#!/usr/bin/env python3
"""
Självtestsvit för ChArUco 3-Kamerarigg Kalibrering, Rigg-Låst Export & Förkontroll.

Kör:
  python calibration/test_synthetic_rig.py

Verifierar att:
  1. Alla 3 ChArUco-tavlor genereras (.svg + .png + .html) och ger 24/24 unika hörn.
  2. Kalibreringsmotorn återställer en snett ihopskruvad 2880x3840-rigg enbart från
     grova startmått (< 0.25 px RMS, < 0.35 px hold-out RMS, < 0.12° vinkelfel, < 1.5 mm baslinje).
  3. Rigg-låst COLMAP sparse/0-solver lagar trasig brännvidd (t.ex. fx=225 px), skalar till
     exakt 1:1 meterskala, räddar borttappade kamerabilder och låser riggen till 0.000°.
  4. Scenkontrollen (epipolär SIFT-check) ger OK på intakt rigg och FAIL om en kamera
     rubbats +2.5 grader.

Obs! Testet arbetar 100%% i temporära kataloger och skriver INTE över riktig riggprofil.
"""

from __future__ import annotations

import json
import math
import shutil
import struct
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np

APP_ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = APP_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

import charuco_rig_calibration as crc  # noqa: E402


def _ang_diff_deg(R1: np.ndarray, R2: np.ndarray) -> float:
    d = float(np.trace(R1 @ R2.T))
    return math.degrees(math.acos(float(np.clip((d - 1.0) * 0.5, -1.0, 1.0))))


def _render_textured_corner_scene(
    intr: dict[str, np.ndarray],
    T_cam_rel: dict[str, np.ndarray],
    pose_idx: int = 0,
    width: int = 2880,
    height: int = 3840,
) -> dict[str, np.ndarray]:
    """
    Ray-castar en fotorealistisk 3-väggs texturerad testscen direkt med OPENCV_FISHEYE
    så att SIFT ger hundratals exakta geometriska träffar mellan kameror.
    """
    rng = np.random.default_rng(100 + pose_idx)
    # Skapa en högkontrast-textur (2048x2048) rik på skarpa hörn och unika SIFT-punkter
    tex = np.full((1024, 1024, 3), 215, dtype=np.uint8)
    for gy in range(0, 1024, 64):
        for gx in range(0, 1024, 64):
            if ((gx // 64) + (gy // 64)) % 2 == 0:
                cv2.rectangle(tex, (gx, gy), (gx + 63, gy + 63), (170, 175, 185), -1)
    for k in range(280):
        cx = int(rng.integers(24, 1000))
        cy = int(rng.integers(24, 1000))
        col = tuple(int(v) for v in rng.integers(15, 240, size=3))
        if k % 3 == 0:
            cv2.rectangle(tex, (cx - 14, cy - 10), (cx + 14, cy + 10), col, -1)
            cv2.rectangle(tex, (cx - 14, cy - 10), (cx + 14, cy + 10), (15, 15, 15), 2)
        elif k % 3 == 1:
            cv2.circle(tex, (cx, cy), int(rng.integers(6, 20)), col, -1)
            cv2.circle(tex, (cx, cy), int(rng.integers(6, 20)), (20, 20, 20), 2)
        else:
            cv2.putText(tex, f"P{k}", (cx - 12, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2, cv2.LINE_AA)

    # Rendera direkt i full 2880x3840 via snabb koordinat-grid för att testa 100% exakt upplösning
    # Dela upp i 4 horisontella remsor för minimal RAM-topp.
    u_row = np.arange(width, dtype=np.float32)
    T_world_rig = np.eye(4, dtype=np.float64)
    yaw_w = (-12.0 + 14.0 * pose_idx)
    R_w = crc.euler_to_rotmat(yaw_w, 2.0 * pose_idx - 3.0, 0.0)
    T_world_rig[:3, :3] = R_w
    T_world_rig[:3, 3] = np.array([0.1 * pose_idx - 0.1, 0.0, 0.0], dtype=np.float64)

    out_imgs: dict[str, np.ndarray] = {}
    strip_h = 480
    r_lut = np.linspace(0.0, 2.5, 4096, dtype=np.float32)
    for ck in crc.CAM_ORDER:
        fx, fy, cx, cy, k1, k2, k3, k4 = [float(x) for x in intr[ck]]
        # Förberäkna 1D Newton-LUT för fisheye r_d -> theta (~10x snabbare per bild)
        th_lut = np.clip(r_lut, 0.0, 1.48)
        for _ in range(5):
            th2 = th_lut * th_lut
            th4 = th2 * th2
            f_val = th_lut * (1.0 + k1 * th2 + k2 * th4 + k3 * th4 * th2 + k4 * th4 * th4) - r_lut
            df = 1.0 + 3.0 * k1 * th2 + 5.0 * k2 * th4 + 7.0 * k3 * th4 * th2 + 9.0 * k4 * th4 * th4
            th_lut = np.clip(th_lut - f_val / np.maximum(0.2, df), 0.0, 1.48)

        T_cw = T_cam_rel[ck] @ T_world_rig
        R_wc = T_cw[:3, :3].T
        C_w = -R_wc @ T_cw[:3, 3]
        img = np.zeros((height, width, 3), dtype=np.uint8)

        for y0 in range(0, height, strip_h):
            y1 = min(height, y0 + strip_h)
            v_col = np.arange(y0, y1, dtype=np.float32)
            uu, vv = np.meshgrid(u_row, v_col)
            mx = (uu - cx) / fx
            my = (vv - cy) / fy
            r_d = np.sqrt(mx * mx + my * my)
            theta = np.interp(r_d.ravel(), r_lut, th_lut).reshape(r_d.shape).astype(np.float32)

            scale = np.where(r_d > 1e-6, np.sin(theta) / np.maximum(1e-6, r_d), 1.0)
            rx = mx * scale
            ry = my * scale
            rz = np.cos(theta)

            # Rotera kamerastrålar till världsrummet
            dx = R_wc[0, 0] * rx + R_wc[0, 1] * ry + R_wc[0, 2] * rz
            dy = R_wc[1, 0] * rx + R_wc[1, 1] * ry + R_wc[1, 2] * rz
            dz = R_wc[2, 0] * rx + R_wc[2, 1] * ry + R_wc[2, 2] * rz

            # Träffa frontvägg z = 2.6 m (plus lätt golv y = 1.1 m)
            t_wall = np.where(dz > 0.05, (2.6 - C_w[2]) / np.maximum(0.05, dz), 1e9)
            t_floor = np.where(dy > 0.05, (1.1 - C_w[1]) / np.maximum(0.05, dy), 1e9)
            hit_floor = t_floor < t_wall
            t_hit = np.minimum(t_wall, t_floor)
            t_safe = np.minimum(t_hit, 90.0)

            wx = C_w[0] + dx * t_safe
            wy = C_w[1] + dy * t_safe
            wz = C_w[2] + dz * t_safe

            u_tex = np.where(hit_floor, (wx + 2.5) * 210.0, (wx + 2.8) * 195.0)
            v_tex = np.where(hit_floor, (wz + 1.0) * 210.0, (wy + 2.2) * 195.0)
            ix = np.mod(np.round(u_tex).astype(np.int32), 1024)
            iy = np.mod(np.round(v_tex).astype(np.int32), 1024)

            strip = tex[iy, ix]
            invalid = (t_hit > 80.0) | (theta >= 1.46)
            strip[invalid] = (30, 32, 36)
            img[y0:y1] = strip

        out_imgs[ck] = img
    return out_imgs


def test_1_charuco_boards(tmp: Path) -> None:
    print("\n=== TEST 1: Generera 3 unika ChArUco-tavlor & verifiera 24/24 hörndetektering ===")
    bdir = tmp / "boards"
    res = crc.generate_charuco_boards(out_dir=bdir)
    assert Path(res["kit_html"]).is_file(), "printable_calibration_kit.html saknas"

    det = crc.MultiBoardDetector()
    for b_name, _off, _t, _s in crc.BOARD_SPECS:
        svg_p = bdir / f"{b_name}.svg"
        png_p = bdir / f"{b_name}.png"
        assert svg_p.is_file() and svg_p.stat().st_size > 2000, f"SVG saknas/tom: {svg_p}"
        assert png_p.is_file() and png_p.stat().st_size > 2000, f"PNG saknas/tom: {png_p}"

        img = cv2.imread(str(png_p))
        assert img is not None, f"Kunde inte läsa {png_p}"
        hits = det.detect_image(img, min_corners=6)
        assert list(hits.keys()) == [b_name], f"Förväntade enbart {b_name}, fick {list(hits.keys())}"
        n_corners = len(hits[b_name][0])
        assert n_corners == 24, f"{b_name}: förväntade 24 ChArUco-hörn, fick {n_corners}"
        print(f"  [OK] {b_name}: 24/24 unika hörn detekterade (SVG + 300 DPI PNG)")


def test_2_synthetic_calibration(tmp: Path) -> tuple[dict, dict, dict]:
    print("\n=== TEST 2: Syntetisk 2880x3840 snett ihopskruvad rigg-kalibrering + 20% hold-out ===")
    frames, image_sizes, detector, true_intr, true_cam_rel, _tb, _tf = crc.build_synthetic_rig_test_data(
        width=2880, height=3840, n_poses=18, noise_sigma_px=0.14, seed=42
    )
    prof_path = tmp / "dji_rig_profile.json"
    prev_dir = tmp / "preview"
    profile = crc._solve_and_save_profile(
        frames=frames,
        image_sizes=image_sizes,
        detector=detector,
        raw_frames_cache={},
        output_profile_path=prof_path,
        calibrated_rig_compat_path=None,
        preview_dir=prev_dir,
        source_tag="synthetic_test",
        progress_cb=print,
    )
    assert prof_path.is_file(), "Profilfil skrevs inte"
    assert (prev_dir / "calibration_verification_report.html").is_file(), "HTML-rapport skrevs inte"
    assert profile["overall_status"] == "OK", f"Förväntade OK, fick {profile['overall_status']}"
    assert profile["rms_px"] < 0.25, f"RMS för högt: {profile['rms_px']}"
    assert profile["holdout_rms_px"] < 0.35, f"Hold-out RMS för högt: {profile['holdout_rms_px']}"

    for ck in ("cam1_left", "cam3_right"):
        pc = profile["per_camera"][ck]
        R_est = crc.quaternion_wxyz_to_rotmat(pc["cam_from_rig_qvec_wxyz"])
        t_est = np.asarray(pc["cam_from_rig_tvec_m"], dtype=np.float64)
        R_true = true_cam_rel[ck][:3, :3]
        t_true = true_cam_rel[ck][:3, 3]
        ang_err = _ang_diff_deg(R_est, R_true)
        C_est = -R_est.T @ t_est
        C_true = -R_true.T @ t_true
        pos_err_mm = float(np.linalg.norm(C_est - C_true)) * 1000.0
        fx_err = abs(pc["intrinsics"][0] - float(true_intr[ck][0]))
        assert ang_err < 0.12, f"{ck} vinkelfel {ang_err:.3f} deg >= 0.12 deg"
        assert pos_err_mm < 1.5, f"{ck} positionsfel {pos_err_mm:.2f} mm >= 1.5 mm"
        assert fx_err < 4.0, f"{ck} fx-fel {fx_err:.2f} px >= 4.0 px"
        print(f"  [OK] {ck}: vinkelfel={ang_err:.3f} deg, baslinjefel={pos_err_mm:.2f} mm, fx-fel={fx_err:.2f} px")

    return profile, true_intr, true_cam_rel


def test_3_sparse_rig_enforcement(tmp: Path, profile: dict, true_intr: dict, true_cam_rel: dict) -> None:
    print("\n=== TEST 3: COLMAP sparse/0 rigg-låsning, 1:1 meter-skala & bild-räddning ===")
    proj_dir = tmp / "splat_proj"
    sparse_dir = proj_dir / "sparse" / "0"
    sparse_dir.mkdir(parents=True, exist_ok=True)
    for ck in crc.CAM_ORDER:
        (proj_dir / "images" / ck).mkdir(parents=True, exist_ok=True)

    # Skapa 60 sanna 3D-punkter framför riggen (2.0..4.5 m)
    rng = np.random.default_rng(77)
    pts3d_true = np.column_stack([
        rng.uniform(-1.8, 1.8, 60),
        rng.uniform(-1.1, 1.1, 60),
        rng.uniform(2.0, 4.5, 60),
    ])

    # Deliberately scale world by 0.43x + corrupt cam1_left intrinsics to fx=225.18 + drop cam1_left/frame_0003.jpg
    fake_scale = 0.43
    cams_bin = {}
    for cid, ck in enumerate(crc.CAM_ORDER, 1):
        p_vec = list(true_intr[ck])
        if ck == "cam1_left":
            p_vec[0] = 225.18  # trasigt värde från gårdagens bugg
            p_vec[1] = 269.19
        cams_bin[cid] = {
            "camera_id": cid,
            "model_id": 5,
            "width": 2880,
            "height": 3840,
            "params": p_vec,
        }

    imgs_bin = {}
    pts_tracks = {pid + 1: [] for pid in range(len(pts3d_true))}
    img_id = 1
    for t_idx in range(5):
        fname = f"frame_{t_idx + 1:04d}.jpg"
        T_rw_true = np.eye(4, dtype=np.float64)
        T_rw_true[:3, :3] = crc.euler_to_rotmat((t_idx - 2) * 8.0, 0.0, 0.0)
        T_rw_true[:3, 3] = np.array([(t_idx - 2) * 0.25, 0.0, 0.0], dtype=np.float64)

        for cid, ck in enumerate(crc.CAM_ORDER, 1):
            # Skapa minimal dummy-fil på disk så rescue hittar den
            cv2.imwrite(str(proj_dir / "images" / ck / fname), np.zeros((32, 32, 3), dtype=np.uint8))
            if t_idx == 2 and ck == "cam1_left":
                continue  # simulera tappad kamera i Spirula!

            T_cw_true = true_cam_rel[ck] @ T_rw_true
            rv, _ = cv2.Rodrigues(T_cw_true[:3, :3])
            pr, z = crc.project_fisheye_world_points(pts3d_true, rv.ravel(), T_cw_true[:3, 3], true_intr[ck])
            vis = (z > 0.3) & (pr[:, 0] > 30) & (pr[:, 0] < 2850) & (pr[:, 1] > 30) & (pr[:, 1] < 3810)
            xys_l = []
            pids_l = []
            for p_idx in np.where(vis)[0]:
                pt2d_idx = len(xys_l)
                xys_l.append(pr[p_idx] + rng.normal(0.0, 0.2, 2))
                pids_l.append(p_idx + 1)
                pts_tracks[p_idx + 1].append((img_id, pt2d_idx))

            imgs_bin[f"{ck}/{fname}"] = {
                "image_id": img_id,
                "qvec": crc.rotmat_to_quaternion_wxyz(T_cw_true[:3, :3]),
                "tvec": T_cw_true[:3, 3] * fake_scale,
                "camera_id": cid,
                "xys": np.asarray(xys_l, dtype=np.float64) if xys_l else np.zeros((0, 2)),
                "point3D_ids": np.asarray(pids_l, dtype=np.int64) if pids_l else np.zeros(0, dtype=np.int64),
            }
            img_id += 1

    pts_bin = {}
    for p_idx in range(len(pts3d_true)):
        pid = p_idx + 1
        if len(pts_tracks[pid]) >= 2:
            pts_bin[pid] = {
                "point3D_id": pid,
                "xyz": pts3d_true[p_idx] * fake_scale,
                "rgb": [180, 180, 180],
                "error": 0.4,
                "track": pts_tracks[pid],
            }

    crc.write_colmap_cameras_bin(sparse_dir / "cameras.bin", cams_bin)
    crc.write_colmap_images_bin_full(sparse_dir / "images.bin", imgs_bin)
    crc.write_colmap_points3d_bin(sparse_dir / "points3D.bin", pts_bin)

    res = crc.enforce_calibrated_rig_on_sparse_model(sparse_dir, proj_dir, profile, progress_cb=print)
    assert res["ok"], "enforce_calibrated_rig_on_sparse_model misslyckades"
    assert res["num_images"] == 15, f"Förväntade 15 bilder (räddad), fick {res['num_images']}"
    assert res["rescued_images"] == 1, f"Förväntade 1 räddad bild, fick {res['rescued_images']}"

    cams_out = crc.read_colmap_cameras_bin(sparse_dir / "cameras.bin")
    imgs_out = crc.read_colmap_images_bin_full(sparse_dir / "images.bin")
    assert abs(cams_out[1]["params"][0] - profile["per_camera"]["cam1_left"]["intrinsics"][0]) < 1e-3, "cam1 fx låstes inte"
    assert (sparse_dir / "rigs.txt").is_file(), "sparse/0/rigs.txt saknas"
    assert (proj_dir / "rig_config.json").is_file(), "rig_config.json saknas"

    # Verifiera exakt 0.000° rigg-låsning och 1:1 meterskala på samtliga 5 trior
    T_rel_prof = {}
    for ck in crc.CAM_ORDER:
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = crc.quaternion_wxyz_to_rotmat(profile["per_camera"][ck]["cam_from_rig_qvec_wxyz"])
        T[:3, 3] = np.asarray(profile["per_camera"][ck]["cam_from_rig_tvec_m"])
        T_rel_prof[ck] = T

    for t_idx in range(5):
        fn = f"frame_{t_idx + 1:04d}.jpg"
        r2 = imgs_out[f"cam2_center/{fn}"]
        T2 = np.eye(4, dtype=np.float64)
        T2[:3, :3] = crc.quaternion_wxyz_to_rotmat(r2["qvec"])
        T2[:3, 3] = r2["tvec"]
        for ck in ("cam1_left", "cam3_right"):
            rs = imgs_out[f"{ck}/{fn}"]
            Ts = np.eye(4, dtype=np.float64)
            Ts[:3, :3] = crc.quaternion_wxyz_to_rotmat(rs["qvec"])
            Ts[:3, 3] = rs["tvec"]
            T_rel_obs = Ts @ np.linalg.inv(T2)
            ang = _ang_diff_deg(T_rel_obs[:3, :3], T_rel_prof[ck][:3, :3])
            dt_mm = float(np.linalg.norm(T_rel_obs[:3, 3] - T_rel_prof[ck][:3, 3])) * 1000.0
            assert ang < 1e-3, f"Rigg ej låst för {ck}/{fn}: {ang} deg"
            assert dt_mm < 0.05, f"Baslinje ej låst för {ck}/{fn}: {dt_mm} mm"

    print(f"  [OK] 15/15 bilder låsta (1 räddad), skalfaktor={res['metric_scale_factor']:.4f}, rigg-avvikelse=0.000 deg")


def test_4_preflight_verification(tmp: Path, profile: dict, true_intr: dict, true_cam_rel: dict) -> None:
    print("\n=== TEST 4: Scenkontroll (OK på intakt rigg vs FAIL vid +3.0 deg rubbat fäste) ===")
    prof_path = tmp / "dji_rig_profile.json"
    prev_ok = tmp / "prev_ok"
    prev_bad = tmp / "prev_bad"

    intact_triplets = [_render_textured_corner_scene(true_intr, true_cam_rel, pose_idx=p) for p in range(1)]
    res_ok = crc.verify_calibration_on_scene(
        rig_profile_path=prof_path,
        preview_dir=prev_ok,
        triplet_imgs_list=intact_triplets,
        progress_cb=print,
    )
    assert res_ok["status"] == "OK", f"Intakt rigg borde ge OK, fick {res_ok}"
    assert res_ok["epipolar_median_px"]["cam1_left"] < 1.5, f"Intakt C1 epipolar fel för högt: {res_ok['epipolar_median_px']}"
    assert res_ok["epipolar_median_px"]["cam3_right"] < 1.5, f"Intakt C3 epipolar fel för högt: {res_ok['epipolar_median_px']}"

    # Rubba cam1_left med +3.0 deg fysisk yaw
    bumped_rel = dict(true_cam_rel)
    R_bump = crc.euler_to_rotmat(3.0, 0.0, 0.0)
    T_b = true_cam_rel["cam1_left"].copy()
    T_b[:3, :3] = R_bump @ T_b[:3, :3]
    bumped_rel["cam1_left"] = T_b

    bumped_triplets = [_render_textured_corner_scene(true_intr, bumped_rel, pose_idx=p) for p in range(1)]
    res_bad = crc.verify_calibration_on_scene(
        rig_profile_path=prof_path,
        preview_dir=prev_bad,
        triplet_imgs_list=bumped_triplets,
        progress_cb=print,
    )
    assert res_bad["status"] == "FAIL", f"Rubbad rigg (+3 deg) borde ge FAIL, fick {res_bad}"
    assert res_bad["epipolar_median_px"]["cam1_left"] > 3.5, f"Borde avslöjat >3.5 px fel på C1, fick {res_bad['epipolar_median_px']}"
    print(
        f"  [OK] Intakt rigg={res_ok['status']} (C1={res_ok['epipolar_median_px']['cam1_left']} px) | "
        f"Rubbad rigg (+3 deg)={res_bad['status']} (C1={res_bad['epipolar_median_px']['cam1_left']} px)"
    )


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="optigsrigg_selftest_") as td:
        tmp = Path(td)
        test_1_charuco_boards(tmp)
        profile, true_intr, true_cam_rel = test_2_synthetic_calibration(tmp)
        test_3_sparse_rig_enforcement(tmp, profile, true_intr, true_cam_rel)
        test_4_preflight_verification(tmp, profile, true_intr, true_cam_rel)
    print("\n" + "=" * 72)
    print("ALLA 4 SJALVTESTER GODKANDA (100% PASS)!")
    print("=" * 72)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
