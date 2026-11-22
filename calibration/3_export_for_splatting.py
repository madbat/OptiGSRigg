#!/usr/bin/env python3
"""
Steg 3: Exportera riktig skanning med LÅST rigg-kalibrering till
LichtFeld Studio 0.5.4.1 (3DGUT) & Spirula Studio 2026.10.30.

Arbetsflöde:
  1. Kontrollerar upplösning & epipolär geometri mot dji_rig_profile.json (valfritt att hoppa över).
  2. Ljudklapp-synkar och extraherar skarpa bild-trior (valfritt D-Log M LUT).
  3. Kör SfM (Spirula eller COLMAP) med låsta OPENCV_FISHEYE-linser.
  4. Låser sparse/0 exakt till riggens uppmätta 6-DoF-geometri & 1:1 meterskala.

Exempel:
  python calibration/3_export_for_splatting.py ^
    --rig-profile config/dji_rig_profile.json ^
    --videos scene_left.mp4 scene_center.mp4 scene_right.mp4 ^
    --output-dir D:/3DGS_Captures/MinScen/processed ^
    --fps 2.0
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import tempfile
from pathlib import Path

APP_ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = APP_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from charuco_rig_calibration import (  # noqa: E402
    CAM_ORDER,
    DJI_RIG_PROFILE_PATH,
    PREVIEW_DIR,
    enforce_calibrated_rig_on_sparse_model,
    verify_calibration_on_scene,
)
import process_3dgs_rig as pr  # noqa: E402


def _load_settings() -> dict:
    cfg = APP_ROOT / "config" / "settings.json"
    if cfg.exists():
        try:
            return json.loads(cfg.read_text(encoding="utf-8"))
        except Exception:
            pass
    return {}


def main() -> int:
    settings = _load_settings()
    default_spirula = settings.get("spirula_exe", pr.DEFAULT_SPIRULA_BIN)
    default_colmap = settings.get("colmap_exe", pr.DEFAULT_COLMAP_BIN)

    parser = argparse.ArgumentParser(description="Exportera rigg-låst 3DGUT-dataset för LichtFeld Studio & Spirula Studio.")
    parser.add_argument("--rig-profile", type=Path, default=DJI_RIG_PROFILE_PATH, help="Sökväg till dji_rig_profile.json")
    parser.add_argument("--videos", nargs=3, metavar=("LEFT", "CENTER", "RIGHT"), type=Path, help="Tre synkade scen-videor")
    parser.add_argument("--raw-dir", type=Path, help="Mapp med cam1_left.mp4, cam2_center.mp4, cam3_right.mp4")
    parser.add_argument("--output-dir", required=True, type=Path, help="Målmapp för färdig dataset (images/, sparse/0/, rig_config.json)")
    parser.add_argument("--fps", type=float, default=2.5, help="Ungefär hur många bild-trior per sekund som ska plockas ut (default: 2.5 -> 0.4s fönster)")
    parser.add_argument("--video-fps", type=float, default=50.0, help="Kamerornas inspelade FPS (default: 50.0)")
    parser.add_argument("--skip-start-sec", type=float, default=3.0, help="Sekunder efter klappen innan bildplock startar (default: 3.0)")
    parser.add_argument("--lut", type=Path, default=None, help="Valfri .cube LUT (t.ex. D-Log M -> Rec.709)")
    parser.add_argument("--no-audio-sync", action="store_true", help="Hoppa över automatisk ljudsynk")
    parser.add_argument("--skip-preflight", action="store_true", help="Hoppa över automatisk epipolär kontroll")
    parser.add_argument("--skip-extract", action="store_true", help="Använd befintliga images/ i --output-dir")
    parser.add_argument("--sfm-backend", choices=["auto", "spirula", "colmap", "none"], default="auto", help="SfM-motor (default: auto)")
    parser.add_argument("--spirula-bin", type=str, default=default_spirula, help="Sökväg till spirula.exe")
    parser.add_argument("--colmap-bin", type=str, default=default_colmap, help="Sökväg till colmap.exe")
    args = parser.parse_args()

    if not args.rig_profile.exists():
        raise FileNotFoundError(
            f"Hittade ingen riggprofil ({args.rig_profile}). Kör Steg 2 (2_calibrate_rig.py) med 3 ChArUco-tavlor först!"
        )

    profile = json.loads(args.rig_profile.read_text(encoding="utf-8"))
    out_dir = args.output_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    baseline_mm = float(profile.get("baselines_mm", {}).get("left_to_right_mm", 300.0))

    # Kopiera riggprofilen in i projektmappen för 100 % spårbarhet
    (out_dir / "dji_rig_profile.json").write_text(json.dumps(profile, indent=2, ensure_ascii=False), encoding="utf-8")

    # 1. Videofil-upplösning & extraktion
    if not args.skip_extract:
        if args.videos:
            videos = {
                "cam1_left": Path(args.videos[0]).resolve(),
                "cam2_center": Path(args.videos[1]).resolve(),
                "cam3_right": Path(args.videos[2]).resolve(),
            }
        elif args.raw_dir:
            videos = {ck: (args.raw_dir / f"{ck}.mp4").resolve() for ck in CAM_ORDER}
        else:
            parser.error("Ange --videos LEFT CENTER RIGHT eller --raw-dir PATH (eller --skip-extract).")

        for ck, vp in videos.items():
            if not vp.is_file():
                raise FileNotFoundError(f"Saknar video för {ck}: {vp}")

        if args.no_audio_sync:
            offsets = {ck: 0 for ck in CAM_ORDER}
        else:
            offsets = pr.compute_frame_offsets(videos, out_dir, fps=args.video_fps)

        window_sec = max(0.08, 1.0 / max(0.1, float(args.fps)))
        pr.extract_sharpest_synced_triplets(
            videos=videos,
            offsets=offsets,
            output_images_dir=out_dir / "images",
            lut_path=args.lut,
            fps=args.video_fps,
            window_sec=window_sec,
            skip_start_sec=args.skip_start_sec,
            spirula_bin=args.spirula_bin,
            extract_backend="auto",
        )

    # 2. Förkontroll mot upplösning & rubbat fäste
    if not args.skip_preflight:
        ver = verify_calibration_on_scene(
            rig_profile_path=args.rig_profile,
            project_dir=out_dir,
            preview_dir=PREVIEW_DIR,
        )
        print(f"[Förkontroll] {ver['status']}: {ver['summary']}")

    # 3. Generera rig_config.json / rig_config_legacy.json baserat på uppmätt ChArUco-profil
    pr.generate_colmap_rig_config(out_dir / "rig_config.json", mode="calibrated", custom_geom=profile)

    # 4. Kör SfM + tvinga låst 1:1 rigg-geometri på sparse/0
    if args.sfm_backend != "none":
        pr.run_colmap_rig_sfm(
            out_dir,
            mode="calibrated",
            baseline_mm=baseline_mm,
            colmap_bin=args.colmap_bin,
            spirula_bin=args.spirula_bin,
            sfm_backend=args.sfm_backend,
        )
        sparse_dir = out_dir / "sparse" / "0"
        if (sparse_dir / "cameras.bin").exists():
            lock_res = enforce_calibrated_rig_on_sparse_model(sparse_dir, out_dir, profile)
            print(
                f"[Export KLAR] Låsta bilder: {lock_res['num_images']} (räddade {lock_res['rescued_images']}), "
                f"3D-punkter: {lock_res['num_points3d']}, RMS: {lock_res['rms_px']} px"
            )

    print("\n" + "=" * 72)
    print(f"FÄRDIGT DATASET FOR 3DGUT: {out_dir}")
    print("  - LichtFeld Studio (0.5.4.1): Dra in mappen ovan och välj 3DGUT-träning.")
    print("  - Spirula Studio (2026.10.30): Öppna/kör 3DGUT direkt mot denna COLMAP-mapp.")
    print("=" * 72)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
