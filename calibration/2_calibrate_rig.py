#!/usr/bin/env python3
"""
Steg 2: Kalibrera 3-kameroriggen (DJI Osmo Action 4) med 3 ChArUco-tavlor.
Beräknar både exakta OPENCV_FISHEYE-linsprofiler och riggens 6-DoF-geometri
med 20%% hold-out korsvalidering och 5-punkts GO/NO-GO-bedömning.

Exempel:
  python calibration/2_calibrate_rig.py --videos left_calib.mp4 center_calib.mp4 right_calib.mp4
  python calibration/2_calibrate_rig.py --raw-dir D:/3DGS_Captures/Calib_01/raw
  python calibration/2_calibrate_rig.py --demo-synthetic
"""

from __future__ import annotations

import argparse
import shutil
import sys
import time
from pathlib import Path

APP_ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = APP_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))
if str(APP_ROOT) not in sys.path:
    sys.path.insert(0, str(APP_ROOT))

from charuco_rig_calibration import (  # noqa: E402
    CALIBRATED_RIG_PATH,
    CAM_ORDER,
    DEFAULT_MARKER_MM,
    DEFAULT_SQUARE_MM,
    DJI_DEMO_PROFILE_PATH,
    DJI_RIG_PROFILE_PATH,
    PREVIEW_DIR,
    calibrate_rig_from_videos,
    run_synthetic_calibration_demo,
)


def _print_sd_scan_report(scan_data: dict) -> None:
    print(f"\n=== ANSLUTNA KAMEROR / SD-KORT ({scan_data['connected_count']}/3 via .id-filer) ===")
    for ck in CAM_ORDER:
        ci = scan_data["cameras"].get(ck, {})
        if ci.get("connected"):
            print(
                f"  [OK] {ci['label']:15s} | {ci['id_file']:13s} | Enhet: {ci['drive']} | "
                f"{ci['clip_count']} MP4 ({ci.get('free_gb', 0)}/{ci.get('total_gb', 0)} GB ledigt)"
            )
        else:
            print(f"  [--] {ci.get('label', ck):15s} | {ci.get('id_file', '-'):13s} | Ej ansluten")

    unmarked = scan_data.get("unmarked_dcim") or []
    if unmarked:
        print(f"  [VARNING] Minneskort med DCIM utan .id-fil: {', '.join(unmarked)}")

    part = scan_data.get("partial_calib_session")
    if part:
        present_cams = [c for c, ok in part["present"].items() if ok]
        print(
            f"\n[Pågående kalibreringsmapp] {part['name']} ({part['mp4_count']}/3 klara: {', '.join(present_cams)}) -> {part['raw_dir']}"
        )

    cands = scan_data.get("candidates") or []
    if not cands:
        print("\nHittade inga MP4-kalibreringsklipp på anslutna kort.")
        print("Anslut kamerorna (USB-C filöverföring) eller sätt i MicroSD-kort med .id-fil och kör igen.")
        return

    print(f"\n=== FÖRESLAGNA KALIBRERINGSFILER (parning: {scan_data['pairing_mode']}) ===")
    for idx, c in enumerate(cands, 1):
        star = "★ REKOMMENDERAD" if c.get("recommended") else f"Alternativ #{idx}"
        print(f"\n  [{idx}] {star} (poäng {c['score']}): {c['name']} • {c['mtime_str']}")
        print(f"      Metadata : {c['reason']} • {c['total_mb']} MB")
        for ck in CAM_ORDER:
            f_info = c["files"].get(ck)
            if f_info:
                print(f"        - {ck:11s}: {f_info['path']} ({f_info['size_mb']} MB)")
            else:
                print(f"        - {ck:11s}: (saknas på anslutna kort)")
        for w in c.get("warnings") or []:
            print(f"      ! {w}")

    print("\nKör import + kalibrering av förslag #1 med:")
    print(r"  D:\OptiGSRigg\.venv\Scripts\python.exe calibration\2_calibrate_rig.py --from-sd --ruler-100mm 100.0")


def main() -> int:
    parser = argparse.ArgumentParser(description="Kalibrera 3-kamerorigg (ChArUco + OPENCV_FISHEYE + 6-DoF BA).")
    parser.add_argument("--videos", nargs=3, metavar=("LEFT", "CENTER", "RIGHT"), type=Path, help="Tre videofiler: cam1_left cam2_center cam3_right")
    parser.add_argument("--raw-dir", type=Path, help="Mapp innehållande cam1_left.mp4, cam2_center.mp4, cam3_right.mp4")
    parser.add_argument("--scan-sd", action="store_true", help="Skanna anslutna kameror/SD-kort via .id-filer och lista föreslagna kalibreringsklipp")
    parser.add_argument("--from-sd", action="store_true", help="Importera föreslaget kalibreringsklipp från anslutna SD-kort och starta kalibrering")
    parser.add_argument("--take-index", type=int, default=1, help="Vilket föreslaget klipp (1 = rekommenderad) vid --from-sd")
    parser.add_argument("--session-name", type=str, default=None, help="Mappnamn under captures_root vid --from-sd")
    parser.add_argument("--ruler-100mm", type=float, default=100.0, help="Uppmätt längd i mm för utskriven 100 mm-linjal (skalkorrigering)")
    parser.add_argument("--square-mm", type=float, default=DEFAULT_SQUARE_MM, help="Rutstorlek i mm (standard: 34.0)")
    parser.add_argument("--marker-mm", type=float, default=DEFAULT_MARKER_MM, help="Markörstorlek i mm (standard: 25.5)")
    parser.add_argument("--fps", type=float, default=50.0, help="Videons bildfrekvens vid ljudsynk (standard: 50.0, autodetekteras vid behov)")
    parser.add_argument("--no-audio-sync", action="store_true", help="Hoppa över automatisk ljudklapp-synk")
    parser.add_argument("--baseline-mm", type=float, default=300.0, help="Grov nominell baslinje vänster-till-höger i mm (standard: 300.0)")
    parser.add_argument("--output", type=Path, default=None, help="Sökväg att spara profil-JSON till")
    parser.add_argument("--preview-dir", type=Path, default=PREVIEW_DIR, help="Mapp för felsökningsbilder & HTML-rapport")
    parser.add_argument("--demo-synthetic", action="store_true", help="Kör självdemo med simulerad sned rigg (skriver INTE över riktig dji_rig_profile.json)")
    args = parser.parse_args()

    if args.scan_sd or args.from_sd:
        import app as studio_app

        settings = studio_app.load_app_settings()
        captures_root = Path(settings.get("captures_root") or studio_app._DEFAULT_CAPTURES_ROOT)
        sd_status = studio_app._enrich_sd_status_disk_usage(studio_app.scan_sd_cards())
        scan_data = studio_app.build_calibration_sd_candidates(sd_status, captures_root=captures_root)

        if args.scan_sd:
            _print_sd_scan_report(scan_data)
            return 0

        cands = scan_data.get("candidates") or []
        if not cands:
            _print_sd_scan_report(scan_data)
            return 1
        pick_idx = max(0, min(len(cands) - 1, int(args.take_index) - 1))
        cand = cands[pick_idx]
        part = scan_data.get("partial_calib_session")
        session_name = (
            (args.session_name or "").strip()
            or (part["name"] if part else "")
            or cand.get("suggested_project_name")
            or f"Calib_Rig_{time.strftime('%Y%m%d_%H%M')}"
        )
        raw_dir = captures_root / session_name / "raw"
        proc_dir = captures_root / session_name / "processed"
        raw_dir.mkdir(parents=True, exist_ok=True)
        proc_dir.mkdir(parents=True, exist_ok=True)

        print(f"\n[SD-Import] Valde förslag #{pick_idx + 1}: {cand['name']} -> {raw_dir}")
        for ck in CAM_ORDER:
            src_str = cand["paths"].get(ck) or ""
            if not src_str:
                continue
            src_p = Path(src_str)
            dst_p = raw_dir / f"{ck}.mp4"
            if dst_p.is_file() and dst_p.stat().st_size == src_p.stat().st_size:
                print(f"  [Redan klar] {dst_p.name} ({src_p.stat().st_size / (1024*1024):.1f} MB)")
            else:
                print(f"  [Kopierar] {src_p} -> {dst_p} ({src_p.stat().st_size / (1024*1024):.1f} MB)...", flush=True)
                shutil.copy2(src_p, dst_p)
                print(f"  [OK] {dst_p.name} klar!")

        missing_cams = [ck for ck in CAM_ORDER if not (raw_dir / f"{ck}.mp4").is_file()]
        if missing_cams:
            present_cams = [ck for ck in CAM_ORDER if (raw_dir / f"{ck}.mp4").is_file()]
            print(
                f"\n[Delvis importerat] Klara i {raw_dir}: {', '.join(present_cams)}.\n"
                f"Saknar fortfarande: {', '.join(missing_cams)}.\n"
                f"Koppla in resterande kamerakort och kör om --from-sd --session-name \"{session_name}\" så startar kalibreringen automatiskt!"
            )
            return 0

        videos = {ck: raw_dir / f"{ck}.mp4" for ck in CAM_ORDER}
        out_path = args.output or DJI_RIG_PROFILE_PATH
        eff_fps = float(cand.get("fps") or args.fps or 50.0)
        profile = calibrate_rig_from_videos(
            videos=videos,
            output_profile_path=out_path,
            calibrated_rig_compat_path=CALIBRATED_RIG_PATH if out_path == DJI_RIG_PROFILE_PATH else None,
            preview_dir=args.preview_dir,
            ruler_100mm=args.ruler_100mm,
            square_mm=args.square_mm,
            marker_mm=args.marker_mm,
            fps=eff_fps,
            use_audio_sync=not args.no_audio_sync,
            nominal_baseline_mm=args.baseline_mm,
        )
        print(f"\n[KLAR] Riktig riggprofil sparad till: {out_path}")
    elif args.demo_synthetic:
        out_path = args.output or DJI_DEMO_PROFILE_PATH
        profile = run_synthetic_calibration_demo(
            output_profile_path=out_path,
            preview_dir=args.preview_dir,
            save_compat=False,
        )
        print(f"\n[DEMO OK] Syntetisk kalibreringsprofil sparad till: {out_path}")
    else:
        if args.videos:
            videos = {
                "cam1_left": args.videos[0],
                "cam2_center": args.videos[1],
                "cam3_right": args.videos[2],
            }
        elif args.raw_dir:
            videos = {
                "cam1_left": args.raw_dir / "cam1_left.mp4",
                "cam2_center": args.raw_dir / "cam2_center.mp4",
                "cam3_right": args.raw_dir / "cam3_right.mp4",
            }
        else:
            parser.error("Ange antingen --videos LEFT CENTER RIGHT, --raw-dir PATH eller --demo-synthetic.")

        out_path = args.output or DJI_RIG_PROFILE_PATH
        profile = calibrate_rig_from_videos(
            videos=videos,
            output_profile_path=out_path,
            calibrated_rig_compat_path=CALIBRATED_RIG_PATH if out_path == DJI_RIG_PROFILE_PATH else None,
            preview_dir=args.preview_dir,
            ruler_100mm=args.ruler_100mm,
            square_mm=args.square_mm,
            marker_mm=args.marker_mm,
            fps=args.fps,
            use_audio_sync=not args.no_audio_sync,
            nominal_baseline_mm=args.baseline_mm,
        )
        print(f"\n[KLAR] Riktig riggprofil sparad till: {out_path}")

    print(
        f"Status: {profile['overall_status']} | RMS: {profile['rms_px']} px | "
        f"Hold-out RMS: {profile['holdout_rms_px']} px | Baslinje L-R: {profile['baselines_mm']['left_to_right_mm']} mm"
    )
    for chk in profile.get("checks", []):
        print(f"  - [{chk['status']}] {chk.get('title') or chk.get('label')}: {chk['value']} ({chk['detail']})")
    print(f"Rapport: {profile.get('report_html_path')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
