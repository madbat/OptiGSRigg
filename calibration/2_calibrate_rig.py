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
import sys
from pathlib import Path

APP_ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = APP_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from charuco_rig_calibration import (  # noqa: E402
    CALIBRATED_RIG_PATH,
    DEFAULT_MARKER_MM,
    DEFAULT_SQUARE_MM,
    DJI_DEMO_PROFILE_PATH,
    DJI_RIG_PROFILE_PATH,
    PREVIEW_DIR,
    calibrate_rig_from_videos,
    run_synthetic_calibration_demo,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Kalibrera 3-kamerorigg (ChArUco + OPENCV_FISHEYE + 6-DoF BA).")
    parser.add_argument("--videos", nargs=3, metavar=("LEFT", "CENTER", "RIGHT"), type=Path, help="Tre videofiler: cam1_left cam2_center cam3_right")
    parser.add_argument("--raw-dir", type=Path, help="Mapp innehållande cam1_left.mp4, cam2_center.mp4, cam3_right.mp4")
    parser.add_argument("--ruler-100mm", type=float, default=100.0, help="Uppmätt längd i mm för utskriven 100 mm-linjal (skalkorrigering)")
    parser.add_argument("--square-mm", type=float, default=DEFAULT_SQUARE_MM, help="Rutstorlek i mm (standard: 34.0)")
    parser.add_argument("--marker-mm", type=float, default=DEFAULT_MARKER_MM, help="Markörstorlek i mm (standard: 25.5)")
    parser.add_argument("--fps", type=float, default=50.0, help="Videons bildfrekvens vid ljudsynk (standard: 50.0)")
    parser.add_argument("--no-audio-sync", action="store_true", help="Hoppa över automatisk ljudklapp-synk")
    parser.add_argument("--baseline-mm", type=float, default=300.0, help="Grov nominell baslinje vänster-till-höger i mm (standard: 300.0)")
    parser.add_argument("--output", type=Path, default=None, help="Sökväg att spara profil-JSON till")
    parser.add_argument("--preview-dir", type=Path, default=PREVIEW_DIR, help="Mapp för felsökningsbilder & HTML-rapport")
    parser.add_argument("--demo-synthetic", action="store_true", help="Kör självdemo med simulerad sned rigg (skriver INTE över riktig dji_rig_profile.json)")
    args = parser.parse_args()

    if args.demo_synthetic:
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
