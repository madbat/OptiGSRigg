#!/usr/bin/env python3
"""
Steg 4: Förkontroll (Pre-Flight Verification) innan riktig skanning.
Kontrollerar på ett kort testklipp eller extraherade trior att:
  1. Upplösning och bildförhållande matchar kalibreringen.
  2. Den låsta 6-DoF Essential-matrisen (epipolära linjer) stämmer över scenens punkter,
     så att du direkt ser om ett kamerafäste rubbats >= 1 grad sedan kalibreringen.

Exempel:
  python calibration/4_verify_calibration.py --raw-dir D:/3DGS_Captures/Testklipp/raw
  python calibration/4_verify_calibration.py --project-dir D:/3DGS_Captures/MinScen/processed
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
    DJI_RIG_PROFILE_PATH,
    PREVIEW_DIR,
    verify_calibration_on_scene,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Kontrollera att riggens sparade kalibrering fortfarande stämmer 100%%.")
    parser.add_argument("--rig-profile", type=Path, default=DJI_RIG_PROFILE_PATH, help="Sökväg till dji_rig_profile.json")
    parser.add_argument("--project-dir", type=Path, default=None, help="Befintlig processed/-mapp med extraherade images/")
    parser.add_argument("--raw-dir", type=Path, default=None, help="Råmapp med cam1_left.mp4, cam2_center.mp4, cam3_right.mp4")
    parser.add_argument("--preview-dir", type=Path, default=PREVIEW_DIR, help="Mapp där epipolar_*.jpg sparas")
    args = parser.parse_args()

    if not args.project_dir and not args.raw_dir:
        parser.error("Ange minst en av --project-dir eller --raw-dir.")

    res = verify_calibration_on_scene(
        rig_profile_path=args.rig_profile,
        project_dir=args.project_dir,
        raw_dir=args.raw_dir,
        preview_dir=args.preview_dir,
    )
    print(f"\nResultat: [{res['status']}] {res['summary']}")
    print(f"Upplösning: {res['resolution_message']}")
    print(f"Epipolära medianfel: C1={res['epipolar_median_px'].get('cam1_left')} px, C3={res['epipolar_median_px'].get('cam3_right')} px")
    if res.get("charuco_direct_rms_px") is not None:
        print(f"ChArUco låst RMS i testklippet: {res['charuco_direct_rms_px']} px")
    return 0 if res["status"] != "FAIL" else 2


if __name__ == "__main__":
    raise SystemExit(main())
