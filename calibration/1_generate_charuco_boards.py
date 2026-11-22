#!/usr/bin/env python3
"""
Steg 1: Skapar 3 unika ChArUco-kalibreringstavlor (.svg + 300 DPI .png)
samt en utskrivbar A4-sats (printable_calibration_kit.html) med 100 mm-linjal.

Exempel:
  python calibration/1_generate_charuco_boards.py
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
    BOARDS_DIR,
    DEFAULT_BOARD_COLS,
    DEFAULT_BOARD_ROWS,
    DEFAULT_MARKER_MM,
    DEFAULT_SQUARE_MM,
    generate_charuco_boards,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Generera 3 unika ChArUco-tavlor och utskriftskit i 100%% A4-skala.")
    parser.add_argument("--out-dir", type=Path, default=BOARDS_DIR, help="Målmapp för tavlor (standard: calibration/boards)")
    parser.add_argument("--square-mm", type=float, default=DEFAULT_SQUARE_MM, help="Rutstorlek i mm (standard: 34.0)")
    parser.add_argument("--marker-mm", type=float, default=DEFAULT_MARKER_MM, help="ArUco-markörstorlek i mm (standard: 25.5)")
    parser.add_argument("--cols", type=int, default=DEFAULT_BOARD_COLS, help="Antal kolumner (standard: 7)")
    parser.add_argument("--rows", type=int, default=DEFAULT_BOARD_ROWS, help="Antal rader (standard: 5)")
    args = parser.parse_args()

    res = generate_charuco_boards(
        out_dir=args.out_dir,
        square_mm=args.square_mm,
        marker_mm=args.marker_mm,
        cols=args.cols,
        rows=args.rows,
    )
    print(f"[ChArUco] Genererade {len(res['files'])} filer i: {res['out_dir']}")
    print(f"[ChArUco] Tavlor: {args.cols}x{args.rows} ({args.square_mm:.1f} mm rutor, {args.marker_mm:.1f} mm ArUco)")
    print(f"[ChArUco] Öppna för A4-utskrift (100% / Verklig storlek): {res['kit_html']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
