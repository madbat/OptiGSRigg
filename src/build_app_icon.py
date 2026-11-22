#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bygger OptiGSRigg Studio app-ikonpaket (PNG, flerstorleks-.ico och SVG).
========================================================================
Tar emot valfri kvadratisk/rektangulär källbild (--source) eller ritar en
supersamplad geometrisk 3-kamerarigg + 3DGS-ikon (--procedural), applicerar
mjukt kantutjämnad squircle-alfamask + tunn inre ljusram och exporterar:
  - assets/icons/optigsrigg_512.png  (512x512 RGBA)
  - assets/icons/optigsrigg_192.png  (192x192 RGBA)
  - assets/icons/optigsrigg.ico      (16, 32, 48, 64, 128, 256 px 32-bit PNG-inbäddad ICO)
  - assets/icons/optigsrigg.svg      (64x64 vektorikon)
"""

from __future__ import annotations

import argparse
import struct
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

APP_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT_DIR = APP_ROOT / "assets" / "icons"
ICO_SIZES: Tuple[int, ...] = (16, 32, 48, 64, 128, 256)

OPTIGSRIGG_SVG = """<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64" width="64" height="64">
  <defs>
    <linearGradient id="bgGrad" x1="0" y1="0" x2="1" y2="1">
      <stop offset="0%" stop-color="#171d2c"/>
      <stop offset="100%" stop-color="#0c1018"/>
    </linearGradient>
    <radialGradient id="splatGlow" cx="50%" cy="64%" r="36%">
      <stop offset="0%" stop-color="#22d3ee" stop-opacity="0.95"/>
      <stop offset="45%" stop-color="#388bfd" stop-opacity="0.55"/>
      <stop offset="100%" stop-color="#2ea043" stop-opacity="0"/>
    </radialGradient>
    <linearGradient id="rimStroke" x1="0" y1="0" x2="1" y2="1">
      <stop offset="0%" stop-color="#60a5fa" stop-opacity="0.55"/>
      <stop offset="50%" stop-color="#22d3ee" stop-opacity="0.28"/>
      <stop offset="100%" stop-color="#2ea043" stop-opacity="0.45"/>
    </linearGradient>
  </defs>
  <rect x="2" y="2" width="60" height="60" rx="14" ry="14" fill="url(#bgGrad)" stroke="url(#rimStroke)" stroke-width="1.5"/>
  <!-- Konvergerande ljuskoner från de 3 kamerorna -->
  <polygon points="15,24 32,45 27,47" fill="#388bfd" fill-opacity="0.26"/>
  <polygon points="32,16 34.5,44 29.5,44" fill="#22d3ee" fill-opacity="0.30"/>
  <polygon points="49,24 37,47 32,45" fill="#2ea043" fill-opacity="0.26"/>
  <line x1="15" y1="24" x2="32" y2="45" stroke="#388bfd" stroke-width="1.6" stroke-linecap="round" stroke-opacity="0.85"/>
  <line x1="32" y1="16" x2="32" y2="45" stroke="#22d3ee" stroke-width="1.7" stroke-linecap="round" stroke-opacity="0.9"/>
  <line x1="49" y1="24" x2="32" y2="45" stroke="#2ea043" stroke-width="1.6" stroke-linecap="round" stroke-opacity="0.85"/>
  <!-- Rigg-båge i titan -->
  <path d="M 14 25 Q 32 13 50 25" fill="none" stroke="#64748b" stroke-width="3.2" stroke-linecap="round"/>
  <!-- 3 DJI Action 4 fisheye-enheter -->
  <circle cx="15" cy="24" r="4.6" fill="#1e293b" stroke="#60a5fa" stroke-width="1.4"/>
  <circle cx="15" cy="24" r="1.9" fill="#388bfd"/>
  <circle cx="32" cy="16" r="5.0" fill="#1e293b" stroke="#22d3ee" stroke-width="1.5"/>
  <circle cx="32" cy="16" r="2.1" fill="#22d3ee"/>
  <circle cx="49" cy="24" r="4.6" fill="#1e293b" stroke="#2ea043" stroke-width="1.4"/>
  <circle cx="49" cy="24" r="1.9" fill="#2ea043"/>
  <!-- 3D Gaussian Splatting glödande ellipsoid-kärna -->
  <circle cx="32" cy="45" r="12" fill="url(#splatGlow)"/>
  <ellipse cx="32" cy="45" rx="8.5" ry="5.5" transform="rotate(-14 32 45)" fill="none" stroke="#22d3ee" stroke-width="1.2" stroke-opacity="0.85"/>
  <ellipse cx="32" cy="45" rx="8.5" ry="5.5" transform="rotate(24 32 45)" fill="none" stroke="#388bfd" stroke-width="1.1" stroke-opacity="0.75"/>
  <circle cx="32" cy="45" r="3.4" fill="#e0f2fe"/>
  <circle cx="25" cy="44" r="1.4" fill="#388bfd"/>
  <circle cx="39" cy="46" r="1.4" fill="#2ea043"/>
  <circle cx="32" cy="39.5" r="1.2" fill="#22d3ee"/>
</svg>
"""


def _center_crop_square(img: np.ndarray) -> np.ndarray:
    h, w = img.shape[:2]
    side = min(h, w)
    y0 = (h - side) // 2
    x0 = (w - side) // 2
    return img[y0 : y0 + side, x0 : x0 + side].copy()


def _build_rounded_rect_mask(size: int, pad: int, radius: int) -> np.ndarray:
    """
    Skapar en mjukt kantutjämnad alpha-mask (0..255 uint8) via 2x supersampling
    för en squircle/rundad-kvadrat med marginal `pad` runt kanterna.
    """
    ss = size * 2
    p = pad * 2
    r = radius * 2
    x0, y0 = p, p
    x1, y1 = ss - p - 1, ss - p - 1

    mask_hi = np.zeros((ss, ss), dtype=np.uint8)
    # Centrala rektanglar + 4 fyllda hörn-cirklar
    cv2.rectangle(mask_hi, (x0 + r, y0), (x1 - r, y1), 255, -1)
    cv2.rectangle(mask_hi, (x0, y0 + r), (x1, y1 - r), 255, -1)
    for cx, cy in ((x0 + r, y0 + r), (x1 - r, y0 + r), (x0 + r, y1 - r), (x1 - r, y1 - r)):
        cv2.circle(mask_hi, (cx, cy), r, 255, -1, lineType=cv2.LINE_AA)

    mask = cv2.resize(mask_hi, (size, size), interpolation=cv2.INTER_AREA)
    return mask


def _draw_rounded_rect_stroke(
    canvas_bgra: np.ndarray,
    pad: int,
    radius: int,
    color_bgra: Tuple[int, int, int, int],
    thickness: float = 2.5,
) -> None:
    """Ritar en jämn inre ljusram runt den rundade kvadraten med 2x supersampling."""
    size = canvas_bgra.shape[0]
    ss = size * 2
    p = pad * 2
    r = radius * 2
    x0, y0 = p, p
    x1, y1 = ss - p - 1, ss - p - 1
    thick_ss = max(1, int(round(thickness * 2.0)))

    stroke_hi = np.zeros((ss, ss), dtype=np.uint8)
    cv2.line(stroke_hi, (x0 + r, y0), (x1 - r, y0), 255, thick_ss, cv2.LINE_AA)
    cv2.line(stroke_hi, (x0 + r, y1), (x1 - r, y1), 255, thick_ss, cv2.LINE_AA)
    cv2.line(stroke_hi, (x0, y0 + r), (x0, y1 - r), 255, thick_ss, cv2.LINE_AA)
    cv2.line(stroke_hi, (x1, y0 + r), (x1, y1 - r), 255, thick_ss, cv2.LINE_AA)
    cv2.ellipse(stroke_hi, (x0 + r, y0 + r), (r, r), 0, 180, 270, 255, thick_ss, cv2.LINE_AA)
    cv2.ellipse(stroke_hi, (x1 - r, y0 + r), (r, r), 0, 270, 360, 255, thick_ss, cv2.LINE_AA)
    cv2.ellipse(stroke_hi, (x1 - r, y1 - r), (r, r), 0, 0, 90, 255, thick_ss, cv2.LINE_AA)
    cv2.ellipse(stroke_hi, (x0 + r, y1 - r), (r, r), 0, 90, 180, 255, thick_ss, cv2.LINE_AA)

    stroke_mask = cv2.resize(stroke_hi, (size, size), interpolation=cv2.INTER_AREA).astype(np.float32) / 255.0
    alpha_fg = stroke_mask * (color_bgra[3] / 255.0)
    for c in range(3):
        ch = canvas_bgra[:, :, c].astype(np.float32)
        canvas_bgra[:, :, c] = np.clip(ch * (1.0 - alpha_fg) + float(color_bgra[c]) * alpha_fg, 0, 255).astype(np.uint8)


def render_procedural_icon_bgr(size: int = 1024) -> np.ndarray:
    """
    Reserv- och fristående renderer som skapar en ren, högkontrast
    3-kamerarigg + 3D Gaussian Splatting-ikon i OpenCV/NumPy vid `size x size`.
    """
    ss = size * 2  # 2048x2048 för knivskarp anti-aliasing
    yy, xx = np.mgrid[0:ss, 0:ss].astype(np.float32)
    u = xx / float(ss)
    v = yy / float(ss)

    # Djup obsidian-skiffer bakgrundsgradient + mjuk glöd i mitten
    bg = np.zeros((ss, ss, 3), dtype=np.float32)
    grad = 0.55 * u + 0.45 * v
    bg[:, :, 0] = 24.0 + 18.0 * (1.0 - grad)   # B
    bg[:, :, 1] = 17.0 + 11.0 * (1.0 - grad)   # G
    bg[:, :, 2] = 13.0 + 8.0 * (1.0 - grad)    # R

    # Radial spotlight runt splat-kärnan
    dist_focus = np.sqrt((u - 0.50) ** 2 + (v - 0.65) ** 2)
    glow = np.exp(-(dist_focus ** 2) / (2.0 * 0.19 ** 2))
    bg[:, :, 0] += glow * 90.0
    bg[:, :, 1] += glow * 52.0
    bg[:, :, 2] += glow * 14.0

    # Övre rigg-båge (subtil stålskugga + highlight)
    canvas = np.clip(bg, 0, 255).astype(np.uint8)
    focus = (int(round(ss * 0.50)), int(round(ss * 0.67)))
    cams = [
        (int(round(ss * 0.23)), int(round(ss * 0.37)), (253, 139, 56), "left"),    # Blå #388bfd (BGR)
        (int(round(ss * 0.50)), int(round(ss * 0.25)), (238, 211, 34), "center"),  # Cyan #22d3ee (BGR)
        (int(round(ss * 0.77)), int(round(ss * 0.37)), (67, 160, 46), "right"),    # Grön #2ea043 (BGR)
    ]

    # Konvergerande ljuskoner (additiv/halvtransparent overlay)
    beam_layer = np.zeros_like(canvas, dtype=np.float32)
    for cx, cy, col, _ in cams:
        tri = np.array([[cx, cy], [focus[0] - 58, focus[1]], [focus[0] + 58, focus[1]]], dtype=np.int32)
        tmp = np.zeros_like(canvas)
        cv2.fillConvexPoly(tmp, tri, col, lineType=cv2.LINE_AA)
        tmp = cv2.GaussianBlur(tmp, (0, 0), 34)
        beam_layer += tmp.astype(np.float32) * 0.34
        cv2.line(beam_layer, (cx, cy), focus, tuple(float(c) * 0.85 for c in col), 18, cv2.LINE_AA)

    canvas = np.clip(canvas.astype(np.float32) + beam_layer, 0, 255).astype(np.uint8)

    # Kolfiber-/titanbåge bakom de 3 kamerorna
    arc_pts = []
    for t in np.linspace(0.0, 1.0, 40):
        px = (1 - t) ** 2 * cams[0][0] + 2 * (1 - t) * t * cams[1][0] + t ** 2 * cams[2][0]
        py = (1 - t) ** 2 * (cams[0][1] + 24) + 2 * (1 - t) * t * (cams[1][1] - 55) + t ** 2 * (cams[2][1] + 24)
        arc_pts.append([int(round(px)), int(round(py))])
    arc_arr = np.array(arc_pts, dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(canvas, [arc_arr], False, (96, 82, 70), 46, cv2.LINE_AA)
    cv2.polylines(canvas, [arc_arr], False, (165, 148, 132), 14, cv2.LINE_AA)

    # Ritar de 3 Action-4 fisheye-linsenheterna
    for cx, cy, col, kind in cams:
        r_outer = 128 if kind == "center" else 114
        cv2.circle(canvas, (cx, cy), r_outer + 10, (18, 14, 12), -1, cv2.LINE_AA)
        cv2.circle(canvas, (cx, cy), r_outer, (46, 38, 32), -1, cv2.LINE_AA)
        cv2.circle(canvas, (cx, cy), r_outer - 4, col, 14, cv2.LINE_AA)
        cv2.circle(canvas, (cx, cy), int(r_outer * 0.66), (32, 24, 20), -1, cv2.LINE_AA)
        cv2.circle(canvas, (cx, cy), int(r_outer * 0.42), col, -1, cv2.LINE_AA)
        cv2.circle(canvas, (cx - 22, cy - 22), int(r_outer * 0.16), (255, 250, 242), -1, cv2.LINE_AA)

    # 3D Gaussian Splatting ellipsoid-kärna + omgivande splats
    splat_glow = np.zeros_like(canvas, dtype=np.float32)
    cv2.ellipse(splat_glow, focus, (240, 140), -16, 0, 360, (238, 211, 34), -1, cv2.LINE_AA)
    cv2.ellipse(splat_glow, focus, (220, 125), 28, 0, 360, (253, 139, 56), -1, cv2.LINE_AA)
    splat_glow = cv2.GaussianBlur(splat_glow, (0, 0), 55)
    canvas = np.clip(canvas.astype(np.float32) + splat_glow * 0.55, 0, 255).astype(np.uint8)

    cv2.ellipse(canvas, focus, (215, 118), -16, 0, 360, (238, 211, 34), 12, cv2.LINE_AA)
    cv2.ellipse(canvas, focus, (215, 118), 26, 0, 360, (253, 139, 56), 10, cv2.LINE_AA)
    cv2.ellipse(canvas, focus, (140, 74), 5, 0, 360, (175, 245, 220), 8, cv2.LINE_AA)
    cv2.circle(canvas, focus, 72, (255, 252, 235), -1, cv2.LINE_AA)

    sat_splats = [
        (focus[0] - 180, focus[1] - 25, 26, (253, 139, 56)),
        (focus[0] + 185, focus[1] + 20, 26, (67, 160, 46)),
        (focus[0] - 95, focus[1] + 95, 20, (238, 211, 34)),
        (focus[0] + 105, focus[1] - 92, 22, (253, 175, 80)),
    ]
    for sx, sy, sr, scol in sat_splats:
        cv2.circle(canvas, (sx, sy), sr, scol, -1, cv2.LINE_AA)

    return cv2.resize(canvas, (size, size), interpolation=cv2.INTER_AREA)


def _apply_small_icon_sharpen(bgra: np.ndarray) -> np.ndarray:
    """Diskret uppskärpning på RGB-kanaler för 16/32/48 px Windows-ikoner med intakt alfa."""
    out = bgra.copy()
    rgb = out[:, :, :3]
    blur = cv2.GaussianBlur(rgb, (0, 0), 0.8)
    out[:, :, :3] = cv2.addWeighted(rgb, 1.25, blur, -0.25, 0)
    return out


def _encode_ico_png_bytes(bgra: np.ndarray) -> bytes:
    ok, buf = cv2.imencode(".png", bgra)
    if not ok:
        raise RuntimeError("Kunde inte PNG-koda ikonstorlek för .ico")
    return buf.tobytes()


def write_multi_size_ico(master_bgra: np.ndarray, ico_path: Path, sizes: Tuple[int, ...] = ICO_SIZES) -> None:
    """Skriver standard Windows .ico med 32-bitars RGBA PNG-ramar i flera upplösningar."""
    frames: List[Tuple[int, bytes]] = []
    for sz in sizes:
        resized = cv2.resize(master_bgra, (sz, sz), interpolation=cv2.INTER_AREA)
        if sz <= 48:
            resized = _apply_small_icon_sharpen(resized)
        frames.append((sz, _encode_ico_png_bytes(resized)))

    header = struct.pack("<HHH", 0, 1, len(frames))
    offset = 6 + 16 * len(frames)
    entries = []
    blobs = []
    for sz, png_bytes in frames:
        w_b = 0 if sz >= 256 else sz
        h_b = 0 if sz >= 256 else sz
        entries.append(struct.pack("<BBBBHHII", w_b, h_b, 0, 0, 1, 32, len(png_bytes), offset))
        blobs.append(png_bytes)
        offset += len(png_bytes)

    ico_path.write_bytes(header + b"".join(entries) + b"".join(blobs))


def build_icon_pack(
    source_path: Optional[Path] = None,
    out_dir: Path = DEFAULT_OUT_DIR,
) -> dict:
    """Bygger komplett ikonpaket (512 PNG, 192 PNG, flerstorleks .ico och .svg)."""
    out_dir.mkdir(parents=True, exist_ok=True)
    master_size = 1024
    pad = 18
    radius = 220

    if source_path is not None and source_path.is_file():
        src = cv2.imread(str(source_path), cv2.IMREAD_COLOR)
        if src is None:
            raise RuntimeError(f"Kunde inte läsa källbild: {source_path}")
        sq = _center_crop_square(src)
        bgr = cv2.resize(sq, (master_size, master_size), interpolation=cv2.INTER_AREA)
    else:
        bgr = render_procedural_icon_bgr(master_size)

    alpha = _build_rounded_rect_mask(master_size, pad=pad, radius=radius)
    bgra = np.dstack([bgr, alpha])
    # Håll ytan utanför squircle helt ren & transparent
    bgra[alpha == 0, :3] = 0
    _draw_rounded_rect_stroke(
        bgra,
        pad=pad,
        radius=radius,
        color_bgra=(250, 165, 96, 108),  # rgba(96, 165, 250, 0.42) i BGRA
        thickness=2.5,
    )

    p512 = out_dir / "optigsrigg_512.png"
    p192 = out_dir / "optigsrigg_192.png"
    p64 = out_dir / "optigsrigg_64.png"
    p32 = out_dir / "optigsrigg_32.png"
    pico = out_dir / "optigsrigg.ico"
    psvg = out_dir / "optigsrigg.svg"

    img512 = cv2.resize(bgra, (512, 512), interpolation=cv2.INTER_AREA)
    img192 = cv2.resize(bgra, (192, 192), interpolation=cv2.INTER_AREA)
    img64 = _apply_small_icon_sharpen(cv2.resize(bgra, (64, 64), interpolation=cv2.INTER_AREA))
    img32 = _apply_small_icon_sharpen(cv2.resize(bgra, (32, 32), interpolation=cv2.INTER_AREA))
    cv2.imwrite(str(p512), img512)
    cv2.imwrite(str(p192), img192)
    cv2.imwrite(str(p64), img64)
    cv2.imwrite(str(p32), img32)
    write_multi_size_ico(bgra, pico, ICO_SIZES)
    psvg.write_text(OPTIGSRIGG_SVG, encoding="utf-8")

    return {
        "png_512": str(p512),
        "png_192": str(p192),
        "png_64": str(p64),
        "png_32": str(p32),
        "ico": str(pico),
        "svg": str(psvg),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Bygg OptiGSRigg Studio app-ikon (.ico, .png, .svg).")
    parser.add_argument("--source", type=Path, default=None, help="Sökväg till källbild (PNG/JPG)")
    parser.add_argument("--procedural", action="store_true", help="Generera geometrisk ikon direkt i kod")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR, help="Målmapp för ikonfiler")
    args = parser.parse_args()

    src = None if args.procedural else args.source
    res = build_icon_pack(source_path=src, out_dir=args.out_dir)
    for k, v in res.items():
        print(f"  [OK] {k}: {v}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
