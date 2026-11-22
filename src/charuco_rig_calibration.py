#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OptiGSRigg – ChArUco 3-Kamerarigg Kalibrering, Förkontroll & Rig-Låst Export
===============================================================================
Matematiskt exakt kalibrering av 3x DJI Osmo Action 4 i 90° porträttläge
(stöder både 4K 4:3 2880x3840 och 16:9 2160x3840, Wide FOV, RockSteady OFF)
med Kannala-Brandt `OPENCV_FISHEYE` [fx, fy, cx, cy, k1, k2, k3, k4] och 6-DoF
rigg-geometri relativt `cam2_center`.

Innehåller:
  1. Utskrivbara A4 ChArUco-tavlor (3 unika ID-set + 100 mm linjal-kontroll).
  2. Global gles Bundle Adjustment över ChArUco-trior med 20 % hold-out korsvalidering.
  3. Förkontroll på valfritt testklipp / riktig scen (epipolär fisheye-geometri + ChArUco).
  4. Rig-låst 1:1 meterskalad post-lösning på COLMAP/Spirula `sparse/0` (.bin + rigs.txt).
"""

from __future__ import annotations

import datetime
import html as html_lib
import json
import math
import os
import shutil
import struct
import sys
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import cv2
import numpy as np
from scipy import optimize, sparse

APP_ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = Path(__file__).resolve().parent
CONFIG_DIR = APP_ROOT / "config"
CALIB_DIR = APP_ROOT / "calibration"
BOARDS_DIR = CALIB_DIR / "boards"
PREVIEW_DIR = CONFIG_DIR / "calib_preview"

DJI_RIG_PROFILE_PATH = CONFIG_DIR / "dji_rig_profile.json"
DJI_DEMO_PROFILE_PATH = CONFIG_DIR / "dji_rig_profile_demo.json"
CALIBRATED_RIG_PATH = CONFIG_DIR / "calibrated_rig.json"

CAM_ORDER = ("cam1_left", "cam2_center", "cam3_right")
CAM_LABELS = {
    "cam1_left": "Cam 1 Vänster",
    "cam2_center": "Cam 2 Mitten (Ref)",
    "cam3_right": "Cam 3 Höger",
}

# 3 distinkta ChArUco-tavlor i A4-liggande (7x5 rutor -> 24 inre hörn, 17 markörer vardera)
DEFAULT_BOARD_COLS = 7
DEFAULT_BOARD_ROWS = 5
DEFAULT_SQUARE_MM = 34.0
DEFAULT_MARKER_MM = 25.5

BOARD_SPECS: List[Tuple[str, int, str, str]] = [
    (
        "board_0_left",
        0,
        "Tavla 0 – VÄNSTER VÄGG",
        "Placera på vänster vägg/skiva i hörnet så att Cam 1 (-45°) och gärna Cam 2 ser den.",
    ),
    (
        "board_1_center",
        20,
        "Tavla 1 – MITTEN (LITE HÖGRE UPP)",
        "Placera rakt fram i hörnet lite högre upp för att matcha Cam 2 Mitten (+8° uppåt).",
    ),
    (
        "board_2_right",
        40,
        "Tavla 2 – HÖGER VÄGG",
        "Placera på höger vägg/skiva i hörnet så att Cam 3 (+45°) och gärna Cam 2 ser den.",
    ),
]


# =============================================================================
# 1. ROTATIONS- & RIGG-MATEMATIK (COLMAP KONVENTION: R_cw, t_cw = -R_cw @ C)
# =============================================================================

def euler_to_rotmat(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    """Bygger rotationsmatris R = R_roll @ R_pitch @ R_yaw i COLMAP-kameraram (X=höger, Y=ner, Z=framåt)."""
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    R_yaw = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]], dtype=np.float64)
    R_pitch = np.array([[1.0, 0.0, 0.0], [0.0, cp, -sp], [0.0, sp, cp]], dtype=np.float64)
    R_roll = np.array([[cr, -sr, 0.0], [sr, cr, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return R_roll @ R_pitch @ R_yaw


def rotmat_to_euler_deg(R: np.ndarray) -> Tuple[float, float, float]:
    """Extraherar (yaw_deg, pitch_deg, roll_deg) från R = R_roll @ R_pitch @ R_yaw."""
    pitch = math.asin(float(np.clip(R[2, 1], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-6:
        yaw = math.atan2(-R[2, 0], R[2, 2])
        roll = math.atan2(-R[0, 1], R[1, 1])
    else:
        yaw = math.atan2(R[0, 2], R[0, 0])
        roll = 0.0
    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


def rotmat_to_quaternion_wxyz(R: np.ndarray) -> List[float]:
    """Konverterar 3x3 rotationsmatris till enhetskvaternion [qw, qx, qy, qz]."""
    trace = float(R[0, 0] + R[1, 1] + R[2, 2])
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    q = np.array([qw, qx, qy, qz], dtype=np.float64)
    q /= max(1e-12, float(np.linalg.norm(q)))
    if q[0] < 0:
        q = -q
    return [float(x) for x in q]


def quaternion_wxyz_to_rotmat(q: List[float] | np.ndarray) -> np.ndarray:
    """Konverterar kvaternion [qw, qx, qy, qz] till 3x3 rotationsmatris."""
    q_arr = np.asarray(q, dtype=np.float64)
    q_arr = q_arr / max(1e-12, float(np.linalg.norm(q_arr)))
    qw, qx, qy, qz = q_arr
    return np.array(
        [
            [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
            [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
            [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


def _se3_from_rvec_tvec(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return T


def _rvec_tvec_from_se3(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    return rvec.reshape(3).astype(np.float64), T[:3, 3].copy().astype(np.float64)


def _project_so3(M: np.ndarray) -> np.ndarray:
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1.0
        R = U @ Vt
    return R


def _median_se3(transforms: List[np.ndarray]) -> np.ndarray:
    if len(transforms) == 1:
        return transforms[0].copy()
    R_stack = np.stack([T[:3, :3] for T in transforms], axis=0)
    t_stack = np.stack([T[:3, 3] for T in transforms], axis=0)
    R_med = _project_so3(np.median(R_stack, axis=0))
    t_med = np.median(t_stack, axis=0)
    T_out = np.eye(4, dtype=np.float64)
    T_out[:3, :3] = R_med
    T_out[:3, 3] = t_med
    return T_out


def nominal_rig_rel_se3(side_cam: str, baseline_mm: float = 300.0, flipped: bool = False) -> np.ndarray:
    """
    Bygger passiv SE(3)-transform T_side_from_center enligt CAD-specifikation:
      Standard (`flipped=False`): Mitten +8° upp, Vänster -150mm/-45°/-12°, Höger +150mm/+45°/-12°
      180° Flip (`flipped=True`): Mitten -8° ned, Sidor ±150mm/±45°/+12° upp
    """
    half_m = (baseline_mm / 1000.0) / 2.0
    center_tilt = -8.0 if flipped else +8.0
    side_tilt = +12.0 if flipped else -12.0
    if side_cam == "cam1_left":
        pos = np.array([-half_m, 0.0, 0.0], dtype=np.float64)
        phys_yaw = +45.0 if flipped else -45.0
    elif side_cam == "cam3_right":
        pos = np.array([+half_m, 0.0, 0.0], dtype=np.float64)
        phys_yaw = -45.0 if flipped else +45.0
    else:
        return np.eye(4, dtype=np.float64)

    R_center_rig = euler_to_rotmat(0.0, -center_tilt, 0.0)
    R_side_rig = euler_to_rotmat(-phys_yaw, -side_tilt, 0.0)
    R_rel = R_side_rig @ R_center_rig.T
    t_rel = -R_rel @ pos
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R_rel
    T[:3, 3] = t_rel
    return T


def physical_angles_from_rel(R_rel: np.ndarray, center_tilt_up_deg: float = 8.0) -> Tuple[float, float, float]:
    """Konverterar relativ `R_side_from_center` till bommens fysiska (Yaw, Lutning, Roll)."""
    R_center_rig = euler_to_rotmat(0.0, -center_tilt_up_deg, 0.0)
    R_side_rig = R_rel @ R_center_rig
    ry, rp, rr = rotmat_to_euler_deg(R_side_rig)
    return -ry, -rp, rr


# =============================================================================
# 2. EXAKT OPENCV_FISHEYE PROJEKTION (KANNALA-BRANDT 8 PARAMS)
# =============================================================================

def project_fisheye_world_points(
    pts_world: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    cam_params: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Projicerar 3D-punkter (N, 3) till pixlar (N, 2) via passiv transform (rvec eller 3x3 R, tvec)
    och Kannala-Brandt `OPENCV_FISHEYE` [fx, fy, cx, cy, k1, k2, k3, k4].
    Returnerar (pixels (N,2), Z_cam (N,)).
    """
    fx, fy, cx, cy, k1, k2, k3, k4 = [float(x) for x in cam_params]
    rv_arr = np.asarray(rvec, dtype=np.float64)
    if rv_arr.shape == (3, 3):
        R = rv_arr
    else:
        R, _ = cv2.Rodrigues(rv_arr.reshape(3, 1))
    t = np.asarray(tvec, dtype=np.float64).reshape(3)
    pts_cam = pts_world @ R.T + t
    z = pts_cam[:, 2]
    safe_z = np.where(z > 1e-4, z, 1e-4)
    a = pts_cam[:, 0] / safe_z
    b = pts_cam[:, 1] / safe_z
    r = np.hypot(a, b)
    theta = np.arctan(r)
    t2 = theta * theta
    theta_d = theta * (1.0 + k1 * t2 + k2 * (t2 ** 2) + k3 * (t2 ** 3) + k4 * (t2 ** 4))
    scale = np.where(r > 1e-8, theta_d / np.maximum(r, 1e-8), 1.0)
    u = fx * scale * a + cx
    v = fy * scale * b + cy
    return np.column_stack([u, v]), z


def undistort_fisheye_to_rays(pts_px: np.ndarray, cam_params: np.ndarray) -> np.ndarray:
    """Avdistorderar pixelkoordinater (N, 2) till normaliserade enhetsstrålar (N, 3) via cv2.fisheye."""
    fx, fy, cx, cy, k1, k2, k3, k4 = [float(x) for x in cam_params]
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    D = np.array([k1, k2, k3, k4], dtype=np.float64)
    pts_in = np.asarray(pts_px, dtype=np.float64).reshape(-1, 1, 2)
    norm = cv2.fisheye.undistortPoints(pts_in, K, D).reshape(-1, 2)
    rays = np.column_stack([norm[:, 0], norm[:, 1], np.ones(len(norm), dtype=np.float64)])
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    return rays


# =============================================================================
# 3. STEG 1: CHARUCO-TAVLOR (SVG + PNG + UTSKRIVBAR A4 HTML-KIT MED 100MM LINJAL)
# =============================================================================

def _make_charuco_subdict(id_offset: int, count: int = 20) -> Any:
    base = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    return cv2.aruco.Dictionary(base.bytesList[id_offset : id_offset + count].copy(), 4)


def make_charuco_board(
    id_offset: int,
    square_mm: float = DEFAULT_SQUARE_MM,
    marker_mm: float = DEFAULT_MARKER_MM,
    cols: int = DEFAULT_BOARD_COLS,
    rows: int = DEFAULT_BOARD_ROWS,
) -> Any:
    sub_dict = _make_charuco_subdict(id_offset, count=20)
    return cv2.aruco.CharucoBoard(
        (int(cols), int(rows)),
        float(square_mm) / 1000.0,
        float(marker_mm) / 1000.0,
        sub_dict,
    )


def _render_charuco_svg(
    board_name: str,
    title: str,
    subtitle: str,
    id_offset: int,
    square_mm: float,
    marker_mm: float,
    cols: int = DEFAULT_BOARD_COLS,
    rows: int = DEFAULT_BOARD_ROWS,
) -> str:
    """
    Genererar exakt 1:1 millimeter-skalad SVG identisk med OpenCV `CharucoBoard.generateImage()`.
    Cell (r=0, c=0) överst till vänster är svart; vita rutor har centrerad 4x4+kant ArUco-markör.
    """
    sub_dict = _make_charuco_subdict(id_offset, count=20)
    board_w = cols * square_mm
    board_h = rows * square_mm
    margin_x = 18.0
    margin_top = 12.0
    margin_bottom = 14.0
    page_w = board_w + 2.0 * margin_x
    page_h = board_h + margin_top + margin_bottom
    pad = (square_mm - marker_mm) / 2.0
    cell_sz = marker_mm / 6.0  # 4x4 bits + 1 cell svart ram runt om = 6x6

    elems: List[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{page_w:.2f}mm" height="{page_h:.2f}mm" '
        f'viewBox="0 0 {page_w:.3f} {page_h:.3f}">',
        f'<rect x="0" y="0" width="{page_w:.3f}" height="{page_h:.3f}" fill="#ffffff"/>',
        f'<text x="{margin_x:.2f}" y="7.5" font-family="Segoe UI, Arial, sans-serif" '
        f'font-size="4.4" font-weight="700" fill="#111111">{html_lib.escape(title)}</text>',
        f'<text x="{page_w - margin_x:.2f}" y="7.5" text-anchor="end" font-family="Consolas, monospace" '
        f'font-size="3.4" fill="#444444">{cols}x{rows} • Ruta {square_mm:.1f} mm • Markör {marker_mm:.1f} mm • ID {id_offset}..{id_offset+16}</text>',
        f'<g transform="translate({margin_x:.3f},{margin_top:.3f})">',
        f'<rect x="0" y="0" width="{board_w:.3f}" height="{board_h:.3f}" fill="#ffffff" stroke="#d0d7de" stroke-width="0.25"/>',
    ]

    local_marker_id = 0
    for r in range(rows):
        for c in range(cols):
            x0 = c * square_mm
            y0 = r * square_mm
            if (r + c) % 2 == 0:
                elems.append(
                    f'<rect x="{x0:.3f}" y="{y0:.3f}" width="{square_mm:.3f}" height="{square_mm:.3f}" fill="#000000"/>'
                )
            else:
                mk_grid = cv2.aruco.generateImageMarker(sub_dict, local_marker_id, 6)
                mx = x0 + pad
                my = y0 + pad
                for br in range(6):
                    for bc in range(6):
                        if int(mk_grid[br, bc]) < 128:
                            elems.append(
                                f'<rect x="{mx + bc * cell_sz:.3f}" y="{my + br * cell_sz:.3f}" '
                                f'width="{cell_sz + 0.02:.3f}" height="{cell_sz + 0.02:.3f}" fill="#000000"/>'
                            )
                local_marker_id += 1

    elems.append("</g>")
    # 100.0 mm kontroll-linjal längst ner i vänsterkant
    ruler_y = margin_top + board_h + 6.5
    elems.extend(
        [
            f'<g transform="translate({margin_x:.3f},{ruler_y:.3f})">',
            '<line x1="0" y1="0" x2="100" y2="0" stroke="#000000" stroke-width="0.6"/>',
            '<line x1="0" y1="-2.5" x2="0" y2="2.5" stroke="#000000" stroke-width="0.6"/>',
            '<line x1="50" y1="-1.5" x2="50" y2="1.5" stroke="#000000" stroke-width="0.4"/>',
            '<line x1="100" y1="-2.5" x2="100" y2="2.5" stroke="#000000" stroke-width="0.6"/>',
            '<text x="103" y="1.2" font-family="Consolas, monospace" font-size="3.4" font-weight="700" fill="#000000">'
            'KONTROLLMÅTT: Exakt 100.0 mm (Mät med linjal efter utskrift!)</text>',
            "</g>",
            f'<text x="{page_w - margin_x:.2f}" y="{ruler_y + 1.2:.2f}" text-anchor="end" '
            f'font-family="Segoe UI, Arial, sans-serif" font-size="3.0" fill="#555555">{html_lib.escape(board_name)}</text>',
            "</svg>",
        ]
    )
    return "\n".join(elems)


_PRINTABLE_KIT_TEMPLATE = """<!DOCTYPE html>
<html lang="sv">
<head>
  <meta charset="UTF-8" />
  <title>OptiGSRigg ChArUco-Kalibreringskit (3 Tavlor • 100% A4)</title>
  <style>
    @page {
      size: A4 landscape;
      margin: 5mm;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      padding: 16px;
      font-family: "Segoe UI", Arial, sans-serif;
      background: #141517;
      color: #e2e8f0;
    }
    .print-bar {
      max-width: 287mm;
      margin: 0 auto 12px auto;
      padding: 10px 14px;
      background: #1b1c1f;
      border: 1px solid #388bfd;
      border-radius: 5px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      flex-wrap: wrap;
      gap: 10px;
    }
    .print-actions {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
    }
    .print-btn {
      background: #2563eb;
      color: #fff;
      border: 1px solid #60a5fa;
      padding: 7px 13px;
      font-size: 12.5px;
      font-weight: 600;
      border-radius: 4px;
      cursor: pointer;
    }
    .print-btn.secondary {
      background: #0f766e;
      border-color: #2dd4bf;
    }
    .screen-sim {
      max-width: 287mm;
      margin: 0 auto 18px auto;
      background: #181a20;
      border: 1px solid #3b82f6;
      border-radius: 6px;
      padding: 14px 16px;
      color: #e2e8f0;
    }
    .sim-top {
      display: flex;
      justify-content: space-between;
      align-items: center;
      flex-wrap: wrap;
      gap: 8px;
      margin-bottom: 10px;
    }
    .sim-title {
      font-size: 14.5px;
      font-weight: 700;
      color: #93c5fd;
    }
    .sim-play-btn {
      background: #16a34a;
      color: #fff;
      border: 1px solid #4ade80;
      padding: 6px 12px;
      border-radius: 4px;
      font-size: 12px;
      font-weight: 700;
      cursor: pointer;
    }
    .sim-play-btn.playing {
      background: #d97706;
      border-color: #fbbf24;
    }
    .sim-phases {
      display: grid;
      grid-template-columns: repeat(6, 1fr);
      gap: 6px;
      margin-bottom: 12px;
    }
    .phase-btn {
      background: #111318;
      color: #cbd5e1;
      border: 1px solid #334155;
      border-radius: 4px;
      padding: 6px 8px;
      font-size: 11px;
      font-weight: 600;
      text-align: left;
      line-height: 1.3;
      cursor: pointer;
    }
    .phase-btn.active {
      background: #1d4ed8;
      color: #ffffff;
      border-color: #60a5fa;
      box-shadow: 0 0 0 1px #60a5fa;
    }
    .sim-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
    }
    .sim-card {
      background: #0f1116;
      border: 1px solid #2d3340;
      border-radius: 5px;
      padding: 8px 10px;
    }
    .sim-card-lbl {
      font-size: 11.5px;
      font-weight: 700;
      color: #cbd5e1;
      margin-bottom: 6px;
      display: flex;
      justify-content: space-between;
    }
    .sim-action-bar {
      margin-top: 10px;
      background: #111827;
      border: 1px solid #334155;
      border-left: 4px solid #38bdf8;
      border-radius: 4px;
      padding: 9px 12px;
      display: grid;
      grid-template-columns: 1fr 1.4fr;
      gap: 12px;
      font-size: 12px;
      line-height: 1.45;
    }
    .sim-rule-strip {
      grid-column: 1 / -1;
      background: rgba(234, 179, 8, 0.12);
      border: 1px solid #ca8a04;
      color: #fde047;
      padding: 5px 9px;
      border-radius: 3px;
      font-size: 11.5px;
      font-weight: 600;
    }
    .page {
      width: 287mm;
      min-height: 196mm;
      margin: 0 auto 18px auto;
      background: #ffffff;
      color: #111827;
      padding: 6mm 7.5mm;
      border-radius: 2px;
      page-break-after: always;
      break-after: page;
    }
    .page:last-child {
      page-break-after: auto;
      break-after: auto;
    }
    h1 { font-size: 15.5pt; margin: 0 0 5px 0; color: #0f172a; }
    h2 { font-size: 11pt; margin: 7px 0 4px 0; color: #1d4ed8; }
    p, li { font-size: 9pt; line-height: 1.35; }
    .cam-boxes {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 6px;
      margin: 5px 0 6px 0;
    }
    .cam-box {
      border: 1px solid #94a3b8;
      background: #f8fafc;
      border-radius: 3px;
      padding: 4px 7px;
      font-size: 8.5pt;
      line-height: 1.3;
    }
    .cam-box strong { display: block; color: #0f172a; font-size: 9pt; }
    .storyboard-top {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 4px;
      margin: 4px 0 5px 0;
    }
    .tl-step {
      flex: 1;
      background: #eff6ff;
      border: 1px solid #93c5fd;
      border-radius: 3px;
      padding: 4px 5px;
      font-size: 8pt;
      font-weight: 700;
      color: #1e3a8a;
      text-align: center;
    }
    .tl-arrow { font-weight: 700; color: #64748b; font-size: 10pt; }
    .story-rule {
      background: #fef9c3;
      border: 1px solid #eab308;
      color: #713f12;
      padding: 4px 8px;
      border-radius: 3px;
      font-size: 8.8pt;
      font-weight: 700;
      margin-bottom: 6px;
      text-align: center;
    }
    .story-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 7px;
    }
    .story-card {
      border: 1px solid #cbd5e1;
      background: #f8fafc;
      border-radius: 4px;
      padding: 5px 7px;
    }
    .story-hdr {
      font-size: 9.3pt;
      font-weight: 700;
      color: #0f172a;
      margin-bottom: 3px;
      border-bottom: 1px solid #e2e8f0;
      padding-bottom: 2px;
    }
    .story-txt {
      font-size: 8.1pt;
      line-height: 1.3;
      color: #334155;
      margin-top: 3px;
    }
    .board-page {
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      padding: 2mm;
    }
    .board-note {
      font-size: 9.5pt;
      color: #374151;
      margin-bottom: 2mm;
    }
    .svg-wrap svg {
      display: block;
      max-width: 100%;
      height: auto;
    }
    @media print {
      body { background: #ffffff; padding: 0; }
      .print-bar, .screen-sim { display: none !important; }
      body.print-boards-only .guide-page { display: none !important; }
      .page { margin: 0; border-radius: 0; width: 100%; min-height: auto; }
    }
  </style>
</head>
<body>
  <div class="print-bar">
    <div>
      <strong>OptiGSRigg ChArUco-Kalibreringskit (3 Tavlor)</strong> —
      Rutstorlek <code>__SQUARE_MM__ mm</code> • Skriv ut i <strong>Liggande A4, Faktisk storlek / 100 % skala</strong> (utan "Anpassa till sida").
    </div>
    <div class="print-actions">
      <button class="print-btn" onclick="printKit(false)">Skriv ut Komplett Kit (2 Visuella Guidesidor + 3 Tavlor)</button>
      <button class="print-btn secondary" onclick="printKit(true)">Skriv ENDAST ut de 3 Tavlorna (3 sidor • 100%)</button>
    </div>
  </div>

  <!-- INTERAKTIV FILM-SIMULATOR (VISAS PÅ SKÄRMEN, DÖLJS VID UTSKRIFT) -->
  <section class="screen-sim">
    <div class="sim-top">
      <div class="sim-title">🎥 Interaktiv Film-Simulator — Klicka eller spela upp för att se EXAKT hur ni står, vrider riggen &amp; vad kameraskärmarna visar (Stopp 1–17)</div>
      <button id="simPlayBtn" class="sim-play-btn" onclick="toggleSimAuto()">▶ Spela upp automatiskt</button>
    </div>

    <div class="sim-phases" id="simPhaseBtns">
      <button class="phase-btn active" data-p="0"><strong>0. Start</strong><br/>REC + 1 Klapp 👏</button>
      <button class="phase-btn" data-p="1"><strong>Stopp 1–3</strong><br/>Punkt A (1.1m) • Mitten / Låg / Hög</button>
      <button class="phase-btn" data-p="2"><strong>Stopp 4–5</strong><br/>Punkt A (1.1m) • Tilta &amp; Ratt ±15°</button>
      <button class="phase-btn" data-p="3"><strong>Stopp 6–8</strong><br/>Punkt A • VRID 40° HÖGER (Cam 1+2)</button>
      <button class="phase-btn" data-p="4"><strong>Stopp 9–11</strong><br/>Punkt A • VRID 40° VÄNSTER (Cam 2+3)</button>
      <button class="phase-btn" data-p="5"><strong>Stopp 12–17</strong><br/>Kliv till Punkt B (0.6m) • SkärmHÖRN!</button>
    </div>

    <div class="sim-grid">
      <div class="sim-card">
        <div class="sim-card-lbl">
          <span>RUMMET UPPIFRÅN — Var står fötterna &amp; hur pekar riggen?</span>
          <span id="simRoomTag" style="color:#60a5fa;">Punkt A (1.1 m)</span>
        </div>
        <svg id="simRoomSvg" viewBox="0 0 430 230" style="width:100%; height:auto; display:block;"></svg>
      </div>
      <div class="sim-card">
        <div class="sim-card-lbl">
          <span>SÅ HÄR SKA KAMERORNAS 3 SKÄRMAR SE UT I DETTA STEG! (Stående 4:3)</span>
          <span id="simCamTag" style="color:#4ade80;">Synkroniserad vy</span>
        </div>
        <svg id="simScreensSvg" viewBox="0 0 430 230" style="width:100%; height:auto; display:block;"></svg>
      </div>
    </div>

    <div class="sim-action-bar">
      <div>
        <div style="color:#93c5fd; font-weight:700; margin-bottom:2px;">Var står dina fötter?</div>
        <div id="simFeetTxt"></div>
      </div>
      <div>
        <div style="color:#4ade80; font-weight:700; margin-bottom:2px;">Vad gör du med riggen just nu?</div>
        <div id="simDoTxt"></div>
      </div>
      <div class="sim-rule-strip">
        VIKTIGT REGEL: Filma EN enda sammanhängande video (~45–55 sek) — tryck INTE stopp mellan lägena! Flytta riggen lugnt och håll den HELT STILLA i 1–2 sekunder ("tusen-ett, tusen-två") vid varje stopp.
      </div>
    </div>
  </section>

  <!-- UTSKRIVBAR GUIDESIDA 1: FÖRBEREDELSE & 90-GRADERS HÖRNMONTERING -->
  <section class="page guide-page">
    <h1>Sida 1: Kamerainställningar, 100 mm-linjal &amp; Montering i 90°-hörn</h1>
    <div class="cam-boxes">
      <div class="cam-box"><strong>1. Format &amp; Orientering</strong><code>4K 4:3</code> Stående (2880×3840) — identiskt på alla 3 kameror som vid scanning!</div>
      <div class="cam-box"><strong>2. Synfält (FOV)</strong><strong>Wide</strong> (rå fisheye). Stäng alltid AV <code>Dewarp</code>!</div>
      <div class="cam-box"><strong>3. Stabilisering (EIS)</strong><strong>AV</strong> (RockSteady / HorizonSteady helt avstängd — absolut krav!).</div>
      <div class="cam-box"><strong>4. FPS &amp; Slutare</strong><code>50/60 fps</code>, manuell slutare <code>1/250s–1/500s</code>, låst WB &amp; ISO.</div>
    </div>

    <p style="margin:2px 0 4px 0;"><strong>Kontrollmät linjalen efter utskrift:</strong> Limma/tejpa de 3 tavlorna helt plant på styv kartong, kapaskiva eller direkt mot slät vägg. Mät svarta <strong>100.0 mm-kontrollinjen</strong> längst ner på tavlan med en vanlig linjal. Visar den t.ex. <code>98.5 mm</code>, skriv in <code>98.5</code> i OptiGSRigg (<code>Uppmätt 100mm-linjal</code>) så blir skalan exakt!</p>

    <h2>Montering i ett 90°-hörn — VIK ALDRIG EN TAVLA!</h2>
    <div style="background:#fef2f2; border:1px solid #ef4444; color:#991b1b; padding:4px 8px; border-radius:3px; font-size:8.8pt; margin-bottom:4px;">
      <strong>VIKTIGT:</strong> Böj eller vik <strong>ALDRIG</strong> Tavla 1 inuti själva hörnvecket — alla 3 A4-papper måste vara <strong>100 % plana</strong>! Välj <strong>Metod A</strong> (direkt på de 2 väggarna) eller <strong>Metod B</strong> (plan skiva snett över hörnet):
    </div>
    <svg width="268mm" height="58mm" viewBox="0 0 820 175" style="display:block; margin:2px auto; border:1px solid #cbd5e1; background:#f8fafc;">
      <!-- METOD A -->
      <rect x="6" y="6" width="398" height="163" fill="#ffffff" stroke="#cbd5e1" stroke-width="1"/>
      <text x="16" y="22" font-family="Segoe UI, Arial" font-size="11.5" font-weight="700" fill="#1e293b">Metod A: Vanligt 90°-hörn (direkt på 2 väggar)</text>
      <polyline points="80,128 205,34 330,128" fill="none" stroke="#94a3b8" stroke-width="5"/>
      <text x="205" y="29" font-family="Segoe UI, Arial" font-size="9" font-weight="700" fill="#ef4444" text-anchor="middle">Hörnveck 90° — håll tomt (vik INTE!)</text>
      <line x1="95" y1="117" x2="145" y2="79" stroke="#2563eb" stroke-width="7" stroke-linecap="butt"/>
      <text x="90" y="88" font-family="Segoe UI, Arial" font-size="10" font-weight="700" fill="#1e40af" text-anchor="end">Tavla 0</text>
      <text x="90" y="99" font-family="Segoe UI, Arial" font-size="8.5" fill="#475569" text-anchor="end">~40 cm ut</text>
      <line x1="215" y1="42" x2="262" y2="77" stroke="#16a34a" stroke-width="7" stroke-linecap="butt"/>
      <text x="248" y="48" font-family="Segoe UI, Arial" font-size="10" font-weight="700" fill="#166534">Tavla 1 (Mitten)</text>
      <text x="248" y="59" font-family="Segoe UI, Arial" font-size="8.5" fill="#15803d">5–10 cm från veck, +20 cm upp</text>
      <line x1="278" y1="89" x2="328" y2="126" stroke="#d97706" stroke-width="7" stroke-linecap="butt"/>
      <text x="334" y="108" font-family="Segoe UI, Arial" font-size="10" font-weight="700" fill="#92400e">Tavla 2</text>
      <text x="334" y="119" font-family="Segoe UI, Arial" font-size="8.5" fill="#475569">~55 cm ut</text>
      <circle cx="205" cy="144" r="7" fill="#111827"/>
      <line x1="205" y1="144" x2="122" y2="100" stroke="#2563eb" stroke-width="1.5" stroke-dasharray="4,3"/>
      <line x1="205" y1="144" x2="238" y2="62" stroke="#16a34a" stroke-width="1.5" stroke-dasharray="4,3"/>
      <line x1="205" y1="144" x2="302" y2="108" stroke="#d97706" stroke-width="1.5" stroke-dasharray="4,3"/>
      <text x="205" y="162" font-family="Segoe UI, Arial" font-size="9.5" font-weight="700" fill="#111827" text-anchor="middle">Punkt A (~1.1 m framför hörnet)</text>

      <!-- METOD B -->
      <rect x="416" y="6" width="398" height="163" fill="#ffffff" stroke="#cbd5e1" stroke-width="1"/>
      <text x="426" y="22" font-family="Segoe UI, Arial" font-size="11.5" font-weight="700" fill="#1e293b">Metod B: Plan skiva snett (45°) över 90°-hörnet</text>
      <polyline points="490,128 615,34 740,128" fill="none" stroke="#94a3b8" stroke-width="5"/>
      <line x1="560" y1="75" x2="670" y2="75" stroke="#475569" stroke-width="3" stroke-dasharray="4,2"/>
      <line x1="582" y1="75" x2="648" y2="75" stroke="#16a34a" stroke-width="7"/>
      <text x="615" y="62" font-family="Segoe UI, Arial" font-size="10" font-weight="700" fill="#166534" text-anchor="middle">Tavla 1 på plan skiva (+20 cm upp)</text>
      <line x1="502" y1="119" x2="552" y2="81" stroke="#2563eb" stroke-width="7"/>
      <text x="498" y="92" font-family="Segoe UI, Arial" font-size="10" font-weight="700" fill="#1e40af" text-anchor="end">Tavla 0</text>
      <line x1="678" y1="81" x2="728" y2="119" stroke="#d97706" stroke-width="7"/>
      <text x="734" y="92" font-family="Segoe UI, Arial" font-size="10" font-weight="700" fill="#92400e">Tavla 2</text>
      <circle cx="615" cy="144" r="7" fill="#111827"/>
      <line x1="615" y1="144" x2="528" y2="100" stroke="#2563eb" stroke-width="1.5" stroke-dasharray="4,3"/>
      <line x1="615" y1="144" x2="615" y2="80" stroke="#16a34a" stroke-width="1.5" stroke-dasharray="4,3"/>
      <line x1="615" y1="144" x2="702" y2="100" stroke="#d97706" stroke-width="1.5" stroke-dasharray="4,3"/>
      <text x="615" y="162" font-family="Segoe UI, Arial" font-size="9.5" font-weight="700" fill="#111827" text-anchor="middle">Även 3 skivor ställda i U-form på ett bord fungerar perfekt!</text>
    </svg>
    <p style="margin:5px 0 0 0; font-size:8.8pt; color:#334155;"><strong>Tips:</strong> Se till att belysningen är jämn utan skarpa lampreflexer/blänk i pappret, och rör inte tavlorna eller riggens skruvar mellan det att ni tryckt REC och ni har avslutat klippet.</p>
  </section>

  <!-- UTSKRIVBAR GUIDESIDA 2: VISUELL FILM-STORYBOARD / FUSKBLAD -->
  <section class="page guide-page">
    <h1 style="font-size:14.5pt; margin-bottom:4px;">Sida 2: Visuellt Inspelningsschema — Håll detta blad framför er när ni filmar (~50 sek i ETT klipp)</h1>

    <div class="storyboard-top">
      <div class="tl-step">1. Tryck REC ● på alla 3</div><span class="tl-arrow">➔</span>
      <div class="tl-step">2. Klappa HÖGT 1 gång 👏</div><span class="tl-arrow">➔</span>
      <div class="tl-step">3. Punkt A (1.1 m): Stopp 1–5</div><span class="tl-arrow">➔</span>
      <div class="tl-step">4. Punkt A: Vrid ±40° (Stopp 6–11)</div><span class="tl-arrow">➔</span>
      <div class="tl-step">5. Punkt B (0.6 m): Hörnen (12–17)</div><span class="tl-arrow">➔</span>
      <div class="tl-step">6. Tryck STOPP ■</div>
    </div>

    <div class="story-rule">
      Stäng INTE av kamerorna mellan stoppen! Flytta lugnt ➔ Frys HELT STILLA i 1–2 sek ("tusen-ett, tusen-två") ➔ flytta vidare.
    </div>

    <div class="story-grid">
      <!-- RUTA A: GOLVKARTA -->
      <div class="story-card">
        <div class="story-hdr">A. GOLVKARTA UPPIFRÅN: Var står dina fötter?</div>
        <svg viewBox="0 0 390 128" style="width:100%; height:41mm; display:block; background:#fff; border:1px solid #e2e8f0;">
          <polyline points="55,95 195,15 335,95" fill="none" stroke="#94a3b8" stroke-width="5"/>
          <line x1="78" y1="81" x2="128" y2="53" stroke="#2563eb" stroke-width="7"/>
          <text x="72" y="64" font-size="9.5" font-weight="700" fill="#1e40af" text-anchor="end">Tavla 0</text>
          <line x1="205" y1="21" x2="252" y2="48" stroke="#16a34a" stroke-width="7"/>
          <text x="234" y="25" font-size="9.5" font-weight="700" fill="#166534">Tavla 1 (+20cm)</text>
          <line x1="274" y1="60" x2="322" y2="87" stroke="#d97706" stroke-width="7"/>
          <text x="328" y="74" font-size="9.5" font-weight="700" fill="#92400e">Tavla 2</text>

          <line x1="195" y1="15" x2="195" y2="112" stroke="#cbd5e1" stroke-width="1.2" stroke-dasharray="3,3"/>
          <circle cx="195" cy="60" r="14" fill="#dcfce7" stroke="#16a34a" stroke-width="2"/>
          <text x="195" y="63" font-size="9.5" font-weight="700" fill="#166534" text-anchor="middle">B</text>
          <text x="216" y="63" font-size="9" font-weight="700" fill="#15803d">Punkt B = 0.6 m (halvt kliv fram • Stopp 12–17)</text>

          <circle cx="195" cy="105" r="15" fill="#dbeafe" stroke="#2563eb" stroke-width="2.2"/>
          <text x="195" y="109" font-size="10" font-weight="700" fill="#1e40af" text-anchor="middle">A</text>
          <text x="217" y="108" font-size="9" font-weight="700" fill="#1e40af">Punkt A = 1.1 m (1 stort kliv ut • Stopp 1–11)</text>
        </svg>
        <div class="story-txt">Gör 1 tejpkryss på golvet vid <strong>Punkt A (1.1 m)</strong> och ett vid <strong>Punkt B (0.6 m)</strong> så vet du alltid exakt var du står.</div>
      </div>

      <!-- RUTA B: RUNDA 1 -->
      <div class="story-card">
        <div class="story-hdr">B. RUNDA 1 (Punkt A, 1.1 m): Rakt mot hörnet — 5 frys-stopp</div>
        <svg viewBox="0 0 390 128" style="width:100%; height:41mm; display:block; background:#fff; border:1px solid #e2e8f0;">
          <line x1="12" y1="64" x2="378" y2="64" stroke="#cbd5e1" stroke-width="1" stroke-dasharray="4,3"/>
          <text x="14" y="61" font-size="8" fill="#64748b">Brösthöjd</text>
          <!-- 1 -->
          <rect x="26" y="59" width="44" height="10" rx="2" fill="#1e293b"/>
          <text x="48" y="22" font-size="9.5" font-weight="700" fill="#1d4ed8" text-anchor="middle">1. Mitten</text>
          <text x="48" y="116" font-size="8.5" fill="#475569" text-anchor="middle">Vågrätt</text>
          <!-- 2 -->
          <path d="M 118,64 L 118,88" stroke="#2563eb" stroke-width="2"/>
          <rect x="96" y="88" width="44" height="10" rx="2" fill="#1e293b"/>
          <text x="118" y="22" font-size="9.5" font-weight="700" fill="#1d4ed8" text-anchor="middle">2. Sänk</text>
          <text x="118" y="116" font-size="8.5" fill="#475569" text-anchor="middle">-30 cm ner</text>
          <!-- 3 -->
          <path d="M 190,64 L 190,38" stroke="#2563eb" stroke-width="2"/>
          <rect x="168" y="34" width="44" height="10" rx="2" fill="#1e293b"/>
          <text x="190" y="22" font-size="9.5" font-weight="700" fill="#1d4ed8" text-anchor="middle">3. Höj</text>
          <text x="190" y="116" font-size="8.5" fill="#475569" text-anchor="middle">+30 cm upp</text>
          <!-- 4 -->
          <g transform="translate(264,62) rotate(-15)">
            <rect x="-22" y="-5" width="44" height="10" rx="2" fill="#0f766e"/>
          </g>
          <path d="M 244,44 Q 264,34 284,44" fill="none" stroke="#0f766e" stroke-width="1.8"/>
          <text x="264" y="22" font-size="9.5" font-weight="700" fill="#0f766e" text-anchor="middle">4. Luta ±15°</text>
          <text x="264" y="116" font-size="8.5" fill="#475569" text-anchor="middle">Uppåt / Neråt</text>
          <!-- 5 -->
          <g transform="translate(344,64) rotate(18)">
            <rect x="-22" y="-5" width="44" height="10" rx="2" fill="#9333ea"/>
          </g>
          <text x="344" y="22" font-size="9.5" font-weight="700" fill="#7e22ce" text-anchor="middle">5. Ratt ↺↻</text>
          <text x="344" y="116" font-size="8.5" fill="#475569" text-anchor="middle">Tippa ±15°</text>
        </svg>
        <div class="story-txt">Håll riggen framför dig och peka rakt mot hörnet: <strong>Mitten</strong>, <strong>midjehöjd</strong>, <strong>ögonhöjd</strong>, <strong>vinkla upp/ner</strong> och <strong>tippa som en ratt</strong>.</div>
      </div>

      <!-- RUTA C: RUNDA 2 -->
      <div class="story-card">
        <div class="story-hdr">C. RUNDA 2 (Punkt A, 1.1 m): Vrid ±40° (Låser ihop kamerorna!)</div>
        <svg viewBox="0 0 390 128" style="width:100%; height:41mm; display:block; background:#fff; border:1px solid #e2e8f0;">
          <!-- Vänstra rutan: Vrid 40° HÖGER -->
          <text x="97" y="14" font-size="9.2" font-weight="700" fill="#1e293b" text-anchor="middle">Stopp 6–8: Vrid ~40° HÖGER (3 höjder)</text>
          <polyline points="20,76 97,26 174,76" fill="none" stroke="#94a3b8" stroke-width="3.5"/>
          <line x1="36" y1="65" x2="62" y2="49" stroke="#2563eb" stroke-width="5"/>
          <line x1="103" y1="30" x2="129" y2="47" stroke="#16a34a" stroke-width="5"/>
          <line x1="142" y1="56" x2="168" y2="73" stroke="#d97706" stroke-width="5"/>
          <polygon points="94,95 96,30 132,50" fill="rgba(22,163,74,0.22)" stroke="#16a34a" stroke-width="1"/>
          <polygon points="103,95 135,54 172,76" fill="rgba(217,119,6,0.22)" stroke="#d97706" stroke-width="1"/>
          <g transform="translate(97,98) rotate(38)">
            <rect x="-16" y="-4" width="32" height="8" rx="2" fill="#111827"/>
          </g>
          <text x="97" y="121" font-size="8.2" font-weight="700" fill="#15803d" text-anchor="middle">Cam 1 ser T1 • Cam 2 ser T2</text>

          <line x1="195" y1="6" x2="195" y2="122" stroke="#cbd5e1" stroke-width="1"/>

          <!-- Högra rutan: Vrid 40° VÄNSTER -->
          <text x="292" y="14" font-size="9.2" font-weight="700" fill="#1e293b" text-anchor="middle">Stopp 9–11: Vrid ~40° VÄNSTER (3 höjder)</text>
          <polyline points="216,76 292,26 368,76" fill="none" stroke="#94a3b8" stroke-width="3.5"/>
          <line x1="232" y1="65" x2="258" y2="49" stroke="#2563eb" stroke-width="5"/>
          <line x1="298" y1="30" x2="324" y2="47" stroke="#16a34a" stroke-width="5"/>
          <line x1="337" y1="56" x2="363" y2="73" stroke="#d97706" stroke-width="5"/>
          <polygon points="286,95 224,62 268,42" fill="rgba(37,99,235,0.22)" stroke="#2563eb" stroke-width="1"/>
          <polygon points="296,95 292,30 330,50" fill="rgba(22,163,74,0.22)" stroke="#16a34a" stroke-width="1"/>
          <g transform="translate(292,98) rotate(-38)">
            <rect x="-16" y="-4" width="32" height="8" rx="2" fill="#111827"/>
          </g>
          <text x="292" y="121" font-size="8.2" font-weight="700" fill="#1e40af" text-anchor="middle">Cam 2 ser T0 • Cam 3 ser T1</text>
        </svg>
        <div class="story-txt"><strong>Viktigast!</strong> Stå kvar på Punkt A, vrid hela riggen 40° åt höger (3 höjder) och 40° åt vänster (3 höjder) så flera kameror ser samma tavlor.</div>
      </div>

      <!-- RUTA D: RUNDA 3 -->
      <div class="story-card">
        <div class="story-hdr">D. RUNDA 3 (Punkt B, 0.6 m nära): Skärmarna — Tavlor ända ut i HÖRNEN!</div>
        <svg viewBox="0 0 390 128" style="width:100%; height:41mm; display:block; background:#fff; border:1px solid #e2e8f0;">
          <g transform="translate(18,14)">
            <rect x="0" y="0" width="74" height="96" rx="4" fill="#0f172a" stroke="#475569" stroke-width="2"/>
            <rect x="6" y="7" width="32" height="24" fill="#22c55e" stroke="#fff" stroke-width="1.2"/>
            <text x="22" y="22" font-size="8.5" font-weight="700" fill="#fff" text-anchor="middle">ChArUco</text>
            <text x="37" y="109" font-size="8.3" font-weight="700" fill="#0f172a" text-anchor="middle">12: Övre hörn</text>
          </g>
          <g transform="translate(110,14)">
            <rect x="0" y="0" width="74" height="96" rx="4" fill="#0f172a" stroke="#475569" stroke-width="2"/>
            <rect x="36" y="65" width="32" height="24" fill="#22c55e" stroke="#fff" stroke-width="1.2"/>
            <text x="52" y="80" font-size="8.5" font-weight="700" fill="#fff" text-anchor="middle">ChArUco</text>
            <text x="37" y="109" font-size="8.3" font-weight="700" fill="#0f172a" text-anchor="middle">13: Nedre hörn</text>
          </g>
          <g transform="translate(202,14)">
            <rect x="0" y="0" width="74" height="96" rx="4" fill="#0f172a" stroke="#475569" stroke-width="2"/>
            <rect x="5" y="36" width="30" height="26" fill="#2563eb" stroke="#fff" stroke-width="1.2"/>
            <text x="20" y="52" font-size="8.5" font-weight="700" fill="#fff" text-anchor="middle">T0/T1</text>
            <text x="37" y="109" font-size="8.3" font-weight="700" fill="#0f172a" text-anchor="middle">14–15: Kanter</text>
          </g>
          <g transform="translate(294,14)">
            <rect x="0" y="0" width="74" height="96" rx="4" fill="#0f172a" stroke="#475569" stroke-width="2"/>
            <rect x="39" y="10" width="30" height="26" fill="#d97706" stroke="#fff" stroke-width="1.2"/>
            <rect x="4" y="62" width="30" height="26" fill="#2563eb" stroke="#fff" stroke-width="1.2"/>
            <text x="37" y="109" font-size="8.3" font-weight="700" fill="#0f172a" text-anchor="middle">16–17: Sidohörn</text>
          </g>
        </svg>
        <div class="story-txt">Kliv fram till <strong>Punkt B (0.6 m)</strong> och sikta lite upp, ner, vänster och höger i 6 stopp så tavlorna ligger i skärmarnas ytterhörn! Tryck sedan <strong>STOPP</strong>.</div>
      </div>
    </div>
  </section>

__BOARD_PAGES__

  <script>
    function printKit(boardsOnly) {
      if (boardsOnly) {
        document.body.classList.add("print-boards-only");
      } else {
        document.body.classList.remove("print-boards-only");
      }
      window.print();
    }

    const SIM_PHASES = [
      {
        roomTag: "Start • På Punkt A (1.1 m)",
        camTag: "REC på alla 3 + 1 Klapp",
        feet: "Stå stadigt på <strong>Punkt A (1.1 m framför hörnet)</strong> med riggen rakt framför dig i brösthöjd.",
        action: "Tryck <strong>REC ●</strong> på alla 3 DJI Action 4. Gör <strong>EN tydlig, hög handklapp</strong> framför riggen så ljudvågen syns på alla tre filmer.",
        rigX: 215, rigY: 168, rigDeg: 0, activeSpot: "A",
        posture: "BRÖSTHÖJD • VÅGRÄTT",
        postureMode: "clap",
        screensMode: "clap"
      },
      {
        roomTag: "Punkt A (1.1 m) • Stopp 1–3",
        camTag: "T0 i C1 • T1 i C2 • T2 i C3",
        feet: "Stå kvar på <strong>Punkt A (1.1 m)</strong>, rakt vänd mot hörnet.",
        action: "Frys ~1–2 sek vid <strong>Stopp 1 (Brösthöjd)</strong>, flytta ner ~30 cm och frys vid <strong>Stopp 2 (Midjehöjd)</strong>, lyft upp ~30 cm över bröstet och frys vid <strong>Stopp 3 (Ögonhöjd)</strong>.",
        rigX: 215, rigY: 168, rigDeg: 0, activeSpot: "A",
        posture: "HÖJD: MITTEN / -30 / +30 CM",
        postureMode: "height",
        screensMode: "straight"
      },
      {
        roomTag: "Punkt A (1.1 m) • Stopp 4–5",
        camTag: "Lutade tavlor i alla 3",
        feet: "Stå kvar på <strong>Punkt A (1.1 m)</strong>, rakt vänd mot hörnet.",
        action: "<strong>Stopp 4:</strong> Vinkla riggen uppåt (~15°) och neråt (~15°), frys ~1–2 sek i varje läge. <strong>Stopp 5:</strong> Tippa hela riggen i sidled som en ratt (~15° åt varje håll) och frys ~1–2 sek.",
        rigX: 215, rigY: 168, rigDeg: 0, activeSpot: "A",
        posture: "TILTA ±15° & RATT-LUTA ±15°",
        postureMode: "tilt",
        screensMode: "tilt"
      },
      {
        roomTag: "Punkt A (1.1 m) • VRID 40° HÖGER",
        camTag: "Cam 1 & Cam 2 delar tavlor!",
        feet: "Stå kvar på <strong>Punkt A (1.1 m)</strong> men vrid hela kroppen + riggen <strong>~40° åt HÖGER</strong>.",
        action: "<strong>Stopp 6–8 (3 lägen i lite olika höjd, 1–2 s var):</strong> Nu pekar Vänsterkamera (C1) på Mittentavlan (T1) och Mittenkameran (C2) på Högertavlan (T2) — detta binder ihop C1 och C2!",
        rigX: 215, rigY: 168, rigDeg: 38, activeSpot: "A",
        posture: "VRID HÖGER +40° (3 HÖJDER)",
        postureMode: "turnR",
        screensMode: "turnRight"
      },
      {
        roomTag: "Punkt A (1.1 m) • VRID 40° VÄNSTER",
        camTag: "Cam 2 & Cam 3 delar tavlor!",
        feet: "Stå kvar på <strong>Punkt A (1.1 m)</strong> och vrid istället kroppen + riggen <strong>~40° åt VÄNSTER</strong>.",
        action: "<strong>Stopp 9–11 (3 lägen i lite olika höjd, 1–2 s var):</strong> Nu pekar Mittenkameran (C2) på Vänstertavlan (T0) och Högerkameran (C3) på Mittentavlan (T1) — detta låser ihop hela riggen!",
        rigX: 215, rigY: 168, rigDeg: -38, activeSpot: "A",
        posture: "VRID VÄNSTER -40° (3 HÖJDER)",
        postureMode: "turnL",
        screensMode: "turnLeft"
      },
      {
        roomTag: "Punkt B (0.6 m) • NÄRA FÖR HÖRNEN",
        camTag: "Tavlor i bildernas YTTRE HÖRN!",
        feet: "Ta ett halvt kliv framåt till <strong>Punkt B (ca 0.6 m från tavlorna)</strong>.",
        action: "<strong>Stopp 12–17 (6 stopp à 1–2 s):</strong> Vinkla riggen lätt upp, ner, vänster och höger så ChArUco-rutorna hamnar ända ut i bildskärmarnas hörn &amp; kanter. Tryck sedan <strong>STOPP ■</strong>!",
        rigX: 215, rigY: 112, rigDeg: 0, activeSpot: "B",
        posture: "NÄRA 0.6 m • SIKTA MOT HÖRNEN",
        postureMode: "close",
        screensMode: "corners"
      }
    ];

    function renderCheckerRect(x, y, w, h, color, label, rotDeg) {
      const tr = rotDeg ? ` transform="rotate(${rotDeg} ${x + w / 2} ${y + h / 2})"` : "";
      const cw = w / 4;
      const ch = h / 3;
      let cells = "";
      for (let r = 0; r < 3; r++) {
        for (let c = 0; c < 4; c++) {
          const fill = (r + c) % 2 === 0 ? "#0f172a" : "#f8fafc";
          cells += `<rect x="${(x + c * cw).toFixed(1)}" y="${(y + r * ch).toFixed(1)}" width="${cw.toFixed(1)}" height="${ch.toFixed(1)}" fill="${fill}"/>`;
        }
      }
      return `<g${tr}>
        ${cells}
        <rect x="${x}" y="${y}" width="${w}" height="${h}" fill="none" stroke="${color}" stroke-width="2.4"/>
        <rect x="${x + w / 2 - 11}" y="${y + h / 2 - 7}" width="22" height="14" rx="2" fill="${color}"/>
        <text x="${x + w / 2}" y="${y + h / 2 + 3.5}" font-size="8.5" font-weight="700" fill="#fff" text-anchor="middle">${label}</text>
      </g>`;
    }

    function renderScreensSvg(mode) {
      const camX = [18, 155, 292];
      const camTitles = ["CAM 1 VÄNSTER", "CAM 2 MITTEN", "CAM 3 HÖGER"];
      let out = `<rect x="0" y="0" width="430" height="230" rx="4" fill="#090b0f"/>`;
      camX.forEach((x, idx) => {
        out += `<rect x="${x}" y="18" width="120" height="170" rx="6" fill="#111827" stroke="#475569" stroke-width="2"/>`;
        out += `<rect x="${x + 6}" y="28" width="108" height="144" fill="#05070a" stroke="#1e293b" stroke-width="1"/>`;
        out += `<text x="${x + 60}" y="199" font-size="9.5" font-weight="700" fill="#cbd5e1" text-anchor="middle">${camTitles[idx]}</text>`;
        if (mode === "corners") {
          out += `<rect x="${x + 9}" y="31" width="102" height="138" fill="none" stroke="#22c55e" stroke-width="1" stroke-dasharray="4,3"/>`;
        }
        out += `<circle cx="${x + 15}" cy="37" r="3.5" fill="#ef4444"/><text x="${x + 22}" y="40" font-size="8" font-weight="700" fill="#fca5a5">REC</text>`;
      });

      if (mode === "clap") {
        camX.forEach((x) => {
          out += `<circle cx="${x + 60}" cy="100" r="18" fill="none" stroke="#facc15" stroke-width="2" stroke-dasharray="3,2"/>`;
          out += `<circle cx="${x + 60}" cy="100" r="30" fill="none" stroke="#facc15" stroke-width="1.3" stroke-dasharray="4,3"/>`;
          out += `<text x="${x + 60}" y="97" font-size="11" font-weight="700" fill="#fde047" text-anchor="middle">KLAPP!</text>`;
          out += `<text x="${x + 60}" y="112" font-size="9" fill="#fef08a" text-anchor="middle">Ljudsynk</text>`;
        });
      } else if (mode === "straight") {
        out += renderCheckerRect(camX[0] + 34, 82, 52, 38, "#3b82f6", "T0", 0);
        out += renderCheckerRect(camX[1] + 34, 82, 52, 38, "#22c55e", "T1", 0);
        out += renderCheckerRect(camX[2] + 34, 82, 52, 38, "#f59e0b", "T2", 0);
        camX.forEach((x) => {
          out += `<path d="M ${x + 60},76 L ${x + 60},55 M ${x + 60},126 L ${x + 60},147" stroke="#60a5fa" stroke-width="2"/>`;
          out += `<text x="${x + 60}" y="164" font-size="8.5" fill="#93c5fd" text-anchor="middle">Åker upp/ner ±30 cm</text>`;
        });
      } else if (mode === "tilt") {
        out += renderCheckerRect(camX[0] + 32, 74, 54, 40, "#3b82f6", "T0", -15);
        out += renderCheckerRect(camX[1] + 32, 86, 54, 40, "#22c55e", "T1", 15);
        out += renderCheckerRect(camX[2] + 32, 78, 54, 40, "#f59e0b", "T2", 15);
        camX.forEach((x) => {
          out += `<text x="${x + 60}" y="164" font-size="8.5" fill="#c084fc" text-anchor="middle">Tilta &amp; ratt-luta ±15°</text>`;
        });
      } else if (mode === "turnRight") {
        out += renderCheckerRect(camX[0] + 54, 80, 50, 36, "#22c55e", "T1", 0);
        out += renderCheckerRect(camX[0] + 8, 92, 26, 20, "#3b82f6", "T0", 0);
        out += renderCheckerRect(camX[1] + 10, 82, 30, 24, "#22c55e", "T1", 0);
        out += renderCheckerRect(camX[1] + 56, 80, 50, 36, "#f59e0b", "T2", 0);
        out += `<text x="${camX[2] + 60}" y="102" font-size="9" fill="#64748b" text-anchor="middle">Ut åt höger</text>`;
        out += `<path d="M ${camX[0] + 78},74 Q ${(camX[0] + camX[1]) / 2 + 52},36 ${camX[1] + 25},74" fill="none" stroke="#4ade80" stroke-width="2.5"/>`;
        out += `<rect x="44" y="207" width="224" height="18" rx="3" fill="#14532d" stroke="#22c55e"/>`;
        out += `<text x="156" y="219" font-size="9" font-weight="700" fill="#86efac" text-anchor="middle">SAMMA TAVLA (T1) = Låser ihop Cam 1 &amp; Cam 2!</text>`;
      } else if (mode === "turnLeft") {
        out += `<text x="${camX[0] + 60}" y="102" font-size="9" fill="#64748b" text-anchor="middle">Ut åt vänster</text>`;
        out += renderCheckerRect(camX[1] + 14, 80, 50, 36, "#3b82f6", "T0", 0);
        out += renderCheckerRect(camX[1] + 80, 82, 30, 24, "#22c55e", "T1", 0);
        out += renderCheckerRect(camX[2] + 14, 80, 50, 36, "#22c55e", "T1", 0);
        out += renderCheckerRect(camX[2] + 86, 92, 26, 20, "#f59e0b", "T2", 0);
        out += `<path d="M ${camX[1] + 95},74 Q ${(camX[1] + camX[2]) / 2 + 55},36 ${camX[2] + 38},74" fill="none" stroke="#4ade80" stroke-width="2.5"/>`;
        out += `<rect x="172" y="207" width="234" height="18" rx="3" fill="#14532d" stroke="#22c55e"/>`;
        out += `<text x="289" y="219" font-size="9" font-weight="700" fill="#86efac" text-anchor="middle">SAMMA TAVLA (T1) = Låser ihop Cam 2 &amp; Cam 3!</text>`;
      } else if (mode === "corners") {
        out += renderCheckerRect(camX[0] + 8, 32, 42, 30, "#3b82f6", "T0", 0);
        out += renderCheckerRect(camX[0] + 68, 138, 42, 30, "#22c55e", "T1", 0);
        out += renderCheckerRect(camX[1] + 68, 32, 42, 30, "#22c55e", "T1", 0);
        out += renderCheckerRect(camX[1] + 8, 138, 42, 30, "#3b82f6", "T0", 0);
        out += renderCheckerRect(camX[2] + 68, 32, 42, 30, "#f59e0b", "T2", 0);
        out += renderCheckerRect(camX[2] + 8, 138, 42, 30, "#22c55e", "T1", 0);
        out += `<rect x="65" y="207" width="300" height="18" rx="3" fill="#064e3b" stroke="#22c55e"/>`;
        out += `<text x="215" y="219" font-size="9" font-weight="700" fill="#86efac" text-anchor="middle">Tryck ut tavlorna i ALLA 4 HÖRN för 155° fisheye (k1..k4)!</text>`;
      }
      return out;
    }

    function renderRoomSvg(ph) {
      const aActive = ph.activeSpot === "A";
      const bActive = ph.activeSpot === "B";
      const degRad = (a) => (a * Math.PI) / 180;
      const baseDeg = ph.rigDeg - 90; // 0 deg right; -90 deg up toward corner
      const fovCone = (camLocalOffsetX, camAngleOffset, dist, spreadDeg, color) => {
        const cx = ph.rigX + Math.cos(degRad(ph.rigDeg)) * camLocalOffsetX;
        const cy = ph.rigY + Math.sin(degRad(ph.rigDeg)) * camLocalOffsetX;
        const a1 = degRad(baseDeg + camAngleOffset - spreadDeg / 2);
        const a2 = degRad(baseDeg + camAngleOffset + spreadDeg / 2);
        const x1 = cx + Math.cos(a1) * dist;
        const y1 = cy + Math.sin(a1) * dist;
        const x2 = cx + Math.cos(a2) * dist;
        const y2 = cy + Math.sin(a2) * dist;
        return `<polygon points="${cx.toFixed(1)},${cy.toFixed(1)} ${x1.toFixed(1)},${y1.toFixed(1)} ${x2.toFixed(1)},${y2.toFixed(1)}" fill="${color}" fill-opacity="0.16" stroke="${color}" stroke-width="1"/>`;
      };

      let postureIcon = "";
      if (ph.postureMode === "clap") {
        postureIcon = `<text x="58" y="34" font-size="14" text-anchor="middle">👏</text><text x="58" y="48" font-size="8.5" font-weight="700" fill="#fde047" text-anchor="middle">1 Hög Klapp</text>`;
      } else if (ph.postureMode === "height") {
        postureIcon = `<rect x="38" y="20" width="40" height="5" rx="2" fill="#60a5fa"/><rect x="38" y="33" width="40" height="6" rx="2" fill="#e2e8f0"/><rect x="38" y="47" width="40" height="5" rx="2" fill="#60a5fa"/><path d="M 88,20 L 88,52" stroke="#60a5fa" stroke-width="1.8"/>`;
      } else if (ph.postureMode === "tilt") {
        postureIcon = `<g transform="translate(58,36) rotate(-18)"><rect x="-22" y="-4" width="44" height="8" rx="2" fill="#c084fc"/></g><path d="M 30,26 Q 58,15 86,26" fill="none" stroke="#c084fc" stroke-width="1.6"/>`;
      } else if (ph.postureMode === "turnR") {
        postureIcon = `<path d="M 35,44 A 24,24 0 0 1 82,36" fill="none" stroke="#4ade80" stroke-width="2.5"/><text x="58" y="54" font-size="8.5" font-weight="700" fill="#4ade80" text-anchor="middle">+40° HÖGER ↻</text>`;
      } else if (ph.postureMode === "turnL") {
        postureIcon = `<path d="M 82,44 A 24,24 0 0 0 35,36" fill="none" stroke="#60a5fa" stroke-width="2.5"/><text x="58" y="54" font-size="8.5" font-weight="700" fill="#60a5fa" text-anchor="middle">-40° VÄNSTER ↺</text>`;
      } else {
        postureIcon = `<path d="M 58,48 L 58,22" stroke="#4ade80" stroke-width="2.5"/><polygon points="58,16 53,25 63,25" fill="#4ade80"/><text x="58" y="56" font-size="8" font-weight="700" fill="#4ade80" text-anchor="middle">Kliv till B!</text>`;
      }

      const dCone = ph.activeSpot === "B" ? 86 : 124;
      return `
        <rect x="0" y="0" width="430" height="230" rx="4" fill="#090b0f"/>
        <polyline points="55,135 215,24 375,135" fill="none" stroke="#64748b" stroke-width="6" stroke-linecap="round"/>
        <text x="215" y="15" font-size="9.5" font-weight="700" fill="#94a3b8" text-anchor="middle">90° Hörnveck (håll tomt)</text>

        <line x1="88" y1="112" x2="148" y2="70" stroke="#3b82f6" stroke-width="8"/>
        <text x="84" y="82" font-size="10.5" font-weight="700" fill="#60a5fa" text-anchor="end">Tavla 0</text>
        <line x1="227" y1="32" x2="282" y2="70" stroke="#22c55e" stroke-width="8"/>
        <text x="266" y="36" font-size="10.5" font-weight="700" fill="#4ade80">Tavla 1 (+20 cm)</text>
        <line x1="305" y1="86" x2="365" y2="128" stroke="#f59e0b" stroke-width="8"/>
        <text x="370" y="102" font-size="10.5" font-weight="700" fill="#fbbf24">Tavla 2</text>

        <!-- Punkt B & A floor spots -->
        <circle cx="215" cy="112" r="15" fill="${bActive ? 'rgba(34,197,94,0.28)' : 'rgba(100,116,139,0.12)'}" stroke="${bActive ? '#4ade80' : '#475569'}" stroke-width="2" stroke-dasharray="3,2"/>
        <text x="236" y="115" font-size="9.5" font-weight="700" fill="${bActive ? '#86efac' : '#64748b'}">PUNKT B (0.6 m)</text>

        <circle cx="215" cy="168" r="16" fill="${aActive ? 'rgba(59,130,246,0.28)' : 'rgba(100,116,139,0.12)'}" stroke="${aActive ? '#60a5fa' : '#475569'}" stroke-width="2" stroke-dasharray="3,2"/>
        <text x="238" y="172" font-size="9.5" font-weight="700" fill="${aActive ? '#93c5fd' : '#64748b'}">PUNKT A (1.1 m)</text>

        ${fovCone(-18, -45, dCone, 46, "#3b82f6")}
        ${fovCone(0, 0, dCone, 46, "#22c55e")}
        ${fovCone(18, 45, dCone, 46, "#f59e0b")}

        <g transform="translate(${ph.rigX},${ph.rigY}) rotate(${ph.rigDeg})">
          <rect x="-26" y="-6" width="52" height="12" rx="4" fill="#f8fafc" stroke="#0f172a" stroke-width="1.5"/>
          <circle cx="-18" cy="0" r="3.8" fill="#3b82f6"/>
          <circle cx="0" cy="0" r="3.8" fill="#22c55e"/>
          <circle cx="18" cy="0" r="3.8" fill="#f59e0b"/>
        </g>

        <!-- Posture inset -->
        <g transform="translate(10,156)">
          <rect x="0" y="0" width="116" height="64" rx="4" fill="#111827" stroke="#334155"/>
          <text x="58" y="12" font-size="8" font-weight="700" fill="#94a3b8" text-anchor="middle">${ph.posture}</text>
          ${postureIcon}
        </g>
      `;
    }

    let simIdx = 0;
    let simTimer = null;

    function selectSimPhase(idx) {
      simIdx = idx;
      const ph = SIM_PHASES[idx];
      document.querySelectorAll("#simPhaseBtns .phase-btn").forEach((b, i) => {
        b.classList.toggle("active", i === idx);
      });
      document.getElementById("simRoomTag").textContent = ph.roomTag;
      document.getElementById("simCamTag").textContent = ph.camTag;
      document.getElementById("simFeetTxt").innerHTML = ph.feet;
      document.getElementById("simDoTxt").innerHTML = ph.action;
      document.getElementById("simRoomSvg").innerHTML = renderRoomSvg(ph);
      document.getElementById("simScreensSvg").innerHTML = renderScreensSvg(ph.screensMode);
    }

    function toggleSimAuto() {
      const btn = document.getElementById("simPlayBtn");
      if (simTimer) {
        clearInterval(simTimer);
        simTimer = null;
        btn.textContent = "▶ Spela upp automatiskt";
        btn.classList.remove("playing");
      } else {
        btn.textContent = "⏸ Pausa uppspelning";
        btn.classList.add("playing");
        simTimer = setInterval(() => {
          selectSimPhase((simIdx + 1) % SIM_PHASES.length);
        }, 3500);
      }
    }

    document.querySelectorAll("#simPhaseBtns .phase-btn").forEach(btn => {
      btn.addEventListener("click", () => {
        if (simTimer) toggleSimAuto();
        selectSimPhase(Number(btn.dataset.p || 0));
      });
    });
    selectSimPhase(0);
  </script>
</body>
</html>
"""


def _build_printable_kit_html(svg_blocks: Dict[str, str], square_mm: float) -> str:
    board_pages: List[str] = []
    for b_name, _id_off, title, subtitle in BOARD_SPECS:
        board_pages.append(
            f"""  <section class="page board-page">
    <div class="board-note"><strong>{html_lib.escape(title)}</strong> — {html_lib.escape(subtitle)}</div>
    <div class="svg-wrap">{svg_blocks[b_name]}</div>
  </section>"""
        )
    return (
        _PRINTABLE_KIT_TEMPLATE
        .replace("__SQUARE_MM__", f"{square_mm:.1f}")
        .replace("__BOARD_PAGES__", "\n".join(board_pages))
    )


def generate_charuco_boards(
    out_dir: Path = BOARDS_DIR,
    square_mm: float = DEFAULT_SQUARE_MM,
    marker_mm: float = DEFAULT_MARKER_MM,
    cols: int = DEFAULT_BOARD_COLS,
    rows: int = DEFAULT_BOARD_ROWS,
) -> Dict[str, Any]:
    """
    Skapar 3 unika ChArUco-tavlor (.svg + 300 DPI .png) samt en utskrivbar A4-sats
    (`printable_calibration_kit.html`) med 100 mm-kontrollinjal och interaktiv film-simulator.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    generated_files: List[str] = []
    svg_blocks: Dict[str, str] = {}

    px_per_m = 11811.0  # ~300 DPI
    png_w = int(round((cols * square_mm / 1000.0) * px_per_m))
    png_h = int(round((rows * square_mm / 1000.0) * px_per_m))

    for b_name, id_off, title, subtitle in BOARD_SPECS:
        svg_text = _render_charuco_svg(b_name, title, subtitle, id_off, square_mm, marker_mm, cols, rows)
        svg_path = out_dir / f"{b_name}.svg"
        svg_path.write_text(svg_text, encoding="utf-8")
        generated_files.append(str(svg_path))
        svg_blocks[b_name] = svg_text

        board = make_charuco_board(id_off, square_mm=square_mm, marker_mm=marker_mm, cols=cols, rows=rows)
        png_img = board.generateImage((png_w, png_h), marginSize=40, borderBits=1)
        png_path = out_dir / f"{b_name}.png"
        cv2.imwrite(str(png_path), png_img)
        generated_files.append(str(png_path))

    kit_html = _build_printable_kit_html(svg_blocks=svg_blocks, square_mm=square_mm)
    html_path = out_dir / "printable_calibration_kit.html"
    html_path.write_text(kit_html, encoding="utf-8")
    generated_files.append(str(html_path))

    return {
        "ok": True,
        "out_dir": str(out_dir),
        "html_path": str(html_path),
        "kit_html": str(html_path),
        "square_mm": float(square_mm),
        "marker_mm": float(marker_mm),
        "files": generated_files,
    }


# =============================================================================
# 4. DETECTION AV CHARUCO I SYNKADE BILD-TRIOR / VIDEOR
# =============================================================================

class MultiBoardDetector:
    """Snabb detektor för alla 3 unika ChArUco-tavlor i OpenCV 5.0."""

    def __init__(
        self,
        square_mm: float = DEFAULT_SQUARE_MM,
        marker_mm: float = DEFAULT_MARKER_MM,
        cols: int = DEFAULT_BOARD_COLS,
        rows: int = DEFAULT_BOARD_ROWS,
    ):
        self.boards: Dict[str, Any] = {}
        self.detectors: Dict[str, Any] = {}
        self.obj_corners: Dict[str, np.ndarray] = {}
        for b_name, id_off, _, _ in BOARD_SPECS:
            board = make_charuco_board(id_off, square_mm=square_mm, marker_mm=marker_mm, cols=cols, rows=rows)
            self.boards[b_name] = board
            self.detectors[b_name] = cv2.aruco.CharucoDetector(board)
            self.obj_corners[b_name] = np.asarray(board.getChessboardCorners(), dtype=np.float64).reshape(-1, 3)

    def detect_image(self, gray_or_bgr: np.ndarray, min_corners: int = 6) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
        """
        Returnerar { board_name: (corner_ids (K,), img_pts (K, 2)) } i källbildens pixelkoordinater.
        Använder snabbskala (bredd ~1440 px) för markörletning och förfinar i full upplösning.
        """
        gray = cv2.cvtColor(gray_or_bgr, cv2.COLOR_BGR2GRAY) if gray_or_bgr.ndim == 3 else gray_or_bgr
        h, w = gray.shape[:2]
        out: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        for b_name, det in self.detectors.items():
            cc, ci, _mc, _mi = det.detectBoard(gray)
            if cc is not None and ci is not None and len(ci) >= min_corners:
                ids_flat = ci.reshape(-1).astype(np.int32)
                pts = cc.reshape(-1, 2).astype(np.float64)
                # Förfina med cornerSubPix i full upplösning för maximal skärpa
                if h >= 1200 and w >= 1200:
                    pts32 = pts.reshape(-1, 1, 2).astype(np.float32)
                    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.01)
                    cv2.cornerSubPix(gray, pts32, (7, 7), (-1, -1), crit)
                    pts = pts32.reshape(-1, 2).astype(np.float64)
                out[b_name] = (ids_flat, pts)
        return out


def _shared_corner_median_shift_px(
    obs_a: Dict[str, Dict[str, Tuple[np.ndarray, np.ndarray]]],
    obs_b: Dict[str, Dict[str, Tuple[np.ndarray, np.ndarray]]],
) -> Tuple[int, float]:
    """Beräknar antal gemensamma ChArUco-hörn och deras medianförflyttning (px) mellan två tidpunkter."""
    shifts: List[float] = []
    for ck, boards_a in obs_a.items():
        boards_b = obs_b.get(ck)
        if not boards_b:
            continue
        for b_name, (ids_a, pts_a) in boards_a.items():
            pair_b = boards_b.get(b_name)
            if pair_b is None:
                continue
            ids_b, pts_b = pair_b
            map_b = {int(cid): pt for cid, pt in zip(ids_b.reshape(-1), pts_b)}
            for cid, pt_a in zip(ids_a.reshape(-1), pts_a):
                pt_b = map_b.get(int(cid))
                if pt_b is not None:
                    shifts.append(float(np.linalg.norm(pt_a - pt_b)))
    if not shifts:
        return 0, float("inf")
    return len(shifts), float(np.median(shifts))


def _extract_candidate_triplets_from_videos(
    videos: Dict[str, Path],
    offsets: Dict[str, int],
    detector: MultiBoardDetector,
    fps: float = 50.0,
    step_sec: float = 0.35,
    skip_start_sec: float = 2.5,
    max_poses: int = 36,
    progress_cb: Optional[Callable[[str], None]] = None,
) -> Tuple[List[Dict[str, Any]], Dict[str, Tuple[int, int]], Dict[str, Dict[str, np.ndarray]]]:
    """
    Skannar igenom de tre ljudsynkade kalibreringsvideorna, mäter rörelse ruta-till-ruta
    (optical flow på frys-stopp), filtrerar bort rutor i rörelse, klustrar per stopp
    och väljer automatiskt den stillaste & skarpaste vinnarbilden per frys-stopp med
    skydd för närbilder i fisheye-ytterkanterna.
    """
    caps: Dict[str, cv2.VideoCapture] = {}
    image_sizes: Dict[str, Tuple[int, int]] = {}
    frame_counts: List[int] = []
    for ck in CAM_ORDER:
        cap = cv2.VideoCapture(str(videos[ck]))
        if not cap.isOpened():
            for c_prev in caps.values():
                c_prev.release()
            raise RuntimeError(f"Kunde inte öppna kalibreringsvideo: {videos[ck]}")
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        image_sizes[ck] = (w, h)
        caps[ck] = cap
        frame_counts.append(n - int(offsets.get(ck, 0)))

    try:
        max_sync = max(0, min(frame_counts) - 2)
        start_f = max(0, int(round(skip_start_sec * fps)))
        step_f = max(2, int(round(step_sec * fps)))
        candidates = list(range(start_f, max_sync, step_f))
        if len(candidates) > 120:
            stride = len(candidates) / 120.0
            candidates = sorted({candidates[int(i * stride)] for i in range(120)})

        ref_max_dim = max(max(w, h) for (w, h) in image_sizes.values())
        thr_px = max(2.0, 6.5 * (float(ref_max_dim) / 3840.0))

        if progress_cb:
            progress_cb(
                f"[ChArUco] Skannar {len(candidates)} synkroniserade tidpunkter efter klappen "
                f"({image_sizes['cam2_center'][0]}x{image_sizes['cam2_center'][1]}, rörelsegräns {thr_px:.1f} px/ruta)..."
            )

        collected: List[Dict[str, Any]] = []

        for idx, base_f in enumerate(candidates):
            imgs: Dict[str, np.ndarray] = {}
            next_imgs: Dict[str, np.ndarray] = {}
            ok_read = True
            for ck in CAM_ORDER:
                tf = max(0, int(base_f + offsets.get(ck, 0)))
                caps[ck].set(cv2.CAP_PROP_POS_FRAMES, tf)
                ret, frame = caps[ck].read()
                if not ret or frame is None:
                    ok_read = False
                    break
                imgs[ck] = frame
                # Sekventiell läsning av nästa ruta (tf + 1) utan extra video-seek för att mäta stillhet
                ret2, next_frame = caps[ck].read()
                if ret2 and next_frame is not None:
                    next_imgs[ck] = next_frame
            if not ok_read:
                continue

            obs_cam_boards: Dict[str, Dict[str, Tuple[np.ndarray, np.ndarray]]] = {}
            total_corners = 0
            cams_with_hits = 0
            for ck in CAM_ORDER:
                hits = detector.detect_image(imgs[ck], min_corners=6)
                if hits:
                    obs_cam_boards[ck] = hits
                    cams_with_hits += 1
                    total_corners += sum(len(v[0]) for v in hits.values())

            if total_corners < 8:
                continue

            outer_corners = 0
            motion_vals: List[float] = []
            sharp_vals: List[float] = []

            for ck, hits in obs_cam_boards.items():
                w_c, h_c = image_sizes[ck]
                pts_all = np.vstack([pts for (_ids, pts) in hits.values()]).astype(np.float64)
                rad = np.sqrt(
                    ((pts_all[:, 0] - w_c * 0.5) / max(1.0, w_c * 0.5)) ** 2
                    + ((pts_all[:, 1] - h_c * 0.5) / max(1.0, h_c * 0.5)) ** 2
                )
                outer_corners += int(np.sum(rad >= 0.62))

                flow_scale = min(1.0, 960.0 / float(max(w_c, h_c)))
                img0 = imgs[ck]
                if flow_scale < 1.0:
                    nw = max(32, int(round(w_c * flow_scale)))
                    nh = max(32, int(round(h_c * flow_scale)))
                    img0_s = cv2.resize(img0, (nw, nh), interpolation=cv2.INTER_AREA)
                else:
                    img0_s = img0
                g0 = cv2.cvtColor(img0_s, cv2.COLOR_BGR2GRAY) if img0_s.ndim == 3 else img0_s
                sharp_vals.append(float(cv2.Laplacian(g0, cv2.CV_64F).var()))

                if ck in next_imgs:
                    img1 = next_imgs[ck]
                    if flow_scale < 1.0:
                        img1_s = cv2.resize(img1, (g0.shape[1], g0.shape[0]), interpolation=cv2.INTER_AREA)
                    else:
                        img1_s = img1
                    g1 = cv2.cvtColor(img1_s, cv2.COLOR_BGR2GRAY) if img1_s.ndim == 3 else img1_s

                    p0 = (pts_all.astype(np.float32) * flow_scale).reshape(-1, 1, 2)
                    p1, st, _err = cv2.calcOpticalFlowPyrLK(
                        g0,
                        g1,
                        p0,
                        None,
                        winSize=(21, 21),
                        maxLevel=3,
                        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01),
                    )
                    valid = (
                        st is not None
                        and p1 is not None
                        and (st.reshape(-1) == 1)
                    )
                    if isinstance(valid, np.ndarray) and int(np.sum(valid)) >= max(3, int(0.5 * len(p0))):
                        disps_full = (
                            np.linalg.norm(
                                p1.reshape(-1, 2)[valid] - p0.reshape(-1, 2)[valid],
                                axis=1,
                            )
                            / flow_scale
                        )
                        cam_motion = float(np.median(disps_full))
                    else:
                        mean_diff = float(np.mean(np.abs(g1.astype(np.float32) - g0.astype(np.float32))))
                        cam_motion = 0.0 if mean_diff < 1.5 else max(thr_px * 2.5, mean_diff / flow_scale)
                    motion_vals.append(cam_motion)

            motion_px = float(max(motion_vals)) if motion_vals else 0.0
            sharpness = float(np.mean(sharp_vals)) if sharp_vals else 0.0
            quality_score = (
                cams_with_hits * 450.0
                + total_corners * 8.0
                + outer_corners * 14.0
                - motion_px * 75.0
                + min(sharpness, 600.0) * 0.25
            )

            ts_name = f"calib_{idx + 1:03d}_f{base_f:05d}"
            collected.append(
                {
                    "frame_name": ts_name,
                    "frame_idx": base_f,
                    "cand_idx": idx,
                    "cams_with_hits": cams_with_hits,
                    "total_corners": total_corners,
                    "outer_corners": outer_corners,
                    "motion_px": motion_px,
                    "sharpness": sharpness,
                    "quality_score": quality_score,
                    "observations": obs_cam_boards,
                }
            )

            if (idx + 1) % 10 == 0 or (idx + 1) == len(candidates):
                if progress_cb:
                    progress_cb(
                        f"[ChArUco] Skanning {idx + 1}/{len(candidates)}: hittat {len(collected)} användbara bild-trior..."
                    )

        still_pool = [c for c in collected if c["motion_px"] <= thr_px]
        n_discarded_motion = len(collected) - len(still_pool)
        if len(still_pool) < min(8, len(collected)) and len(collected) > 0:
            by_motion = sorted(collected, key=lambda c: c["motion_px"])
            keep_n = min(len(collected), max(12, len(still_pool)))
            still_pool = sorted(by_motion[:keep_n], key=lambda c: c["frame_idx"])
            if progress_cb:
                progress_cb(
                    f"[ChArUco] Varning: få helt stilla frys-stopp hittades ({len(collected) - n_discarded_motion} st), "
                    f"använder de {len(still_pool)} lugnaste tidpunkterna."
                )

        still_pool.sort(key=lambda o: o["frame_idx"])
        pose_shift_thr_px = 0.018 * float(ref_max_dim)
        clusters: List[List[Dict[str, Any]]] = []
        for curr in still_pool:
            if not clusters:
                clusters.append([curr])
                continue
            cluster = clusters[-1]
            prev = cluster[-1]
            anchor = cluster[0]
            adj_scan = (curr["cand_idx"] - prev["cand_idx"]) <= 2
            n_sh_prev, shift_prev = _shared_corner_median_shift_px(prev["observations"], curr["observations"])
            n_sh_anch, shift_anch = _shared_corner_median_shift_px(anchor["observations"], curr["observations"])
            same_stop = (
                adj_scan
                and (n_sh_prev >= 6 and shift_prev <= pose_shift_thr_px)
                and (n_sh_anch >= 6 and shift_anch <= pose_shift_thr_px * 1.25)
            )
            if same_stop:
                cluster.append(curr)
            else:
                clusters.append([curr])

        deduped: List[Dict[str, Any]] = [max(cl, key=lambda c: c["quality_score"]) for cl in clusters]

        if len(deduped) < 2 and len(still_pool) >= 4:
            used_names = {c["frame_name"] for c in deduped}
            rem = [c for c in still_pool if c["frame_name"] not in used_names]
            need = min(12, len(still_pool)) - len(deduped)
            if rem and need > 0:
                idxs = np.linspace(0, len(rem) - 1, min(need, len(rem)), dtype=int)
                for i in sorted(set(int(x) for x in idxs)):
                    deduped.append(rem[i])

        selected = list(deduped)
        if len(selected) > max_poses:
            outer_reserve_cap = min(10, max_poses // 3)
            outer_candidates = sorted(
                [c for c in selected if c["outer_corners"] >= 4],
                key=lambda c: (c["outer_corners"], c["quality_score"]),
                reverse=True,
            )[:outer_reserve_cap]
            picked_names = {c["frame_name"] for c in outer_candidates}
            rest_candidates = sorted(
                [c for c in selected if c["frame_name"] not in picked_names],
                key=lambda c: (c["cams_with_hits"], c["quality_score"]),
                reverse=True,
            )
            slots_left = max(0, max_poses - len(outer_candidates))
            selected = outer_candidates + rest_candidates[:slots_left]

        selected.sort(key=lambda o: o["frame_idx"])

        raw_frames_cache: Dict[str, Dict[str, np.ndarray]] = {}
        top_for_preview = sorted(
            selected, key=lambda c: (c["cams_with_hits"], c["quality_score"]), reverse=True
        )[:4]
        for item in top_for_preview:
            f_imgs: Dict[str, np.ndarray] = {}
            ok_p = True
            for ck in CAM_ORDER:
                tf_p = max(0, int(item["frame_idx"] + offsets.get(ck, 0)))
                caps[ck].set(cv2.CAP_PROP_POS_FRAMES, tf_p)
                ret_p, frm_p = caps[ck].read()
                if not ret_p or frm_p is None:
                    ok_p = False
                    break
                f_imgs[ck] = frm_p
            if ok_p:
                raw_frames_cache[item["frame_name"]] = f_imgs

        if progress_cb:
            med_mot = float(np.median([c["motion_px"] for c in selected])) if selected else 0.0
            progress_cb(
                f"[ChArUco] Valde {len(selected)} stilla & skarpa frys-stopp ur {len(collected)} träffar "
                f"(filtrerade bort {n_discarded_motion} rutor i rörelse, medianrörelse {med_mot:.2f} px/ruta)."
            )

        return selected, image_sizes, raw_frames_cache
    finally:
        for cap in caps.values():
            cap.release()


# =============================================================================
# 5. GLOBAL GLES BUNDLE ADJUSTMENT & HOLD-OUT KORSVALIDERING
# =============================================================================

def _calibrate_single_camera_intrinsics(
    cam_key: str,
    width: int,
    height: int,
    views: List[Tuple[str, np.ndarray, np.ndarray]],
    detector: MultiBoardDetector,
) -> Tuple[np.ndarray, Dict[Tuple[int, str], Tuple[np.ndarray, np.ndarray]]]:
    """
    Kalibrerar initial `OPENCV_FISHEYE` [fx, fy, cx, cy, k1, k2, k3, k4] och enskilda
    tavle-poser (rvec, tvec) för en kamera via iterative PnP + gles Huber Bundle Adjustment.
    """
    f0 = 0.55 * float(min(width, height))
    K_params = np.array([f0, f0, width * 0.5, height * 0.5, -0.04, 0.008, -0.002, 0.0005], dtype=np.float64)

    view_poses: Dict[int, Tuple[np.ndarray, np.ndarray, str, np.ndarray, np.ndarray]] = {}
    valid_views: List[Tuple[int, str, np.ndarray, np.ndarray]] = []

    for v_idx, (b_name, ids_arr, img_pts) in enumerate(views):
        obj_pts = detector.obj_corners[b_name][ids_arr]
        rays = undistort_fisheye_to_rays(img_pts, K_params)
        norm_2d = (rays[:, :2] / rays[:, 2:3]).reshape(-1, 1, 2).astype(np.float64)
        ok, rvec, tvec, _inl = cv2.solvePnPRansac(
            obj_pts.reshape(-1, 1, 3),
            norm_2d,
            np.eye(3, dtype=np.float64),
            None,
            flags=cv2.SOLVEPNP_IPPE,
            reprojectionError=0.025,
            confidence=0.99,
        )
        if not ok or float(tvec[2, 0]) <= 0.05:
            ok, rvec, tvec, _inl = cv2.solvePnPRansac(
                obj_pts.reshape(-1, 1, 3),
                norm_2d,
                np.eye(3, dtype=np.float64),
                None,
                flags=cv2.SOLVEPNP_ITERATIVE,
                reprojectionError=0.025,
                confidence=0.99,
            )
        if not ok or float(tvec[2, 0]) <= 0.05:
            continue
        rvec_f = rvec.reshape(3).astype(np.float64)
        tvec_f = tvec.reshape(3).astype(np.float64)

        def _pnp_res(x):
            pred, z = project_fisheye_world_points(obj_pts, x[:3], x[3:6], K_params)
            pen = np.where(z <= 0.05, 50.0, 0.0)[:, None]
            return ((pred - img_pts) + pen).ravel()

        sol = optimize.least_squares(
            _pnp_res,
            np.concatenate([rvec_f, tvec_f]),
            loss="huber",
            f_scale=2.0,
            x_scale="jac",
            max_nfev=120,
        )
        pred_v, _ = project_fisheye_world_points(obj_pts, sol.x[:3], sol.x[3:6], K_params)
        v_rms = float(math.sqrt(float(np.mean(np.sum((pred_v - img_pts) ** 2, axis=1)))))
        if v_rms > 25.0:
            continue
        view_poses[v_idx] = (sol.x[:3], sol.x[3:6], b_name, obj_pts, img_pts)
        valid_views.append((v_idx, b_name, obj_pts, img_pts))

    if not valid_views:
        raise RuntimeError(f"Kunde inte estimera några initiala ChArUco-poser för {cam_key}.")

    n_views = len(valid_views)
    x0 = np.concatenate([K_params] + [np.concatenate([view_poses[v[0]][0], view_poses[v[0]][1]]) for v in valid_views])

    n_res = sum(len(v[2]) * 2 for v in valid_views)
    n_par = 8 + 6 * n_views
    rows_sp: List[int] = []
    cols_sp: List[int] = []
    r_cur = 0
    for i, (_v_idx, _b_name, obj_pts, _img_pts) in enumerate(valid_views):
        m = len(obj_pts) * 2
        v_col = 8 + 6 * i
        for rr in range(r_cur, r_cur + m):
            rows_sp.extend([rr] * 14)
            cols_sp.extend(list(range(8)) + list(range(v_col, v_col + 6)))
        r_cur += m
    jac_sp = sparse.coo_matrix((np.ones(len(rows_sp), dtype=bool), (rows_sp, cols_sp)), shape=(n_res, n_par))

    def _intrinsics_res(x):
        intr = x[:8]
        out = np.empty(n_res, dtype=np.float64)
        pos = 0
        for i, (_v_idx, _b_name, obj_pts, img_pts) in enumerate(valid_views):
            off = 8 + 6 * i
            pred, z = project_fisheye_world_points(obj_pts, x[off : off + 3], x[off + 3 : off + 6], intr)
            diff = pred - img_pts
            bad = z <= 0.05
            if np.any(bad):
                diff[bad] = 250.0
            m = len(obj_pts) * 2
            out[pos : pos + m] = diff.ravel()
            pos += m
        return out

    lb = np.full(n_par, -np.inf)
    ub = np.full(n_par, np.inf)
    lb[0], ub[0] = 0.25 * width, 1.8 * max(width, height)
    lb[1], ub[1] = 0.25 * height, 1.8 * max(width, height)
    lb[2], ub[2] = width * 0.35, width * 0.65
    lb[3], ub[3] = height * 0.35, height * 0.65
    lb[4:8], ub[4:8] = -2.5, 2.5

    sol = optimize.least_squares(
        _intrinsics_res,
        x0,
        bounds=(lb, ub),
        jac_sparsity=jac_sp,
        method="trf",
        loss="huber",
        f_scale=1.5,
        x_scale="jac",
        max_nfev=550,
    )

    refined_intr = sol.x[:8].copy()
    refined_poses: Dict[Tuple[int, str], Tuple[np.ndarray, np.ndarray]] = {}
    for i, (v_idx, b_name, _obj_pts, _img_pts) in enumerate(valid_views):
        off = 8 + 6 * i
        refined_poses[(v_idx, b_name)] = (sol.x[off : off + 3].copy(), sol.x[off + 3 : off + 6].copy())
    return refined_intr, refined_poses


def _initialize_rig_and_boards(
    frames: List[Dict[str, Any]],
    view_poses_by_cam: Dict[str, Dict[Tuple[int, str], Tuple[np.ndarray, np.ndarray]]],
    baseline_mm: float = 300.0,
) -> Tuple[str, Dict[str, np.ndarray], Dict[str, np.ndarray], Dict[int, np.ndarray], Dict[str, int]]:
    """
    Kedjar ihop tavlornas inbördes 3D-lägen (`T_refB_from_B`), kamerornas rigg-lägen
    (`T_cam_from_rig`) relativt `cam2_center`, och riggens frame-poser (`T_rig_from_refB`)
    via minsta-hopp BFS över gemensamma observationer (med CAD-prior endast som reserv).
    """
    board_names = [b[0] for b in BOARD_SPECS]
    # Räkna vilka tavlor som ses av center-kameran
    center_counts = {b: 0 for b in board_names}
    total_counts = {b: 0 for b in board_names}
    for t_idx, fr in enumerate(frames):
        obs = fr["observations"]
        for ck, bhits in obs.items():
            for b in bhits:
                if (t_idx, b) in view_poses_by_cam.get(ck, {}):
                    total_counts[b] += 1
                    if ck == "cam2_center":
                        center_counts[b] += 1

    ref_board = max(board_names, key=lambda b: (center_counts[b] > 0, b == "board_1_center", center_counts[b], total_counts[b]))
    T_refB_from_B: Dict[str, np.ndarray] = {ref_board: np.eye(4, dtype=np.float64)}
    T_cam_from_rig: Dict[str, np.ndarray] = {"cam2_center": np.eye(4, dtype=np.float64)}

    # Iterativ BFS: propagation mellan tavlor i samma kamera och mellan kameror över samma tavla
    cam_overlap_counts = {"cam1_left": 0, "cam3_right": 0}
    for _ in range(6):
        changed = False
        # 1) Nya tavlor via samma kamera där ref_board/känd tavla ses
        for b_new in board_names:
            if b_new in T_refB_from_B:
                continue
            cand_T: List[np.ndarray] = []
            for t_idx in range(len(frames)):
                for ck in CAM_ORDER:
                    vmap = view_poses_by_cam.get(ck, {})
                    if (t_idx, b_new) not in vmap:
                        continue
                    T_c_new = _se3_from_rvec_tvec(*vmap[(t_idx, b_new)])
                    for b_known, T_rb_known in list(T_refB_from_B.items()):
                        if (t_idx, b_known) in vmap:
                            T_c_known = _se3_from_rvec_tvec(*vmap[(t_idx, b_known)])
                            # p_known = inv(T_c_known) @ T_c_new @ p_new
                            T_known_new = np.linalg.inv(T_c_known) @ T_c_new
                            cand_T.append(T_rb_known @ T_known_new)
            if cand_T:
                T_refB_from_B[b_new] = _median_se3(cand_T)
                changed = True

        # 2) Sidokamera relativt cam2_center via kända tavlor vid samma tidpunkt
        for side_cam in ("cam1_left", "cam3_right"):
            cand_rel: List[np.ndarray] = []
            c_map = view_poses_by_cam.get("cam2_center", {})
            s_map = view_poses_by_cam.get(side_cam, {})
            for t_idx in range(len(frames)):
                ref_cands: List[np.ndarray] = []
                side_cands: List[np.ndarray] = []
                for b_k, T_rb_b in T_refB_from_B.items():
                    inv_rb_b = np.linalg.inv(T_rb_b)
                    if (t_idx, b_k) in c_map:
                        ref_cands.append(_se3_from_rvec_tvec(*c_map[(t_idx, b_k)]) @ inv_rb_b)
                    if (t_idx, b_k) in s_map:
                        side_cands.append(_se3_from_rvec_tvec(*s_map[(t_idx, b_k)]) @ inv_rb_b)
                if ref_cands and side_cands:
                    T_0_rb = _median_se3(ref_cands)
                    T_s_rb = _median_se3(side_cands)
                    cand_rel.append(T_s_rb @ np.linalg.inv(T_0_rb))
            if cand_rel:
                cam_overlap_counts[side_cam] = len(cand_rel)
                if side_cam not in T_cam_from_rig:
                    T_cam_from_rig[side_cam] = _median_se3(cand_rel)
                    changed = True

        # 3) Om en sidokamera redan har känd T_cam_from_rig kan vi även härleda nya tavlor via cam2_center <-> side_cam
        for b_new in board_names:
            if b_new in T_refB_from_B:
                continue
            cand_T = []
            for t_idx in range(len(frames)):
                for ck in ("cam1_left", "cam3_right"):
                    if ck not in T_cam_from_rig:
                        continue
                    vmap = view_poses_by_cam.get(ck, {})
                    cmap = view_poses_by_cam.get("cam2_center", {})
                    if (t_idx, b_new) not in vmap:
                        continue
                    T_s_new = _se3_from_rvec_tvec(*vmap[(t_idx, b_new)])
                    T_0_s = np.linalg.inv(T_cam_from_rig[ck])
                    for b_known, T_rb_known in list(T_refB_from_B.items()):
                        if (t_idx, b_known) in cmap:
                            T_0_known = _se3_from_rvec_tvec(*cmap[(t_idx, b_known)])
                            T_known_new = np.linalg.inv(T_0_known) @ T_0_s @ T_s_new
                            cand_T.append(T_rb_known @ T_known_new)
            if cand_T:
                T_refB_from_B[b_new] = _median_se3(cand_T)
                changed = True

        if not changed:
            break

    # Fallback med nominell CAD-prior för ev. kamera/tavla som helt saknar gemensamt fält
    for side_cam in ("cam1_left", "cam3_right"):
        if side_cam not in T_cam_from_rig:
            T_cam_from_rig[side_cam] = nominal_rig_rel_se3(side_cam, baseline_mm=baseline_mm)

    for b_new in board_names:
        if b_new not in T_refB_from_B:
            cand_T = []
            for t_idx in range(len(frames)):
                for ck in CAM_ORDER:
                    vmap = view_poses_by_cam.get(ck, {})
                    cmap = view_poses_by_cam.get("cam2_center", {})
                    if (t_idx, b_new) not in vmap:
                        continue
                    T_c_new = _se3_from_rvec_tvec(*vmap[(t_idx, b_new)])
                    T_0_c = np.linalg.inv(T_cam_from_rig[ck])
                    for b_known, T_rb_known in list(T_refB_from_B.items()):
                        if (t_idx, b_known) in cmap:
                            T_0_known = _se3_from_rvec_tvec(*cmap[(t_idx, b_known)])
                            cand_T.append(T_rb_known @ np.linalg.inv(T_0_known) @ T_0_c @ T_c_new)
            T_refB_from_B[b_new] = _median_se3(cand_T) if cand_T else np.eye(4, dtype=np.float64)

    # Beräkna initial T_rig_from_refB per timestamp
    T_rig_frames: Dict[int, np.ndarray] = {}
    for t_idx in range(len(frames)):
        cands: List[np.ndarray] = []
        for ck in CAM_ORDER:
            inv_cam_rel = np.linalg.inv(T_cam_from_rig[ck])
            vmap = view_poses_by_cam.get(ck, {})
            for b_name, T_rb_b in T_refB_from_B.items():
                if (t_idx, b_name) in vmap:
                    T_c_b = _se3_from_rvec_tvec(*vmap[(t_idx, b_name)])
                    cands.append(inv_cam_rel @ T_c_b @ np.linalg.inv(T_rb_b))
        if cands:
            T_rig_frames[t_idx] = _median_se3(cands)

    return ref_board, T_cam_from_rig, T_refB_from_B, T_rig_frames, cam_overlap_counts


def _run_joint_rig_bundle_adjustment(
    train_frame_indices: List[int],
    frames: List[Dict[str, Any]],
    detector: MultiBoardDetector,
    intrinsics: Dict[str, np.ndarray],
    ref_board: str,
    T_cam_from_rig: Dict[str, np.ndarray],
    T_refB_from_B: Dict[str, np.ndarray],
    T_rig_frames: Dict[int, np.ndarray],
    max_nfev: int = 750,
) -> Tuple[Dict[str, np.ndarray], Dict[str, np.ndarray], Dict[str, np.ndarray], Dict[int, np.ndarray], float]:
    """
    Kör gles gemensam Levenberg-Marquardt / TRF Bundle Adjustment över:
      - 3 kamerors 8-param OPENCV_FISHEYE intrinsics
      - 2 sidokamerors 6-DoF rigg-extrinsics (relativt cam2_center)
      - icke-referenstavlor (6-DoF relativt ref_board)
      - riggens 6-DoF pose per bild-trio
    """
    other_boards = [b[0] for b in BOARD_SPECS if b[0] != ref_board and b[0] in T_refB_from_B]
    train_indices = [ti for ti in train_frame_indices if ti in T_rig_frames]
    t_to_slot = {ti: idx for idx, ti in enumerate(train_indices)}

    # Paketera startvektor
    x_parts: List[np.ndarray] = []
    cam_param_offset: Dict[str, int] = {}
    cur = 0
    for ck in CAM_ORDER:
        cam_param_offset[ck] = cur
        x_parts.append(intrinsics[ck].copy())
        cur += 8

    ext_param_offset: Dict[str, int] = {}
    for ck in ("cam1_left", "cam3_right"):
        ext_param_offset[ck] = cur
        rv, tv = _rvec_tvec_from_se3(T_cam_from_rig[ck])
        x_parts.append(np.concatenate([rv, tv]))
        cur += 6

    board_param_offset: Dict[str, int] = {}
    for bn in other_boards:
        board_param_offset[bn] = cur
        rv, tv = _rvec_tvec_from_se3(T_refB_from_B[bn])
        x_parts.append(np.concatenate([rv, tv]))
        cur += 6

    frame_param_offset: Dict[int, int] = {}
    for ti in train_indices:
        frame_param_offset[ti] = cur
        rv, tv = _rvec_tvec_from_se3(T_rig_frames[ti])
        x_parts.append(np.concatenate([rv, tv]))
        cur += 6

    n_params = cur
    x0 = np.concatenate(x_parts)

    # Platta ut alla observationer i träningssetet
    obs_list: List[Tuple[int, str, str, np.ndarray, np.ndarray]] = []
    for ti in train_indices:
        obs_dict = frames[ti]["observations"]
        for ck in CAM_ORDER:
            for bn, (ids_arr, img_pts) in obs_dict.get(ck, {}).items():
                if bn != ref_board and bn not in board_param_offset:
                    continue
                obj_pts = detector.obj_corners[bn][ids_arr]
                obs_list.append((ti, ck, bn, obj_pts, img_pts))

    n_res = sum(len(o[3]) * 2 for o in obs_list)
    if n_res == 0:
        return intrinsics, T_cam_from_rig, T_refB_from_B, T_rig_frames, 999.0

    # Gles Jacobian-struktur för snabb LM/TRF
    sp_rows: List[int] = []
    sp_cols: List[int] = []
    r_cur = 0
    for ti, ck, bn, obj_pts, _img_pts in obs_list:
        m = len(obj_pts) * 2
        cols: List[int] = []
        c_off = cam_param_offset[ck]
        cols.extend(range(c_off, c_off + 8))
        if ck in ext_param_offset:
            e_off = ext_param_offset[ck]
            cols.extend(range(e_off, e_off + 6))
        if bn in board_param_offset:
            b_off = board_param_offset[bn]
            cols.extend(range(b_off, b_off + 6))
        f_off = frame_param_offset[ti]
        cols.extend(range(f_off, f_off + 6))
        for rr in range(r_cur, r_cur + m):
            sp_rows.extend([rr] * len(cols))
            sp_cols.extend(cols)
        r_cur += m

    jac_sp = sparse.coo_matrix((np.ones(len(sp_rows), dtype=bool), (sp_rows, sp_cols)), shape=(n_res, n_params))

    def _residual_fn(x: np.ndarray) -> np.ndarray:
        out = np.empty(n_res, dtype=np.float64)
        cam_se3: Dict[str, np.ndarray] = {"cam2_center": np.eye(4, dtype=np.float64)}
        for ck in ("cam1_left", "cam3_right"):
            eo = ext_param_offset[ck]
            cam_se3[ck] = _se3_from_rvec_tvec(x[eo : eo + 3], x[eo + 3 : eo + 6])

        board_se3: Dict[str, np.ndarray] = {ref_board: np.eye(4, dtype=np.float64)}
        for bn, bo in board_param_offset.items():
            board_se3[bn] = _se3_from_rvec_tvec(x[bo : bo + 3], x[bo + 3 : bo + 6])

        frame_se3: Dict[int, np.ndarray] = {}
        for ti, fo in frame_param_offset.items():
            frame_se3[ti] = _se3_from_rvec_tvec(x[fo : fo + 3], x[fo + 3 : fo + 6])

        pos = 0
        for ti, ck, bn, obj_pts, img_pts in obs_list:
            T_c_b = cam_se3[ck] @ frame_se3[ti] @ board_se3[bn]
            co = cam_param_offset[ck]
            pred, z = project_fisheye_world_points(obj_pts, T_c_b[:3, :3], T_c_b[:3, 3], x[co : co + 8])
            diff = pred - img_pts
            bad = z <= 0.05
            if np.any(bad):
                diff[bad] = 250.0
            m = len(obj_pts) * 2
            out[pos : pos + m] = diff.ravel()
            pos += m
        return out

    sol = optimize.least_squares(
        _residual_fn,
        x0,
        jac_sparsity=jac_sp,
        method="trf",
        loss="huber",
        f_scale=1.5,
        x_scale="jac",
        max_nfev=max_nfev,
    )

    # Packa upp optimerade värden
    res_vec = _residual_fn(sol.x).reshape(-1, 2)
    rms_px = float(math.sqrt(float(np.mean(np.sum(res_vec ** 2, axis=1)))))

    new_intr = {ck: sol.x[cam_param_offset[ck] : cam_param_offset[ck] + 8].copy() for ck in CAM_ORDER}
    new_cam_rel = {"cam2_center": np.eye(4, dtype=np.float64)}
    for ck in ("cam1_left", "cam3_right"):
        eo = ext_param_offset[ck]
        new_cam_rel[ck] = _se3_from_rvec_tvec(sol.x[eo : eo + 3], sol.x[eo + 3 : eo + 6])

    new_boards = {ref_board: np.eye(4, dtype=np.float64)}
    for bn, bo in board_param_offset.items():
        new_boards[bn] = _se3_from_rvec_tvec(sol.x[bo : bo + 3], sol.x[bo + 3 : bo + 6])

    new_frames = dict(T_rig_frames)
    for ti, fo in frame_param_offset.items():
        new_frames[ti] = _se3_from_rvec_tvec(sol.x[fo : fo + 3], sol.x[fo + 3 : fo + 6])

    return new_intr, new_cam_rel, new_boards, new_frames, rms_px


def _evaluate_holdout_frames(
    holdout_indices: List[int],
    frames: List[Dict[str, Any]],
    detector: MultiBoardDetector,
    intrinsics: Dict[str, np.ndarray],
    T_cam_from_rig: Dict[str, np.ndarray],
    T_refB_from_B: Dict[str, np.ndarray],
    T_rig_frames: Dict[int, np.ndarray],
) -> float:
    """Utvärderar Hold-Out RMS med låsta kameror, rigg-extrinsics & tavlor (löser bara 6-DoF pose per trio)."""
    sq_errs: List[np.ndarray] = []
    for ti in holdout_indices:
        if ti not in T_rig_frames:
            continue
        obs_dict = frames[ti]["observations"]
        frame_obs: List[Tuple[str, str, np.ndarray, np.ndarray]] = []
        for ck in CAM_ORDER:
            for bn, (ids_arr, img_pts) in obs_dict.get(ck, {}).items():
                if bn in T_refB_from_B:
                    frame_obs.append((ck, bn, detector.obj_corners[bn][ids_arr], img_pts))
        if not frame_obs:
            continue
        rv0, tv0 = _rvec_tvec_from_se3(T_rig_frames[ti])

        def _pose_res(x):
            T_rig = _se3_from_rvec_tvec(x[:3], x[3:6])
            errs = []
            for ck, bn, obj_pts, img_pts in frame_obs:
                T_cb = T_cam_from_rig[ck] @ T_rig @ T_refB_from_B[bn]
                pred, _z = project_fisheye_world_points(obj_pts, T_cb[:3, :3], T_cb[:3, 3], intrinsics[ck])
                errs.append((pred - img_pts).ravel())
            return np.concatenate(errs)

        sol = optimize.least_squares(
            _pose_res,
            np.concatenate([rv0, tv0]),
            loss="huber",
            f_scale=2.0,
            x_scale="jac",
            max_nfev=140,
        )
        err2d = _pose_res(sol.x).reshape(-1, 2)
        sq_errs.append(np.sum(err2d ** 2, axis=1))

    if not sq_errs:
        return 0.0
    all_sq = np.concatenate(sq_errs)
    return float(math.sqrt(float(np.mean(all_sq))))


def _compute_rig_stability_spread(
    frames: List[Dict[str, Any]],
    detector: MultiBoardDetector,
    intrinsics: Dict[str, np.ndarray],
    T_cam_from_rig: Dict[str, np.ndarray],
    T_refB_from_B: Dict[str, np.ndarray],
    T_rig_frames: Dict[int, np.ndarray],
) -> Dict[str, Dict[str, float]]:
    """Mäter hur mycket varje enskild trio avviker från den låsta rigg-geometrin (grader & mm)."""
    res: Dict[str, Dict[str, float]] = {
        ck: {"spread_deg": 0.0, "spread_mm": 0.0, "multi_frames": 0} for ck in CAM_ORDER
    }
    for side_cam in ("cam1_left", "cam3_right"):
        ang_diffs: List[float] = []
        pos_diffs_mm: List[float] = []
        T_cal = T_cam_from_rig[side_cam]
        C_cal = -T_cal[:3, :3].T @ T_cal[:3, 3]

        for ti, fr in enumerate(frames):
            obs = fr["observations"]
            c_hits = [(bn, detector.obj_corners[bn][ids], pts) for bn, (ids, pts) in obs.get("cam2_center", {}).items() if bn in T_refB_from_B]
            s_hits = [(bn, detector.obj_corners[bn][ids], pts) for bn, (ids, pts) in obs.get(side_cam, {}).items() if bn in T_refB_from_B]
            if not c_hits or not s_hits or ti not in T_rig_frames:
                continue

            # Lös separat 6-DoF pose mot ref_board för center och side
            def _solve_free(ck_name, hits_list, T_init):
                rv0, tv0 = _rvec_tvec_from_se3(T_init)

                def _fn(x):
                    T_rb = _se3_from_rvec_tvec(x[:3], x[3:6])
                    e = []
                    for bn, obj_pts, img_pts in hits_list:
                        rv, tv = _rvec_tvec_from_se3(T_rb @ T_refB_from_B[bn])
                        pr, _ = project_fisheye_world_points(obj_pts, rv, tv, intrinsics[ck_name])
                        e.append((pr - img_pts).ravel())
                    return np.concatenate(e)

                sol = optimize.least_squares(_fn, np.concatenate([rv0, tv0]), max_nfev=25)
                return _se3_from_rvec_tvec(sol.x[:3], sol.x[3:6])

            T_0_free = _solve_free("cam2_center", c_hits, T_rig_frames[ti])
            T_s_free = _solve_free(side_cam, s_hits, T_cal @ T_rig_frames[ti])
            T_rel_free = T_s_free @ np.linalg.inv(T_0_free)

            dR = T_rel_free[:3, :3] @ T_cal[:3, :3].T
            cos_th = float(np.clip((np.trace(dR) - 1.0) * 0.5, -1.0, 1.0))
            ang_diffs.append(math.degrees(math.acos(cos_th)))
            C_free = -T_rel_free[:3, :3].T @ T_rel_free[:3, 3]
            pos_diffs_mm.append(float(np.linalg.norm(C_free - C_cal) * 1000.0))

        if ang_diffs:
            res[side_cam] = {
                "spread_deg": round(float(np.median(ang_diffs)), 3),
                "spread_mm": round(float(np.median(pos_diffs_mm)), 2),
                "multi_frames": len(ang_diffs),
            }
    return res


# =============================================================================
# 6. VISUELLA VERIFIERINGSBILDER & KONTROLLRAPPORT
# =============================================================================

def _draw_hud_banner(img: np.ndarray, line1: str, line2: str, color_bgr: Tuple[int, int, int]) -> None:
    h, w = img.shape[:2]
    bar_h = max(95, int(h * 0.045))
    overlay = img.copy()
    cv2.rectangle(overlay, (0, 0), (w, bar_h), (12, 14, 18), -1)
    cv2.addWeighted(overlay, 0.82, img, 0.18, 0.0, img)
    cv2.line(img, (0, bar_h), (w, bar_h), color_bgr, max(3, w // 700))
    f_scale = max(0.85, w / 1600.0)
    thick = max(2, int(round(f_scale * 1.8)))
    cv2.putText(img, line1, (26, int(bar_h * 0.43)), cv2.FONT_HERSHEY_SIMPLEX, f_scale, (240, 244, 248), thick, cv2.LINE_AA)
    cv2.putText(img, line2, (26, int(bar_h * 0.82)), cv2.FONT_HERSHEY_SIMPLEX, f_scale * 0.78, color_bgr, max(1, thick - 1), cv2.LINE_AA)


def _render_calibration_visuals(
    frames: List[Dict[str, Any]],
    detector: MultiBoardDetector,
    image_sizes: Dict[str, Tuple[int, int]],
    intrinsics: Dict[str, np.ndarray],
    T_cam_from_rig: Dict[str, np.ndarray],
    T_refB_from_B: Dict[str, np.ndarray],
    T_rig_frames: Dict[int, np.ndarray],
    raw_frames_cache: Dict[str, Dict[str, np.ndarray]],
    preview_dir: Path,
) -> Tuple[Dict[str, Dict[str, Any]], Dict[str, str]]:
    """
    Ritar:
      - `reproj_<cam>.jpg`: Gröna ringar (detekterade hörn) vs röda + (rigg-lås projekterat)
      - `coverage_<cam>.jpg`: Alla insamlade hörn på 184° fisheye-sensorn + % i yttre zonen (>70% radie)
    """
    preview_dir.mkdir(parents=True, exist_ok=True)
    cov_stats: Dict[str, Dict[str, Any]] = {}
    preview_paths: Dict[str, str] = {}

    # Samla alla hörn + per-kamera RMS
    all_pts_by_cam: Dict[str, List[np.ndarray]] = {ck: [] for ck in CAM_ORDER}
    all_err_by_cam: Dict[str, List[np.ndarray]] = {ck: [] for ck in CAM_ORDER}

    for ti, fr in enumerate(frames):
        if ti not in T_rig_frames:
            continue
        for ck in CAM_ORDER:
            for bn, (ids_arr, img_pts) in fr["observations"].get(ck, {}).items():
                if bn not in T_refB_from_B:
                    continue
                obj_pts = detector.obj_corners[bn][ids_arr]
                T_cb = T_cam_from_rig[ck] @ T_rig_frames[ti] @ T_refB_from_B[bn]
                rv, tv = _rvec_tvec_from_se3(T_cb)
                pred, _ = project_fisheye_world_points(obj_pts, rv, tv, intrinsics[ck])
                err = np.linalg.norm(pred - img_pts, axis=1)
                all_pts_by_cam[ck].append(img_pts)
                all_err_by_cam[ck].append(err)

    # Välj bästa trio som finns i raw_frames_cache
    best_fr = None
    best_score = -1
    for fr in frames:
        if fr["frame_name"] in raw_frames_cache and fr.get("frame_idx") is not None:
            sc = fr["cams_with_hits"] * 1000 + fr["total_corners"]
            if sc > best_score:
                best_score = sc
                best_fr = fr

    best_ti = frames.index(best_fr) if best_fr is not None else -1

    for ck in CAM_ORDER:
        w, h = image_sizes[ck]
        pts_list = all_pts_by_cam[ck]
        err_list = all_err_by_cam[ck]
        pts_cat = np.vstack(pts_list) if pts_list else np.zeros((0, 2), dtype=np.float64)
        err_cat = np.concatenate(err_list) if err_list else np.zeros(0, dtype=np.float64)

        rms_c = float(math.sqrt(float(np.mean(err_cat ** 2)))) if len(err_cat) else 999.0
        cx, cy = float(intrinsics[ck][2]), float(intrinsics[ck][3])
        half_w = max(1.0, 0.5 * float(w))
        half_h = max(1.0, 0.5 * float(h))
        outer_thresh = 0.62
        if len(pts_cat) > 0:
            norm_r = np.hypot((pts_cat[:, 0] - cx) / half_w, (pts_cat[:, 1] - cy) / half_h)
            outer_pct = round(100.0 * float(np.mean(norm_r >= outer_thresh)), 1)
        else:
            norm_r = np.zeros(0)
            outer_pct = 0.0

        cov_stats[ck] = {
            "total_corners": int(len(pts_cat)),
            "rms_px": round(rms_c, 3),
            "outer_zone_pct": outer_pct,
        }

        # 1) Coverage-bild (skalad till max ~1080 px bredd för snabb UI-visning)
        scale_out = min(1.0, 1080.0 / max(1, w))
        cw, ch = int(round(w * scale_out)), int(round(h * scale_out))
        cov_img = np.full((ch, cw, 3), (14, 16, 20), dtype=np.uint8)
        for g in range(1, 4):
            cv2.line(cov_img, (int(cw * g / 4), 0), (int(cw * g / 4), ch), (34, 38, 46), 1)
            cv2.line(cov_img, (0, int(ch * g / 4)), (cw, int(ch * g / 4)), (34, 38, 46), 1)
        c_center = (int(round(cx * scale_out)), int(round(cy * scale_out)))
        ell_axes = (int(round(outer_thresh * 0.5 * cw)), int(round(outer_thresh * 0.5 * ch)))
        cv2.ellipse(cov_img, c_center, ell_axes, 0, 0, 360, (70, 130, 180), 2, cv2.LINE_AA)
        cv2.drawMarker(cov_img, c_center, (200, 200, 200), cv2.MARKER_CROSS, 18, 1)

        for pt, nr in zip(pts_cat, norm_r):
            px_i = int(round(pt[0] * scale_out))
            py_i = int(round(pt[1] * scale_out))
            col = (60, 220, 90) if nr >= outer_thresh else (235, 160, 50)
            cv2.circle(cov_img, (px_i, py_i), 3, col, -1, cv2.LINE_AA)

        col_ok = (63, 185, 80) if len(pts_cat) >= 120 and outer_pct >= 12.0 else (34, 153, 210)
        _draw_hud_banner(
            cov_img,
            f"{CAM_LABELS[ck]} ({w}x{h}) - Kanthörn & täckning",
            f"Hörn: {len(pts_cat)} st | Yttre zon (>62% radie): {outer_pct:.1f}% | Kamera-RMS: {rms_c:.3f} px",
            col_ok,
        )
        cov_path = preview_dir / f"coverage_{ck}.jpg"
        cv2.imwrite(str(cov_path), cov_img, [cv2.IMWRITE_JPEG_QUALITY, 92])
        preview_paths[f"coverage_{ck}"] = str(cov_path)

        # 2) Reprojicerings-bild
        if best_fr is not None and best_fr["frame_name"] in raw_frames_cache and best_ti in T_rig_frames:
            src_img = raw_frames_cache[best_fr["frame_name"]][ck].copy()
        else:
            src_img = np.full((h, w, 3), (18, 20, 25), dtype=np.uint8)

        scale_rep = min(1.0, 1200.0 / max(1, w))
        rw, rh = int(round(w * scale_rep)), int(round(h * scale_rep))
        rep_img = cv2.resize(src_img, (rw, rh), interpolation=cv2.INTER_AREA)

        fr_errs: List[float] = []
        if best_fr is not None and best_ti >= 0:
            for bn, (ids_arr, img_pts) in best_fr["observations"].get(ck, {}).items():
                if bn not in T_refB_from_B:
                    continue
                obj_pts = detector.obj_corners[bn][ids_arr]
                T_cb = T_cam_from_rig[ck] @ T_rig_frames[best_ti] @ T_refB_from_B[bn]
                rv, tv = _rvec_tvec_from_se3(T_cb)
                pred, _ = project_fisheye_world_points(obj_pts, rv, tv, intrinsics[ck])
                for p_meas, p_proj in zip(img_pts, pred):
                    e_px = float(np.linalg.norm(p_proj - p_meas))
                    fr_errs.append(e_px)
                    x1, y1 = int(round(p_meas[0] * scale_rep)), int(round(p_meas[1] * scale_rep))
                    x2, y2 = int(round(p_proj[0] * scale_rep)), int(round(p_proj[1] * scale_rep))
                    # Förstärkt felpil (x12) så subpixel-riktningen syns tydligt
                    dx = int(round((p_proj[0] - p_meas[0]) * 12.0 * scale_rep))
                    dy = int(round((p_proj[1] - p_meas[1]) * 12.0 * scale_rep))
                    cv2.line(rep_img, (x1, y1), (x1 + dx, y1 + dy), (0, 220, 255), 2, cv2.LINE_AA)
                    cv2.circle(rep_img, (x1, y1), 7, (50, 230, 80), 2, cv2.LINE_AA)
                    cv2.drawMarker(rep_img, (x2, y2), (50, 60, 255), cv2.MARKER_CROSS, 11, 2, cv2.LINE_AA)

        fr_rms = float(math.sqrt(float(np.mean(np.square(fr_errs))))) if fr_errs else rms_c
        hud_col = (63, 185, 80) if fr_rms < 0.50 else ((34, 153, 210) if fr_rms < 1.0 else (73, 81, 248))
        _draw_hud_banner(
            rep_img,
            f"{CAM_LABELS[ck]} - Rigg-låst återprojektion",
            f"Grön ring=Mätt | Röd +=Låst Rigg | Trio-RMS: {fr_rms:.3f} px (Totalt {rms_c:.3f} px)",
            hud_col,
        )
        rep_path = preview_dir / f"reproj_{ck}.jpg"
        cv2.imwrite(str(rep_path), rep_img, [cv2.IMWRITE_JPEG_QUALITY, 92])
        preview_paths[f"reproj_{ck}"] = str(rep_path)

    return cov_stats, preview_paths


def _evaluate_5point_checks(
    rms_px: float,
    holdout_rms_px: float,
    stability: Dict[str, Dict[str, float]],
    cov_stats: Dict[str, Dict[str, Any]],
    per_cam_data: Dict[str, Dict[str, Any]],
    baselines_mm: Dict[str, float],
    cam_overlap_counts: Dict[str, int],
) -> Tuple[List[Dict[str, str]], str]:
    checks: List[Dict[str, str]] = []

    # 1. RMS återprojektionsfel
    st1 = "OK" if rms_px < 0.50 else ("WARN" if rms_px < 1.0 else "FAIL")
    checks.append(
        {
            "id": "reproj_rms",
            "title": "1. ChArUco återprojektionsfel (RMS)",
            "value": f"{rms_px:.3f} px",
            "target": "< 0.50 px (helst 0.1–0.3 px)",
            "status": st1,
            "detail": "Mäter hur exakt den låsta OPENCV_FISHEYE-linsen & riggen träffar uppmätta ChArUco-hörn.",
        }
    )

    # 2. Hold-out & rigg-stabilitet
    max_spread_deg = max(stability["cam1_left"]["spread_deg"], stability["cam3_right"]["spread_deg"])
    st2 = (
        "OK"
        if holdout_rms_px < 0.65 and max_spread_deg < 0.25
        else ("WARN" if holdout_rms_px < 1.25 and max_spread_deg < 0.80 else "FAIL")
    )
    checks.append(
        {
            "id": "holdout_stability",
            "title": "2. Hold-Out validering (20%) & Rigg-stabilitet",
            "value": f"{holdout_rms_px:.3f} px • ±{max_spread_deg:.3f}°",
            "target": "< 0.65 px & < 0.25°",
            "status": st2,
            "detail": "Testar låst kalibrering på helt nya osparade bild-trior så modellen inte överanpassats.",
        }
    )

    # 3. Hörntäckning i 184° fisheye-kanterna
    min_corners = min(c["total_corners"] for c in cov_stats.values())
    min_outer = min(c["outer_zone_pct"] for c in cov_stats.values())
    st3 = (
        "OK"
        if min_corners >= 120 and min_outer >= 12.0
        else ("WARN" if min_corners >= 40 else "FAIL")
    )
    checks.append(
        {
            "id": "corner_coverage",
            "title": "3. Fisheye hörntäckning & k1–k4 kanter",
            "value": f"Min {min_corners} hörn • {min_outer:.1f}% i ytterkant",
            "target": ">= 120 hörn & >= 12% i ytterzon",
            "status": st3,
            "detail": "Behövs för att k1–k4-distorsionen ska bli rätt ända ut i bildkanterna (3DGUT).",
        }
    )

    # 4. Fysiskt måttbandstest (baslinjer & vinklar)
    b_lr = baselines_mm["left_to_right_mm"]
    l_yaw = per_cam_data["cam1_left"]["phys_yaw"]
    r_yaw = per_cam_data["cam3_right"]["phys_yaw"]
    plausible = (240.0 <= b_lr <= 380.0) and (-65.0 <= l_yaw <= -25.0) and (25.0 <= r_yaw <= 65.0)
    st4 = "OK" if plausible else "WARN"
    checks.append(
        {
            "id": "physical_geometry",
            "title": "4. Måttbands- & vinkelkontroll (C1–C3)",
            "value": f"{b_lr:.1f} mm • Yaw {l_yaw:+.1f}° / {r_yaw:+.1f}°",
            "target": "~300 mm & ca ±45° / -12°/+8°",
            "status": st4,
            "detail": "Kontrollmät avståndet C1-lins till C3-lins med måttband — det ska matcha millimetern ovan!",
        }
    )

    # 5. Överlapp mellan kameror
    min_ov = min(cam_overlap_counts.values())
    st5 = "OK" if min_ov >= 3 else ("WARN" if min_ov >= 1 else "FAIL")
    checks.append(
        {
            "id": "cross_overlap",
            "title": "5. Gemensam tavel-täckning mellan kameror",
            "value": f"L↔C: {cam_overlap_counts['cam1_left']} • R↔C: {cam_overlap_counts['cam3_right']} trior",
            "target": ">= 3 delade trior per sida",
            "status": st5,
            "detail": "Vrid riggen -45°..+45° under kalibrering så sidokameror & mittenkamera ser gemensamma tavlor.",
        }
    )

    statuses = [c["status"] for c in checks]
    overall = "FAIL" if "FAIL" in statuses else ("WARN" if "WARN" in statuses else "OK")
    return checks, overall


def _write_verification_report_html(profile: Dict[str, Any], out_html: Path) -> None:
    out_html.parent.mkdir(parents=True, exist_ok=True)
    overall = profile.get("overall_status", "WARN")
    ov_color = "#2ea043" if overall == "OK" else ("#d29922" if overall == "WARN" else "#f85149")
    ov_txt = "GODKÄND (REDO FÖR FÄLT)" if overall == "OK" else ("KONTROLLERA VARNINGAR" if overall == "WARN" else "EJ GODKÄND — KALIBRERA OM")

    check_rows = ""
    for ch in profile.get("checks", []):
        c_col = "#3fb950" if ch["status"] == "OK" else ("#f0b72f" if ch["status"] == "WARN" else "#f85149")
        check_rows += (
            f"<tr><td><strong>{html_lib.escape(ch['title'])}</strong><br><small style='color:#9da3ae;'>{html_lib.escape(ch['detail'])}</small></td>"
            f"<td><code>{html_lib.escape(ch['value'])}</code></td>"
            f"<td><code>{html_lib.escape(ch['target'])}</code></td>"
            f"<td style='color:{c_col}; font-weight:700;'>{html_lib.escape(ch['status'])}</td></tr>"
        )

    cam_rows = ""
    for ck in CAM_ORDER:
        cd = profile["per_camera"][ck]
        intr = cd["intrinsics"]
        pos = cd["pos_mm"]
        cam_rows += (
            f"<tr><td><strong>{ck}</strong></td>"
            f"<td>{cd['width']}×{cd['height']}</td>"
            f"<td>{intr[0]:.1f} / {intr[1]:.1f}</td>"
            f"<td>{intr[2]:.1f}, {intr[3]:.1f}</td>"
            f"<td>[{intr[4]:+.4f}, {intr[5]:+.4f}, {intr[6]:+.4f}, {intr[7]:+.4f}]</td>"
            f"<td>{pos[0]:+.1f}, {pos[1]:+.1f}, {pos[2]:+.1f} mm</td>"
            f"<td>{cd['phys_yaw']:+.2f}° / {cd['phys_tilt']:+.2f}° / {cd['phys_roll']:+.2f}°</td></tr>"
        )

    b = profile.get("baselines_mm", {})
    html_str = f"""<!DOCTYPE html>
<html lang="sv">
<head>
  <meta charset="UTF-8" />
  <title>OptiGSRigg Kalibreringsrapport</title>
  <style>
    body {{ font-family: "Segoe UI", Arial, sans-serif; background:#141517; color:#dcdfe4; margin:20px; }}
    .wrap {{ max-width: 1180px; margin: 0 auto; }}
    .banner {{ padding: 14px 18px; border-left: 5px solid {ov_color}; background:#1b1c1f; margin-bottom:18px; }}
    table {{ width:100%; border-collapse: collapse; background:#111214; margin-bottom: 18px; font-size:12px; }}
    th, td {{ border:1px solid #2c2e33; padding:8px 10px; text-align:left; vertical-align:top; }}
    th {{ background:#1b1c1f; color:#9da3ae; text-transform:uppercase; font-size:11px; }}
    code {{ font-family: Consolas, monospace; color:#93c5fd; }}
    .grid3 {{ display:grid; grid-template-columns:repeat(3,1fr); gap:10px; margin-bottom:18px; }}
    .grid3 img {{ width:100%; border:1px solid #2c2e33; border-radius:2px; }}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="banner">
      <div style="font-size:18px; font-weight:700; color:{ov_color};">{ov_txt}</div>
      <div style="font-size:12px; color:#9da3ae; margin-top:4px;">
        Skapad: {html_lib.escape(str(profile.get('created_at', '-')))} • Källa: <code>{html_lib.escape(str(profile.get('source', '-')))}</code>
        • Linjalfaktor: <code>{profile.get('ruler_100mm', 100.0):.1f} mm / 100.0 mm</code>
        • Uppmätt spännvidd C1–C3: <code>{b.get('left_to_right_mm', 0.0):.1f} mm</code> (C1–C2: {b.get('left_to_center_mm', 0.0):.1f} mm, C2–C3: {b.get('center_to_right_mm', 0.0):.1f} mm)
      </div>
    </div>

    <h3>5-Punkters Förkontroll (GO / NO-GO)</h3>
    <table>
      <thead><tr><th>Kontrollpunkt</th><th>Uppmätt värde</th><th>Målvärde</th><th>Status</th></tr></thead>
      <tbody>{check_rows}</tbody>
    </table>

    <h3>Uppmätt OPENCV_FISHEYE &amp; 3D-Rigggeometri (1:1 millimeter)</h3>
    <table>
      <thead><tr><th>Kamera</th><th>Upplösning</th><th>fx / fy</th><th>cx, cy</th><th>k1..k4</th><th>Pos relativt Cam2 (X,Y,Z)</th><th>Yaw / Lutning / Roll</th></tr></thead>
      <tbody>{cam_rows}</tbody>
    </table>

    <h3>Visuell Återprojektion (Grön ring = uppmätt ChArUco • Röd + = låst rigg)</h3>
    <div class="grid3">
      <img src="reproj_cam1_left.jpg" alt="reproj cam1" />
      <img src="reproj_cam2_center.jpg" alt="reproj cam2" />
      <img src="reproj_cam3_right.jpg" alt="reproj cam3" />
    </div>

    <h3>Fisheye Lins- &amp; Hörntäckning (184° Wide FOV)</h3>
    <div class="grid3">
      <img src="coverage_cam1_left.jpg" alt="coverage cam1" />
      <img src="coverage_cam2_center.jpg" alt="coverage cam2" />
      <img src="coverage_cam3_right.jpg" alt="coverage cam3" />
    </div>
  </div>
</body>
</html>
"""
    out_html.write_text(html_str, encoding="utf-8")


# =============================================================================
# 7. HUVUDFUNKTIONER FÖR KALIBRERING (RIKTIGA VIDEOR & SYNTETISKT TEST)
# =============================================================================

def _solve_and_save_profile(
    frames: List[Dict[str, Any]],
    image_sizes: Dict[str, Tuple[int, int]],
    detector: MultiBoardDetector,
    raw_frames_cache: Dict[str, Dict[str, np.ndarray]],
    output_profile_path: Path,
    calibrated_rig_compat_path: Optional[Path],
    preview_dir: Path,
    source_tag: str,
    ruler_100mm: float = 100.0,
    nominal_baseline_mm: float = 300.0,
    progress_cb: Optional[Callable[[str], None]] = None,
) -> Dict[str, Any]:
    if len(frames) < 4:
        raise RuntimeError(
            f"Hittade bara {len(frames)} användbara ChArUco bild-trior (behöver minst 4, helst 15–25). "
            "Kontrollera att alla 3 tavlor syns tydligt och att RockSteady är AV."
        )

    # Steg A: Kalibrera varje kameras egna fisheye-intrinsics separat
    intrinsics: Dict[str, np.ndarray] = {}
    view_poses_by_cam: Dict[str, Dict[Tuple[int, str], Tuple[np.ndarray, np.ndarray]]] = {}

    for ck in CAM_ORDER:
        w, h = image_sizes[ck]
        views: List[Tuple[str, np.ndarray, np.ndarray]] = []
        v_to_tidx: List[Tuple[int, str]] = []
        for t_idx, fr in enumerate(frames):
            for bn, (ids_arr, img_pts) in fr["observations"].get(ck, {}).items():
                views.append((bn, ids_arr, img_pts))
                v_to_tidx.append((t_idx, bn))
        if len(views) < 3:
            raise RuntimeError(f"För få ChArUco-bilder ({len(views)}) för {ck}. Behöver minst 3 vinklar per kamera.")
        if progress_cb:
            progress_cb(f"[ChArUco] Löser OPENCV_FISHEYE-lins för {ck} ({len(views)} tavelvyer)...")
        intr, vposes = _calibrate_single_camera_intrinsics(ck, w, h, views, detector)
        intrinsics[ck] = intr
        mapped_poses: Dict[Tuple[int, str], Tuple[np.ndarray, np.ndarray]] = {}
        for (v_idx, bn), pose in vposes.items():
            mapped_poses[v_to_tidx[v_idx]] = pose
        view_poses_by_cam[ck] = mapped_poses

    # Steg B: Kedja ihop tavlor & kamerorigg
    ref_board, T_cam_from_rig, T_refB_from_B, T_rig_frames, cam_overlap_counts = _initialize_rig_and_boards(
        frames, view_poses_by_cam, baseline_mm=nominal_baseline_mm
    )

    # Steg C1: 80% Train / 20% Hold-Out validering
    multi_indices = [ti for ti, fr in enumerate(frames) if fr["cams_with_hits"] >= 2 and ti in T_rig_frames]
    holdout_indices = [ti for k, ti in enumerate(multi_indices) if k % 5 == 2]
    if not holdout_indices and len(frames) >= 6:
        holdout_indices = [len(frames) // 2]
    train_indices = [ti for ti in range(len(frames)) if ti not in set(holdout_indices) and ti in T_rig_frames]

    if progress_cb:
        progress_cb(
            f"[ChArUco] Kör korsvalidering (träning {len(train_indices)} trior, hold-out {len(holdout_indices)} trior)..."
        )
    tr_intr, tr_cam_rel, tr_boards, tr_frames, _tr_rms = _run_joint_rig_bundle_adjustment(
        train_indices,
        frames,
        detector,
        intrinsics,
        ref_board,
        T_cam_from_rig,
        T_refB_from_B,
        T_rig_frames,
        max_nfev=650,
    )
    holdout_rms_px = _evaluate_holdout_frames(
        holdout_indices, frames, detector, tr_intr, tr_cam_rel, tr_boards, T_rig_frames
    )

    # Steg C2: Slutlig full gles Bundle Adjustment över 100% av datan
    all_indices = [ti for ti in range(len(frames)) if ti in T_rig_frames]
    if progress_cb:
        progress_cb(f"[ChArUco] Kör slutlig global Sparse Bundle Adjustment över alla {len(all_indices)} bild-trior...")
    warm_frames = {**T_rig_frames, **tr_frames}
    final_intr, final_cam_rel, final_boards, final_frames, rms_px = _run_joint_rig_bundle_adjustment(
        all_indices,
        frames,
        detector,
        tr_intr,
        ref_board,
        tr_cam_rel,
        tr_boards,
        warm_frames,
        max_nfev=800,
    )

    # Rigg-stabilitet (per-frame avvikelse)
    stability = _compute_rig_stability_spread(frames, detector, final_intr, final_cam_rel, final_boards, final_frames)

    # Baslinjer & fysiska vinklar
    pos_by_cam = {}
    for ck in CAM_ORDER:
        T_cr = final_cam_rel[ck]
        pos_by_cam[ck] = -T_cr[:3, :3].T @ T_cr[:3, 3]

    baselines_mm = {
        "left_to_center_mm": round(float(np.linalg.norm(pos_by_cam["cam1_left"] - pos_by_cam["cam2_center"]) * 1000.0), 2),
        "center_to_right_mm": round(float(np.linalg.norm(pos_by_cam["cam3_right"] - pos_by_cam["cam2_center"]) * 1000.0), 2),
        "left_to_right_mm": round(float(np.linalg.norm(pos_by_cam["cam3_right"] - pos_by_cam["cam1_left"]) * 1000.0), 2),
    }

    per_cam_data: Dict[str, Dict[str, Any]] = {}
    compat_dict: Dict[str, Any] = {}
    center_tilt_up = 8.0

    for ck in CAM_ORDER:
        w, h = image_sizes[ck]
        T_cr = final_cam_rel[ck]
        R_rel = T_cr[:3, :3]
        t_rel = T_cr[:3, 3]
        pos_m = pos_by_cam[ck]
        yaw_rel, pitch_rel, roll_rel = rotmat_to_euler_deg(R_rel)
        if ck == "cam2_center":
            phys_yaw, phys_tilt, phys_roll = 0.0, +center_tilt_up, 0.0
        else:
            phys_yaw, phys_tilt, phys_roll = physical_angles_from_rel(R_rel, center_tilt_up_deg=center_tilt_up)

        q_wxyz = rotmat_to_quaternion_wxyz(R_rel)
        per_cam_data[ck] = {
            "width": int(w),
            "height": int(h),
            "camera_model": "OPENCV_FISHEYE",
            "intrinsics": [float(x) for x in final_intr[ck]],
            "cam_from_rig_qvec_wxyz": q_wxyz,
            "cam_from_rig_tvec_m": [float(t_rel[0]), float(t_rel[1]), float(t_rel[2])],
            "pos_in_ref_m": [float(pos_m[0]), float(pos_m[1]), float(pos_m[2])],
            "pos_mm": [round(float(x * 1000.0), 2) for x in pos_m],
            "rel_euler_deg": [round(yaw_rel, 3), round(pitch_rel, 3), round(roll_rel, 3)],
            "phys_yaw": round(phys_yaw, 2),
            "phys_tilt": round(phys_tilt, 2),
            "phys_roll": round(phys_roll, 2),
            "spread_deg": stability[ck]["spread_deg"],
            "spread_mm": stability[ck]["spread_mm"],
        }
        compat_dict[ck] = {
            "tx": float(t_rel[0]),
            "ty": float(t_rel[1]),
            "tz": float(t_rel[2]),
            "pos_x_m": float(pos_m[0]),
            "pos_y_m": float(pos_m[1]),
            "pos_z_m": float(pos_m[2]),
            "yaw": round(yaw_rel, 3),
            "pitch": round(pitch_rel, 3),
            "roll": round(roll_rel, 3),
            "phys_yaw": round(phys_yaw, 2),
            "phys_tilt": round(phys_tilt, 2),
            "phys_roll": round(phys_roll, 2),
            "spread_deg": stability[ck]["spread_deg"],
            "qvec_wxyz": q_wxyz,
            "intrinsics": [float(x) for x in final_intr[ck]],
            "width": int(w),
            "height": int(h),
        }

    # Visualiseringar
    cov_stats, preview_paths = _render_calibration_visuals(
        frames,
        detector,
        image_sizes,
        final_intr,
        final_cam_rel,
        final_boards,
        final_frames,
        raw_frames_cache,
        preview_dir,
    )
    for ck in CAM_ORDER:
        per_cam_data[ck].update(cov_stats[ck])

    checks, overall_status = _evaluate_5point_checks(
        rms_px, holdout_rms_px, stability, cov_stats, per_cam_data, baselines_mm, cam_overlap_counts
    )

    profile: Dict[str, Any] = {
        "schema_version": 2,
        "source": source_tag,
        "created_at": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "image_size": list(image_sizes["cam2_center"]),
        "ruler_100mm": float(ruler_100mm),
        "n_frames_used": len(all_indices),
        "n_holdout_frames": len(holdout_indices),
        "ref_board": ref_board,
        "rms_px": round(rms_px, 4),
        "holdout_rms_px": round(holdout_rms_px, 4),
        "cross_cam_overlap": cam_overlap_counts,
        "baselines_mm": baselines_mm,
        "overall_status": overall_status,
        "checks": checks,
        "per_camera": per_cam_data,
        "preview_paths": preview_paths,
    }

    output_profile_path.parent.mkdir(parents=True, exist_ok=True)
    output_profile_path.write_text(json.dumps(profile, indent=2, ensure_ascii=False), encoding="utf-8")

    compat_dict["_meta"] = {
        "source": source_tag,
        "created_at": profile["created_at"],
        "rms_px": profile["rms_px"],
        "holdout_rms_px": profile["holdout_rms_px"],
        "overall_status": overall_status,
        "baselines_mm": baselines_mm,
    }
    if calibrated_rig_compat_path is not None:
        calibrated_rig_compat_path.parent.mkdir(parents=True, exist_ok=True)
        calibrated_rig_compat_path.write_text(json.dumps(compat_dict, indent=2, ensure_ascii=False), encoding="utf-8")

    report_path = preview_dir / "calibration_verification_report.html"
    _write_verification_report_html(profile, report_path)
    profile["report_html_path"] = str(report_path)

    return profile


def calibrate_rig_from_videos(
    videos: Dict[str, Path],
    output_profile_path: Path = DJI_RIG_PROFILE_PATH,
    calibrated_rig_compat_path: Optional[Path] = CALIBRATED_RIG_PATH,
    preview_dir: Path = PREVIEW_DIR,
    ruler_100mm: float = 100.0,
    square_mm: float = DEFAULT_SQUARE_MM,
    marker_mm: float = DEFAULT_MARKER_MM,
    fps: float = 50.0,
    use_audio_sync: bool = True,
    nominal_baseline_mm: float = 300.0,
    progress_cb: Optional[Callable[[str], None]] = print,
) -> Dict[str, Any]:
    """Fullständig ChArUco-kalibrering från 3 synkade kalibreringsvideor."""
    for ck in CAM_ORDER:
        if ck not in videos or not Path(videos[ck]).is_file():
            raise FileNotFoundError(f"Saknar kalibreringsvideo för {ck}: {videos.get(ck)}")

    scale_corr = max(0.5, min(2.0, float(ruler_100mm) / 100.0))
    eff_sq = float(square_mm) * scale_corr
    eff_mk = float(marker_mm) * scale_corr

    if use_audio_sync:
        if str(SRC_DIR) not in sys.path:
            sys.path.insert(0, str(SRC_DIR))
        import process_3dgs_rig as pr

        temp_sync_dir = output_profile_path.parent / "_temp_charuco_sync"
        temp_sync_dir.mkdir(parents=True, exist_ok=True)
        try:
            offsets = pr.compute_frame_offsets({k: Path(v) for k, v in videos.items()}, temp_sync_dir, fps=fps)
        finally:
            shutil.rmtree(temp_sync_dir, ignore_errors=True)
    else:
        offsets = {ck: 0 for ck in CAM_ORDER}

    detector = MultiBoardDetector(square_mm=eff_sq, marker_mm=eff_mk)
    frames, image_sizes, raw_frames_cache = _extract_candidate_triplets_from_videos(
        {k: Path(v) for k, v in videos.items()},
        offsets=offsets,
        detector=detector,
        fps=fps,
        progress_cb=progress_cb,
    )

    return _solve_and_save_profile(
        frames=frames,
        image_sizes=image_sizes,
        detector=detector,
        raw_frames_cache=raw_frames_cache,
        output_profile_path=output_profile_path,
        calibrated_rig_compat_path=calibrated_rig_compat_path,
        preview_dir=preview_dir,
        source_tag="charuco",
        ruler_100mm=ruler_100mm,
        nominal_baseline_mm=nominal_baseline_mm,
        progress_cb=progress_cb,
    )


def build_synthetic_rig_test_data(
    width: int = 2880,
    height: int = 3840,
    n_poses: int = 20,
    noise_sigma_px: float = 0.14,
    seed: int = 42,
) -> Tuple[
    List[Dict[str, Any]],
    Dict[str, Tuple[int, int]],
    MultiBoardDetector,
    Dict[str, np.ndarray],
    Dict[str, np.ndarray],
    Dict[str, np.ndarray],
    Dict[int, np.ndarray],
]:
    """Genererar realistisk snett ihopskruvad 2880x3840 3-kamera ChArUco-rigg för självtest & demo."""
    rng = np.random.default_rng(seed)
    detector = MultiBoardDetector(square_mm=DEFAULT_SQUARE_MM, marker_mm=DEFAULT_MARKER_MM)

    # Lätt asymmetriska linser (typisk fabriks-variation mellan 3 Action 4-kameror)
    true_intr = {
        "cam1_left": np.array([1582.4, 1584.1, 1444.2, 1916.8, -0.0415, 0.0092, -0.0024, 0.0005], dtype=np.float64),
        "cam2_center": np.array([1591.2, 1590.5, 1437.5, 1923.4, -0.0431, 0.0085, -0.0019, 0.0004], dtype=np.float64),
        "cam3_right": np.array([1576.8, 1578.0, 1441.9, 1918.1, -0.0408, 0.0078, -0.0021, 0.0006], dtype=np.float64),
    }

    # Snett ihopskruvad rigg (~3° fel & ±6 mm jämfört med CAD-idealet)
    def _custom_rel(px: float, py: float, pz: float, phys_yaw: float, phys_tilt: float, phys_roll: float) -> np.ndarray:
        R_c = euler_to_rotmat(0.0, -8.0, 0.0)
        R_s = euler_to_rotmat(-phys_yaw, -phys_tilt, phys_roll)
        R_rel = R_s @ R_c.T
        t_rel = -R_rel @ np.array([px, py, pz], dtype=np.float64)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R_rel
        T[:3, 3] = t_rel
        return T

    true_cam_rel = {
        "cam2_center": np.eye(4, dtype=np.float64),
        "cam1_left": _custom_rel(-0.1535, 0.0042, 0.0061, -43.2, -13.8, 1.4),
        "cam3_right": _custom_rel(+0.1482, -0.0031, 0.0048, +46.7, -10.9, -1.1),
    }

    # 3 tavlor monterade i ett ~90-graders rumshörn
    def _board_T(tx: float, ty: float, tz: float, yaw_d: float, pitch_d: float, roll_d: float) -> np.ndarray:
        R = euler_to_rotmat(yaw_d, pitch_d, roll_d)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = np.array([tx, ty, tz], dtype=np.float64)
        return T

    true_boards = {
        "board_1_center": np.eye(4, dtype=np.float64),
        "board_0_left": _board_T(-0.42, 0.08, 0.24, -40.0, 4.0, -1.5),
        "board_2_right": _board_T(+0.45, 0.06, 0.22, +38.0, 3.0, +1.8),
    }

    frames: List[Dict[str, Any]] = []
    true_rig_frames: Dict[int, np.ndarray] = {}
    image_sizes = {ck: (width, height) for ck in CAM_ORDER}

    for i in range(n_poses):
        # Svep riggen mellan -45°..+45° yaw, pitch -26°..+18°, roll -12°..+12°, avstånd 0.72..1.24 m så hörn når ytterkanterna
        phase = (i / max(1, n_poses - 1)) * 2.0 - 1.0
        yaw_rig = 42.0 * math.sin(phase * math.pi * 0.9)
        pitch_rig = 22.0 * math.cos(i * 1.3) - 4.0
        roll_rig = 9.0 * math.sin(i * 1.7)
        dist = 0.72 + 0.52 * abs(math.cos(i * 0.8))
        # Flytta tavlor framåt framför cam2 (z ~ dist) i riggkoordinater: T_rig_from_refB
        T_r_b = _board_T(0.24 * phase, 0.34 * math.sin(i * 1.3), dist, yaw_rig, pitch_rig, roll_rig)
        true_rig_frames[i] = T_r_b

        obs_dict: Dict[str, Dict[str, Tuple[np.ndarray, np.ndarray]]] = {}
        cams_hit = 0
        tot_c = 0
        for ck in CAM_ORDER:
            cam_obs: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
            for bn, obj_all in detector.obj_corners.items():
                T_cb = true_cam_rel[ck] @ T_r_b @ true_boards[bn]
                rv, tv = _rvec_tvec_from_se3(T_cb)
                pts2, z = project_fisheye_world_points(obj_all, rv, tv, true_intr[ck])
                vis = (z > 0.25) & (pts2[:, 0] > 45.0) & (pts2[:, 0] < width - 45.0) & (pts2[:, 1] > 45.0) & (pts2[:, 1] < height - 45.0)
                if int(np.sum(vis)) >= 12:
                    ids_v = np.where(vis)[0].astype(np.int32)
                    pts_noisy = pts2[ids_v] + rng.normal(0.0, noise_sigma_px, size=(len(ids_v), 2))
                    cam_obs[bn] = (ids_v, pts_noisy)
                    tot_c += len(ids_v)
            if cam_obs:
                obs_dict[ck] = cam_obs
                cams_hit += 1

        frames.append(
            {
                "frame_name": f"synth_{i:03d}",
                "frame_idx": i,
                "cams_with_hits": cams_hit,
                "total_corners": tot_c,
                "observations": obs_dict,
            }
        )

    return frames, image_sizes, detector, true_intr, true_cam_rel, true_boards, true_rig_frames


def run_synthetic_calibration_demo(
    output_profile_path: Path = DJI_DEMO_PROFILE_PATH,
    preview_dir: Path = PREVIEW_DIR,
    save_compat: bool = False,
    progress_cb: Optional[Callable[[str], None]] = print,
) -> Dict[str, Any]:
    """Kör komplett syntetisk kalibrerings- & verifieringspipeline och skriver förhandsvisning."""
    if progress_cb:
        progress_cb("[ChArUco Demo] Simulerar en snett ihopskruvad 2880×3840 Action 4-rigg över 18 vinklar...")
    frames, image_sizes, detector, true_intr, true_cam_rel, _tb, _tf = build_synthetic_rig_test_data()
    compat = CALIBRATED_RIG_PATH if save_compat else None
    profile = _solve_and_save_profile(
        frames=frames,
        image_sizes=image_sizes,
        detector=detector,
        raw_frames_cache={},
        output_profile_path=output_profile_path,
        calibrated_rig_compat_path=compat,
        preview_dir=preview_dir,
        source_tag="synthetic_demo",
        ruler_100mm=100.0,
        progress_cb=progress_cb,
    )
    profile["ground_truth_baselines_mm"] = {
        "left_to_right_mm": round(
            float(
                np.linalg.norm(
                    (-true_cam_rel["cam3_right"][:3, :3].T @ true_cam_rel["cam3_right"][:3, 3])
                    - (-true_cam_rel["cam1_left"][:3, :3].T @ true_cam_rel["cam1_left"][:3, 3])
                )
                * 1000.0
            ),
            2,
        )
    }
    return profile


# =============================================================================
# 8. PRE-FLIGHT SCENKONTROLL INNAN FÄLTFILMNING (EPIPOLÄR + CHARUCO)
# =============================================================================

def _compute_essential_matrix(T_s_from_0: np.ndarray) -> np.ndarray:
    """E = [t]_x R så att ray_s^T E ray_0 = 0 för en sann gemensam 3D-punkt."""
    R = T_s_from_0[:3, :3]
    t = T_s_from_0[:3, 3]
    tx = np.array([[0.0, -t[2], t[1]], [t[2], 0.0, -t[0]], [-t[1], t[0], 0.0]], dtype=np.float64)
    return tx @ R


def _epipolar_pixel_errors(
    pts_center: np.ndarray,
    pts_side: np.ndarray,
    intr_center: np.ndarray,
    intr_side: np.ndarray,
    E_s_0: np.ndarray,
) -> np.ndarray:
    """
    Beräknar epipolärt vinkelfel omräknat till pixlar (vinkelavstånd till epipolära planet × brännvidd)
    under den LÅSTA kalibrerade riggens Essential-matris E = [t_rel]_x R_rel.
    """
    r0 = undistort_fisheye_to_rays(pts_center, intr_center)
    r1 = undistort_fisheye_to_rays(pts_side, intr_side)
    # Normalvektor till epipolarplanet i side-kamera: n_s = E @ r0_i (normerad)
    lines_s = (E_s_0 @ r0.T).T
    n_norm = np.linalg.norm(lines_s, axis=1)
    safe = n_norm > 1e-10
    ang_err = np.full(len(r0), np.pi, dtype=np.float64)
    sin_err = np.abs(np.sum(r1[safe] * lines_s[safe], axis=1)) / n_norm[safe]
    ang_err[safe] = np.arcsin(np.clip(sin_err, 0.0, 1.0))
    f_avg = 0.5 * (float(intr_side[0]) + float(intr_side[1]))
    return ang_err * f_avg


def scale_intrinsics_for_resolution(intr: List[float] | np.ndarray, cal_w: int, cal_h: int, cur_w: int, cur_h: int) -> Tuple[np.ndarray, str]:
    """Skalar (fx, fy, cx, cy) proportionellt om bildstorleken skiljer sig och varnar vid annat bildförhållande."""
    arr = np.asarray(intr, dtype=np.float64).copy()
    if cal_w == cur_w and cal_h == cur_h:
        return arr, "EXACT"
    sx = float(cur_w) / max(1, cal_w)
    sy = float(cur_h) / max(1, cal_h)
    arr[0] *= sx
    arr[1] *= sy
    arr[2] *= sx
    arr[3] *= sy
    ratio_diff = abs((cur_w / max(1.0, float(cur_h))) - (cal_w / max(1.0, float(cal_h))))
    kind = "ASPECT_MISMATCH" if ratio_diff > 0.03 else "SCALED"
    return arr, kind


def verify_calibration_on_scene(
    rig_profile_path: Path = DJI_RIG_PROFILE_PATH,
    project_dir: Optional[Path] = None,
    raw_dir: Optional[Path] = None,
    preview_dir: Path = PREVIEW_DIR,
    triplet_imgs_list: Optional[List[Dict[str, np.ndarray]]] = None,
    progress_cb: Optional[Callable[[str], None]] = print,
) -> Dict[str, Any]:
    """
    Kontrollerar på ett testklipp/scen om den sparade kalibreringen fortfarande stämmer till 100 %:
      1. Matchar klippets upplösning & bildförhållande kalibreringen?
      2. Om ChArUco-tavlor syns i klippet: hur stort blir det låsta 6-DoF-felet i pixlar?
      3. Utan tavlor (valfri inne-/utemiljö): matchar SIFT mellan kameror och mäter epipolärt
         pixel-fel under den FROSTADE kalibrerade Essential-matrisen (fångar direkt om ett fäste rubbats >=1°!).
    """
    if not rig_profile_path.exists():
        raise FileNotFoundError(f"Saknar kalibreringsprofil: {rig_profile_path}. Kalibrera riggen i Steg 0 först!")

    profile = json.loads(rig_profile_path.read_text(encoding="utf-8"))
    cal_w, cal_h = int(profile["image_size"][0]), int(profile["image_size"][1])

    # Hämta 3-5 synkade test-trior
    triplets: List[Tuple[str, Dict[str, np.ndarray]]] = []
    if triplet_imgs_list:
        for idx, d in enumerate(triplet_imgs_list, 1):
            triplets.append((f"sample_{idx:02d}", d))
    else:
        img_root = (project_dir / "images") if project_dir else None
        if img_root and (img_root / "cam2_center").exists() and any((img_root / "cam2_center").glob("*.jpg")):
            c2_files = sorted((img_root / "cam2_center").glob("*.jpg"))
            picks = [c2_files[int(k * (len(c2_files) - 1) / max(1, min(4, len(c2_files) - 1)))] for k in range(min(5, len(c2_files)))]
            for fp in picks:
                fn = fp.name
                d = {}
                for ck in CAM_ORDER:
                    p = img_root / ck / fn
                    if p.exists():
                        im = cv2.imread(str(p))
                        if im is not None:
                            d[ck] = im
                if len(d) == 3:
                    triplets.append((fn, d))

        if not triplets and raw_dir:
            vids = {ck: Path(raw_dir) / f"{ck}.mp4" for ck in CAM_ORDER}
            if all(v.is_file() for v in vids.values()):
                if str(SRC_DIR) not in sys.path:
                    sys.path.insert(0, str(SRC_DIR))
                import process_3dgs_rig as pr

                sync_dir = preview_dir / "_tmp_verify_sync"
                sync_dir.mkdir(parents=True, exist_ok=True)
                try:
                    offsets = pr.compute_frame_offsets(vids, sync_dir, fps=50.0)
                finally:
                    shutil.rmtree(sync_dir, ignore_errors=True)

                caps = {ck: cv2.VideoCapture(str(vids[ck])) for ck in CAM_ORDER}
                try:
                    n_min = min(int(caps[ck].get(cv2.CAP_PROP_FRAME_COUNT)) - offsets.get(ck, 0) for ck in CAM_ORDER)
                    for k, frac in enumerate((0.18, 0.36, 0.54, 0.72), 1):
                        bf = max(125, int(n_min * frac))
                        d = {}
                        for ck in CAM_ORDER:
                            caps[ck].set(cv2.CAP_PROP_POS_FRAMES, max(0, bf + offsets.get(ck, 0)))
                            ret, frame = caps[ck].read()
                            if ret and frame is not None:
                                d[ck] = frame
                        if len(d) == 3:
                            triplets.append((f"raw_{k:02d}_f{bf}", d))
                finally:
                    for c in caps.values():
                        c.release()

    if not triplets:
        raise RuntimeError("Hittade inga bild-trior eller råvideor (cam1_left/cam2_center/cam3_right.mp4) att kontrollera.")

    sample_h, sample_w = triplets[0][1]["cam2_center"].shape[:2]
    scaled_intr: Dict[str, np.ndarray] = {}
    res_status = "OK"
    res_msg = f"Exakt match ({sample_w}×{sample_h})"
    for ck in CAM_ORDER:
        intr_c, kind = scale_intrinsics_for_resolution(profile["per_camera"][ck]["intrinsics"], cal_w, cal_h, sample_w, sample_h)
        scaled_intr[ck] = intr_c
        if kind == "ASPECT_MISMATCH":
            res_status = "FAIL"
            res_msg = f"FEL BILDFÖRHÅLLANDE! Kalibrerad för {cal_w}×{cal_h}, klippet är {sample_w}×{sample_h}!"
        elif kind == "SCALED" and res_status != "FAIL":
            res_status = "WARN"
            res_msg = f"Skalad från {cal_w}×{cal_h} -> {sample_w}×{sample_h}"

    T_cam_rel = {"cam2_center": np.eye(4, dtype=np.float64)}
    for ck in ("cam1_left", "cam3_right"):
        cd = profile["per_camera"][ck]
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = quaternion_wxyz_to_rotmat(cd["cam_from_rig_qvec_wxyz"])
        T[:3, 3] = np.asarray(cd["cam_from_rig_tvec_m"], dtype=np.float64)
        T_cam_rel[ck] = T

    if progress_cb:
        progress_cb(f"[Förkontroll] Kör epipolär 3D-kontroll över {len(triplets)} synkade bild-trior ({sample_w}×{sample_h})...")

    # Matcha SIFT-punkter (oberoende av rigg först så att även rubbat fäste avslöjas!)
    sift = cv2.SIFT_create(nfeatures=2800)
    flann = cv2.FlannBasedMatcher(dict(algorithm=1, trees=5), dict(checks=48))
    scale_sift = min(0.45, 1300.0 / max(1, sample_w))

    pair_errors: Dict[str, List[float]] = {"cam1_left": [], "cam3_right": []}
    best_vis: Dict[str, Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, str]] = {}

    for t_name, im_dict in triplets:
        grays = {}
        kps = {}
        dess = {}
        for ck in CAM_ORDER:
            g = cv2.cvtColor(im_dict[ck], cv2.COLOR_BGR2GRAY) if im_dict[ck].ndim == 3 else im_dict[ck]
            g_small = cv2.resize(g, (0, 0), fx=scale_sift, fy=scale_sift, interpolation=cv2.INTER_AREA)
            grays[ck] = g_small
            kp, des = sift.detectAndCompute(g_small, None)
            kps[ck] = kp
            dess[ck] = des

        if dess["cam2_center"] is None or len(kps["cam2_center"]) < 20:
            continue

        for side_cam in ("cam1_left", "cam3_right"):
            if dess[side_cam] is None or len(kps[side_cam]) < 20:
                continue
            matches = flann.knnMatch(dess["cam2_center"], dess[side_cam], k=2)
            good = [m for m, n in matches if m.distance < 0.72 * n.distance]
            if len(good) < 12:
                continue

            pts_c = np.float64([kps["cam2_center"][m.queryIdx].pt for m in good]) / scale_sift
            pts_s = np.float64([kps[side_cam][m.trainIdx].pt for m in good]) / scale_sift

            # Filtrera bort falska SIFT-matchar med fri 5-punkts RANSAC (oberoende av sparad rigg!)
            r0 = undistort_fisheye_to_rays(pts_c, scaled_intr["cam2_center"])
            r1 = undistort_fisheye_to_rays(pts_s, scaled_intr[side_cam])
            n0 = (r0[:, :2] / r0[:, 2:3]).astype(np.float64)
            n1 = (r1[:, :2] / r1[:, 2:3]).astype(np.float64)
            _E_free, mask = cv2.findEssentialMat(n0, n1, np.eye(3), method=cv2.RANSAC, prob=0.999, threshold=0.008)
            if mask is None:
                continue
            inl = mask.ravel().astype(bool)
            if int(np.sum(inl)) < 8:
                continue

            pts_c_in = pts_c[inl]
            pts_s_in = pts_s[inl]
            E_locked = _compute_essential_matrix(T_cam_rel[side_cam])
            errs = _epipolar_pixel_errors(pts_c_in, pts_s_in, scaled_intr["cam2_center"], scaled_intr[side_cam], E_locked)
            pair_errors[side_cam].extend(errs.tolist())

            if side_cam not in best_vis or len(pts_c_in) > len(best_vis[side_cam][2]):
                best_vis[side_cam] = (im_dict["cam2_center"], im_dict[side_cam], pts_c_in, pts_s_in, errs, t_name)

    # Rita ut epipolära testbilder i calib_preview/
    preview_dir.mkdir(parents=True, exist_ok=True)
    pair_medians: Dict[str, float] = {}
    pair_counts: Dict[str, int] = {}
    overall_epipolar = "OK"

    if best_vis:
        ref_src = next(iter(best_vis.values()))[0]
        scale_vis = min(1.0, 1100.0 / max(1, sample_w))
        vw, vh = int(round(sample_w * scale_vis)), int(round(sample_h * scale_vis))
        c_vis = cv2.resize(ref_src, (vw, vh), interpolation=cv2.INTER_AREA)
        _draw_hud_banner(
            c_vis,
            f"Cam 2 Mitten (Referens) - {res_msg}",
            f"Kontrollerad över {len(triplets)} synkade bild-trior",
            (63, 185, 80) if res_status == "OK" else (34, 153, 210),
        )
        cv2.imwrite(str(preview_dir / "epipolar_cam2_center.jpg"), c_vis, [cv2.IMWRITE_JPEG_QUALITY, 90])

    for side_cam in ("cam1_left", "cam3_right"):
        errs_all = pair_errors[side_cam]
        pair_counts[side_cam] = len(errs_all)
        med_err = float(np.median(errs_all)) if errs_all else 999.0
        pair_medians[side_cam] = round(med_err, 2)
        if not errs_all:
            overall_epipolar = "WARN" if overall_epipolar == "OK" else overall_epipolar
        elif med_err >= 3.5:
            overall_epipolar = "FAIL"
        elif med_err >= 1.8 and overall_epipolar == "OK":
            overall_epipolar = "WARN"

        if side_cam in best_vis:
            im_c, im_s, pts_c_in, pts_s_in, errs, t_name = best_vis[side_cam]
            scale_vis = min(1.0, 1100.0 / max(1, sample_w))
            vw, vh = int(round(sample_w * scale_vis)), int(round(sample_h * scale_vis))
            vis_s = cv2.resize(im_s, (vw, vh), interpolation=cv2.INTER_AREA)

            # Rita böjda fisheye-epipolära linjer för upp till 14 tydliga punkter
            T_rel = T_cam_rel[side_cam]
            R_rel = T_rel[:3, :3]
            t_rel = T_rel[:3, 3]
            step_k = max(1, len(pts_c_in) // 14)
            depths = np.array([0.4, 0.65, 1.0, 1.6, 2.5, 4.5, 9.0, 22.0, 60.0], dtype=np.float64)
            for idx_p in range(0, len(pts_c_in), step_k):
                pt_c = pts_c_in[idx_p : idx_p + 1]
                pt_s = pts_s_in[idx_p]
                ep_err = float(errs[idx_p])
                col = (50, 230, 80) if ep_err < 1.8 else ((34, 170, 230) if ep_err < 3.5 else (60, 70, 250))

                ray_c = undistort_fisheye_to_rays(pt_c, scaled_intr["cam2_center"])[0]
                pts_3d_s = ((R_rel @ ray_c)[None, :] * depths[:, None]) + t_rel[None, :]
                ep_curve, z_c = project_fisheye_world_points(
                    pts_3d_s, np.zeros(3), np.zeros(3), scaled_intr[side_cam]
                )
                valid_curve = ep_curve[z_c > 0.05]
                if len(valid_curve) >= 2:
                    poly = np.round(valid_curve * scale_vis).astype(np.int32).reshape(-1, 1, 2)
                    cv2.polylines(vis_s, [poly], False, (255, 190, 50), 2, cv2.LINE_AA)

                sx_i, sy_i = int(round(pt_s[0] * scale_vis)), int(round(pt_s[1] * scale_vis))
                cv2.circle(vis_s, (sx_i, sy_i), 7, col, -1, cv2.LINE_AA)
                cv2.circle(vis_s, (sx_i, sy_i), 8, (15, 15, 15), 2, cv2.LINE_AA)

            col_b = (63, 185, 80) if med_err < 1.8 else ((34, 153, 210) if med_err < 3.5 else (73, 81, 248))
            verdict_s = "RIGG STÄMMER" if med_err < 1.8 else ("LITEN AVVIKELSE" if med_err < 3.5 else "FÄSTET HAR RÖRT SIG!")
            _draw_hud_banner(
                vis_s,
                f"{CAM_LABELS[side_cam]} - {verdict_s} ({t_name})",
                f"Median epipolärt fel: {med_err:.2f} px ({len(errs_all)} SIFT-punkter)",
                col_b,
            )
            cv2.imwrite(str(preview_dir / f"epipolar_{side_cam}.jpg"), vis_s, [cv2.IMWRITE_JPEG_QUALITY, 90])

    # Kolla även om ChArUco-tavlor råkar finnas i testklippet!
    charuco_locked_rms = None
    try:
        r100 = float(profile.get("ruler_100mm", 100.0))
        det = MultiBoardDetector(square_mm=DEFAULT_SQUARE_MM * r100 / 100.0, marker_mm=DEFAULT_MARKER_MM * r100 / 100.0)
        ch_errs: List[np.ndarray] = []
        for _tn, im_dict in triplets[:3]:
            hits_per_cam = {ck: det.detect_image(im_dict[ck], min_corners=6) for ck in CAM_ORDER}
            if sum(sum(len(v[0]) for v in bh.values()) for bh in hits_per_cam.values()) < 10:
                continue
            # Enkel pose-kontroll per tavla som ses av båda sidor skulle kräva T_boards; mät åtminstone per kamera med låsta intrinsics
            for ck, bh in hits_per_cam.items():
                for bn, (ids_arr, img_pts) in bh.items():
                    obj_pts = det.obj_corners[bn][ids_arr]
                    rays = undistort_fisheye_to_rays(img_pts, scaled_intr[ck])
                    norm_2d = (rays[:, :2] / rays[:, 2:3]).reshape(-1, 1, 2)
                    ok, rv, tv, _ = cv2.solvePnPRansac(obj_pts.reshape(-1, 1, 3), norm_2d, np.eye(3), None)
                    if ok:
                        pr, _ = project_fisheye_world_points(obj_pts, rv.ravel(), tv.ravel(), scaled_intr[ck])
                        ch_errs.append(np.sum((pr - img_pts) ** 2, axis=1))
        if ch_errs:
            charuco_locked_rms = round(float(math.sqrt(float(np.mean(np.concatenate(ch_errs))))), 3)
    except Exception:
        pass

    final_status = "FAIL" if ("FAIL" in (res_status, overall_epipolar)) else ("WARN" if "WARN" in (res_status, overall_epipolar) else "OK")
    if final_status == "OK":
        summary = f"GODKÄND (100%): Upplösning matchar och epipolärt fel är lågt (C1: {pair_medians['cam1_left']} px, C3: {pair_medians['cam3_right']} px)."
    elif final_status == "FAIL":
        summary = f"VARNING / FEL: Riggen eller inställningarna avviker kraftigt! ({res_msg} • C1: {pair_medians['cam1_left']} px, C3: {pair_medians['cam3_right']} px). Kalibrera om i Steg 0!"
    else:
        summary = f"KONTROLLERA: {res_msg} • Epipolärt fel C1: {pair_medians['cam1_left']} px, C3: {pair_medians['cam3_right']} px."

    verify_result = {
        "ok": True,
        "checked_at": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "status": final_status,
        "resolution_status": res_status,
        "resolution_message": res_msg,
        "scene_size": [sample_w, sample_h],
        "calibrated_size": [cal_w, cal_h],
        "n_triplets_checked": len(triplets),
        "epipolar_median_px": pair_medians,
        "epipolar_matches": pair_counts,
        "charuco_direct_rms_px": charuco_locked_rms,
        "summary": summary,
    }
    (preview_dir / "scene_verification.json").write_text(
        json.dumps(verify_result, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    if progress_cb:
        progress_cb(f"[Förkontroll] Klar ({final_status}): {summary}")
    return verify_result


# =============================================================================
# 9. RIG-LÅST POST-SOLVER FÖR COLMAP / SPIRULA `sparse/0` + LICHTFELD
# =============================================================================

def read_colmap_cameras_bin(path: Path) -> Dict[int, Dict[str, Any]]:
    data = path.read_bytes()
    off = 0
    num = struct.unpack_from("<Q", data, off)[0]
    off += 8
    param_counts = {0: 3, 1: 4, 2: 4, 3: 5, 4: 8, 5: 8, 6: 12, 7: 5}
    cams: Dict[int, Dict[str, Any]] = {}
    for _ in range(num):
        cid, mid, w, h = struct.unpack_from("<IiQQ", data, off)
        off += 24
        n_par = param_counts.get(mid, 8)
        pars = list(struct.unpack_from(f"<{n_par}d", data, off))
        off += 8 * n_par
        cams[cid] = {"camera_id": cid, "model_id": mid, "width": w, "height": h, "params": pars}
    return cams


def write_colmap_cameras_bin(path: Path, cams: Dict[int, Dict[str, Any]]) -> None:
    buf = bytearray(struct.pack("<Q", len(cams)))
    for cid in sorted(cams.keys()):
        c = cams[cid]
        pars = [float(x) for x in c["params"]]
        buf.extend(struct.pack("<IiQQ", int(cid), int(c["model_id"]), int(c["width"]), int(c["height"])))
        buf.extend(struct.pack(f"<{len(pars)}d", *pars))
    path.write_bytes(bytes(buf))


def read_colmap_images_bin_full(path: Path) -> Dict[str, Dict[str, Any]]:
    data = path.read_bytes()
    off = 0
    num = struct.unpack_from("<Q", data, off)[0]
    off += 8
    imgs: Dict[str, Dict[str, Any]] = {}
    for _ in range(num):
        img_id = struct.unpack_from("<I", data, off)[0]
        vals = struct.unpack_from("<7d", data, off + 4)
        cam_id = struct.unpack_from("<I", data, off + 60)[0]
        off += 64
        end = data.index(b"\x00", off)
        name = data[off:end].decode("utf-8", errors="replace").replace("\\", "/")
        off = end + 1
        n_pts = struct.unpack_from("<Q", data, off)[0]
        off += 8
        pts_arr = np.empty((n_pts, 2), dtype=np.float64)
        pids_arr = np.empty(n_pts, dtype=np.int64)
        for j in range(n_pts):
            x, y, pid = struct.unpack_from("<2dq", data, off)
            pts_arr[j, 0] = x
            pts_arr[j, 1] = y
            pids_arr[j] = pid
            off += 24
        imgs[name] = {
            "image_id": int(img_id),
            "qvec": list(vals[:4]),
            "tvec": np.array(vals[4:7], dtype=np.float64),
            "camera_id": int(cam_id),
            "xys": pts_arr,
            "point3D_ids": pids_arr,
        }
    return imgs


def write_colmap_images_bin_full(path: Path, imgs: Dict[str, Dict[str, Any]]) -> None:
    ordered = sorted(imgs.items(), key=lambda kv: kv[1]["image_id"])
    buf = bytearray(struct.pack("<Q", len(ordered)))
    for name, rec in ordered:
        q = [float(x) for x in rec["qvec"]]
        t = [float(x) for x in rec["tvec"]]
        buf.extend(struct.pack("<I7dI", int(rec["image_id"]), *q, *t, int(rec["camera_id"])))
        buf.extend(name.encode("utf-8") + b"\x00")
        xys = rec["xys"]
        pids = rec["point3D_ids"]
        n_pts = len(xys)
        buf.extend(struct.pack("<Q", n_pts))
        for j in range(n_pts):
            buf.extend(struct.pack("<2dq", float(xys[j, 0]), float(xys[j, 1]), int(pids[j])))
    path.write_bytes(bytes(buf))


def read_colmap_points3d_bin(path: Path) -> Dict[int, Dict[str, Any]]:
    data = path.read_bytes()
    off = 0
    num = struct.unpack_from("<Q", data, off)[0]
    off += 8
    pts: Dict[int, Dict[str, Any]] = {}
    for _ in range(num):
        pid = struct.unpack_from("<Q", data, off)[0]
        x, y, z, r, g, b, err = struct.unpack_from("<3d3Bd", data, off + 8)
        off += 43
        tlen = struct.unpack_from("<Q", data, off)[0]
        off += 8
        track = list(struct.unpack_from(f"<{2 * tlen}I", data, off)) if tlen > 0 else []
        off += 8 * tlen
        track_pairs = [(track[2 * k], track[2 * k + 1]) for k in range(tlen)]
        pts[int(pid)] = {
            "point3D_id": int(pid),
            "xyz": np.array([x, y, z], dtype=np.float64),
            "rgb": (int(r), int(g), int(b)),
            "error": float(err),
            "track": track_pairs,
        }
    return pts


def write_colmap_points3d_bin(path: Path, pts: Dict[int, Dict[str, Any]]) -> None:
    buf = bytearray(struct.pack("<Q", len(pts)))
    for pid in sorted(pts.keys()):
        p = pts[pid]
        xyz = p["xyz"]
        rgb = p["rgb"]
        track = p["track"]
        buf.extend(
            struct.pack(
                "<Q3d3BdQ",
                int(pid),
                float(xyz[0]),
                float(xyz[1]),
                float(xyz[2]),
                int(rgb[0]),
                int(rgb[1]),
                int(rgb[2]),
                float(p["error"]),
                len(track),
            )
        )
        for im_id, p2_idx in track:
            buf.extend(struct.pack("<2I", int(im_id), int(p2_idx)))
    path.write_bytes(bytes(buf))


def _load_rig_spec_for_enforcement(
    rig_profile_or_geom: Dict[str, Any],
    cam_sizes: Dict[str, Tuple[int, int]],
) -> Tuple[Dict[str, np.ndarray], Dict[str, np.ndarray]]:
    """Returnerar (intrinsics_per_cam, T_cam_from_rig_per_cam) oberoende av om input är full profil eller compat-dict."""
    intr: Dict[str, np.ndarray] = {}
    T_rel: Dict[str, np.ndarray] = {"cam2_center": np.eye(4, dtype=np.float64)}

    if "per_camera" in rig_profile_or_geom:
        cal_w, cal_h = int(rig_profile_or_geom["image_size"][0]), int(rig_profile_or_geom["image_size"][1])
        for ck in CAM_ORDER:
            cd = rig_profile_or_geom["per_camera"][ck]
            cw, ch = cam_sizes.get(ck, (cal_w, cal_h))
            intr[ck], _ = scale_intrinsics_for_resolution(cd["intrinsics"], cal_w, cal_h, cw, ch)
            if ck != "cam2_center":
                T = np.eye(4, dtype=np.float64)
                T[:3, :3] = quaternion_wxyz_to_rotmat(cd["cam_from_rig_qvec_wxyz"])
                T[:3, 3] = np.asarray(cd["cam_from_rig_tvec_m"], dtype=np.float64)
                T_rel[ck] = T
    else:
        default_arr = np.array([1585.0, 1585.0, 1440.0, 1920.0, -0.042, 0.008, -0.002, 0.0005], dtype=np.float64)
        for ck in CAM_ORDER:
            g = rig_profile_or_geom[ck]
            cw, ch = cam_sizes.get(ck, (g.get("width", 2880), g.get("height", 3840)))
            if "intrinsics" in g:
                intr[ck], _ = scale_intrinsics_for_resolution(
                    g["intrinsics"], int(g.get("width", cw)), int(g.get("height", ch)), cw, ch
                )
            else:
                arr = default_arr.copy()
                arr[2] = cw * 0.5
                arr[3] = ch * 0.5
                intr[ck] = arr
            if ck != "cam2_center":
                T = np.eye(4, dtype=np.float64)
                if "qvec_wxyz" in g:
                    T[:3, :3] = quaternion_wxyz_to_rotmat(g["qvec_wxyz"])
                else:
                    T[:3, :3] = euler_to_rotmat(float(g["yaw"]), float(g["pitch"]), float(g["roll"]))
                T[:3, 3] = np.array([float(g["tx"]), float(g["ty"]), float(g["tz"])], dtype=np.float64)
                T_rel[ck] = T

    return intr, T_rel


def enforce_calibrated_rig_on_sparse_model(
    sparse_dir: Path,
    project_dir: Path,
    rig_profile_or_geom: Dict[str, Any],
    progress_cb: Optional[Callable[[str], None]] = print,
) -> Dict[str, Any]:
    """
    Låser en färdig Spirula/COLMAP `sparse/0`-rekonstruktion till exakt uppmätt
    `OPENCV_FISHEYE` + 6-DoF rigg-geometri i **exakt 1:1 meterskala**:
      1. Skriver riktiga låsta lins-parametrar till `cameras.bin` (tar bort t.ex. fx=225 px).
      2. Räknar ut exakt metrisk skala utifrån uppmätt baslinje cam1<->cam3/cam2 och skalar världen.
      3. Återställer saknade bilder (t.ex. om någon kamera tappades bort i en buske) till 100 % täckning.
      4. Optimerar varje tios gemensamma 6-DoF rigg-pose + omtriangulerar `points3D.bin` med exakt fisheye.
      5. Skriver `sparse/0/rigs.txt` samt `rig_config.json` / `rig_config_legacy.json`.
    """
    cams_bin = sparse_dir / "cameras.bin"
    imgs_bin = sparse_dir / "images.bin"
    pts_bin = sparse_dir / "points3D.bin"
    if not (cams_bin.exists() and imgs_bin.exists() and pts_bin.exists()):
        raise FileNotFoundError(f"Saknar COLMAP-binärer i {sparse_dir}")

    cams = read_colmap_cameras_bin(cams_bin)
    imgs = read_colmap_images_bin_full(imgs_bin)
    pts = read_colmap_points3d_bin(pts_bin)

    # Knyt camera_id -> kameramapp och upplösning
    cam_sizes: Dict[str, Tuple[int, int]] = {}
    folder_to_camid: Dict[str, int] = {}
    for name, rec in imgs.items():
        parts = name.split("/")
        if len(parts) >= 2 and parts[-2] in CAM_ORDER:
            ck = parts[-2]
            cid = rec["camera_id"]
            folder_to_camid[ck] = cid
            if cid in cams and ck not in cam_sizes:
                cam_sizes[ck] = (int(cams[cid]["width"]), int(cams[cid]["height"]))

    for ck in CAM_ORDER:
        if ck not in cam_sizes:
            sample = next((project_dir / "images" / ck).glob("*.jpg"), None)
            if sample:
                im = cv2.imread(str(sample))
                if im is not None:
                    cam_sizes[ck] = (int(im.shape[1]), int(im.shape[0]))
        if ck not in cam_sizes:
            cam_sizes[ck] = (2880, 3840)

    intr, T_cam_rel = _load_rig_spec_for_enforcement(rig_profile_or_geom, cam_sizes)

    # Uppdatera cameras.bin med exakta OPENCV_FISHEYE (model_id=5) parametrar
    next_cid = max(cams.keys(), default=0) + 1
    for ck in CAM_ORDER:
        cid = folder_to_camid.get(ck)
        if cid is None:
            cid = next_cid
            next_cid += 1
            folder_to_camid[ck] = cid
        w, h = cam_sizes[ck]
        cams[cid] = {
            "camera_id": cid,
            "model_id": 5,  # OPENCV_FISHEYE
            "width": w,
            "height": h,
            "params": [float(x) for x in intr[ck]],
        }

    # Gruppera per bildfil (trio)
    triplets: Dict[str, Dict[str, Tuple[str, Dict[str, Any]]]] = {}
    for name, rec in imgs.items():
        parts = name.split("/")
        if len(parts) >= 2 and parts[-2] in CAM_ORDER:
            triplets.setdefault(parts[-1], {})[parts[-2]] = (name, rec)

    # Estimera exakt metrisk skalfaktor från kameracentrums observerade baslinjer relativt kalibrerad rigg
    scale_ratios: List[float] = []
    for fname, tmap in triplets.items():
        if "cam2_center" not in tmap:
            continue
        _, r0 = tmap["cam2_center"]
        R0 = quaternion_wxyz_to_rotmat(r0["qvec"])
        C0 = -R0.T @ r0["tvec"]
        for side_cam in ("cam1_left", "cam3_right"):
            if side_cam not in tmap:
                continue
            _, rs = tmap[side_cam]
            Rs = quaternion_wxyz_to_rotmat(rs["qvec"])
            Cs = -Rs.T @ rs["tvec"]
            R_rel_obs = Rs @ R0.T
            dR = R_rel_obs @ T_cam_rel[side_cam][:3, :3].T
            ang_deg = math.degrees(math.acos(float(np.clip((np.trace(dR) - 1.0) * 0.5, -1.0, 1.0))))
            if ang_deg > 18.0:
                continue
            C_cal = -T_cam_rel[side_cam][:3, :3].T @ T_cam_rel[side_cam][:3, 3]
            cal_len = float(np.linalg.norm(C_cal))
            if cal_len < 0.02:
                continue
            b_obs_in_0 = R0 @ (Cs - C0)
            proj = float(np.dot(b_obs_in_0, C_cal)) / max(1e-9, float(np.dot(b_obs_in_0, b_obs_in_0)))
            if 1e-4 < proj < 1e5:
                # Om b_obs * s == C_cal => s = ||C_cal|| / ||b_obs|| när vinkeln stämmer
                s_cand = cal_len / max(1e-9, float(np.linalg.norm(b_obs_in_0)))
                scale_ratios.append(s_cand)

    metric_scale = float(np.median(scale_ratios)) if scale_ratios else 1.0
    if abs(metric_scale - 1.0) > 1e-6:
        for rec in imgs.values():
            rec["tvec"] = rec["tvec"] * metric_scale
        for p in pts.values():
            p["xyz"] = p["xyz"] * metric_scale

    # Skapa initial 6-DoF rigg-pose per extraherad bild-trio (inkl. ev. saknade bilder på disk)
    disk_frames = sorted(p.name for p in (project_dir / "images" / "cam2_center").glob("*.jpg")) if (project_dir / "images" / "cam2_center").exists() else sorted(triplets.keys())
    all_fnames = sorted(set(triplets.keys()) | set(disk_frames))

    # Karta image_id -> obs för snabb lookup
    id_to_name_rec: Dict[int, Tuple[str, Dict[str, Any]]] = {rec["image_id"]: (nm, rec) for nm, rec in imgs.items()}
    next_img_id = max((r["image_id"] for r in imgs.values()), default=0) + 1

    # Bygg initial T_rig_from_world för alla registrerade trior och optimera gemensam 6-DoF rig-pose per trio!
    rig_world_poses: Dict[str, np.ndarray] = {}
    rescued_count = 0
    for fname in all_fnames:
        tmap = triplets.get(fname, {})
        cands: List[np.ndarray] = []
        for ck in CAM_ORDER:
            if ck not in tmap:
                continue
            _, rec = tmap[ck]
            T_cw = np.eye(4, dtype=np.float64)
            T_cw[:3, :3] = quaternion_wxyz_to_rotmat(rec["qvec"])
            T_cw[:3, 3] = rec["tvec"]
            # T_rig_w = inv(T_cam_rel[ck]) @ T_cw
            T_rw = np.linalg.inv(T_cam_rel[ck]) @ T_cw
            # Föredra cam2_center om båda finns och andra sidans rotation avvek kraftigt
            if ck == "cam2_center":
                cands = [T_rw]
                break
            cands.append(T_rw)
        if not cands:
            continue
        T_rw_init = _median_se3(cands)

        # Samla 2D-3D korrespondenser över alla kameror i denna trio
        obs_rig: List[Tuple[str, np.ndarray, np.ndarray]] = []
        for ck, (_, rec) in tmap.items():
            valid_mask = rec["point3D_ids"] >= 0
            if not np.any(valid_mask):
                continue
            pids_v = rec["point3D_ids"][valid_mask]
            xys_v = rec["xys"][valid_mask]
            pts3_list = []
            xys_keep = []
            for pid, xy in zip(pids_v, xys_v):
                if int(pid) in pts:
                    pts3_list.append(pts[int(pid)]["xyz"])
                    xys_keep.append(xy)
            if pts3_list:
                obs_rig.append((ck, np.asarray(pts3_list, dtype=np.float64), np.asarray(xys_keep, dtype=np.float64)))

        if sum(len(o[1]) for o in obs_rig) >= 8:
            rv0, tv0 = _rvec_tvec_from_se3(T_rw_init)

            def _pose_res(x):
                T_rw = _se3_from_rvec_tvec(x[:3], x[3:6])
                out_e = []
                for ck, pts3, xys2 in obs_rig:
                    T_cw = T_cam_rel[ck] @ T_rw
                    rv, tv = _rvec_tvec_from_se3(T_cw)
                    pr, z = project_fisheye_world_points(pts3, rv, tv, intr[ck])
                    d = pr - xys2
                    bad = z <= 0.02
                    if np.any(bad):
                        d[bad] = 200.0
                    out_e.append(d.ravel())
                return np.concatenate(out_e)

            sol = optimize.least_squares(_pose_res, np.concatenate([rv0, tv0]), loss="huber", f_scale=2.0, max_nfev=35)
            T_rw_opt = _se3_from_rvec_tvec(sol.x[:3], sol.x[3:6])
        else:
            T_rw_opt = T_rw_init

        rig_world_poses[fname] = T_rw_opt

        # Skriv tillbaka låst rigg-pose på alla 3 kameror (och rädda saknade kameror)
        for ck in CAM_ORDER:
            img_name = f"{ck}/{fname}"
            img_file = project_dir / "images" / ck / fname
            if not img_file.exists() and img_name not in imgs:
                continue
            T_cw = T_cam_rel[ck] @ T_rw_opt
            q_new = rotmat_to_quaternion_wxyz(T_cw[:3, :3])
            t_new = T_cw[:3, 3].copy()
            if img_name in imgs:
                imgs[img_name]["qvec"] = q_new
                imgs[img_name]["tvec"] = t_new
                imgs[img_name]["camera_id"] = folder_to_camid[ck]
            else:
                imgs[img_name] = {
                    "image_id": next_img_id,
                    "qvec": q_new,
                    "tvec": t_new,
                    "camera_id": folder_to_camid[ck],
                    "xys": np.zeros((0, 2), dtype=np.float64),
                    "point3D_ids": np.zeros(0, dtype=np.int64),
                }
                id_to_name_rec[next_img_id] = (img_name, imgs[img_name])
                next_img_id += 1
                rescued_count += 1

    # Omtriangulera / förfina 3D-punkter med låsta kameror & äkta OPENCV_FISHEYE
    img_R_t: Dict[int, Tuple[np.ndarray, np.ndarray, str]] = {}
    for name, rec in imgs.items():
        ck = name.split("/")[-2]
        if ck in CAM_ORDER:
            R_cw = quaternion_wxyz_to_rotmat(rec["qvec"])
            img_R_t[rec["image_id"]] = (R_cw, rec["tvec"], ck)

    # Rensa ut grova outliers bland 3D-punkter genom exakt fisheye-projektion & ray-triangulering
    reproj_sq: List[float] = []
    kept_pts: Dict[int, Dict[str, Any]] = {}
    for pid, p in pts.items():
        track = p["track"]
        obs_rays: List[Tuple[np.ndarray, np.ndarray, np.ndarray, str]] = []
        valid_track: List[Tuple[int, int]] = []
        for im_id, p2_idx in track:
            if im_id not in id_to_name_rec or im_id not in img_R_t:
                continue
            _nm, rec = id_to_name_rec[im_id]
            if p2_idx >= len(rec["xys"]):
                continue
            xy = rec["xys"][p2_idx : p2_idx + 1]
            R_cw, t_cw, ck = img_R_t[im_id]
            ray_c = undistort_fisheye_to_rays(xy, intr[ck])[0]
            ray_w = R_cw.T @ ray_c
            C_w = -R_cw.T @ t_cw
            obs_rays.append((C_w, ray_w, xy[0], ck))
            valid_track.append((im_id, p2_idx))

        if len(obs_rays) < 2:
            continue

        # Linjär linje-skärning i 3D för rent startvärde
        A_mat = np.zeros((3, 3), dtype=np.float64)
        b_vec = np.zeros(3, dtype=np.float64)
        for C_w, d_w, _, _ in obs_rays:
            P_mat = np.eye(3, dtype=np.float64) - np.outer(d_w, d_w)
            A_mat += P_mat
            b_vec += P_mat @ C_w
        try:
            X0 = np.linalg.solve(A_mat + 1e-6 * np.eye(3), b_vec)
        except np.linalg.LinAlgError:
            X0 = p["xyz"].copy()

        # Kontrollera djup + återprojektionsfel i alla kameror
        errs_p = []
        depths_ok = True
        for k_i, (im_id, _p2) in enumerate(valid_track):
            R_cw, t_cw, ck = img_R_t[im_id]
            rv, _ = cv2.Rodrigues(R_cw)
            pr, z = project_fisheye_world_points(X0.reshape(1, 3), rv.ravel(), t_cw, intr[ck])
            if float(z[0]) <= 0.05 or float(z[0]) > 250.0:
                depths_ok = False
                break
            errs_p.append(float(np.linalg.norm(pr[0] - obs_rays[k_i][2])))

        if not depths_ok or not errs_p:
            continue
        mean_e = float(np.mean(errs_p))
        if mean_e > 5.0:
            continue
        p_copy = dict(p)
        p_copy["xyz"] = X0
        p_copy["error"] = mean_e
        p_copy["track"] = valid_track
        kept_pts[pid] = p_copy
        reproj_sq.extend(e * e for e in errs_p)

    # Uppdatera point3D_ids i images för ev. droppade punkter
    kept_set = set(kept_pts.keys())
    for rec in imgs.values():
        pids = rec["point3D_ids"]
        if len(pids) > 0:
            for j in range(len(pids)):
                if pids[j] >= 0 and int(pids[j]) not in kept_set:
                    pids[j] = -1

    write_colmap_cameras_bin(cams_bin, cams)
    write_colmap_images_bin_full(imgs_bin, imgs)
    write_colmap_points3d_bin(pts_bin, kept_pts)

    # Skriv även Spirulas `sparse/0/rigs.txt` + COLMAP `rig_config.json` & `rig_config_legacy.json`
    n_trig = max(1, len(rig_world_poses))
    rigs_lines = [
        "# Rig calibration enforced by OptiGSRigg ChArUco calibration, cam_from_rig per member as",
        "# rig member reference qw qx qy qz tx ty tz frames spread_deg, in metres.",
        f"rig0 cam2_center cam2_center 1 0 0 0 0 0 0 {n_trig} 0",
    ]
    cameras_modern = []
    cameras_legacy = []
    cam_id_map = {"cam1_left": 1, "cam2_center": 2, "cam3_right": 3}

    for ck in CAM_ORDER:
        T_cr = T_cam_rel[ck]
        qw, qx, qy, qz = rotmat_to_quaternion_wxyz(T_cr[:3, :3])
        tx, ty, tz = [float(v) for v in T_cr[:3, 3]]
        if ck != "cam2_center":
            rigs_lines.append(f"rig0 {ck} cam2_center {qw:.12g} {qx:.12g} {qy:.12g} {qz:.12g} {tx:.12g} {ty:.12g} {tz:.12g} {n_trig} 0")

        m_entry: Dict[str, Any] = {
            "image_prefix": f"{ck}/",
            "camera_model_name": "OPENCV_FISHEYE",
            "camera_params": [float(x) for x in intr[ck]],
        }
        l_entry: Dict[str, Any] = {
            "camera_id": cam_id_map[ck],
            "image_prefix": f"{ck}/",
            "rel_tvec": [tx, ty, tz],
            "rel_qvec": [qw, qx, qy, qz],
        }
        if ck == "cam2_center":
            m_entry["ref_sensor"] = True
            l_entry["ref_camera"] = True
        else:
            m_entry["cam_from_rig_rotation"] = [qw, qx, qy, qz]
            m_entry["cam_from_rig_translation"] = [tx, ty, tz]
        cameras_modern.append(m_entry)
        cameras_legacy.append(l_entry)

    (sparse_dir / "rigs.txt").write_text("\n".join(rigs_lines) + "\n", encoding="utf-8")
    (project_dir / "rig_config.json").write_text(json.dumps([{"cameras": cameras_modern}], indent=2), encoding="utf-8")
    (project_dir / "rig_config_legacy.json").write_text(
        json.dumps([{"ref_camera_id": 2, "cameras": cameras_legacy}], indent=2), encoding="utf-8"
    )

    rms_final = float(math.sqrt(float(np.mean(reproj_sq)))) if reproj_sq else 0.0
    if progress_cb:
        progress_cb(
            f"[RigLock] Låste {len(imgs)} bilder (räddade {rescued_count} saknade), {len(kept_pts)} 3D-punkter, "
            f"skala ×{metric_scale:.5f} -> 1:1 meter (RMS {rms_final:.2f} px)!"
        )
    return {
        "ok": True,
        "num_images": len(imgs),
        "rescued_images": rescued_count,
        "num_points3d": len(kept_pts),
        "metric_scale_factor": metric_scale,
        "rms_px": round(rms_final, 3),
    }
