#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OptiGSRigg Studio v2.0 – Cross-Platform Desktop App (Windows & macOS)
=====================================================================
Arbetsstation för 3x DJI Osmo Action 4 -> 3DGUT Pipeline.
Innehåller nu dedikerad Steg 1-modul för:
  - Full kamerakontroll (CAM0_LEFT.id, CAM1_MID.id, CAM2_RIGHT.id + diskutrymme)
  - Tagningsinspektion på SD-kort och lokala diskar
  - Snabb videoförhandsvisning (Mittenkamera / Vänster / Höger) med bildrute-scrubber & ljud
  - Komplett EXIF / FFprobe-metadata (Upplösning, Rotation, FPS, Codec, 10-bit färgrymd, Bitrate, Timecode, Ljud, DJI-spår)
  - Eget projektnamn vid import eller omdöpning av tagning
"""

import argparse
import ctypes
import datetime
import json
import os
import platform
import re
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

APP_ROOT = Path(__file__).resolve().parent
SRC_DIR = APP_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))
UI_HTML_PATH = APP_ROOT / "ui" / "index.html"
SRC_SCRIPT = APP_ROOT / "src" / "process_3dgs_rig.py"
CALIB_JSON_PATH = APP_ROOT / "config" / "calibrated_rig.json"
DJI_PROFILE_PATH = APP_ROOT / "config" / "dji_rig_profile.json"
DJI_DEMO_PROFILE_PATH = APP_ROOT / "config" / "dji_rig_profile_demo.json"
CALIB_PREVIEW_DIR = APP_ROOT / "config" / "calib_preview"
CALIB_DIR = APP_ROOT / "calibration"
BOARDS_DIR = CALIB_DIR / "boards"
CALIB_GUIDE_PATH = APP_ROOT / "KALIBRERINGSGUIDE_SV.md"
SETTINGS_JSON_PATH = APP_ROOT / "config" / "settings.json"
DEFAULT_LUT_PATH = APP_ROOT / "assets" / "luts" / "DJI_Action4_DLogM_to_Rec709.cube"
ICONS_DIR = APP_ROOT / "assets" / "icons"
_RUNNING_INSTANCE_FILE = APP_ROOT / "config" / ".running_instance.json"
_STARTUP_LOG_PATH = APP_ROOT / "config" / "last_app_startup.log"


def _ensure_valid_std_streams() -> None:
    """Säkerställer giltiga stdout/stderr-strömmar vid start med pythonw.exe (utan konsol)."""
    if sys.stdout is None:
        try:
            sys.stdout = open(os.devnull, "w", encoding="utf-8")
        except OSError:
            pass
    if sys.stderr is None:
        try:
            _STARTUP_LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
            sys.stderr = open(_STARTUP_LOG_PATH, "a", encoding="utf-8")
        except OSError:
            try:
                sys.stderr = open(os.devnull, "w", encoding="utf-8")
            except OSError:
                pass


_ensure_valid_std_streams()

if platform.system() == "Darwin":
    _DEFAULT_CAPTURES_ROOT = str(Path.home() / "3DGS_Captures")
    _DEFAULT_LICHTFELD_EXE = "/Applications/LichtFeld-Studio.app/Contents/MacOS/LichtFeld-Studio"
    _DEFAULT_SPIRULA_EXE = "/Applications/Spirula.app/Contents/MacOS/spirula"
else:
    _DEFAULT_CAPTURES_ROOT = r"D:\3DGS_Captures" if Path("D:/").exists() else str(Path.home() / "3DGS_Captures")
    _DEFAULT_LICHTFELD_EXE = r"D:\LichtFeld-Studio-windows-v0.5.4.1\bin\LichtFeld-Studio.exe"
    _DEFAULT_SPIRULA_EXE = r"D:\spirula-2026.10.30-windows-vulkan-x86_64\spirula.exe"

_DEFAULT_COLMAP_EXE = "colmap"
_DEFAULT_MCP_URL = "http://127.0.0.1:45677/mcp"


def _is_valid_exe_path(path_str: str) -> bool:
    s = (path_str or "").strip()
    if not s:
        return False
    p = Path(s)
    if p.is_file():
        return True
    return shutil.which(s) is not None


def _glob_first_existing(patterns: list[tuple[Path, str]]) -> str:
    for base, pat in patterns:
        try:
            if base.exists():
                matches = sorted(base.glob(pat), reverse=True)
                for m in matches:
                    if m.is_file():
                        return str(m)
        except OSError:
            pass
    return ""


_SKIP_SCAN_DIRS = {
    "windows",
    "users",
    "programdata",
    "$recycle.bin",
    "system volume information",
    "recovery",
    "perflogs",
}


def _search_drive_subfolders(roots: list[Path], sub_patterns: list[str]) -> str:
    """Söker ett steg djupare i rotkataloger (t.ex. C:\\portabla-program\\, D:\\Splat programs\\)."""
    for root in roots:
        try:
            if not root.exists():
                continue
            for sub in sorted(root.iterdir()):
                try:
                    if not sub.is_dir():
                        continue
                    if sub.name.lower() in _SKIP_SCAN_DIRS:
                        continue
                    for pat in sub_patterns:
                        matches = sorted(sub.glob(pat), reverse=True)
                        for m in matches:
                            if m.is_file():
                                return str(m)
                except OSError:
                    continue
        except OSError:
            continue
    return ""


def auto_find_engine_paths() -> dict:
    """
    Söker automatiskt efter Spirula Studio, LichtFeld Studio, COLMAP samt projektmapp
    i vanliga installationskataloger på Windows, macOS och PATH.
    """
    sys_name = platform.system()
    spirula = ""
    lichtfeld = ""
    colmap = shutil.which("colmap") or ""

    if sys_name == "Darwin":
        for p in (
            "/Applications/Spirula.app/Contents/MacOS/spirula",
            "/Applications/Spirula Studio.app/Contents/MacOS/spirula",
        ):
            if Path(p).is_file():
                spirula = p
                break
        if not spirula:
            spirula = shutil.which("spirula") or ""

        for p in ("/Applications/LichtFeld-Studio.app/Contents/MacOS/LichtFeld-Studio",):
            if Path(p).is_file():
                lichtfeld = p
                break
        if not lichtfeld:
            lichtfeld = shutil.which("LichtFeld-Studio") or ""

        if not colmap:
            for p in ("/opt/homebrew/bin/colmap", "/usr/local/bin/colmap"):
                if Path(p).is_file():
                    colmap = p
                    break
    else:
        if Path(_DEFAULT_SPIRULA_EXE).is_file():
            spirula = _DEFAULT_SPIRULA_EXE
        else:
            spirula = (
                _glob_first_existing(
                    [
                        (Path("D:/"), "spirula*/spirula.exe"),
                        (Path("C:/"), "spirula*/spirula.exe"),
                        (Path("C:/Program Files"), "Spirula*/spirula.exe"),
                        (Path.home(), "spirula*/spirula.exe"),
                    ]
                )
                or _search_drive_subfolders([Path("D:/"), Path("C:/")], ["*spirula*/spirula.exe"])
                or (shutil.which("spirula") or "")
            )

        if Path(_DEFAULT_LICHTFELD_EXE).is_file():
            lichtfeld = _DEFAULT_LICHTFELD_EXE
        else:
            lichtfeld = (
                _glob_first_existing(
                    [
                        (Path("D:/"), "LichtFeld-Studio*/bin/LichtFeld-Studio.exe"),
                        (Path("D:/"), "LichtFeld-Studio*/LichtFeld-Studio.exe"),
                        (Path("C:/"), "LichtFeld-Studio*/bin/LichtFeld-Studio.exe"),
                        (Path("C:/Program Files"), "LichtFeld-Studio*/bin/LichtFeld-Studio.exe"),
                    ]
                )
                or _search_drive_subfolders(
                    [Path("D:/"), Path("C:/")],
                    [
                        "*LichtFeld-Studio*/bin/LichtFeld-Studio.exe",
                        "*LichtFeld-Studio*/LichtFeld-Studio.exe",
                    ],
                )
                or (shutil.which("LichtFeld-Studio") or "")
            )

        if not colmap:
            colmap = _glob_first_existing(
                [
                    (Path("D:/"), "colmap*/bin/colmap.exe"),
                    (Path("D:/"), "colmap*/COLMAP.bat"),
                    (Path("C:/"), "colmap*/bin/colmap.exe"),
                    (Path("C:/Program Files"), "COLMAP*/bin/colmap.exe"),
                ]
            ) or _search_drive_subfolders(
                [Path("D:/"), Path("C:/")],
                ["*colmap*/bin/colmap.exe", "*colmap*/COLMAP.bat"],
            )

    return {
        "spirula_exe": spirula or _DEFAULT_SPIRULA_EXE,
        "lichtfeld_exe": lichtfeld or _DEFAULT_LICHTFELD_EXE,
        "colmap_exe": colmap or _DEFAULT_COLMAP_EXE,
        "captures_root": _DEFAULT_CAPTURES_ROOT,
        "mcp_url": _DEFAULT_MCP_URL,
        "extract_backend": "auto",
    }


def _validate_settings_status(s: dict) -> dict:
    out = dict(s)
    cap = (out.get("captures_root") or "").strip()
    out["spirula_ok"] = _is_valid_exe_path(out.get("spirula_exe", ""))
    out["lichtfeld_ok"] = _is_valid_exe_path(out.get("lichtfeld_exe", ""))
    out["colmap_ok"] = _is_valid_exe_path(out.get("colmap_exe", ""))
    out["captures_root_ok"] = bool(cap) and Path(cap).exists()
    return out


def load_app_settings() -> dict:
    auto_det = auto_find_engine_paths()
    defaults = dict(auto_det)
    if SETTINGS_JSON_PATH.exists():
        try:
            saved = json.loads(SETTINGS_JSON_PATH.read_text(encoding="utf-8"))
            if isinstance(saved, dict):
                for k in ("spirula_exe", "lichtfeld_exe", "colmap_exe", "captures_root", "mcp_url", "extract_backend"):
                    val = str(saved.get(k) or "").strip()
                    if val:
                        defaults[k] = val
                healed = False
                for bin_key in ("spirula_exe", "lichtfeld_exe", "colmap_exe"):
                    if not _is_valid_exe_path(str(defaults.get(bin_key) or "")) and _is_valid_exe_path(
                        str(auto_det.get(bin_key) or "")
                    ):
                        defaults[bin_key] = auto_det[bin_key]
                        healed = True
                if healed:
                    save_app_settings(defaults)
                return defaults
        except Exception:
            pass
    save_app_settings(defaults)
    return defaults


def save_app_settings(data: dict) -> dict:
    clean = {
        "spirula_exe": str(data.get("spirula_exe") or "").strip(),
        "lichtfeld_exe": str(data.get("lichtfeld_exe") or "").strip(),
        "colmap_exe": str(data.get("colmap_exe") or "colmap").strip() or "colmap",
        "captures_root": str(data.get("captures_root") or _DEFAULT_CAPTURES_ROOT).strip() or _DEFAULT_CAPTURES_ROOT,
        "mcp_url": str(data.get("mcp_url") or _DEFAULT_MCP_URL).strip() or _DEFAULT_MCP_URL,
        "extract_backend": str(data.get("extract_backend") or "auto").strip() or "auto",
    }
    try:
        SETTINGS_JSON_PATH.parent.mkdir(parents=True, exist_ok=True)
        SETTINGS_JSON_PATH.write_text(json.dumps(clean, indent=2), encoding="utf-8")
    except Exception:
        pass
    return clean

SD_CARD_ID_FILES = {
    "CAM0_LEFT.id": "cam1_left",
    "CAM1_MID.id": "cam2_center",
    "CAM2_RIGHT.id": "cam3_right",
    "CAM1_LEFT.id": "cam1_left",
    "CAM2_CENTER.id": "cam2_center",
    "CAM3_RIGHT.id": "cam3_right",
}


def scan_sd_cards() -> dict:
    """
    Skannar alla monterade enheter (Windows D:..Z: samt macOS /Volumes/*)
    med ren Python standardbibliotek (kräver varken cv2 eller scipy).
    """
    found_drives: dict[str, tuple[Path, str]] = {}
    unmarked_dcim: list[str] = []

    candidate_roots: list[Path] = []
    if os.name == "nt":
        try:
            k32 = ctypes.windll.kernel32
            bitmask = k32.GetLogicalDrives()
            for idx, letter in enumerate("ABCDEFGHIJKLMNOPQRSTUVWXYZ"):
                if idx < 3:  # Hoppa över A:, B:, C:
                    continue
                if bitmask & (1 << idx):
                    drive_type = k32.GetDriveTypeW(f"{letter}:\\")
                    if drive_type in (2, 3):  # DRIVE_REMOVABLE (2) eller DRIVE_FIXED (3); hoppa över nätverksenheter
                        candidate_roots.append(Path(f"{letter}:/"))
        except Exception:
            for letter in "DEFGHIJKLMNOPQRSTUVWXYZ":
                candidate_roots.append(Path(f"{letter}:/"))
    else:
        for base in (Path("/Volumes"), Path("/media"), Path("/mnt")):
            if base.exists():
                try:
                    candidate_roots.extend([p for p in base.iterdir() if p.is_dir()])
                except OSError:
                    pass

    for drive_root in candidate_roots:
        try:
            if not drive_root.exists():
                continue
            matched_id = None
            for id_filename, target_cam in SD_CARD_ID_FILES.items():
                if (drive_root / id_filename).exists():
                    found_drives[target_cam] = (drive_root, id_filename)
                    matched_id = id_filename
                    break
            if not matched_id and (drive_root / "DCIM").exists():
                unmarked_dcim.append(str(drive_root))
        except OSError:
            continue

    status: dict = {}
    for cam_key, default_id in [
        ("cam1_left", "CAM0_LEFT.id"),
        ("cam2_center", "CAM1_MID.id"),
        ("cam3_right", "CAM2_RIGHT.id"),
    ]:
        if cam_key in found_drives:
            drive_root, id_file = found_drives[cam_key]
            dcim_dir = drive_root / "DCIM"
            search_root = dcim_dir if dcim_dir.exists() else drive_root
            try:
                mp4s = [
                    p
                    for p in search_root.rglob("*")
                    if p.is_file() and p.suffix.lower() == ".mp4" and not p.name.startswith("._")
                ]
                mp4s.sort(key=lambda p: p.stat().st_mtime)
            except OSError:
                mp4s = []

            clips = []
            for p in reversed(mp4s):
                try:
                    st = p.stat()
                    clips.append(
                        {
                            "name": p.name,
                            "path": str(p),
                            "size_mb": round(st.st_size / (1024 * 1024), 1),
                            "mtime": float(st.st_mtime),
                            "mtime_str": datetime.datetime.fromtimestamp(st.st_mtime).strftime("%Y-%m-%d %H:%M:%S"),
                        }
                    )
                except OSError:
                    pass
            status[cam_key] = {
                "connected": True,
                "drive": str(drive_root),
                "id_file": id_file,
                "clip_count": len(clips),
                "clips": clips,
            }
        else:
            status[cam_key] = {
                "connected": False,
                "drive": None,
                "id_file": default_id,
                "clip_count": 0,
                "clips": [],
            }
    status["_unmarked_dcim"] = unmarked_dcim
    return status


def _enrich_sd_status_disk_usage(sd_status: dict) -> dict:
    """Fyller på diskutrymme (GB) och total klippstorlek (MB) för anslutna SD-kort."""
    for cam_key in ("cam1_left", "cam2_center", "cam3_right"):
        cam_info = sd_status.get(cam_key)
        if cam_info and cam_info.get("connected") and cam_info.get("drive"):
            try:
                usage = shutil.disk_usage(cam_info["drive"])
                cam_info["free_gb"] = round(usage.free / (1024**3), 1)
                cam_info["total_gb"] = round(usage.total / (1024**3), 1)
            except Exception:
                cam_info["free_gb"] = 0.0
                cam_info["total_gb"] = 0.0
            cam_info["clips_total_mb"] = round(sum(c.get("size_mb", 0) for c in cam_info.get("clips", [])), 1)
    return sd_status


def build_calibration_sd_candidates(
    sd_status: dict,
    captures_root: Path | None = None,
    max_probe_candidates: int = 6,
) -> dict:
    """
    Analyserar anslutna SD-kort/kameror via .id-filer, parar ihop troliga
    kalibreringstagningar (tidsstämpel + fallback för osynkade kameraklockor),
    berikar toppkandidater med FFprobe/EXIF-metadata och poängsätter lämplighet
    för ChArUco-riggkalibrering.
    """
    cam_order = ("cam1_left", "cam2_center", "cam3_right")
    cam_labels = {
        "cam1_left": "Cam 1 Vänster",
        "cam2_center": "Cam 2 Mitten",
        "cam3_right": "Cam 3 Höger",
    }
    default_ids = {
        "cam1_left": "CAM0_LEFT.id",
        "cam2_center": "CAM1_MID.id",
        "cam3_right": "CAM2_RIGHT.id",
    }

    cameras: dict = {}
    clips_by_cam: dict[str, list[dict]] = {}
    connected_count = 0

    for ck in cam_order:
        raw_cam = dict(sd_status.get(ck) or {})
        is_conn = bool(raw_cam.get("connected"))
        if is_conn:
            connected_count += 1
        raw_clips = list(raw_cam.get("clips") or [])
        norm_clips = []
        for idx, c in enumerate(raw_clips):
            mt = c.get("mtime")
            if mt is None and c.get("path"):
                try:
                    mt = float(Path(c["path"]).stat().st_mtime)
                except OSError:
                    mt = 0.0
            norm_clips.append(
                {
                    "name": c.get("name") or Path(c.get("path") or f"clip_{idx}").name,
                    "path": str(c.get("path") or ""),
                    "size_mb": float(c.get("size_mb") or 0.0),
                    "mtime_str": str(c.get("mtime_str") or "-"),
                    "mtime": float(mt or 0.0),
                    **{k: v for k, v in c.items() if k not in ("name", "path", "size_mb", "mtime_str", "mtime")},
                }
            )
        if norm_clips and any(c["mtime"] > 0.0 for c in norm_clips):
            norm_clips.sort(key=lambda c: c["mtime"], reverse=True)
        clips_by_cam[ck] = norm_clips
        cameras[ck] = {
            "label": cam_labels[ck],
            "connected": is_conn,
            "drive": raw_cam.get("drive"),
            "id_file": raw_cam.get("id_file") or default_ids[ck],
            "clip_count": len(norm_clips),
            "free_gb": float(raw_cam.get("free_gb") or 0.0),
            "total_gb": float(raw_cam.get("total_gb") or 0.0),
            "clips_total_mb": float(
                raw_cam.get("clips_total_mb")
                if raw_cam.get("clips_total_mb") is not None
                else round(sum(c["size_mb"] for c in norm_clips), 1)
            ),
            "clips": norm_clips,
        }

    cams_with_clips = [ck for ck in cam_order if clips_by_cam[ck]]
    raw_groups: list[dict[str, dict]] = []

    if len(cams_with_clips) >= 2:
        max_per_cam = 60
        work_clips = {ck: clips_by_cam[ck][:max_per_cam] for ck in cams_with_clips}
        all_have_mtime = all(cl["mtime"] > 0.0 for ck in cams_with_clips for cl in work_clips[ck])
        edges: list[tuple[float, float, str, int, str, int]] = []
        if all_have_mtime:
            for idx_a in range(len(cams_with_clips)):
                ck_a = cams_with_clips[idx_a]
                for idx_b in range(idx_a + 1, len(cams_with_clips)):
                    ck_b = cams_with_clips[idx_b]
                    for ia, ca in enumerate(work_clips[ck_a]):
                        ta, sa = ca["mtime"], ca["size_mb"]
                        for ib, cb in enumerate(work_clips[ck_b]):
                            dt = abs(ta - cb["mtime"])
                            if dt <= 120.0:
                                edges.append((dt, abs(sa - cb["size_mb"]), ck_a, ia, ck_b, ib))

        if edges:
            pairing_mode = "timestamp"
            clusters: dict[tuple[str, int], dict[str, tuple[int, dict]]] = {}
            node_to_cid: dict[tuple[str, int], tuple[str, int]] = {}
            for ck in cams_with_clips:
                for i, cl in enumerate(work_clips[ck]):
                    nid = (ck, i)
                    clusters[nid] = {ck: (i, cl)}
                    node_to_cid[nid] = nid

            edges.sort(key=lambda e: (e[0], e[1]))
            for _dt, _ds, ck_a, ia, ck_b, ib in edges:
                cid_a = node_to_cid[(ck_a, ia)]
                cid_b = node_to_cid[(ck_b, ib)]
                if cid_a == cid_b:
                    continue
                grp_a = clusters[cid_a]
                grp_b = clusters[cid_b]
                if set(grp_a.keys()) & set(grp_b.keys()):
                    continue
                merged_times = [v[1]["mtime"] for v in grp_a.values()] + [v[1]["mtime"] for v in grp_b.values()]
                if (max(merged_times) - min(merged_times)) > 120.0:
                    continue
                grp_a.update(grp_b)
                for m_ck, (m_idx, _m_cl) in grp_b.items():
                    node_to_cid[(m_ck, m_idx)] = cid_a
                del clusters[cid_b]

            ordered_clusters = sorted(
                clusters.values(),
                key=lambda grp: max(v[1]["mtime"] for v in grp.values()),
                reverse=True,
            )[:12]
            raw_groups = [{ck: pair[1] for ck, pair in grp.items()} for grp in ordered_clusters]
        else:
            pairing_mode = "newest_index"
            max_len = min(12, max(len(clips_by_cam[ck]) for ck in cams_with_clips))
            for idx in range(max_len):
                grp = {ck: clips_by_cam[ck][idx] for ck in cams_with_clips if idx < len(clips_by_cam[ck])}
                if grp:
                    raw_groups.append(grp)
    elif len(cams_with_clips) == 1:
        pairing_mode = "single_card"
        only_ck = cams_with_clips[0]
        for clip in clips_by_cam[only_ck][:12]:
            raw_groups.append({only_ck: clip})
    else:
        pairing_mode = "none"

    # Skanna befintliga sessioner under captures_root för 1-kort-i-taget-flödet
    existing_sessions_map: dict[str, dict] = {}
    partial_candidates: list[tuple[float, dict]] = []
    if captures_root and Path(captures_root).exists():
        try:
            for d in Path(captures_root).iterdir():
                if not d.is_dir() or d.name.startswith("."):
                    continue
                raw_d = d / "raw"
                if not raw_d.exists():
                    continue
                present = {ck: (raw_d / f"{ck}.mp4").is_file() for ck in cam_order}
                mp4_cnt = sum(1 for v in present.values() if v)
                sess_info = {
                    "name": d.name,
                    "raw_dir": str(raw_d),
                    "mp4_count": mp4_cnt,
                    "present": present,
                }
                existing_sessions_map[d.name] = sess_info
                if 1 <= mp4_cnt < 3 and ("calib" in d.name.lower() or "kalib" in d.name.lower()):
                    try:
                        mt_dir = raw_d.stat().st_mtime
                    except OSError:
                        mt_dir = 0.0
                    partial_candidates.append((mt_dir, sess_info))
        except OSError:
            pass
    partial_candidates.sort(key=lambda x: x[0], reverse=True)
    partial_calib_session = partial_candidates[0][1] if partial_candidates else None

    candidates: list[dict] = []
    for rank, group in enumerate(raw_groups):
        ref_clip = group.get("cam2_center") or group.get("cam1_left") or group.get("cam3_right")
        paths = {ck: (group[ck]["path"] if ck in group else "") for ck in cam_order}
        cams_present = len(group)
        total_mb = round(sum(float(c.get("size_mb") or 0.0) for c in group.values()), 1)

        probed_metas: dict[str, dict] = {}
        if rank < max_probe_candidates:
            for ck, clip in group.items():
                p_str = clip.get("path") or ""
                if p_str:
                    m = dict(probe_video_metadata(Path(p_str)))
                    # Tillåt ev. redan berikad test-metadata om ffprobe inte körts på dummyfil
                    if (m.get("duration_sec", 0.0) <= 0.0 or m.get("probe_error")) and clip.get("duration_sec"):
                        for mk in (
                            "duration_sec",
                            "duration_str",
                            "fps",
                            "effective_width",
                            "effective_height",
                            "aspect_ratio",
                            "color_mode",
                            "is_dlog",
                            "bit_depth",
                            "audio_channels",
                        ):
                            if mk in clip:
                                m[mk] = clip[mk]
                        m.pop("probe_error", None)
                        if m.get("codec", "-") == "-":
                            m["codec"] = "HEVC"
                        if "audio_channels" not in clip and m.get("audio_channels", 0) == 0:
                            m["audio_channels"] = 2
                    probed_metas[ck] = m

        valid_metas = [
            m for m in probed_metas.values() if m.get("exists") and not m.get("probe_error") and m.get("codec", "-") != "-"
        ]
        durations = [float(m["duration_sec"]) for m in valid_metas if float(m.get("duration_sec") or 0.0) > 0.0]
        avg_dur = round(sum(durations) / len(durations), 1) if durations else 0.0
        dur_spread = round(max(durations) - min(durations), 1) if len(durations) >= 2 else 0.0

        fps_vals = [float(m["fps"]) for m in valid_metas if float(m.get("fps") or 0.0) > 0.0]
        fps_match = len({round(f, 2) for f in fps_vals}) <= 1 if fps_vals else True
        rep_fps = round(fps_vals[0], 2) if fps_vals else 0.0

        res_vals = [
            f"{m['effective_width']}×{m['effective_height']}"
            for m in valid_metas
            if int(m.get("effective_width") or 0) > 0 and int(m.get("effective_height") or 0) > 0
        ]
        res_match = len(set(res_vals)) <= 1 if res_vals else True
        rep_res = res_vals[0] if res_vals else "-"
        rep_aspect = next((m["aspect_ratio"] for m in valid_metas if m.get("aspect_ratio") and m["aspect_ratio"] != "-"), "-")

        color_modes = [m["color_mode"] for m in valid_metas if m.get("color_mode") and m["color_mode"] != "-"]
        color_mode = color_modes[0] if color_modes else "-"
        is_dlog = any(bool(m.get("is_dlog")) for m in valid_metas)
        audio_missing = any(int(m.get("audio_channels") or 0) == 0 for m in valid_metas)

        warnings: list[str] = []
        if cams_present < 3:
            warnings.append(
                f"Endast {cams_present}/3 kameror anslutna — du kan importera nu och fylla på nästa kamerakort i samma mapp."
            )
        if not res_match:
            warnings.append("VARNING: Olika upplösning/bildförhållande mellan kamerorna!")
        if not fps_match:
            warnings.append("VARNING: Olika bildfrekvens (FPS) mellan kamerorna!")
        if dur_spread > 8.0:
            warnings.append(f"Klippens längd skiljer sig {dur_spread:.1f} s mellan kamerorna.")
        if audio_missing:
            warnings.append("VARNING: Minst en kamera saknar ljudspår (klapp-synk kräver ljud).")
        if pairing_mode == "newest_index":
            warnings.append(
                "Kameraklockorna skiljer sig >3 min — filerna parade i ordning (senaste först). Kontrollera att rätt fil valts."
            )

        score = {3: 45, 2: 22, 1: 10}.get(cams_present, 0)
        score += 28 if rank == 0 else (16 if rank == 1 else (8 if rank == 2 else 0))
        if avg_dur > 0.0:
            if 20.0 <= avg_dur <= 150.0:
                score += 25
            elif (10.0 <= avg_dur < 20.0) or (150.0 < avg_dur <= 300.0):
                score += 10
            else:
                score -= 20
        if valid_metas and res_match and fps_match:
            score += 10
        if len(durations) >= 2 and dur_spread <= 8.0:
            score += 10
        if not res_match:
            score -= 25
        if not fps_match:
            score -= 15
        if audio_missing:
            score -= 15

        mtime_str = str(ref_clip.get("mtime_str") or "-") if ref_clip else "-"
        clean_dt = mtime_str.replace("-", "").replace(":", "").replace(" ", "_")[:13]
        if len(clean_dt) < 8:
            clean_dt = time.strftime("%Y%m%d_%H%M")
        suggested_name = f"Calib_Rig_{clean_dt}"

        dur_txt = f"{avg_dur:.0f} s" if avg_dur > 0.0 else "-"
        fps_txt = f"{rep_fps:g} fps" if rep_fps > 0.0 else "-"
        ref_name = str(ref_clip.get("name") or f"Take_{rank + 1}") if ref_clip else f"Take_{rank + 1}"

        reason_parts = [f"{cams_present}/3 kameror"]
        if avg_dur > 0.0:
            reason_parts.append(f"längd {dur_txt}")
        if rep_res != "-":
            reason_parts.append(f"{rep_res} ({rep_aspect})")
        if rep_fps > 0.0:
            reason_parts.append(fps_txt)
        if color_mode != "-":
            reason_parts.append(color_mode)
        if rank == 0:
            reason_parts.append("senaste tagningen")

        candidates.append(
            {
                "id": f"cal_sd_{rank + 1}",
                "rank": rank,
                "score": score,
                "recommended": False,
                "name": ref_name,
                "mtime_str": mtime_str,
                "cams_present": cams_present,
                "total_mb": total_mb,
                "duration_sec": avg_dur,
                "duration_str": dur_txt,
                "duration_spread_sec": dur_spread,
                "resolution": rep_res,
                "aspect_ratio": rep_aspect,
                "fps": rep_fps,
                "color_mode": color_mode,
                "is_dlog": is_dlog,
                "res_match": res_match,
                "fps_match": fps_match,
                "audio_ok": not audio_missing,
                "warnings": warnings,
                "reason": " • ".join(reason_parts),
                "suggested_project_name": suggested_name,
                "paths": paths,
                "files": {
                    ck: {
                        "name": group[ck]["name"],
                        "path": group[ck]["path"],
                        "size_mb": group[ck]["size_mb"],
                        "mtime_str": group[ck]["mtime_str"],
                        "duration_sec": round(float(probed_metas.get(ck, {}).get("duration_sec") or 0.0), 1),
                        "fps": float(probed_metas.get(ck, {}).get("fps") or 0.0),
                        "color_mode": probed_metas.get(ck, {}).get("color_mode") or "-",
                    }
                    for ck in cam_order
                    if ck in group
                },
            }
        )

    candidates.sort(key=lambda c: (-c["score"], c["rank"]))
    for idx, cand in enumerate(candidates):
        cand["recommended"] = idx == 0
        prefix = "★ FÖRSLAG: " if idx == 0 else f"#{idx + 1}: "
        meta_bits = [f"{cand['cams_present']}/3 kam"]
        if cand["duration_sec"] > 0:
            meta_bits.append(cand["duration_str"])
        if cand["resolution"] != "-":
            meta_bits.append(cand["resolution"])
        if cand["fps"] > 0:
            meta_bits.append(f"{cand['fps']:g} fps")
        if cand["color_mode"] != "-":
            meta_bits.append(cand["color_mode"])
        cand["label"] = f"{prefix}{cand['name']} ({' • '.join(meta_bits)}) • {cand['mtime_str']}"

    rec_cand = candidates[0] if candidates else None
    return {
        "connected_count": connected_count,
        "cameras": cameras,
        "unmarked_dcim": list(sd_status.get("_unmarked_dcim") or []),
        "pairing_mode": pairing_mode,
        "candidates": candidates,
        "recommended_id": rec_cand["id"] if rec_cand else None,
        "suggested_project_name": (
            partial_calib_session["name"]
            if partial_calib_session
            else (rec_cand["suggested_project_name"] if rec_cand else f"Calib_Rig_{time.strftime('%Y%m%d_%H%M')}")
        ),
        "partial_calib_session": partial_calib_session,
        "existing_sessions_map": existing_sessions_map,
    }


def get_python_executable() -> str:
    candidates = [
        APP_ROOT / ".venv" / "Scripts" / "python.exe",
        APP_ROOT / ".venv" / "bin" / "python",
        Path(r"D:\OptiGSRigg\.venv\Scripts\python.exe"),
    ]
    for cand in candidates:
        if cand.is_file():
            return str(cand)
    return sys.executable


# Snabb cache för ffprobe-metadata så att man kan växla mellan klipp utan fördröjning
_FFPROBE_CACHE: dict[tuple[str, float, int], dict] = {}


def probe_video_metadata(video_path: Path) -> dict:
    """
    Läser ut fullständig teknisk metadata & EXIF-data ur en MP4/MOV-fil via ffprobe.
    """
    if not video_path or not video_path.exists() or not video_path.is_file():
        return {"exists": False, "path": str(video_path) if video_path else ""}

    st = video_path.stat()
    cache_key = (str(video_path), round(st.st_mtime, 2), st.st_size)
    if cache_key in _FFPROBE_CACHE:
        return _FFPROBE_CACHE[cache_key]

    info = {
        "exists": True,
        "name": video_path.name,
        "path": str(video_path),
        "size_bytes": st.st_size,
        "size_mb": round(st.st_size / (1024 * 1024), 1),
        "mtime_str": datetime.datetime.fromtimestamp(st.st_mtime).strftime("%Y-%m-%d %H:%M:%S"),
        "encoder": "-",
        "creation_time": "-",
        "timecode": "-",
        "codec": "-",
        "profile": "-",
        "codec_tag": "-",
        "sensor_width": 0,
        "sensor_height": 0,
        "rotation": 0,
        "effective_width": 0,
        "effective_height": 0,
        "aspect_ratio": "-",
        "fps": 0.0,
        "fps_raw": "-",
        "nb_frames": 0,
        "duration_sec": 0.0,
        "duration_str": "-",
        "pix_fmt": "-",
        "bit_depth": "-",
        "color_space": "-",
        "color_primaries": "-",
        "color_transfer": "-",
        "color_range": "-",
        "color_mode": "-",
        "is_dlog": False,
        "video_bitrate_mbps": 0.0,
        "total_bitrate_mbps": 0.0,
        "audio_codec": "Inget ljud",
        "audio_sample_rate": "-",
        "audio_channels": 0,
        "audio_bitrate_kbps": 0,
        "dji_streams": [],
        "has_embedded_cover": False,
        "shutter_speed": "-",
        "shutter_denom": 0,
        "iso_str": "-",
        "iso_val": 0,
        "white_balance": "-",
        "wb_kelvin": 0,
        "aperture_fov": "-",
        "exposure_ev": "-",
        "sharpness_denoise": "-",
        "sensor_hw": "-",
    }

    try:
        cmd = [
            "ffprobe",
            "-v",
            "quiet",
            "-print_format",
            "json",
            "-show_format",
            "-show_streams",
            str(video_path),
        ]
        raw_out = subprocess.check_output(cmd, timeout=6.0)
        data = json.loads(raw_out.decode("utf-8", errors="replace"))

        fmt = data.get("format", {})
        fmt_tags = fmt.get("tags", {}) or {}
        info["encoder"] = fmt_tags.get("encoder") or fmt_tags.get("com.apple.quicktime.make") or "DJI OsmoAction4"
        raw_ct = fmt_tags.get("creation_time") or ""
        if raw_ct:
            info["creation_time"] = raw_ct.replace("T", " ").replace(".000000Z", " UTC")
        else:
            info["creation_time"] = info["mtime_str"]

        info["timecode"] = fmt_tags.get("timecode") or "-"
        if fmt.get("bit_rate"):
            info["total_bitrate_mbps"] = round(int(fmt["bit_rate"]) / 1_000_000, 2)
        if fmt.get("duration"):
            dur = float(fmt["duration"])
            info["duration_sec"] = round(dur, 2)
            mins = int(dur // 60)
            secs = dur % 60
            info["duration_str"] = f"{mins}m {secs:04.1f}s" if mins > 0 else f"{secs:.2f} s"

        streams = data.get("streams", [])
        for s in streams:
            ctype = s.get("codec_type")
            cname = s.get("codec_name", "")
            if ctype == "video":
                if s.get("disposition", {}).get("attached_pic") == 1 or cname == "mjpeg":
                    info["has_embedded_cover"] = True
                    continue
                info["codec"] = cname.upper()
                info["profile"] = s.get("profile", "-")
                info["codec_tag"] = s.get("codec_tag_string", "-")
                w = int(s.get("width") or 0)
                h = int(s.get("height") or 0)
                info["sensor_width"] = w
                info["sensor_height"] = h

                rot = 0
                for sd in s.get("side_data_list", []) or []:
                    if "rotation" in sd:
                        rot = int(sd["rotation"])
                info["rotation"] = rot

                if abs(rot) in (90, 270):
                    ew, eh = h, w
                else:
                    ew, eh = w, h
                info["effective_width"] = ew
                info["effective_height"] = eh

                if ew > 0 and eh > 0:
                    from math import gcd

                    g = gcd(ew, eh)
                    info["aspect_ratio"] = f"{ew // g}:{eh // g}"

                r_fps = s.get("r_frame_rate") or s.get("avg_frame_rate") or "0/1"
                info["fps_raw"] = r_fps
                if "/" in r_fps:
                    num, den = r_fps.split("/", 1)
                    if float(den) > 0:
                        info["fps"] = round(float(num) / float(den), 2)
                info["nb_frames"] = int(s.get("nb_frames") or 0)
                pix = s.get("pix_fmt") or "-"
                info["pix_fmt"] = pix
                info["bit_depth"] = "10-bit" if "10" in pix else ("12-bit" if "12" in pix else "8-bit")
                info["color_space"] = s.get("color_space") or "-"
                info["color_primaries"] = s.get("color_primaries") or "-"
                info["color_transfer"] = s.get("color_transfer") or "-"
                info["color_range"] = s.get("color_range") or "-"
                if s.get("bit_rate"):
                    info["video_bitrate_mbps"] = round(int(s["bit_rate"]) / 1_000_000, 2)
                s_tags = s.get("tags", {}) or {}
                if info["timecode"] == "-" and s_tags.get("timecode"):
                    info["timecode"] = s_tags["timecode"]

            elif ctype == "audio":
                info["audio_codec"] = f"{cname.upper()} ({s.get('profile', '')})".strip()
                info["audio_sample_rate"] = f"{s.get('sample_rate', '-')} Hz"
                info["audio_channels"] = int(s.get("channels") or 0)
                if s.get("bit_rate"):
                    info["audio_bitrate_kbps"] = round(int(s["bit_rate"]) / 1000)

            elif ctype == "data":
                tag_str = (s.get("codec_tag_string") or "").strip()
                handler = ((s.get("tags") or {}).get("handler_name") or "").strip()
                label = f"{tag_str} ({handler})" if (tag_str and handler) else (tag_str or handler)
                if label:
                    info["dji_streams"].append(label)

        # Läs inbäddad DJI-kamerameta (djmd / dvtm_ac203.proto & dbgi / dbginfo_ac203.proto)
        # direkt ur de första 24 KB av MP4-filen (slutarhastighet, ISO, vitbalans, bländare, skärpa, SN)
        _parse_dji_camera_exif(video_path, info)
        _resolve_video_color_mode(video_path, info, fmt_tags, s_tags)

    except Exception as e:
        info["probe_error"] = str(e)

    info.pop("_dji_color_flag", None)
    _FFPROBE_CACHE[cache_key] = info
    return info


def _read_pb_varint(data: bytes, i: int):
    val = 0
    shift = 0
    n = len(data)
    while i < n:
        b = data[i]
        i += 1
        val |= (b & 0x7F) << shift
        if not (b & 0x80):
            return val, i
        shift += 7
        if shift > 63:
            return None, i
    return None, i


def _parse_pb_tree(data: bytes, depth: int = 0) -> dict:
    import struct as _st

    i = 0
    n = len(data)
    res: dict = {}
    while i < n:
        tag, i = _read_pb_varint(data, i)
        if tag is None:
            break
        fn = tag >> 3
        wt = tag & 0x7
        if fn == 0 or fn > 500:
            break
        if wt == 0:
            val, i = _read_pb_varint(data, i)
            if val is None:
                break
            res.setdefault(fn, []).append(val)
        elif wt == 1:
            if i + 8 > n:
                break
            res.setdefault(fn, []).append(_st.unpack("<d", data[i : i + 8])[0])
            i += 8
        elif wt == 2:
            ln, i = _read_pb_varint(data, i)
            if ln is None or ln < 0 or i + ln > n:
                break
            sub = data[i : i + ln]
            i += ln
            try:
                s = sub.decode("utf-8")
                if len(s) > 0 and all(32 <= ord(c) < 127 for c in s):
                    res.setdefault(fn, []).append(s)
                    continue
            except Exception:
                pass
            if depth < 5 and len(sub) > 0:
                sub_d = _parse_pb_tree(sub, depth + 1)
                if sub_d:
                    res.setdefault(fn, []).append(sub_d)
                    continue
            res.setdefault(fn, []).append(sub)
        elif wt == 5:
            if i + 4 > n:
                break
            res.setdefault(fn, []).append(round(_st.unpack("<f", data[i : i + 4])[0], 4))
            i += 4
        else:
            break
    return res


def _read_dji_udta_dbcm(video_path: Path):
    """
    Läser DJI:s färgprofil-flagga ur MP4 ISO-BMFF-boxen moov -> udta -> dbcm (uint32 BE).
    Kända Action 4 / Osmo-värden: 4 = D-Log M, 2 = D-Log, 3 = HLG, 1 = D-Cinelike, 0 = Normal.
    """
    try:
        file_size = video_path.stat().st_size

        def _iter_boxes(f, start: int, end: int):
            pos = start
            while pos + 8 <= end:
                f.seek(pos)
                hdr = f.read(8)
                if len(hdr) < 8:
                    break
                size, btype = struct.unpack(">I4s", hdr)
                hdr_len = 8
                if size == 1:
                    ext = f.read(8)
                    if len(ext) < 8:
                        break
                    size = struct.unpack(">Q", ext)[0]
                    hdr_len = 16
                elif size == 0:
                    size = end - pos
                if size < hdr_len or pos + size > end:
                    break
                yield btype, pos + hdr_len, pos + size
                pos += size

        with open(video_path, "rb") as f:
            for btype, b_start, b_end in _iter_boxes(f, 0, file_size):
                if btype == b"moov":
                    for mtype, m_start, m_end in _iter_boxes(f, b_start, b_end):
                        if mtype == b"udta":
                            for utype, u_start, u_end in _iter_boxes(f, m_start, m_end):
                                if utype == b"dbcm" and (u_end - u_start) >= 4:
                                    f.seek(u_start)
                                    raw = f.read(4)
                                    if len(raw) == 4:
                                        return struct.unpack(">I", raw)[0]
                    return None
    except Exception:
        return None
    return None


def _resolve_video_color_mode(video_path: Path, info: dict, fmt_tags: dict, s_tags: dict):
    """
    Fastställer inspelningens färgläge (D-Log M / D-Log / HLG / D-Cinelike / Normal) och is_dlog-flagga
    i strikt prioritetsordning: texttaggar -> HDR transfer -> 8-bit Normal -> udta.dbcm -> djmd -> fallback.
    """
    bit_depth = info.get("bit_depth", "-")
    if bit_depth == "-" and info.get("pix_fmt", "-") == "-":
        info["color_mode"] = "-"
        info["is_dlog"] = False
        return

    is_10bit = bit_depth in ("10-bit", "12-bit")
    tag_parts = []
    for d in (fmt_tags or {}, s_tags or {}):
        for k, v in d.items():
            tag_parts.append(f"{k}={v}")
    tag_blob = " ".join(tag_parts).lower()

    # 1. Texttaggar i container/stream
    if any(tok in tag_blob for tok in ("d-log m", "dlog-m", "dlog_m", "dlogm")):
        info["color_mode"] = f"D-Log M ({bit_depth})" if bit_depth != "-" else "D-Log M"
        info["is_dlog"] = True
        return
    if any(tok in tag_blob for tok in ("d-log", "dlog")):
        info["color_mode"] = f"D-Log ({bit_depth})" if bit_depth != "-" else "D-Log"
        info["is_dlog"] = True
        return
    if any(tok in tag_blob for tok in ("d-cinelike", "cinelike")):
        info["color_mode"] = f"D-Cinelike ({bit_depth})" if bit_depth != "-" else "D-Cinelike"
        info["is_dlog"] = False
        return
    if "hlg" in tag_blob:
        info["color_mode"] = "HLG (HDR)"
        info["is_dlog"] = False
        return

    # 2. HDR transfer
    transfer = (info.get("color_transfer") or "").lower()
    if transfer == "arib-std-b67":
        info["color_mode"] = "HLG (10-bit HDR)"
        info["is_dlog"] = False
        return
    if transfer == "smpte2084":
        info["color_mode"] = "PQ / HDR10 (10-bit)"
        info["is_dlog"] = False
        return

    # 3. 8-bit Normal
    if bit_depth == "8-bit":
        info["color_mode"] = "Normal (8-bit Rec.709)"
        info["is_dlog"] = False
        return

    # 4. ISO-BMFF moov -> udta -> dbcm
    dbcm = _read_dji_udta_dbcm(video_path)
    dbcm_map = {
        4: (f"D-Log M ({bit_depth})" if is_10bit else "D-Log M", True),
        2: (f"D-Log ({bit_depth})" if is_10bit else "D-Log", True),
        3: ("HLG (10-bit HDR)" if is_10bit else "HLG", False),
        1: (f"D-Cinelike ({bit_depth})" if is_10bit else "D-Cinelike", False),
        0: ("Normal (10-bit Rec.709)" if is_10bit else "Normal (8-bit Rec.709)", False),
    }
    if dbcm in dbcm_map:
        info["color_mode"], info["is_dlog"] = dbcm_map[dbcm]
        return

    # 5. djmd protobuf fallback (d[2][0].get(5))
    dji_flag = info.get("_dji_color_flag")
    if dji_flag == 1 and is_10bit:
        info["color_mode"] = "D-Log M (10-bit)"
        info["is_dlog"] = True
        return

    # 6. Fallback
    info["color_mode"] = "Normal (10-bit Rec.709)" if is_10bit else "Normal"
    info["is_dlog"] = False


def _parse_dji_camera_exif(video_path: Path, info: dict):
    """
    Läser DJI Osmo Action 4 kameratelemetri (djmd / dvtm_ac203.proto & dbgi / dbginfo_ac203.proto)
    direkt ur de första 24 KB av MP4-filen (< 0.5 ms).
    Extraherar Slutartid, ISO, Sensor Gain, Vitbalans (K), Bländare (f/2.8), FOV/EIS,
    Skärpa/Brusreducering, Exponering (EV / Scen-LV) samt Sensor/Serienummer/Firmware.
    """
    try:
        with open(video_path, "rb") as f:
            buf = f.read(24576)
        mdat_pos = buf.find(b"mdat")
        if mdat_pos < 0:
            return
        d = _parse_pb_tree(buf[mdat_pos + 4 :])
        if not d:
            return

        # 1. Hårdvara, serienummer & firmware (från djmd f1[0] och dbgi f1[1])
        sn = ""
        fw = ""
        sensor_name = "OV48C40"
        sensor_mode = ""
        if 1 in d and len(d[1]) >= 1 and isinstance(d[1][0], dict):
            hdr = d[1][0].get(1, [{}])[0]
            if isinstance(hdr, dict):
                sn = str(hdr.get(5, [""])[0] or "")
                fw = str(hdr.get(6, [""])[0] or "")
        if 1 in d and len(d[1]) >= 2 and isinstance(d[1][1], dict):
            s_node = d[1][1].get(2, [{}])[0]
            if isinstance(s_node, dict):
                s_inner = s_node.get(1, [{}])[0]
                if isinstance(s_inner, dict) and s_inner.get(1):
                    sensor_name = str(s_inner[1][0])

        # 2. Videoläge, FOV & EIS (från djmd f2[0])
        fov_label = "Wide (155°/184° Fisheye)"
        eis_label = "EIS: AV"
        if 2 in d and len(d[2]) >= 1 and isinstance(d[2][0], dict):
            vcfg = d[2][0].get(3, [{}])[0]
            if isinstance(vcfg, dict):
                fov_code = vcfg.get(6, [4])[0]
                if fov_code == 4:
                    fov_label = "Wide (155°/184° Fisheye)"
                elif fov_code == 2:
                    fov_label = "Standard (Dewarp)"
                elif fov_code == 3:
                    fov_label = "Ultra Wide"
            eis_cfg = d[2][0].get(4, [{}])[0]
            if isinstance(eis_cfg, dict):
                eis_code = eis_cfg.get(1, [1])[0]
                eis_label = "RockSteady: AV (1:1)" if eis_code == 1 else f"EIS Aktiv (Läge {eis_code})"

        # 3. ISP / AE-register (från dbgi f2[1])
        shutter_us = 0.0
        sensor_gain = 0.0
        iso_dbgi = 0
        base_iso = 100
        f_num = 2.8
        scene_lv = None
        if 2 in d and len(d[2]) >= 2 and isinstance(d[2][1], dict):
            isp = d[2][1].get(1, [{}])[0]
            if isinstance(isp, dict):
                m_node = isp.get(2, [{}])[0]
                if isinstance(m_node, dict) and m_node.get(2):
                    sensor_mode = str(m_node[2][0])
                ae_node = isp.get(4, [{}])[0]
                if isinstance(ae_node, dict):
                    ae200 = ae_node.get(200, [{}])[0]
                    if isinstance(ae200, dict):
                        shutter_us = float(ae200.get(2, [0.0])[0] or 0.0)
                        sensor_gain = float(ae200.get(3, [0.0])[0] or 0.0)
                    ae201 = ae_node.get(201, [{}])[0]
                    if isinstance(ae201, dict):
                        base_iso = int(ae201.get(8, [100])[0] or 100)
                        iso_dbgi = int(ae201.get(10, [0])[0] or 0)
                        if ae201.get(11):
                            shutter_us = float(ae201[11][0] or shutter_us)
                        if ae201.get(12):
                            f_num = float(ae201[12][0] or 2.8)
                        if ae201.get(23) is not None:
                            scene_lv = float(ae201[23][0])

        # 4. Bildruta-EXIF (från djmd f3[0] -> f2)
        shutter_num = 1
        shutter_den = 0
        iso_djmd = 0
        wb_k = 0
        wb_mode = 4
        sharpness = 0
        denoise = 0
        if 3 in d and len(d[3]) >= 1 and isinstance(d[3][0], dict):
            exp = d[3][0].get(2, [{}])[0]
            if isinstance(exp, dict):
                n3 = exp.get(3, [{}])[0]
                if isinstance(n3, dict) and n3.get(1):
                    iso_djmd = int(round(float(n3[1][0])))
                n4 = exp.get(4, [{}])[0]
                if isinstance(n4, dict) and n4.get(1):
                    raw_sh = n4[1][0]
                    if isinstance(raw_sh, (bytes, bytearray)) and len(raw_sh) >= 2:
                        v1, idx2 = _read_pb_varint(raw_sh, 0)
                        v2, _ = _read_pb_varint(raw_sh, idx2)
                        if v1 and v2:
                            shutter_num = int(v1)
                            shutter_den = int(v2)
                n6 = exp.get(6, [{}])[0]
                if isinstance(n6, dict) and n6.get(1):
                    wb_k = int(n6[1][0])
                n7 = exp.get(7, [{}])[0]
                if isinstance(n7, dict) and n7.get(1):
                    wb_mode = int(n7[1][0])
                n11 = exp.get(11, [{}])[0]
                if isinstance(n11, dict) and n11.get(1) is not None:
                    sharpness = int(round(float(n11[1][0])))
                n12 = exp.get(12, [{}])[0]
                if isinstance(n12, dict) and n12.get(1) is not None:
                    denoise = int(round(float(n12[1][0])))

        # Sammanställ formaterade EXIF-fält
        if shutter_den <= 0 and shutter_us > 0:
            shutter_den = int(round(1_000_000.0 / shutter_us))
        if shutter_den > 0:
            ms_val = (1000.0 * shutter_num / shutter_den) if shutter_us <= 0 else (shutter_us / 1000.0)
            info["shutter_denom"] = shutter_den
            info["shutter_speed"] = f"{shutter_num}/{shutter_den} s ({ms_val:.2f} ms)"

        iso_final = iso_djmd or iso_dbgi
        if iso_final > 0:
            info["iso_val"] = iso_final
            gain_part = f" • Gain {sensor_gain:.2f}x" if sensor_gain > 0 else ""
            info["iso_str"] = f"ISO {iso_final}{gain_part} (Bas {base_iso})"

        if wb_k > 0:
            info["wb_kelvin"] = wb_k
            wb_type = "Manuell / Fast K" if wb_mode == 4 else f"Auto (Läge {wb_mode})"
            info["white_balance"] = f"{wb_k} K ({wb_type})"

        info["aperture_fov"] = f"f/{f_num:.1f} • {fov_label} • {eis_label}"

        if scene_lv is not None:
            env_hint = "Utomhus / Dagsljus" if scene_lv >= 10.0 else ("Inomhus / Belysning" if scene_lv >= 5.5 else "Svagt ljus")
            info["exposure_ev"] = f"Scenljus: {scene_lv:.1f} LV ({env_hint})"

        sh_str = f"{sharpness:+d}" if sharpness != 0 else "0 (Std)"
        dn_str = f"{denoise:+d}" if denoise != 0 else "0 (Std)"
        info["sharpness_denoise"] = f"Skärpa: {sh_str} • Brusred: {dn_str}"

        dcg_tag = " (DCG 10-bit)" if "DCG" in sensor_mode else (" (BIN2)" if "BIN2" in sensor_mode else "")
        sn_part = f" • SN: {sn}" if sn else ""
        fw_part = f" • FW: {fw}" if fw else ""
        info["sensor_hw"] = f"{sensor_name}{dcg_tag}{sn_part}{fw_part}"

        if isinstance(d.get(2), list) and d[2] and isinstance(d[2][0], dict):
            raw5 = d[2][0].get(5)
            if isinstance(raw5, list) and raw5:
                v5 = raw5[0]
                if isinstance(v5, dict) and isinstance(v5.get(1), list) and v5[1]:
                    info["_dji_color_flag"] = v5[1][0]
                elif isinstance(v5, int):
                    info["_dji_color_flag"] = v5
    except Exception:
        pass


def _get_windows_desktop_name() -> str:
    if platform.system() != "Windows":
        return "Default"
    try:
        u32 = ctypes.windll.user32
        k32 = ctypes.windll.kernel32
        hdesk = u32.GetThreadDesktop(k32.GetCurrentThreadId())
        if not hdesk:
            return "Default"
        buf = ctypes.create_unicode_buffer(256)
        needed = ctypes.c_ulong(0)
        if u32.GetUserObjectInformationW(hdesk, 2, buf, ctypes.sizeof(buf), ctypes.byref(needed)) and buf.value:
            return str(buf.value)
    except Exception:
        pass
    return "Default"


class RigStudioBackend:
    def __init__(self):
        self._lock = threading.Lock()
        self._settings: dict = load_app_settings()
        self._win_desktop: str = _get_windows_desktop_name()
        self._logs: list[str] = [
            f"[System] OptiGSRigg Studio v2.0 startad på {platform.system()} ({platform.release()})",
            f"[System] Python: {sys.version.split()[0]} • Arbetsyta: {APP_ROOT}",
        ]
        if platform.system() == "Windows" and self._win_desktop.lower() != "default":
            self._logs.append(
                f"[VARNING] Servern körs på en isolerad bakgrunds-desktop ('{self._win_desktop}') "
                "i stället för Windows 'Default'! GUI-fönster och Utforskaren kommer inte att "
                "synas på skärmen. Stäng denna bakgrundsprocess och starta start_app.bat direkt i Windows."
            )
        self._proc: subprocess.Popen | None = None
        self._is_running: bool = False
        self._status_title: str = "REDO"
        self._status_detail: str = "Steg 1: Kontrollera kameror, förhandsgranska klipp och namnge ditt projekt."
        self._status_color: str = "#2ea043"
        self._progress: int = 0
        self._step_states: list[str] = ["pending"] * 10
        self._startup_refresh_pulse: int = 2
        self._cancel_copy: bool = False
        self._copy_status: dict | None = None
        self._extract_status: dict | None = None
        self._extract_t0: float = 0.0
        self._mcp_state = {
            "connected": False,
            "state": "-",
            "iteration": 0,
            "max_iterations": 30000,
            "loss": 0.0,
            "splats": 0,
            "mcp_url": self.mcp_url,
        }
        self._mcp_io_lock = threading.Lock()
        self._mcp_rpc_id = 0
        self._mcp_session_ready = False
        self._mcp_tools: dict[str, dict] = {}
        self._mcp_cached_token = ""
        self._mcp_token_mtime = -1.0
        self._mcp_token_path = ""
        self._mcp_auth_backoff_until = 0.0
        self._mcp_loaded_url = ""

        threading.Thread(target=self._mcp_poll_loop, daemon=True).start()
        threading.Thread(target=self._auto_calibrate_extracted_projects, daemon=True).start()

    @property
    def captures_root(self) -> Path:
        val = (self._settings.get("captures_root") or _DEFAULT_CAPTURES_ROOT).strip()
        return Path(val or _DEFAULT_CAPTURES_ROOT)

    @property
    def spirula_exe(self) -> str:
        return (self._settings.get("spirula_exe") or "").strip()

    @property
    def lichtfeld_exe(self) -> str:
        return (self._settings.get("lichtfeld_exe") or "").strip()

    @property
    def colmap_exe(self) -> str:
        return (self._settings.get("colmap_exe") or "colmap").strip() or "colmap"

    @property
    def mcp_url(self) -> str:
        return (self._settings.get("mcp_url") or _DEFAULT_MCP_URL).strip() or _DEFAULT_MCP_URL

    def get_settings(self, _payload: dict = None) -> dict:
        return _validate_settings_status(self._settings)

    def auto_detect_engines(self, _payload: dict = None) -> dict:
        detected = auto_find_engine_paths()
        # Behåll användarens ev. ändrade MCP-URL och extract_backend om de redan satts
        detected["mcp_url"] = self.mcp_url
        detected["extract_backend"] = self._settings.get("extract_backend", "auto")
        return _validate_settings_status(detected)

    def save_settings(self, payload: dict) -> dict:
        updated = save_app_settings(payload or {})
        cap = Path(updated["captures_root"])
        try:
            cap.mkdir(parents=True, exist_ok=True)
        except Exception:
            pass
        old_mcp_url = self.mcp_url
        with self._lock:
            self._settings = updated
            self._mcp_state["mcp_url"] = self.mcp_url
            if self.mcp_url != old_mcp_url:
                self._mcp_session_ready = False
                self._mcp_tools = {}
                self._mcp_auth_backoff_until = 0.0
        validated = _validate_settings_status(updated)
        self._append_log(
            f"[OK] [Inställningar] Sparade motorsökvägar i {SETTINGS_JSON_PATH.name} "
            f"(Spirula: {'OK' if validated['spirula_ok'] else 'SAKNAS'}, "
            f"LichtFeld: {'OK' if validated['lichtfeld_ok'] else 'SAKNAS'})"
        )
        return {"ok": True, "settings": validated}

    def _auto_calibrate_extracted_projects(self):
        time.sleep(0.5)
        try:
            cap_root = self.captures_root
            if not cap_root.exists():
                return
            for d in sorted(cap_root.iterdir(), key=lambda p: p.stat().st_mtime, reverse=True):
                if not d.is_dir():
                    continue
                proc_d = d / "processed"
                img_c2 = proc_d / "images" / "cam2_center"
                calib_f = proc_d / "calibrated_rig.json"
                needs_calib = not calib_f.exists()
                if calib_f.exists():
                    try:
                        cdata = json.loads(calib_f.read_text(encoding="utf-8"))
                        if "phys_yaw" not in cdata.get("cam1_left", {}):
                            needs_calib = True
                    except Exception:
                        needs_calib = True
                if img_c2.exists() and any(img_c2.glob("*.jpg")) and needs_calib:
                    self.run_calibrate_only({"proj_dir": str(proc_d), "baseline_mm": "300.0", "mode": "auto_calibrate"})
                    break
        except Exception:
            pass

    def _append_log(self, line: str):
        with self._lock:
            self._logs.append(line)
            if len(self._logs) > 2500:
                self._logs = self._logs[-2000:]

    def get_overview(self, _payload: dict = None) -> dict:
        sd_status = {}
        if scan_sd_cards:
            try:
                sd_status = scan_sd_cards()
            except Exception as e:
                self._append_log(f"[VARNING] SD-kortsskanning misslyckades: {e}")

        # Berika SD-status med diskutrymme (ledigt / totalt i GB) och total klippstorlek
        _enrich_sd_status_disk_usage(sd_status)

        # Bygg lista med tagningar på inkopplade SD-kort (parade över alla 3 kameror)
        sd_takes = []
        c1_clips = sd_status.get("cam1_left", {}).get("clips", [])
        c2_clips = sd_status.get("cam2_center", {}).get("clips", [])
        c3_clips = sd_status.get("cam3_right", {}).get("clips", [])
        max_sd = max(len(c1_clips), len(c2_clips), len(c3_clips))

        for idx in range(max_sd):
            from_end = idx + 1
            take_num = max_sd - idx
            c1 = c1_clips[idx] if idx < len(c1_clips) else None
            c2 = c2_clips[idx] if idx < len(c2_clips) else None
            c3 = c3_clips[idx] if idx < len(c3_clips) else None
            ref = c2 or c1 or c3
            cams_present = sum(1 for x in (c1, c2, c3) if x is not None)
            total_mb = round(sum(x["size_mb"] for x in (c1, c2, c3) if x is not None), 1)
            sd_takes.append(
                {
                    "id": f"sd_{from_end}",
                    "source": "sd",
                    "from_end": from_end,
                    "take_num": take_num,
                    "name": ref["name"] if ref else f"Take_{take_num}",
                    "mtime_str": ref["mtime_str"] if ref else "-",
                    "cams_present": cams_present,
                    "total_mb": total_mb,
                    "label": f"[SD #{take_num}] {ref['name']} ({cams_present}/3 kam • {total_mb} MB • {ref['mtime_str']})",
                    "paths": {
                        "cam1_left": c1["path"] if c1 else "",
                        "cam2_center": c2["path"] if c2 else "",
                        "cam3_right": c3["path"] if c3 else "",
                    },
                }
            )

        # Skanna befintliga sessioner/tagningar på disk i self.captures_root
        sessions = []
        cap_root = self.captures_root
        if cap_root.exists():
            for d in sorted(cap_root.iterdir(), key=lambda p: p.name, reverse=True):
                if d.is_dir() and not d.name.startswith("."):
                    raw_d = d / "raw"
                    proc_d = d / "processed"
                    if not raw_d.exists() and not proc_d.exists():
                        continue
                    p1 = raw_d / "cam1_left.mp4"
                    p2 = raw_d / "cam2_center.mp4"
                    p3 = raw_d / "cam3_right.mp4"
                    mp4s = sum(1 for p in (p1, p2, p3) if p.exists())
                    total_mb = round(sum(p.stat().st_size for p in (p1, p2, p3) if p.exists()) / (1024 * 1024), 1)
                    ref_p = p2 if p2.exists() else (p1 if p1.exists() else (p3 if p3.exists() else d))
                    mtime_str = datetime.datetime.fromtimestamp(ref_p.stat().st_mtime).strftime("%Y-%m-%d %H:%M:%S")
                    has_sfm = (proc_d / "sparse" / "0" / "points3D.bin").exists()
                    tag = f"[{mp4s}/3 MP4" + (" • 3D-SfM KLAR]" if has_sfm else "]")
                    sessions.append(
                        {
                            "id": f"disk_{d.name}",
                            "source": "disk",
                            "name": d.name,
                            "label": f"{d.name}  {tag}",
                            "session_dir": str(d),
                            "raw_dir": str(raw_d),
                            "proj_dir": str(proc_d),
                            "mp4_count": mp4s,
                            "total_mb": total_mb,
                            "mtime_str": mtime_str,
                            "has_sfm": has_sfm,
                            "paths": {
                                "cam1_left": str(p1) if p1.exists() else "",
                                "cam2_center": str(p2) if p2.exists() else "",
                                "cam3_right": str(p3) if p3.exists() else "",
                            },
                        }
                    )

        sessions.sort(key=lambda s: (s["mp4_count"] > 0 or s["has_sfm"], s["mtime_str"]), reverse=True)

        calibrated_rig = None
        if CALIB_JSON_PATH.exists():
            try:
                calibrated_rig = json.loads(CALIB_JSON_PATH.read_text(encoding="utf-8"))
            except Exception:
                pass

        os_name = "macOS" if platform.system() == "Darwin" else ("Windows" if platform.system() == "Windows" else "Linux")
        return {
            "os_label": f"{os_name} ({platform.release()})",
            "captures_root": str(self.captures_root),
            "settings": _validate_settings_status(self._settings),
            "sd_status": sd_status,
            "sd_takes": sd_takes,
            "sessions": sessions,
            "calibrated_rig": calibrated_rig,
            "default_lut_path": str(DEFAULT_LUT_PATH),
        }

    def inspect_take(self, payload: dict) -> dict:
        """
        Kör djupgående FFprobe/EXIF-inspektion på cam1_left, cam2_center och cam3_right
        för den valda tagningen (antingen på SD-kort eller lokal session på disk)
        och kontrollerar om alla 3 kameror har matchande inställningar.
        """
        paths = payload.get("paths") or {}
        meta = {}
        for cam_key in ("cam1_left", "cam2_center", "cam3_right"):
            p_str = paths.get(cam_key) or ""
            meta[cam_key] = probe_video_metadata(Path(p_str)) if p_str else {"exists": False, "path": ""}

        # Validera konsistens mellan de 3 kamerorna
        warnings = []
        existing = [ (k, m) for k, m in meta.items() if m.get("exists") ]
        if len(existing) < 3:
            missing = [k for k in ("cam1_left", "cam2_center", "cam3_right") if not meta[k].get("exists")]
            warnings.append(f"Endast {len(existing)}/3 kamerafiler tillgängliga (saknar: {', '.join(missing)}).")

        resolutions = {
            k: f"{m['effective_width']}×{m['effective_height']} ({m['aspect_ratio']})"
            for k, m in existing
            if m.get("effective_width")
        }
        if len(set(resolutions.values())) > 1:
            res_str = ", ".join(f"{k}: {v}" for k, v in resolutions.items())
            warnings.append(f"VARNING – Olika upplösning/bildförhållande mellan kamerorna: {res_str}")

        fps_vals = {k: m["fps"] for k, m in existing if m.get("fps")}
        if len(set(fps_vals.values())) > 1:
            fps_str = ", ".join(f"{k}: {v} fps" for k, v in fps_vals.items())
            warnings.append(f"VARNING – Olika bildfrekvens (FPS) mellan kamerorna: {fps_str}")

        wb_vals = {k: m["wb_kelvin"] for k, m in existing if m.get("wb_kelvin")}
        if len(set(wb_vals.values())) > 1:
            wb_str = ", ".join(f"{k}: {v}K" for k, v in wb_vals.items())
            warnings.append(f"VARNING – Olika vitbalans (Kelvin) mellan kamerorna: {wb_str}")

        sh_vals = {k: m["shutter_denom"] for k, m in existing if m.get("shutter_denom")}
        if len(set(sh_vals.values())) > 1:
            sh_str = ", ".join(f"{k}: 1/{v}s" for k, v in sh_vals.items())
            warnings.append(f"OBS – Olika slutartid mellan kamerorna: {sh_str}")

        iso_vals = {k: m["iso_val"] for k, m in existing if m.get("iso_val")}
        if len(set(iso_vals.values())) > 1:
            iso_str = ", ".join(f"{k}: ISO {v}" for k, v in iso_vals.items())
            warnings.append(f"OBS – Olika ISO mellan kamerorna: {iso_str}")

        tex_vals = {k: m["sharpness_denoise"] for k, m in existing if m.get("sharpness_denoise") and m.get("sharpness_denoise") != "-"}
        if len(set(tex_vals.values())) > 1:
            tex_str = ", ".join(f"{k}: [{v}]" for k, v in tex_vals.items())
            warnings.append(f"OBS – Olika skärpa/brusreducering mellan kamerorna: {tex_str}")

        cm_vals = {k: m["color_mode"] for k, m in existing if m.get("color_mode") and m.get("color_mode") != "-"}
        if len(set(cm_vals.values())) > 1:
            cm_str = ", ".join(f"{k}: [{v}]" for k, v in cm_vals.items())
            warnings.append(f"VARNING – Olika färgläge/LUT-profil mellan kamerorna: {cm_str}")

        durations = [m["duration_sec"] for _, m in existing if m.get("duration_sec")]
        if len(durations) >= 2 and (max(durations) - min(durations)) > 5.0:
            warnings.append(
                f"OBS – Klippens längd skiljer sig med {max(durations) - min(durations):.1f} sekunder ({min(durations):.1f}s vs {max(durations):.1f}s)."
            )

        for k, m in existing:
            if m.get("audio_channels", 0) == 0:
                warnings.append(f"VARNING – {k} saknar ljudspår! Ljudsynkronisering på klapp kräver ljud.")

        return {
            "cameras": meta,
            "warnings": warnings,
            "all_ok": len(existing) == 3 and len(warnings) == 0,
            "is_dlog": any(bool(m.get("is_dlog")) for _, m in existing),
        }

    def write_sd_id_file(self, payload: dict) -> dict:
        """
        Skapar en .id-fil (CAM0_LEFT.id, CAM1_MID.id eller CAM2_RIGHT.id) på ett omärkt SD-kort.
        """
        drive = Path(payload.get("drive") or "")
        id_file = str(payload.get("id_file") or "")
        if not drive.exists() or id_file not in ("CAM0_LEFT.id", "CAM1_MID.id", "CAM2_RIGHT.id"):
            return {"ok": False, "error": "Ogiltig enhet eller ID-fil."}
        try:
            target = drive / id_file
            target.write_text(f"OptiGSRigg Camera ID: {id_file}\n", encoding="utf-8")
            self._append_log(f"[OK] Skapade {target} på minneskortet!")
            return {"ok": True}
        except Exception as e:
            self._append_log(f"[FEL] Kunde inte skriva {id_file} till {drive}: {e}")
            return {"ok": False, "error": str(e)}

    def _start_chunked_copy_job(
        self,
        file_tasks: list[dict],
        safe_name: str,
        target_raw_dir: Path,
        target_proj_dir: Path,
        on_complete=None,
        copy_context: str = "step1",
    ):
        """
        Kopierar videofiler i 4 MB-block med realtidstelemetri (MB kopierat av totalt,
        MB/s, ETA i sekunder samt individuell status för Fil 1/3, Fil 2/3, Fil 3/3).
        """
        with self._lock:
            if self._is_running:
                self._append_log("[VARNING] En process eller kopiering pågår redan.")
                return
            self._is_running = True
            self._cancel_copy = False
            self._step_states[0] = "done"
            self._step_states[1] = "active"

            total_bytes = sum(t["size_bytes"] for t in file_tasks)
            total_mb = round(total_bytes / (1024 * 1024), 1)
            files_state = []
            for idx, t in enumerate(file_tasks, start=1):
                files_state.append(
                    {
                        "idx": idx,
                        "cam": t["cam"],
                        "cam_label": t["cam_label"],
                        "src_name": t["src"].name,
                        "dst_name": t["dst"].name,
                        "copied_mb": 0.0,
                        "size_mb": round(t["size_bytes"] / (1024 * 1024), 1),
                        "pct": 0,
                        "state": "waiting",
                    }
                )

            self._copy_status = {
                "active": True,
                "done": False,
                "error": None,
                "context": copy_context,
                "project_name": safe_name,
                "raw_dir": str(target_raw_dir),
                "proj_dir": str(target_proj_dir),
                "current_file_idx": 1,
                "total_files": len(file_tasks),
                "total_copied_mb": 0.0,
                "total_size_mb": total_mb,
                "overall_pct": 0,
                "speed_mbps": 0.0,
                "eta_sec": 0,
                "files": files_state,
            }
            self._status_title = f"KOPIERAR TILL '{safe_name}' (0 / {total_mb} MB)"
            self._status_detail = f"Förbereder kopiering av {len(file_tasks)} kamerafiler..."
            self._status_color = "#388bfd"
            self._progress = 1

        self._append_log(f"\n=== STARTAR FILKOPIERING TILL PROJEKT: {safe_name} ({total_mb} MB totalt) ===")

        def copy_worker():
            chunk_size = 4 * 1024 * 1024  # 4 MB per block
            total_bytes_all = max(1, sum(t["size_bytes"] for t in file_tasks))
            copied_bytes_all = 0
            t_start_all = time.time()

            try:
                for i, task in enumerate(file_tasks):
                    src_p: Path = task["src"]
                    dst_p: Path = task["dst"]
                    f_size = task["size_bytes"]
                    f_size_mb = round(f_size / (1024 * 1024), 1)
                    file_num = i + 1

                    with self._lock:
                        self._copy_status["current_file_idx"] = file_num
                        self._copy_status["files"][i]["state"] = "copying"

                    # Om filen redan finns med exakt samma storlek, hoppa över direkt
                    if dst_p.exists() and dst_p.stat().st_size == f_size:
                        copied_bytes_all += f_size
                        with self._lock:
                            self._copy_status["files"][i]["copied_mb"] = f_size_mb
                            self._copy_status["files"][i]["pct"] = 100
                            self._copy_status["files"][i]["state"] = "done"
                            self._copy_status["total_copied_mb"] = round(copied_bytes_all / (1024 * 1024), 1)
                            self._copy_status["overall_pct"] = int((copied_bytes_all * 100) / total_bytes_all)
                            self._progress = self._copy_status["overall_pct"]
                        self._append_log(
                            f"[Fil {file_num}/{len(file_tasks)}] {src_p.name} -> {dst_p.name} ({f_size_mb} MB) fanns redan, hoppar över."
                        )
                        continue

                    self._append_log(
                        f"[Fil {file_num}/{len(file_tasks)}] Kopierar {src_p} -> {dst_p.name} ({f_size_mb} MB)..."
                    )
                    part_p = dst_p.with_suffix(".mp4.part")
                    copied_this_file = 0
                    last_ui_update = 0.0

                    with open(src_p, "rb") as f_in, open(part_p, "wb") as f_out:
                        while True:
                            if self._cancel_copy:
                                raise RuntimeError("Kopieringen avbröts av användaren.")
                            buf = f_in.read(chunk_size)
                            if not buf:
                                break
                            f_out.write(buf)
                            n = len(buf)
                            copied_this_file += n
                            copied_bytes_all += n

                            now = time.time()
                            if (now - last_ui_update) >= 0.08 or copied_this_file >= f_size:
                                last_ui_update = now
                                elapsed = max(0.05, now - t_start_all)
                                speed_bps = copied_bytes_all / elapsed
                                speed_mbps = round(speed_bps / (1024 * 1024), 1)
                                rem_bytes = max(0, total_bytes_all - copied_bytes_all)
                                eta_s = int(rem_bytes / max(1.0, speed_bps))

                                cur_mb = round(copied_this_file / (1024 * 1024), 1)
                                tot_cur_mb = round(copied_bytes_all / (1024 * 1024), 1)
                                file_pct = int((copied_this_file * 100) / max(1, f_size))
                                all_pct = int((copied_bytes_all * 100) / total_bytes_all)

                                with self._lock:
                                    f_entry = self._copy_status["files"][i]
                                    f_entry["copied_mb"] = cur_mb
                                    f_entry["pct"] = file_pct
                                    self._copy_status["total_copied_mb"] = tot_cur_mb
                                    self._copy_status["overall_pct"] = all_pct
                                    self._copy_status["speed_mbps"] = speed_mbps
                                    self._copy_status["eta_sec"] = eta_s

                                    self._progress = all_pct
                                    self._status_title = (
                                        f"KOPIERAR FIL {file_num}/{len(file_tasks)} ({dst_p.name}): "
                                        f"{cur_mb:.0f} / {f_size_mb:.0f} MB ({file_pct}%)"
                                    )
                                    self._status_detail = (
                                        f"Totalt: {tot_cur_mb:.0f} / {total_mb:.0f} MB ({all_pct}%) • "
                                        f"Hastighet: {speed_mbps} MB/s • Kvar: ~{eta_s} s"
                                    )

                    if dst_p.exists():
                        dst_p.unlink()
                    part_p.rename(dst_p)
                    try:
                        shutil.copystat(src_p, dst_p)
                    except Exception:
                        pass

                    with self._lock:
                        self._copy_status["files"][i]["copied_mb"] = f_size_mb
                        self._copy_status["files"][i]["pct"] = 100
                        self._copy_status["files"][i]["state"] = "done"

                    self._append_log(f"[OK] Fil {file_num}/{len(file_tasks)} klar: {dst_p.name} ({f_size_mb} MB)")

                total_elapsed = max(0.1, time.time() - t_start_all)
                avg_speed = round(total_mb / total_elapsed, 1)
                with self._lock:
                    self._is_running = False
                    self._copy_status["active"] = False
                    self._copy_status["done"] = True
                    self._copy_status["overall_pct"] = 100
                    self._copy_status["eta_sec"] = 0
                    self._step_states[1] = "done"
                    self._step_states[2] = "done"
                    self._progress = 100
                    self._status_title = f"KLAR – Alla {len(file_tasks)} filer kopierade till '{safe_name}'!"
                    self._status_detail = f"Kopierade {total_mb} MB på {total_elapsed:.1f} s ({avg_speed} MB/s). Projektet är redo för Steg 2–10."
                    self._status_color = "#2ea043"

                self._append_log(
                    f"[OK] === KOPIERING KLAR: {total_mb} MB till {target_raw_dir} ({avg_speed} MB/s) ==="
                )
                if callable(on_complete):
                    try:
                        on_complete()
                    except Exception as cb_err:
                        self._append_log(f"[FEL] Efterföljande åtgärd efter kopiering misslyckades: {cb_err}")

            except Exception as e:
                # Städa bort ev. halvfärdig .part-fil
                for task in file_tasks:
                    part_p = task["dst"].with_suffix(".mp4.part")
                    if part_p.exists():
                        try:
                            part_p.unlink()
                        except Exception:
                            pass
                with self._lock:
                    self._is_running = False
                    if self._copy_status:
                        self._copy_status["active"] = False
                        self._copy_status["error"] = str(e)
                    self._status_title = f"AVBRUTEN / FEL: {e}"
                    self._status_color = "#f85149"
                self._append_log(f"[FEL] Kopiering avbruten: {e}")

        threading.Thread(target=copy_worker, daemon=True).start()

    def start_named_project(self, payload: dict) -> dict:
        """
        Skapar (eller döper om) ett projekt med användarens valda projektnamn:
          - Om källan är 'sd' (eller 'copy'): Kopierar valda kamerafiler i 4 MB-block
            med realtidsfeedback (MB kopierat av totalt, MB/s, ETA, per fil 1..3).
          - Om källan är 'disk' och mode == 'rename': Döper om befintlig sessionsmapp
            till D:\\3DGS_Captures\\<Projektnamn> och aktiverar den direkt.
        """
        raw_name = (payload.get("project_name") or "").strip()
        safe_name = re.sub(r'[<>:"/\\|?*]+', "_", raw_name).strip(" ._")
        if not safe_name:
            safe_name = f"Project_{time.strftime('%Y%m%d_%H%M%S')}"

        source_type = payload.get("source") or "sd"
        action_mode = payload.get("action_mode") or ("sd" if source_type == "sd" else "rename")
        cap_root = self.captures_root
        target_session_dir = cap_root / safe_name
        target_raw_dir = target_session_dir / "raw"
        target_proj_dir = target_session_dir / "processed"

        cam_labels = {
            "cam1_left": "Cam 1 Vänster (cam1_left.mp4)",
            "cam2_center": "Cam 2 Mitten (cam2_center.mp4)",
            "cam3_right": "Cam 3 Höger (cam3_right.mp4)",
        }

        if source_type == "sd" or action_mode == "copy":
            paths_dict = payload.get("paths") or {}
            # Om paths inte skickades med, slå upp från SD-kort eller session_dir
            if not any(paths_dict.values()):
                if source_type == "sd":
                    take_from_end = int(payload.get("from_end") or 1)
                    sd_st = scan_sd_cards()
                    for ck in ("cam1_left", "cam2_center", "cam3_right"):
                        clips = sd_st.get(ck, {}).get("clips", [])
                        idx = take_from_end - 1
                        if 0 <= idx < len(clips):
                            paths_dict[ck] = clips[idx]["path"]
                else:
                    old_raw = Path(payload.get("session_dir") or "") / "raw"
                    for ck in ("cam1_left", "cam2_center", "cam3_right"):
                        p = old_raw / f"{ck}.mp4"
                        if p.exists():
                            paths_dict[ck] = str(p)

            file_tasks = []
            for ck in ("cam1_left", "cam2_center", "cam3_right"):
                src_str = paths_dict.get(ck) or ""
                if src_str:
                    src_p = Path(src_str)
                    if src_p.exists() and src_p.is_file():
                        file_tasks.append(
                            {
                                "cam": ck,
                                "cam_label": cam_labels[ck],
                                "src": src_p,
                                "dst": target_raw_dir / f"{ck}.mp4",
                                "size_bytes": src_p.stat().st_size,
                            }
                        )

            if not file_tasks:
                return {"ok": False, "error": "Hittade inga källvideofiler att kopiera för vald tagning."}

            target_raw_dir.mkdir(parents=True, exist_ok=True)
            target_proj_dir.mkdir(parents=True, exist_ok=True)
            self._start_chunked_copy_job(file_tasks, safe_name, target_raw_dir, target_proj_dir)
            return {
                "ok": True,
                "copy_started": True,
                "project_name": safe_name,
                "raw_dir": str(target_raw_dir),
                "proj_dir": str(target_proj_dir),
            }

        # Källa är lokal session på disk med mode == 'rename'
        old_session_dir = Path(payload.get("session_dir") or "")
        if not old_session_dir.exists():
            return {"ok": False, "error": f"Källmappen {old_session_dir} hittades inte."}

        if old_session_dir.resolve() == target_session_dir.resolve():
            self._append_log(f"[OK] Projektet '{safe_name}' är nu valt som aktivt projekt.")
            return {
                "ok": True,
                "project_name": safe_name,
                "raw_dir": str(target_raw_dir),
                "proj_dir": str(target_proj_dir),
            }

        if target_session_dir.exists():
            return {
                "ok": False,
                "error": f"Det finns redan en projektmapp med namnet '{safe_name}' i {cap_root}.",
            }

        try:
            old_session_dir.rename(target_session_dir)
            self._append_log(f"[OK] Döpte om '{old_session_dir.name}' -> '{safe_name}' ({target_session_dir})")
            return {
                "ok": True,
                "project_name": safe_name,
                "raw_dir": str(target_raw_dir),
                "proj_dir": str(target_proj_dir),
            }
        except Exception as e:
            self._append_log(f"[FEL] Kunde inte döpa om projekt '{safe_name}': {e}")
            return {"ok": False, "error": str(e)}

    def analyze_audio_sync(self, payload: dict) -> dict:
        """
        Kör blixtsnabb (~0.2 s) FFT-ljudsynksanalys på de första 25 sekunderna av
        raw/cam1_left.mp4, raw/cam2_center.mp4 och raw/cam3_right.mp4 via ffmpeg + numpy.
        Returnerar exakt klapp-tidpunkt, millisekund-förskjutning, bildrute-offset vid vald FPS
        samt 250-punkters ljudkurvor (0..25 s) för grafisk visualisering i Steg 2.
        """
        import numpy as np

        raw_dir = Path(payload.get("raw_dir") or "")
        proj_dir = Path(payload.get("proj_dir") or "")
        fps = float(payload.get("fps") or 50.0)
        sr = 16000
        dur_sec = 25.0

        videos = {
            "cam1_left": raw_dir / "cam1_left.mp4",
            "cam2_center": raw_dir / "cam2_center.mp4",
            "cam3_right": raw_dir / "cam3_right.mp4",
        }
        missing = [k for k, p in videos.items() if not (p.exists() and p.is_file())]
        if missing:
            return {"ok": False, "error": f"Saknar videofiler i raw/: {', '.join(missing)}"}

        t0 = time.time()
        signals = {}
        envelopes = {}
        n_bins = 250  # 0.1s per punkt över 25s

        for cam_id, vpath in videos.items():
            cmd = [
                "ffmpeg",
                "-v",
                "error",
                "-t",
                str(int(dur_sec)),
                "-i",
                str(vpath),
                "-vn",
                "-ac",
                "1",
                "-ar",
                str(sr),
                "-f",
                "f32le",
                "-",
            ]
            raw_bytes = subprocess.check_output(cmd, timeout=8.0)
            arr = np.abs(np.frombuffer(raw_bytes, dtype=np.float32))
            signals[cam_id] = arr

            # Skapa 250-punkters kuvert (max-amplitud per 0.1s-fönster) för vågformsgrafen
            target_len = int(dur_sec * sr)
            if len(arr) < target_len:
                padded = np.zeros(target_len, dtype=np.float32)
                padded[: len(arr)] = arr
            else:
                padded = arr[:target_len]
            reshaped = padded.reshape(n_bins, -1)
            peak_env = reshaped.max(axis=1)
            max_v = float(peak_env.max()) if float(peak_env.max()) > 1e-6 else 1.0
            envelopes[cam_id] = [round(float(x / max_v), 3) for x in peak_env]

        ref = signals["cam2_center"]
        peak_idx = int(np.argmax(ref)) if len(ref) > 0 else 0
        clap_peak_sec = round(peak_idx / float(sr), 2)
        bg_mean = float(np.mean(ref)) if len(ref) > 0 else 1e-6
        peak_val = float(ref[peak_idx]) if len(ref) > 0 else 0.0
        snr_ratio = round(peak_val / max(1e-6, bg_mean), 1)

        lags_ms = {"cam2_center": 0.0}
        raw_offsets = {"cam2_center": 0}

        for cam_id in ("cam1_left", "cam3_right"):
            sig = signals[cam_id]
            n_fft = 1 << (len(sig) + len(ref) - 1).bit_length()
            corr = np.fft.irfft(np.fft.rfft(sig, n_fft) * np.conj(np.fft.rfft(ref, n_fft)), n_fft)
            corr = np.concatenate((corr[-(len(ref) - 1) :], corr[: len(sig)]))
            lag_samples = int(np.argmax(corr)) - (len(ref) - 1)
            lag_s = lag_samples / float(sr)
            lags_ms[cam_id] = round(lag_s * 1000.0, 1)
            raw_offsets[cam_id] = int(round(lag_s * fps))

        min_off = min(raw_offsets.values())
        norm_offsets = {k: v - min_off for k, v in raw_offsets.items()}
        elapsed_ms = int(round((time.time() - t0) * 1000))

        sync_info = {
            "ok": True,
            "fps": fps,
            "duration_analyzed_sec": dur_sec,
            "clap_peak_sec": clap_peak_sec,
            "clap_snr": snr_ratio,
            "lags_ms": lags_ms,
            "raw_frame_offsets": raw_offsets,
            "normalized_offsets": norm_offsets,
            "envelopes": envelopes,
            "elapsed_ms": elapsed_ms,
        }

        if str(proj_dir):
            try:
                proj_dir.mkdir(parents=True, exist_ok=True)
                (proj_dir / "sync_offsets.json").write_text(json.dumps(sync_info, indent=2), encoding="utf-8")
            except Exception:
                pass

        self._append_log(
            f"[Ljudsynk] Klapp detekterad vid {clap_peak_sec:.2f}s (SNR {snr_ratio}x, analys {elapsed_ms} ms) -> "
            f"cam1_left: {lags_ms['cam1_left']:+.1f} ms ({raw_offsets['cam1_left']:+d} f), "
            f"cam2_center: 0.0 ms (0 f), "
            f"cam3_right: {lags_ms['cam3_right']:+.1f} ms ({raw_offsets['cam3_right']:+d} f)"
        )
        return sync_info

    def check_files(self, payload: dict) -> dict:
        raw_dir = Path(payload.get("raw_dir") or "")
        proj_dir = Path(payload.get("proj_dir") or "")

        files_info = []
        found = 0
        raw_paths = {}
        for cam_key, name in [
            ("cam1_left", "cam1_left.mp4"),
            ("cam2_center", "cam2_center.mp4"),
            ("cam3_right", "cam3_right.mp4"),
        ]:
            p = raw_dir / name if str(raw_dir) else Path("__nonexistent__")
            if p.exists() and p.is_file():
                found += 1
                mb = round(p.stat().st_size / (1024 * 1024), 1)
                files_info.append({"name": name, "exists": True, "size_mb": mb, "path": str(p)})
                raw_paths[cam_key] = str(p)
            else:
                files_info.append({"name": name, "exists": False, "size_mb": 0.0, "path": ""})
                raw_paths[cam_key] = ""

        sfm_bin = proj_dir / "sparse" / "0" / "points3D.bin" if str(proj_dir) else Path("__nonexistent__")
        img_dir = proj_dir / "images" / "cam2_center" if str(proj_dir) else Path("__nonexistent__")
        frame_names = sorted([p.name for p in img_dir.glob("*.jpg")]) if img_dir.exists() else []
        n_imgs = len(frame_names)

        sfm_ready = sfm_bin.exists()
        all_mp4 = found == 3

        ref_mp4 = None
        for cam_key in ("cam2_center", "cam1_left", "cam3_right"):
            cand = raw_paths.get(cam_key)
            if cand and Path(cand).exists():
                ref_mp4 = Path(cand)
                break
        if ref_mp4 is None:
            for cand in raw_paths.values():
                if cand and Path(cand).exists():
                    ref_mp4 = Path(cand)
                    break

        if ref_mp4 is not None:
            cm_meta = probe_video_metadata(ref_mp4)
            color_info = {
                "color_mode": cm_meta.get("color_mode", "-"),
                "is_dlog": bool(cm_meta.get("is_dlog", False)),
                "bit_depth": cm_meta.get("bit_depth", "-"),
                "source_cam": ref_mp4.name,
            }
        else:
            color_info = {
                "color_mode": "-",
                "is_dlog": False,
                "bit_depth": "-",
                "source_cam": "",
            }

        # Läs befintlig ljudsynksanalys för detta projekt (eller kör den automatiskt på ~0.2s om alla 3 MP4 finns)
        sync_info = None
        sync_file = proj_dir / "sync_offsets.json" if str(proj_dir) else Path("__nonexistent__")
        if sync_file.exists():
            try:
                sync_info = json.loads(sync_file.read_text(encoding="utf-8"))
                sync_info["ok"] = True
            except Exception:
                sync_info = None

        if (sync_info is None or "envelopes" not in sync_info) and all_mp4 and not self._is_running:
            try:
                sync_info = self.analyze_audio_sync({"raw_dir": str(raw_dir), "proj_dir": str(proj_dir), "fps": 50.0})
            except Exception:
                pass

        # Viktigt: Returnera ENDAST kalibreringsdata för auto_calibrate om DETTA projekt faktiskt har kalibrerats!
        project_calibration = None
        proj_calib_file = proj_dir / "calibrated_rig.json" if str(proj_dir) else Path("__nonexistent__")
        if proj_calib_file.exists():
            try:
                project_calibration = json.loads(proj_calib_file.read_text(encoding="utf-8"))
            except Exception:
                project_calibration = None
        elif n_imgs > 0 and not self._is_running:
            # Om bild-trior redan är extraherade men projektet saknar calibrated_rig.json:
            # starta automatiskt den snabba (4s) Fisheye-kamerakalibreringen!
            self.run_calibrate_only({"proj_dir": str(proj_dir), "baseline_mm": "300.0", "mode": "auto_calibrate"})

        # Fasta/sparade presets om användaren aktivt väljer mode_a, mode_b eller calibrated (ChArUco / Ladda sparad)
        saved_global_calib = None
        if DJI_PROFILE_PATH.exists():
            try:
                raw_prof = json.loads(DJI_PROFILE_PATH.read_text(encoding="utf-8"))
                pc_map = raw_prof.get("per_camera") or {}
                if pc_map:
                    saved_global_calib = {}
                    for ck in ("cam1_left", "cam2_center", "cam3_right"):
                        pc = pc_map.get(ck) or {}
                        pos_m = pc.get("pos_in_ref_m") or [0.0, 0.0, 0.0]
                        tv_m = pc.get("cam_from_rig_tvec_m") or [0.0, 0.0, 0.0]
                        eul = pc.get("rel_euler_deg") or [0.0, 0.0, 0.0]
                        saved_global_calib[ck] = {
                            "tx": float(tv_m[0]),
                            "ty": float(tv_m[1]),
                            "tz": float(tv_m[2]),
                            "pos_x_m": float(pos_m[0]),
                            "pos_y_m": float(pos_m[1]),
                            "pos_z_m": float(pos_m[2]),
                            "yaw": float(eul[0]),
                            "pitch": float(eul[1]),
                            "roll": float(eul[2]),
                            "phys_yaw": float(pc.get("phys_yaw", 0.0)),
                            "phys_tilt": float(pc.get("phys_tilt", 0.0)),
                            "phys_roll": float(pc.get("phys_roll", 0.0)),
                            "spread_deg": float(pc.get("spread_deg", 0.0)),
                            "intrinsics": list(pc.get("intrinsics") or []),
                        }
            except Exception:
                saved_global_calib = None

        if saved_global_calib is None and CALIB_JSON_PATH.exists():
            try:
                saved_global_calib = json.loads(CALIB_JSON_PATH.read_text(encoding="utf-8"))
            except Exception:
                pass

        preset_calibrations = {
            "calibrated": saved_global_calib,
            "mode_a": {
                "cam1_left": {"pos_x_m": -0.150, "yaw": 46.516, "pitch": 17.587, "roll": -5.926, "phys_yaw": -45.0, "phys_tilt": -12.0, "phys_roll": 0.0},
                "cam2_center": {"pos_x_m": 0.000, "yaw": 0.0, "pitch": 0.0, "roll": 0.0, "phys_yaw": 0.0, "phys_tilt": 8.0, "phys_roll": 0.0},
                "cam3_right": {"pos_x_m": 0.150, "yaw": -46.516, "pitch": 17.587, "roll": 5.926, "phys_yaw": 45.0, "phys_tilt": -12.0, "phys_roll": 0.0},
            },
            "mode_b": {
                "cam1_left": {"pos_x_m": -0.150, "yaw": -46.516, "pitch": -17.587, "roll": -5.926, "phys_yaw": 45.0, "phys_tilt": 12.0, "phys_roll": 0.0},
                "cam2_center": {"pos_x_m": 0.000, "yaw": 0.0, "pitch": 0.0, "roll": 0.0, "phys_yaw": 0.0, "phys_tilt": -8.0, "phys_roll": 0.0},
                "cam3_right": {"pos_x_m": 0.150, "yaw": 46.516, "pitch": -17.587, "roll": 5.926, "phys_yaw": -45.0, "phys_tilt": 12.0, "phys_roll": 0.0},
            },
        }

        with self._lock:
            if not self._is_running:
                if sfm_ready:
                    for i in range(8):
                        self._step_states[i] = "done"
                elif n_imgs > 0 and all_mp4:
                    for i in range(6):
                        self._step_states[i] = "done"
                    for i in range(6, 10):
                        self._step_states[i] = "pending"
                elif all_mp4:
                    self._step_states[0] = "done"
                    self._step_states[1] = "done"
                    self._step_states[2] = "done"
                    for i in range(3, 10):
                        self._step_states[i] = "pending"
                else:
                    for i in range(2, 10):
                        self._step_states[i] = "pending"

            if sfm_ready:
                summary = f"3D-SfM & Kalibrering KLAR ({n_imgs} bild-trior / {n_imgs*3} bilder + points3D.bin)"
            elif n_imgs > 0:
                summary = f"Steg 2 Klar ({n_imgs} synkade bild-trior extraherade) – Redo för Steg 3 (Rigg-SfM & Kalibrering)"
            elif all_mp4:
                summary = "Alla 3 MP4-filer finns i raw/ – Redo för Steg 2 (Ljudsynk & Bildextraktion)"
            else:
                summary = f"Väntar på videofiler ({found}/3 MP4 i raw/)"

            return {
                "files": files_info,
                "raw_paths": raw_paths,
                "all_mp4": all_mp4,
                "sfm_ready": sfm_ready,
                "n_triplets": n_imgs,
                "frames": frame_names,
                "sync_info": sync_info,
                "project_calibration": project_calibration,
                "color_info": color_info,
                "default_lut_path": str(DEFAULT_LUT_PATH),
                "default_lut_exists": DEFAULT_LUT_PATH.exists(),
                "has_charuco_profile": DJI_PROFILE_PATH.exists(),
                "preset_calibrations": preset_calibrations,
                "summary": summary,
                "step_states": list(self._step_states),
            }

    def browse_directory(self, payload: dict) -> dict:
        cap_root_str = str(self.captures_root)
        initial = payload.get("initial_dir") or cap_root_str
        if not Path(initial).exists():
            initial = cap_root_str if Path(cap_root_str).exists() else str(APP_ROOT)
        try:
            if platform.system() == "Darwin":
                script = f'POSIX path of (choose folder with prompt "Välj mapp" default location POSIX file "{initial}")'
                res = subprocess.run(["osascript", "-e", script], capture_output=True, text=True)
                chosen = res.stdout.strip()
                return {"path": chosen} if chosen else {}
            else:
                # Kör Tkinter-mappväljaren i en egen subprocess så att den alltid
                # exekveras i en huvudtråd (undviker 'main thread is not in main loop').
                py_code = (
                    "import sys, tkinter as tk; from tkinter import filedialog; "
                    "root = tk.Tk(); root.withdraw(); root.attributes('-topmost', True); "
                    "p = filedialog.askdirectory(initialdir=sys.argv[1]); "
                    "root.destroy(); print(p if p else '')"
                )
                res = subprocess.run(
                    [get_python_executable(), "-c", py_code, str(initial)],
                    capture_output=True,
                    text=True,
                    encoding="utf-8",
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
                )
                chosen = (res.stdout or "").strip()
                if chosen:
                    return {"path": str(Path(chosen))}
                return {}
        except Exception as e:
            self._append_log(f"[VARNING] Kunde inte öppna mappväljare: {e}")
            return {}

    def browse_file(self, payload: dict) -> dict:
        filter_type = (payload.get("filter_type") or "exe").strip().lower()
        raw_init = (payload.get("initial_path") or "").strip()
        default_title = "Välj 3D-LUT (.cube)" if filter_type == "lut" else "Välj programfil"
        title = (payload.get("title") or default_title).strip()
        init_dir = str(APP_ROOT)
        if raw_init:
            p = Path(raw_init)
            if p.is_file() or p.is_dir():
                init_dir = str(p if p.is_dir() else p.parent)
            elif p.parent.exists():
                init_dir = str(p.parent)
        elif Path("D:/").exists():
            init_dir = "D:\\"

        try:
            if platform.system() == "Darwin":
                script = f'POSIX path of (choose file with prompt "{title}" default location POSIX file "{init_dir}")'
                res = subprocess.run(["osascript", "-e", script], capture_output=True, text=True)
                chosen = res.stdout.strip()
                return {"path": chosen} if chosen else {}
            else:
                ft_spec = (
                    "[('3D-LUT (.cube)', '*.cube'), ('Alla filer', '*.*')]"
                    if filter_type == "lut"
                    else "[('Körbara filer (.exe, .bat, .cmd)', '*.exe;*.bat;*.cmd'), ('Alla filer', '*.*')]"
                )
                py_code = (
                    "import sys, tkinter as tk; from tkinter import filedialog; "
                    "root = tk.Tk(); root.withdraw(); root.attributes('-topmost', True); "
                    "p = filedialog.askopenfilename(initialdir=sys.argv[1], title=sys.argv[2], "
                    f"filetypes={ft_spec}); "
                    "root.destroy(); print(p if p else '')"
                )
                res = subprocess.run(
                    [get_python_executable(), "-c", py_code, init_dir, title],
                    capture_output=True,
                    text=True,
                    encoding="utf-8",
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
                )
                chosen = (res.stdout or "").strip()
                if chosen:
                    return {"path": str(Path(chosen))}
                return {}
        except Exception as e:
            self._append_log(f"[VARNING] Kunde inte öppna filväljare: {e}")
            return {}

    def _hidden_desktop_error(self) -> str | None:
        if platform.system() == "Windows" and self._win_desktop.lower() != "default":
            return (
                f"OptiGSRigg-servern körs just nu på en isolerad bakgrunds-desktop ('{self._win_desktop}') "
                "i stället för Windows vanliga skrivbord ('Default'), så fönster som öppnas härifrån syns inte. "
                f"Starta i stället {APP_ROOT / 'start_app.bat'} direkt i Windows så öppnas fönstren synligt."
            )
        return None

    def _resolve_open_folder_target(self, raw_path: str) -> Path:
        candidate = (raw_path or "").strip()
        target = Path(candidate) if candidate else self.captures_root
        while not target.exists() and target.parent != target:
            target = target.parent
        if not target.exists():
            target = self.captures_root if self.captures_root.exists() else APP_ROOT
        if target.is_file():
            target = target.parent
        return target

    def open_folder(self, payload: dict) -> dict:
        hidden_err = self._hidden_desktop_error()
        if hidden_err:
            self._append_log(f"[FEL] [Utforskaren] {hidden_err}")
            return {"ok": False, "error": hidden_err}

        target = self._resolve_open_folder_target(str((payload or {}).get("path") or ""))
        try:
            if platform.system() == "Windows":
                norm = os.path.normpath(str(target.resolve()))
                si = subprocess.STARTUPINFO()
                si.dwFlags |= subprocess.STARTF_USESHOWWINDOW
                si.wShowWindow = 1  # SW_SHOWNORMAL
                subprocess.Popen(["explorer.exe", norm], startupinfo=si)
                self._append_log(f"[OK] [Utforskaren] Öppnade mapp: {norm}")
                return {"ok": True, "path": norm}
            elif platform.system() == "Darwin":
                subprocess.Popen(["open", str(target)])
            else:
                subprocess.Popen(["xdg-open", str(target)])
            return {"ok": True, "path": str(target)}
        except Exception as e:
            msg = f"Kunde inte öppna mapp '{target}': {e}"
            self._append_log(f"[FEL] [Utforskaren] {msg}")
            return {"ok": False, "error": msg}

    def _spawn_visible_gui(self, cmd: list[str], label: str, cwd: Path | None = None) -> subprocess.Popen:
        """
        Startar en desktop-GUI synligt på skärmen och frikopplad från konsolen.
        Loggar eventuella tidiga startfel via loggfil utan att lämna öppna pipes som kan ge BrokenPipe.
        """
        log_slug = re.sub(r"[^a-z0-9]+", "_", label.lower()).strip("_") or "gui"
        log_dir = APP_ROOT / "config"
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / f"last_{log_slug}.log"

        popen_kwargs: dict = {
            "stdin": subprocess.DEVNULL,
            "cwd": str(cwd) if (cwd and cwd.exists()) else None,
        }
        if platform.system() == "Windows":
            si = subprocess.STARTUPINFO()
            si.dwFlags |= subprocess.STARTF_USESHOWWINDOW
            si.wShowWindow = 1  # SW_SHOWNORMAL
            popen_kwargs["startupinfo"] = si
            popen_kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.DETACHED_PROCESS

        log_fp = None
        try:
            log_fp = open(log_path, "w", encoding="utf-8", errors="replace")
            popen_kwargs["stdout"] = log_fp
            popen_kwargs["stderr"] = subprocess.STDOUT
            proc = subprocess.Popen(cmd, **popen_kwargs)
        finally:
            if log_fp is not None:
                try:
                    log_fp.close()
                except Exception:
                    pass

        def _watch_early_exit():
            try:
                rc = proc.wait(timeout=1.5)
            except subprocess.TimeoutExpired:
                return
            except Exception:
                return
            if rc != 0:
                snippet = ""
                try:
                    if log_path.exists():
                        lines = [
                            ln.strip()
                            for ln in log_path.read_text(encoding="utf-8", errors="replace").splitlines()
                            if ln.strip()
                        ]
                        if lines:
                            snippet = " | ".join(lines[-4:])[:360]
                except Exception:
                    pass
                detail = f": {snippet}" if snippet else ""
                self._append_log(f"[FEL] [{label}] Processen avslutades direkt (kod {rc}){detail}")

        threading.Thread(target=_watch_early_exit, daemon=True).start()
        return proc

    def open_spirula_gui(self, payload: dict) -> dict:
        hidden_err = self._hidden_desktop_error()
        if hidden_err:
            self._append_log(f"[FEL] [Spirula GUI] {hidden_err}")
            return {"ok": False, "error": hidden_err}
        proj_dir = (payload.get("proj_dir") or "").strip()
        exe = self.spirula_exe
        if not _is_valid_exe_path(exe):
            msg = f"Hittade inte Spirula Studio på '{exe}'. Ange rätt sökväg i Inställningar."
            self._append_log(f"[FEL] {msg}")
            return {"ok": False, "error": msg}
        proj_p = Path(proj_dir) if proj_dir else None
        cmd = [exe, str(proj_p)] if (proj_p and proj_p.exists()) else [exe]
        exe_path = Path(exe).resolve()
        cwd = exe_path.parent if exe_path.exists() else None
        try:
            self._spawn_visible_gui(cmd, "Spirula GUI", cwd=cwd)
        except Exception as e:
            msg = f"Kunde inte starta Spirula Studio: {e}"
            self._append_log(f"[FEL] {msg}")
            return {"ok": False, "error": msg}
        self._append_log(f"[OK] [Spirula GUI] Öppnar {Path(exe).name}" + (f" ({proj_dir})" if len(cmd) > 1 else ""))
        return {"ok": True}

    def open_lichtfeld_gui(self, payload: dict) -> dict:
        hidden_err = self._hidden_desktop_error()
        if hidden_err:
            self._append_log(f"[FEL] [LichtFeld GUI] {hidden_err}")
            return {"ok": False, "error": hidden_err}
        proj_dir = (payload.get("proj_dir") or "").strip()
        exe = self.lichtfeld_exe
        if not _is_valid_exe_path(exe):
            msg = f"Hittade inte LichtFeld Studio på '{exe}'. Ange rätt sökväg i Inställningar."
            self._append_log(f"[FEL] {msg}")
            return {"ok": False, "error": msg}
        proj_p = Path(proj_dir) if proj_dir else None
        sfm_ok = bool(
            proj_p
            and proj_p.exists()
            and ((proj_p / "sparse" / "0" / "images.bin").exists() or (proj_p / "sparse" / "0" / "images.txt").exists())
        )
        if sfm_ok:
            out_dir = proj_p / "trained_models" / "lichtfeld"
            out_dir.mkdir(parents=True, exist_ok=True)
            cmd = [
                exe,
                "-d",
                str(proj_p),
                "-o",
                str(out_dir),
                "--output-name",
                "model_lichtfeld",
                "--strategy",
                "mrnf",
                "--gut",
                "--max-cap",
                "1500000",
            ]
            log_msg = f"[OK] [LichtFeld GUI] Öppnar LichtFeld Studio med dataset & 3DGUT (-d {proj_p} -o {out_dir})"
            tip_msg = None
        else:
            cmd = [exe]
            log_msg = f"[OK] [LichtFeld GUI] Öppnar {Path(exe).name}."
            tip_msg = (
                "[TIPS] Projektet saknar sparse/0 ännu – kör Steg 3 (3D Rigg-SfM) först om du vill ladda bild-triorna i LichtFeld Studio."
                if (proj_p and (proj_p / "images").exists())
                else None
            )
        exe_path = Path(exe).resolve()
        cwd = exe_path.parent if exe_path.exists() else None
        try:
            self._spawn_visible_gui(cmd, "LichtFeld GUI", cwd=cwd)
        except Exception as e:
            msg = f"Kunde inte starta LichtFeld Studio: {e}"
            self._append_log(f"[FEL] {msg}")
            return {"ok": False, "error": msg}
        self._append_log(log_msg)
        if tip_msg:
            self._append_log(tip_msg)
        return {"ok": True}

    def _start_subprocess(self, args: list[str], title: str):
        is_extract_job = ("--calibrate_only" not in args) and ("--skip_extract" not in args)
        with self._lock:
            if self._is_running:
                self._append_log("[VARNING] En process körs redan. Vänta eller klicka på Avbryt.")
                return
            self._is_running = True
            self._extract_t0 = time.time()
            if is_extract_job:
                self._extract_status = {
                    "active": True,
                    "done": False,
                    "error": None,
                    "phase": "Ljudsynk & förberedelse...",
                    "current_triplet": 0,
                    "total_triplets": 0,
                    "saved_images": 0,
                    "total_images": 0,
                    "pct": 0,
                    "frame_name": "-",
                    "sharpness": 0.0,
                    "eta_sec": 0,
                }
            self._status_title = f"ARBETAR: {title}"
            self._status_color = "#388bfd"
            self._progress = 6

        self._append_log(f"\n=== STARTAR: {title} ===")
        self._append_log("Kommando: " + " ".join(f'"{a}"' if " " in str(a) else str(a) for a in args))

        def worker():
            try:
                env = os.environ.copy()
                env["PYTHONIOENCODING"] = "utf-8"
                env["PYTHONUNBUFFERED"] = "1"
                self._proc = subprocess.Popen(
                    args,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    bufsize=1,
                    env=env,
                )
                for raw_line in self._proc.stdout:
                    line = raw_line.rstrip("\r\n")
                    if line.startswith("[ExtractProgress]"):
                        self._parse_progress(line)
                    else:
                        self._append_log(line)
                        self._parse_progress(line)
                rc = self._proc.wait()
                with self._lock:
                    self._is_running = False
                    if rc == 0:
                        if self._extract_status:
                            self._extract_status["active"] = False
                            self._extract_status["done"] = True
                            self._extract_status["pct"] = 100
                            self._extract_status["eta_sec"] = 0
                            self._extract_status["phase"] = "Klar"
                        self._status_title = "KLAR – Processen slutfördes utan fel"
                        self._status_color = "#2ea043"
                        self._progress = 100
                    else:
                        if self._extract_status:
                            self._extract_status["active"] = False
                            self._extract_status["error"] = f"Avslutades med kod {rc}"
                        self._status_title = f"FEL – Processen avslutades med kod {rc}"
                        self._status_color = "#f85149"
            except Exception as e:
                with self._lock:
                    self._is_running = False
                    if self._extract_status:
                        self._extract_status["active"] = False
                        self._extract_status["error"] = str(e)
                    self._status_title = f"FEL: {e}"
                    self._status_color = "#f85149"
                self._append_log(f"[FEL] {e}")

        threading.Thread(target=worker, daemon=True).start()

    def _parse_progress(self, line: str):
        if line.startswith("[ExtractProgress]"):
            # Format: [ExtractProgress] win_idx/total_windows|saved_imgs|frame_name|score
            try:
                payload = line[len("[ExtractProgress]") :].strip()
                parts = payload.split("|")
                win_str, tot_str = parts[0].split("/")
                cur_t = int(win_str)
                tot_t = max(1, int(tot_str))
                saved_imgs = int(parts[1])
                fname = parts[2]
                score = float(parts[3])
                pct = int(round((cur_t * 100.0) / tot_t))
                elapsed = max(0.1, time.time() - self._extract_t0)
                rate = cur_t / elapsed
                eta_s = int(round(max(0, tot_t - cur_t) / max(0.05, rate)))
                with self._lock:
                    if self._extract_status is None:
                        self._extract_status = {}
                    sharp_txt = f"Laplacian-skärpa: {score:.1f}" if score > 0 else "Vulkan GPU + Flertrådad"
                    phase_label = "GPU-Bildextraktion" if score <= 0 else "Laplacian Skärpefilter & Bildextraktion"
                    self._extract_status.update(
                        {
                            "active": True,
                            "done": False,
                            "error": None,
                            "phase": f"{phase_label} ({fname})",
                            "current_triplet": cur_t,
                            "total_triplets": tot_t,
                            "saved_images": saved_imgs,
                            "total_images": tot_t * 3,
                            "pct": pct,
                            "frame_name": fname,
                            "sharpness": score,
                            "eta_sec": eta_s,
                        }
                    )
                    self._progress = pct
                    self._status_title = f"EXTRAHERAR BILD-TRIO {cur_t} AV {tot_t} ({pct}%)"
                    self._status_detail = (
                        f"Sparat {saved_imgs} av {tot_t * 3} bilder ({fname}) • "
                        f"{sharp_txt} • Kvar: ~{eta_s} s"
                    )
            except Exception:
                pass
            return

        low = line.lower()
        with self._lock:
            self._status_detail = line[:125]
            if "kopierar" in low or "sd-ingest" in low:
                self._step_states[1] = "active"
                self._progress = max(self._progress, 18)
            elif "ljudsynk" in low or "korskorrelation" in low or "steg 1/3" in low:
                self._step_states[1] = "done"
                self._step_states[2] = "done"
                self._step_states[3] = "active"
                if self._extract_status:
                    self._extract_status["phase"] = "Steg 2A: FFT-ljudsynkronisering pågår..."
                self._progress = 15
            elif "laplacian" in low or "extraherar" in low or "bild-trior" in low:
                self._step_states[3] = "done"
                self._step_states[4] = "active"
                self._extract_t0 = time.time()
            elif "rig_config.json" in low:
                self._step_states[4] = "done"
                self._step_states[5] = "done"
                self._progress = 64
            elif "spirula" in low or "sfm" in low or "feature" in low or "bundle" in low:
                self._step_states[6] = "active"
                if self._extract_status:
                    self._extract_status["phase"] = "Steg 3: Kör Rigg-SfM & 3D-rekonstruktion..."
                self._progress = 78
            elif "auto-kalibrer" in low or "calibrated_rig.json" in low or "baslinje" in low or "[rigcalib]" in low:
                self._step_states[6] = "done"
                self._step_states[7] = "active"
                if self._extract_status:
                    self._extract_status["phase"] = "Steg 3: Beräknar kameravinklar (Yaw/Pitch/Roll) & 1:1 meterskala..."
                self._progress = 92
            elif "pipeline klar" in low or "alla 3 kameror" in low:
                self._step_states[1] = "done"
                self._progress = 100

    def stop_process(self, _payload: dict = None) -> dict:
        self._cancel_copy = True
        if self._proc and self._is_running:
            try:
                self._proc.terminate()
                self._append_log("[VARNING] Användaren avbröt pågående process.")
            except Exception as e:
                self._append_log(f"[FEL] Kunde inte avbryta process: {e}")
        return {"ok": True}

    def import_all_takes(self, _payload: dict = None) -> dict:
        args = [
            get_python_executable(),
            str(SRC_SCRIPT),
            "--import_all_takes",
            "--captures_root",
            str(self.captures_root),
        ]
        self._start_subprocess(args, "Steg 2: Kopierar ALLA tagningar från SD-korten")
        return {"ok": True}

    def _append_engine_bin_args(self, args: list[str]):
        if self.spirula_exe:
            args.extend(["--spirula_bin", self.spirula_exe])
        if self.lichtfeld_exe:
            args.extend(["--lichtfeld_bin", self.lichtfeld_exe])
        if self.colmap_exe:
            args.extend(["--colmap_bin", self.colmap_exe])

    def _append_lut_args(self, args: list[str], payload: dict):
        if payload.get("apply_lut"):
            raw_lut = (payload.get("lut_path") or "").strip()
            lut_p = Path(raw_lut) if raw_lut else DEFAULT_LUT_PATH
            if lut_p.exists():
                args.extend(["--lut", str(lut_p)])
            else:
                print(f"[Steg 2] Varning: Vald LUT-fil saknas ({lut_p}), extraherar utan LUT.", flush=True)

    def run_calibrate_only(self, payload: dict) -> dict:
        proj_dir = str(payload.get("proj_dir") or "")
        args = [
            get_python_executable(),
            str(SRC_SCRIPT),
            "--project_dir",
            proj_dir,
            "--mode",
            str(payload.get("mode") or "auto_calibrate"),
            "--baseline_mm",
            str(payload.get("baseline_mm") or "300.0"),
            "--calibrate_only",
        ]
        self._append_engine_bin_args(args)
        self._start_subprocess(args, "Steg 2C: Snabb Fisheye-Kamerakalibrering (Yaw/Pitch/Roll) ur bild-trior")
        return {"ok": True}

    def run_step1_sfm(self, payload: dict) -> dict:
        proj_dir = Path(str(payload.get("proj_dir") or ""))
        existing_imgs = list((proj_dir / "images" / "cam2_center").glob("*.jpg")) if proj_dir.exists() else []
        ext_backend = str(payload.get("extract_backend") or self._settings.get("extract_backend") or "auto")
        args = [
            get_python_executable(),
            str(SRC_SCRIPT),
            "--raw_dir",
            str(payload.get("raw_dir")),
            "--project_dir",
            str(proj_dir),
            "--mode",
            str(payload.get("mode") or "auto_calibrate"),
            "--baseline_mm",
            str(payload.get("baseline_mm") or "300.0"),
            "--fps",
            str(payload.get("fps") or "50.0"),
            "--window_sec",
            str(payload.get("window_sec") or "0.40"),
            "--sfm_backend",
            str(payload.get("sfm_backend") or "auto"),
            "--extract_backend",
            ext_backend,
            "--engine",
            "none",
        ]
        self._append_engine_bin_args(args)
        self._append_lut_args(args, payload)
        if existing_imgs and payload.get("skip_extract", True):
            args.append("--skip_extract")
            title = f"Steg 3: Rigg-SfM & 3D-Punktmoln (på {len(existing_imgs)} extraherade bild-trior)"
        else:
            title = "Steg 2 + 3: Ljudsynk + GPU-Bildextraktion + Rigg-SfM"
        self._start_subprocess(args, title)
        return {"ok": True}

    def run_sync_only(self, payload: dict) -> dict:
        ext_backend = str(payload.get("extract_backend") or self._settings.get("extract_backend") or "auto")
        args = [
            get_python_executable(),
            str(SRC_SCRIPT),
            "--raw_dir",
            str(payload.get("raw_dir")),
            "--project_dir",
            str(payload.get("proj_dir")),
            "--mode",
            str(payload.get("mode") or "auto_calibrate"),
            "--baseline_mm",
            str(payload.get("baseline_mm") or "300.0"),
            "--fps",
            str(payload.get("fps") or "50.0"),
            "--window_sec",
            str(payload.get("window_sec") or "0.40"),
            "--extract_backend",
            ext_backend,
            "--sfm_backend",
            "none",
            "--engine",
            "none",
        ]
        self._append_engine_bin_args(args)
        self._append_lut_args(args, payload)
        self._start_subprocess(args, "Steg 2: Klapp-Synk + GPU-Bildextraktion + Kamerakalibrering")
        return {"ok": True}

    def _parse_mcp_host_port(self) -> tuple[str, int]:
        u = urlparse(self.mcp_url)
        host = u.hostname or "127.0.0.1"
        port = u.port or (443 if u.scheme == "https" else 80)
        return host, int(port)

    def _is_mcp_port_open(self, timeout: float = 0.25) -> bool:
        try:
            host, port = self._parse_mcp_host_port()
            with socket.create_connection((host, port), timeout=timeout):
                return True
        except OSError:
            return False

    def _read_lichtfeld_mcp_token(self) -> str:
        candidates: list[Path] = []
        lfs_home = (os.environ.get("LFS_HOME") or "").strip()
        if lfs_home:
            candidates.append(Path(lfs_home).expanduser() / "mcp_token")
        home = Path.home()
        candidates.append(home / ".lichtfeld" / "mcp_token")
        candidates.append(home / ".lichtfeld" / "mcp" / "mcp_token")
        for env_key in ("LOCALAPPDATA", "APPDATA"):
            val = (os.environ.get(env_key) or "").strip()
            if val:
                candidates.append(Path(val) / "LichtFeldStudio" / "mcp_token")

        for cand in candidates:
            try:
                if cand.is_file():
                    mtime = cand.stat().st_mtime
                    cand_str = str(cand)
                    if cand_str == self._mcp_token_path and mtime == self._mcp_token_mtime and self._mcp_cached_token:
                        return self._mcp_cached_token
                    tok = cand.read_text(encoding="utf-8", errors="replace").strip()
                    if tok:
                        self._mcp_token_path = cand_str
                        self._mcp_token_mtime = mtime
                        self._mcp_cached_token = tok
                        return tok
            except OSError:
                continue
        return ""

    def _mcp_rpc(
        self,
        method: str,
        params: dict | None = None,
        *,
        is_notification: bool = False,
        timeout: float = 3.0,
    ) -> dict:
        payload: dict = {"jsonrpc": "2.0", "method": method}
        if not is_notification:
            self._mcp_rpc_id += 1
            payload["id"] = self._mcp_rpc_id
        if params is not None:
            payload["params"] = params

        headers = {
            "Content-Type": "application/json",
            "Accept": "application/json",
        }
        token = self._read_lichtfeld_mcp_token()
        if token:
            headers["Authorization"] = f"Bearer {token}"

        req = urllib.request.Request(
            self.mcp_url,
            data=json.dumps(payload).encode("utf-8"),
            headers=headers,
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                raw = resp.read().decode("utf-8", errors="replace").strip()
        except urllib.error.HTTPError as he:
            err_txt = ""
            try:
                err_txt = he.read().decode("utf-8", errors="replace").strip()
            except Exception:
                pass
            self._mcp_session_ready = False
            if he.code in (401, 403):
                self._mcp_auth_backoff_until = time.time() + 15.0
                detail = f" ({err_txt})" if err_txt else ""
                raise RuntimeError(f"MCP autentiseringsfel (HTTP {he.code}): kontrollera ~/.lichtfeld/mcp_token{detail}") from he
            detail = f": {err_txt}" if err_txt else ""
            raise RuntimeError(f"MCP HTTP {he.code}{detail}") from he

        if not raw:
            return {}

        body = json.loads(raw)
        if isinstance(body, dict) and body.get("error"):
            err = body["error"]
            msg = err.get("message", str(err)) if isinstance(err, dict) else str(err)
            if "not initialized" in msg.lower():
                self._mcp_session_ready = False
            raise RuntimeError(f"MCP RPC-fel: {msg}")
        return body if isinstance(body, dict) else {}

    def _mcp_ensure_initialized(self, timeout: float = 3.0):
        cur_url = self.mcp_url
        if self._mcp_session_ready and self._mcp_loaded_url == cur_url:
            return
        self._mcp_rpc(
            "initialize",
            {
                "protocolVersion": "2024-11-05",
                "capabilities": {},
                "clientInfo": {"name": "OptiGSRigg", "version": "2.0"},
            },
            timeout=timeout,
        )
        self._mcp_rpc("notifications/initialized", {}, is_notification=True, timeout=timeout)
        tools_resp = self._mcp_rpc("tools/list", {}, timeout=timeout)
        tools_list = tools_resp.get("result", {}).get("tools", []) if isinstance(tools_resp, dict) else []
        self._mcp_tools = {
            str(t["name"]): t
            for t in tools_list
            if isinstance(t, dict) and t.get("name")
        }
        self._mcp_session_ready = True
        self._mcp_loaded_url = cur_url
        self._mcp_auth_backoff_until = 0.0

    def _resolve_mcp_tool_name(self, preferred: str) -> str:
        candidates = (
            preferred,
            preferred.replace(".", "_"),
            preferred.replace("_", "."),
        )
        for cand in candidates:
            if cand in self._mcp_tools:
                return cand
        return preferred

    def _mcp_call(
        self,
        tool_name: str,
        arguments: dict | None = None,
        *,
        timeout: float = 5.0,
        allow_idle_errors: bool = False,
    ) -> dict:
        with self._mcp_io_lock:
            self._mcp_ensure_initialized(timeout=min(timeout, 4.0))
            actual_name = self._resolve_mcp_tool_name(tool_name)
            try:
                body = self._mcp_rpc(
                    "tools/call",
                    {"name": actual_name, "arguments": arguments or {}},
                    timeout=timeout,
                )
            except RuntimeError as e:
                if "not initialized" not in str(e).lower():
                    raise
                self._mcp_session_ready = False
                self._mcp_ensure_initialized(timeout=min(timeout, 4.0))
                actual_name = self._resolve_mcp_tool_name(tool_name)
                body = self._mcp_rpc(
                    "tools/call",
                    {"name": actual_name, "arguments": arguments or {}},
                    timeout=timeout,
                )

            result = body.get("result") or {}
            is_error = bool(result.get("isError"))
            parsed: dict = {}
            if isinstance(result.get("structuredContent"), dict):
                parsed = dict(result["structuredContent"])
            else:
                content = result.get("content") or []
                txt = ""
                if isinstance(content, list) and content and isinstance(content[0], dict):
                    txt = str(content[0].get("text") or "")
                stripped = txt.strip()
                if stripped.startswith("{") or stripped.startswith("["):
                    try:
                        loaded = json.loads(stripped)
                        parsed = loaded if isinstance(loaded, dict) else {"data": loaded}
                    except Exception:
                        parsed = {"error": txt} if is_error else {"message": txt}
                elif txt:
                    parsed = {"error": txt} if is_error else {"message": txt}

            err_msg = parsed.get("error_message") or parsed.get("error") or (parsed.get("message") if is_error else "")
            if allow_idle_errors and err_msg:
                low = str(err_msg).lower()
                if any(p in low for p in ("training system not initialized", "no model loaded", "no active trainer")):
                    return {
                        "state": "idle",
                        "iteration": 0,
                        "max_iterations": 30000,
                        "loss": 0.0,
                        "num_gaussians": 0,
                    }

            has_err_obj = isinstance(parsed.get("error"), (str, dict)) and bool(parsed.get("error"))
            if is_error or parsed.get("success") is False or has_err_obj:
                raise RuntimeError(f"{actual_name}: {err_msg or json.dumps(parsed, ensure_ascii=False)}")
            return parsed

    def start_lichtfeld_training(self, payload: dict) -> dict:
        proj = str(payload.get("proj_dir") or "").strip()
        proj_p = Path(proj) if proj else None
        sfm_ok = bool(
            proj_p
            and proj_p.exists()
            and ((proj_p / "sparse" / "0" / "images.bin").exists() or (proj_p / "sparse" / "0" / "images.txt").exists())
        )
        if not sfm_ok:
            msg = "Steg 3 (3D Rigg-SfM) måste köras först! Mappen sparse/0 saknas i projektet."
            with self._lock:
                self._status_title = "KÖR STEG 3 FÖRST (SPARSE/0 SAKNAS)"
                self._status_color = "#d29922"
            self._append_log(
                f"[VARNING] Kan inte starta Steg 4 ännu: sparse/0 saknas i '{proj}'. "
                "Kör 'Steg 3: Kör 3D Rigg-SfM' först så att 3D-punktmoln och kamerapositioner skapas!"
            )
            return {"ok": False, "error": msg}

        licht_out = proj_p / "trained_models" / "lichtfeld"
        licht_out.mkdir(parents=True, exist_ok=True)
        mcp_url = self.mcp_url

        try:
            with self._lock:
                self._step_states[8] = "active"

            if not self._is_mcp_port_open(timeout=0.35):
                exe = self.lichtfeld_exe
                if not _is_valid_exe_path(exe):
                    raise RuntimeError(
                        f"LichtFeld Studio körs inte (MCP {mcp_url} ej nåbar) och sökväg till exe saknas ({exe})."
                    )
                cmd = [
                    exe,
                    "-d",
                    str(proj_p),
                    "-o",
                    str(licht_out),
                    "--output-name",
                    "model_lichtfeld",
                    "--strategy",
                    "mrnf",
                    "--gut",
                    "--max-cap",
                    "1500000",
                    "--export",
                    "ply,spz",
                    "--train",
                ]
                exe_path = Path(exe).resolve()
                cwd = exe_path.parent if exe_path.exists() else None
                self._spawn_visible_gui(cmd, "LichtFeld 3DGUT", cwd=cwd)
                with self._lock:
                    self._step_states[8] = "done"
                    self._step_states[9] = "active"
                    self._status_title = "TRÄNAR 3DGUT I LICHTFELD STUDIO"
                    self._status_color = "#388bfd"
                self._append_log(
                    "[OK] LichtFeld Studio var inte öppet – startar LichtFeld GUI direkt med 3DGUT "
                    "(--gut --strategy mrnf --max-cap 1500000 --train)!"
                )
                self._append_log(
                    "[INFO] Vid Iteration 0 kompilerar LichtFeld 3DGUT CUDA-kärnor (~4 min första gången). "
                    "MCP ansluter automatiskt när fönstret öppnats."
                )
                return {"ok": True, "mode": "gui_autostart"}

            self._append_log(f"[MCP] Ansluter till öppen LichtFeld Studio ({mcp_url})...")
            self._mcp_call(
                "scene.load_dataset",
                {
                    "path": str(proj_p),
                    "output_path": str(licht_out),
                    "strategy": "mrnf",
                    "max_iterations": 30000,
                },
                timeout=30.0,
            )
            has_prop_set = ("property.set" in self._mcp_tools) or ("property_set" in self._mcp_tools)
            if has_prop_set:
                self._mcp_call("property.set", {"path": "training_params.gut", "value": True}, timeout=5.0)
                self._mcp_call("property.set", {"path": "training_params.max_cap", "value": 1500000}, timeout=5.0)
            self._mcp_call("training.start", {}, timeout=15.0)
            with self._lock:
                self._step_states[8] = "done"
                self._step_states[9] = "active"
                self._status_title = "TRÄNAR 3DGUT I LICHTFELD STUDIO"
                self._status_color = "#388bfd"
            self._append_log("[OK] [MCP] Projekt laddat i LichtFeld Studio (MRNF + OPENCV_FISHEYE 3DGUT) och träning startad!")
            if not has_prop_set:
                self._append_log(
                    "[INFO] [MCP] Tips: Kontrollera i LichtFeld Studio-panelen att GUT (3DGUT) är ikryssat "
                    "för OPENCV_FISHEYE (aktiveras annars direkt vid GUI-autostart)."
                )
            return {"ok": True, "mode": "mcp"}
        except Exception as e:
            with self._lock:
                self._step_states[8] = "pending"
                self._status_title = "MCP-FEL MOT LICHTFELD STUDIO"
                self._status_color = "#f85149"
            self._append_log(f"[FEL] [MCP] {e}")
            return {"ok": False, "error": str(e)}

    def _mcp_poll_loop(self):
        while True:
            try:
                port_open = self._is_mcp_port_open(timeout=0.25)
                if not port_open:
                    self._mcp_session_ready = False
                if not port_open or time.time() < self._mcp_auth_backoff_until:
                    with self._lock:
                        self._mcp_state["connected"] = False
                        self._mcp_state["mcp_url"] = self.mcp_url
                    time.sleep(2.5)
                    continue

                st = self._mcp_call("training.get_state", {}, timeout=2.0, allow_idle_errors=True)
                state = str(
                    st.get("state")
                    or st.get("trainer_state")
                    or ("running" if st.get("is_running") else ("paused" if st.get("is_paused") else "idle"))
                )
                it = int(st.get("iteration") or 0)
                max_it = int(st.get("max_iterations") or 30000)
                loss = float(st.get("loss") or 0.0)
                splats = int(st.get("num_gaussians") or 0)
                with self._lock:
                    self._mcp_state = {
                        "connected": True,
                        "state": state,
                        "iteration": it,
                        "max_iterations": max_it,
                        "loss": loss,
                        "splats": splats,
                        "mcp_url": self.mcp_url,
                    }
                    if state == "running":
                        self._step_states[8] = "done"
                        self._step_states[9] = "active"
                    elif state == "completed":
                        self._step_states[8] = "done"
                        self._step_states[9] = "done"
            except Exception:
                with self._lock:
                    self._mcp_state["connected"] = False
                    self._mcp_state["mcp_url"] = self.mcp_url
            time.sleep(2.5)

    def poll_status(self, payload: dict) -> dict:
        since_idx = int((payload or {}).get("since_index") or 0)
        with self._lock:
            new_logs = self._logs[since_idx:]
            is_run = self._is_running
            if not is_run and self._startup_refresh_pulse > 0:
                # Skicka en kort puls (True -> False) vid nystart så att ett redan öppet
                # fönster automatiskt kör refreshAll() utan att man behöver stänga det!
                is_run = self._startup_refresh_pulse == 2
                self._startup_refresh_pulse -= 1
            return {
                "new_logs": new_logs,
                "next_index": len(self._logs),
                "is_running": is_run,
                "status_title": self._status_title,
                "status_detail": self._status_detail,
                "status_color": self._status_color,
                "progress": self._progress,
                "step_states": list(self._step_states),
                "copy_status": dict(self._copy_status) if self._copy_status else None,
                "extract_status": dict(self._extract_status) if self._extract_status else None,
                "mcp": dict(self._mcp_state),
            }


    def get_charuco_status(self, _payload: dict = None) -> dict:
        boards_ok = all(
            (BOARDS_DIR / f"{b}.png").exists()
            for b in ("board_0_left", "board_1_center", "board_2_right")
        ) and (BOARDS_DIR / "printable_calibration_kit.html").exists()

        has_real = DJI_PROFILE_PATH.exists()
        has_demo = DJI_DEMO_PROFILE_PATH.exists()
        prof_file = DJI_PROFILE_PATH if has_real else (DJI_DEMO_PROFILE_PATH if has_demo else None)
        profile = None
        if prof_file and prof_file.exists():
            try:
                profile = json.loads(prof_file.read_text(encoding="utf-8"))
            except Exception:
                profile = None

        verify_file = CALIB_PREVIEW_DIR / "scene_verification.json"
        scene_ver = None
        if verify_file.exists():
            try:
                scene_ver = json.loads(verify_file.read_text(encoding="utf-8"))
            except Exception:
                scene_ver = None

        previews = {}
        for kind in ("reproj", "coverage", "epipolar"):
            previews[kind] = {
                ck: (CALIB_PREVIEW_DIR / f"{kind}_{ck}.jpg").exists()
                for ck in ("cam1_left", "cam2_center", "cam3_right")
            }

        return {
            "ok": True,
            "boards_generated": boards_ok,
            "boards_dir": str(BOARDS_DIR),
            "kit_html": str(BOARDS_DIR / "printable_calibration_kit.html"),
            "guide_path": str(CALIB_GUIDE_PATH),
            "guide_exists": CALIB_GUIDE_PATH.exists(),
            "report_html": str(CALIB_PREVIEW_DIR / "calibration_verification_report.html"),
            "report_exists": (CALIB_PREVIEW_DIR / "calibration_verification_report.html").exists(),
            "has_real_profile": has_real,
            "has_demo_profile": has_demo,
            "is_demo_profile": bool(not has_real and has_demo),
            "profile": profile,
            "scene_verification": scene_ver,
            "previews": previews,
        }

    def generate_charuco_boards(self, payload: dict = None) -> dict:
        import charuco_rig_calibration as crc

        p = payload or {}
        res = crc.generate_charuco_boards(
            out_dir=BOARDS_DIR,
            square_mm=float(p.get("square_mm") or 34.0),
            marker_mm=float(p.get("marker_mm") or 25.5),
        )
        self._append_log(f"[OK] [ChArUco] Genererade 3 unika kalibreringstavlor + A4-kit -> {BOARDS_DIR}")
        if p.get("open_kit") and not self._hidden_desktop_error():
            try:
                webbrowser.open((BOARDS_DIR / "printable_calibration_kit.html").resolve().as_uri())
            except Exception as e:
                self._append_log(f"[VARNING] [ChArUco] Kunde inte öppna webbläsare för A4-kit: {e}")
        return {"ok": True, "generated": res, **self.get_charuco_status()}

    def scan_calibration_sd_candidates(self, _payload: dict = None) -> dict:
        """Skannar anslutna kameror/SD-kort via .id-filer och föreslår kalibreringsklipp."""
        sd_status = _enrich_sd_status_disk_usage(scan_sd_cards())
        result = build_calibration_sd_candidates(sd_status, captures_root=self.captures_root)
        return {"ok": True, **result}

    def import_calibration_from_sd(self, payload: dict) -> dict:
        """
        Importerar föreslagna/valda kalibreringsklipp från anslutna SD-kort direkt till
        captures_root/<project_name>/raw/ och startar valfritt ChArUco-kalibreringen automatiskt.
        """
        p = payload or {}
        raw_name = str(p.get("project_name") or "").strip()
        safe_name = re.sub(r'[<>:"/\\|?*]+', "_", raw_name).strip(" ._")
        if not safe_name:
            safe_name = f"Calib_Rig_{time.strftime('%Y%m%d_%H%M')}"

        auto_calibrate = bool(p.get("auto_calibrate", False))
        ruler_mm = float(p.get("ruler_100mm") or 100.0)
        req_fps = float(p.get("fps") or 0.0)
        paths = p.get("paths") or {}

        with self._lock:
            if self._is_running:
                return {"ok": False, "error": "En annan process eller kopiering pågår redan. Vänta eller avbryt den först."}

        target_session_dir = self.captures_root / safe_name
        target_raw_dir = target_session_dir / "raw"
        target_proj_dir = target_session_dir / "processed"

        cam_labels = {
            "cam1_left": "Cam 1 Vänster (cam1_left.mp4)",
            "cam2_center": "Cam 2 Mitten (cam2_center.mp4)",
            "cam3_right": "Cam 3 Höger (cam3_right.mp4)",
        }

        file_tasks = []
        existing_cams: set[str] = set()
        selected_cams: set[str] = set()
        probe_candidates: list[Path] = []
        for ck in ("cam1_left", "cam2_center", "cam3_right"):
            dst_p = target_raw_dir / f"{ck}.mp4"
            if dst_p.is_file():
                existing_cams.add(ck)
                probe_candidates.append(dst_p)
            src_str = str(paths.get(ck) or "").strip()
            if src_str:
                src_p = Path(src_str)
                if src_p.is_file():
                    selected_cams.add(ck)
                    probe_candidates.insert(0, src_p)
                    file_tasks.append(
                        {
                            "cam": ck,
                            "cam_label": cam_labels[ck],
                            "src": src_p,
                            "dst": dst_p,
                            "size_bytes": src_p.stat().st_size,
                        }
                    )

        will_have_cams = sorted(existing_cams | selected_cams)
        will_have_all_3 = len(will_have_cams) == 3

        effective_fps = req_fps if req_fps > 0.0 else 0.0
        if effective_fps <= 0.0:
            for p_cand in probe_candidates:
                try:
                    meta = probe_video_metadata(p_cand)
                    if float(meta.get("fps") or 0.0) > 0.0:
                        effective_fps = float(meta["fps"])
                        break
                except Exception:
                    pass
        if effective_fps <= 0.0:
            effective_fps = 50.0

        if not file_tasks:
            if auto_calibrate and will_have_all_3:
                cal_res = self.run_charuco_calibration(
                    {
                        "calib_mode": "videos",
                        "raw_dir": str(target_raw_dir),
                        "ruler_100mm": ruler_mm,
                        "fps": effective_fps,
                        "use_audio_sync": True,
                    }
                )
                if not cal_res.get("ok"):
                    return cal_res
                return {
                    "ok": True,
                    "copy_started": False,
                    "calibration_started": True,
                    "project_name": safe_name,
                    "raw_dir": str(target_raw_dir),
                    "proj_dir": str(target_proj_dir),
                    "detected_fps": effective_fps,
                    "will_have_all_3": True,
                    "will_have_cams": will_have_cams,
                }
            return {
                "ok": False,
                "error": "Hittade inga valda kamerafiler på anslutna SD-kort att importera.",
            }

        on_complete = None
        if auto_calibrate:
            if not will_have_all_3:
                missing_cams = [c for c in ("cam1_left", "cam2_center", "cam3_right") if c not in will_have_cams]
                return {
                    "ok": False,
                    "error": (
                        f"Kan inte starta kalibrering direkt — saknar {', '.join(missing_cams)} "
                        "efter denna import. Importera de kort du har nu och koppla sedan in resterande."
                    ),
                }

            def _start_calib_after_copy():
                self._append_log(
                    f"[ChArUco] Alla 3 kalibreringsfiler klara i {target_raw_dir} — "
                    f"startar ChArUco-kalibrering ({effective_fps:.2f} fps)..."
                )
                cal_res = self.run_charuco_calibration(
                    {
                        "calib_mode": "videos",
                        "raw_dir": str(target_raw_dir),
                        "ruler_100mm": ruler_mm,
                        "fps": effective_fps,
                        "use_audio_sync": True,
                    }
                )
                if not cal_res.get("ok"):
                    self._append_log(f"[FEL] [ChArUco] Kunde inte starta kalibrering: {cal_res.get('error')}")

            on_complete = _start_calib_after_copy

        target_raw_dir.mkdir(parents=True, exist_ok=True)
        target_proj_dir.mkdir(parents=True, exist_ok=True)
        self._start_chunked_copy_job(
            file_tasks,
            safe_name,
            target_raw_dir,
            target_proj_dir,
            on_complete=on_complete,
            copy_context="calib",
        )
        return {
            "ok": True,
            "copy_started": True,
            "calibration_started": False,
            "auto_calibrate": auto_calibrate,
            "project_name": safe_name,
            "raw_dir": str(target_raw_dir),
            "proj_dir": str(target_proj_dir),
            "detected_fps": effective_fps,
            "will_have_all_3": will_have_all_3,
            "will_have_cams": will_have_cams,
        }

    def run_charuco_calibration(self, payload: dict) -> dict:
        p = payload or {}
        calib_mode = str(p.get("calib_mode") or "videos").strip().lower()

        raw_dir_str = str(p.get("raw_dir") or "").strip()
        videos = {}
        if calib_mode != "demo":
            if not raw_dir_str or not Path(raw_dir_str).exists():
                return {"ok": False, "error": f"Ogiltig eller saknad råvideomapp: '{raw_dir_str}'."}
            raw_dir = Path(raw_dir_str)
            missing = []
            for ck in ("cam1_left", "cam2_center", "cam3_right"):
                vp = raw_dir / f"{ck}.mp4"
                if not vp.is_file():
                    missing.append(f"{ck}.mp4")
                videos[ck] = vp
            if missing:
                return {"ok": False, "error": f"Saknar kalibreringsvideor i {raw_dir}: {', '.join(missing)}"}

        with self._lock:
            if self._is_running:
                return {"ok": False, "error": "En annan process körs redan. Vänta eller stoppa den först."}
            self._is_running = True
            self._status_title = (
                "KÖR SYNTETISK CHARUCO-DEMO"
                if calib_mode == "demo"
                else "KALIBRERAR RIGG MED 3 CHARUCO-TAVLOR"
            )
            self._status_detail = "Startar..."
            self._status_color = "active"
            self._progress = 10

        ruler_mm = float(p.get("ruler_100mm") or 100.0)
        fps_val = float(p.get("fps") or 50.0)
        use_sync = bool(p.get("use_audio_sync", True))

        def _worker():
            import importlib
            import traceback
            import charuco_rig_calibration as crc
            crc = importlib.reload(crc)

            def _cb(msg: str):
                self._append_log(str(msg))
                with self._lock:
                    self._status_detail = str(msg)[:125]
                    if "synk" in msg.lower() or "skannar" in msg.lower():
                        self._progress = max(self._progress, 28)
                    elif "frys-stopp" in msg.lower():
                        self._progress = max(self._progress, 42)
                    elif "lins" in msg.lower():
                        self._progress = max(self._progress, 52)
                    elif "korsvalidering" in msg.lower():
                        self._progress = max(self._progress, 72)
                    elif "global sparse bundle" in msg.lower():
                        self._progress = max(self._progress, 88)

            try:
                if calib_mode == "demo":
                    prof = crc.run_synthetic_calibration_demo(
                        output_profile_path=DJI_DEMO_PROFILE_PATH,
                        preview_dir=CALIB_PREVIEW_DIR,
                        save_compat=False,
                        progress_cb=_cb,
                    )
                    tag_txt = "Syntetisk Demo-Profil (skrev EJ över skarp profil)"
                else:
                    prof = crc.calibrate_rig_from_videos(
                        videos=videos,
                        output_profile_path=DJI_PROFILE_PATH,
                        calibrated_rig_compat_path=CALIB_JSON_PATH,
                        preview_dir=CALIB_PREVIEW_DIR,
                        ruler_100mm=ruler_mm,
                        fps=fps_val,
                        use_audio_sync=use_sync,
                        progress_cb=_cb,
                    )
                    tag_txt = "Sparad som skarp dji_rig_profile.json"

                msg_ok = (
                    f"[ChArUco KLAR] Status: {prof['overall_status']} | RMS={prof['rms_px']} px | "
                    f"Hold-out={prof['holdout_rms_px']} px | Baslinje={prof['baselines_mm']['left_to_right_mm']} mm ({tag_txt})"
                )
                self._append_log(msg_ok)
                with self._lock:
                    self._progress = 100
                    self._status_color = "ok" if prof["overall_status"] == "OK" else "warn"
                    self._status_title = f"CHARUCO KLAR ({prof['overall_status']})"
                    self._status_detail = msg_ok[:125]
            except Exception as e:
                self._append_log(f"[FEL] [ChArUco] {e}\n{traceback.format_exc()}")
                with self._lock:
                    self._status_color = "err"
                    self._status_title = "FEL VID CHARUCO-KALIBRERING"
                    self._status_detail = str(e)[:125]
            finally:
                with self._lock:
                    self._is_running = False

        threading.Thread(target=_worker, daemon=True).start()
        return {"ok": True}

    def run_verify_calibration(self, payload: dict) -> dict:
        p = payload or {}
        allow_demo = bool(p.get("allow_demo", True))
        if DJI_PROFILE_PATH.exists():
            prof_path = DJI_PROFILE_PATH
        elif allow_demo and DJI_DEMO_PROFILE_PATH.exists():
            prof_path = DJI_DEMO_PROFILE_PATH
        else:
            return {
                "ok": False,
                "error": "Ingen sparad ChArUco-profil hittades. Kör ChArUco-kalibrering i Steg 0 först!",
            }

        proj_s = str(p.get("proj_dir") or "").strip()
        raw_s = str(p.get("raw_dir") or "").strip()
        proj_dir = Path(proj_s) if proj_s else None
        raw_dir = Path(raw_s) if raw_s else None
        if not ((proj_dir and (proj_dir / "images" / "cam2_center").exists()) or (raw_dir and (raw_dir / "cam2_center.mp4").is_file())):
            return {
                "ok": False,
                "error": "Välj en session/råmapp med antingen extraherade bild-trior (processed/images) eller 3 MP4-videor.",
            }

        with self._lock:
            if self._is_running:
                return {"ok": False, "error": "En annan process körs redan."}
            self._is_running = True
            self._status_title = "KONTROLLERAR RIGG MOT AKTIV SCEN"
            self._status_detail = f"Använder profil {prof_path.name}..."
            self._status_color = "active"
            self._progress = 20

        def _worker():
            import importlib
            import traceback
            import charuco_rig_calibration as crc
            crc = importlib.reload(crc)

            def _cb(msg: str):
                self._append_log(str(msg))
                with self._lock:
                    self._status_detail = str(msg)[:125]
                    self._progress = max(self._progress, 65)

            try:
                res = crc.verify_calibration_on_scene(
                    rig_profile_path=prof_path,
                    project_dir=proj_dir,
                    raw_dir=raw_dir,
                    preview_dir=CALIB_PREVIEW_DIR,
                    progress_cb=_cb,
                )
                with self._lock:
                    self._progress = 100
                    st = res.get("status", "WARN")
                    self._status_color = "ok" if st == "OK" else ("err" if st == "FAIL" else "warn")
                    self._status_title = f"FÖRKONTROLL {st}"
                    self._status_detail = str(res.get("summary") or "")[:125]
            except Exception as e:
                self._append_log(f"[FEL] [Förkontroll] {e}\n{traceback.format_exc()}")
                with self._lock:
                    self._status_color = "err"
                    self._status_title = "FEL VID FÖRKONTROLL"
                    self._status_detail = str(e)[:125]
            finally:
                with self._lock:
                    self._is_running = False

        threading.Thread(target=_worker, daemon=True).start()
        return {"ok": True}

    def open_calib_doc(self, payload: dict) -> dict:
        kind = str((payload or {}).get("kind") or "kit").strip().lower()
        if kind == "boards_dir":
            BOARDS_DIR.mkdir(parents=True, exist_ok=True)
            return self.open_folder({"path": str(BOARDS_DIR)})
        if kind == "preview_dir":
            CALIB_PREVIEW_DIR.mkdir(parents=True, exist_ok=True)
            return self.open_folder({"path": str(CALIB_PREVIEW_DIR)})

        if kind == "kit":
            target = BOARDS_DIR / "printable_calibration_kit.html"
            if not target.exists():
                import charuco_rig_calibration as crc

                crc.generate_charuco_boards(out_dir=BOARDS_DIR)
        elif kind == "report":
            target = CALIB_PREVIEW_DIR / "calibration_verification_report.html"
            if not target.exists():
                return {"ok": False, "error": "Ingen verifieringsrapport skapad ännu. Kör Steg 0B först."}
        elif kind == "guide":
            target = CALIB_GUIDE_PATH
            if not target.exists():
                return {"ok": False, "error": f"Hittade inte guiden: {target}"}
        else:
            return {"ok": False, "error": f"Okänd dokumenttyp: {kind}"}

        hidden_err = self._hidden_desktop_error()
        if hidden_err:
            self._append_log(f"[FEL] [ChArUco] {hidden_err}")
            return {"ok": False, "error": hidden_err}

        try:
            resolved = target.resolve()
            if resolved.suffix.lower() in (".html", ".htm"):
                webbrowser.open(resolved.as_uri())
            elif platform.system() == "Windows":
                os.startfile(str(resolved))  # type: ignore[attr-defined]
            elif platform.system() == "Darwin":
                subprocess.Popen(["open", str(resolved)])
            else:
                subprocess.Popen(["xdg-open", str(resolved)])
            self._append_log(f"[OK] [ChArUco] Öppnade: {resolved}")
            return {"ok": True, "path": str(resolved)}
        except Exception as e:
            msg = f"Kunde inte öppna '{target}': {e}"
            self._append_log(f"[FEL] [ChArUco] {msg}")
            return {"ok": False, "error": msg}

    def create_desktop_shortcut(self, payload: dict) -> dict:
        if platform.system() != "Windows":
            return {"ok": False, "error": "Skrivbordsgenväg (.lnk) stöds bara på Windows. På macOS använder du start_mac.command."}

        ps_path = APP_ROOT / "create_desktop_shortcut.ps1"
        ico_path = ICONS_DIR / "optigsrigg.ico"
        if not ps_path.is_file():
            return {"ok": False, "error": f"Saknar genvägsskript: {ps_path}"}
        if not ico_path.is_file():
            try:
                import build_app_icon

                build_app_icon.build_icon_pack(out_dir=ICONS_DIR)
            except Exception as e:
                return {"ok": False, "error": f"Saknar ikonfil och kunde inte bygga ny: {e}"}

        hidden_err = self._hidden_desktop_error()
        if hidden_err:
            self._append_log(f"[FEL] [Genväg] {hidden_err}")
            return {"ok": False, "error": hidden_err}

        try:
            out = subprocess.check_output(
                [
                    "powershell",
                    "-NoProfile",
                    "-NonInteractive",
                    "-ExecutionPolicy",
                    "Bypass",
                    "-File",
                    str(ps_path),
                ],
                text=True,
                stderr=subprocess.STDOUT,
                timeout=15.0,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            ).strip()
            self._append_log(f"[OK] {out}")
            return {"ok": True, "shortcut_path": out}
        except subprocess.CalledProcessError as e:
            detail = (e.output or str(e)).strip()
            msg = f"Kunde inte skapa skrivbordsgenväg: {detail}"
            self._append_log(f"[FEL] [Genväg] {msg}")
            return {"ok": False, "error": msg}
        except Exception as e:
            msg = f"Kunde inte skapa skrivbordsgenväg: {e}"
            self._append_log(f"[FEL] [Genväg] {msg}")
            return {"ok": False, "error": msg}


BACKEND: RigStudioBackend | None = None
_BACKEND_LOCK = threading.Lock()


def get_backend() -> RigStudioBackend:
    """Returnerar (och vid behov skapar) singleton-instansen av RigStudioBackend."""
    global BACKEND
    if BACKEND is None:
        with _BACKEND_LOCK:
            if BACKEND is None:
                BACKEND = RigStudioBackend()
    return BACKEND


class StudioRequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        req_path = urlparse(self.path).path

        if req_path in ("/", "/index.html"):
            content = UI_HTML_PATH.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        elif req_path == "/favicon.ico":
            ico_path = ICONS_DIR / "optigsrigg.ico"
            if ico_path.exists() and ico_path.is_file():
                data = ico_path.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "image/x-icon")
                self.send_header("Cache-Control", "public, max-age=86400")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_response(404)
                self.end_headers()

        elif req_path in (
            "/assets/icons/optigsrigg_32.png",
            "/assets/icons/optigsrigg_64.png",
            "/assets/icons/optigsrigg_192.png",
            "/assets/icons/optigsrigg_512.png",
        ):
            png_path = ICONS_DIR / Path(req_path).name
            if png_path.exists() and png_path.is_file():
                data = png_path.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "image/png")
                self.send_header("Cache-Control", "public, max-age=86400")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_response(404)
                self.end_headers()

        elif req_path == "/assets/icons/optigsrigg.svg":
            svg_path = ICONS_DIR / "optigsrigg.svg"
            if svg_path.exists() and svg_path.is_file():
                data = svg_path.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "image/svg+xml; charset=utf-8")
                self.send_header("Cache-Control", "public, max-age=86400")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_response(404)
                self.end_headers()

        elif req_path == "/manifest.webmanifest":
            manifest = {
                "name": "OptiGSRigg Control Center",
                "short_name": "OptiGSRigg",
                "start_url": "/",
                "display": "standalone",
                "background_color": "#141517",
                "theme_color": "#222428",
                "icons": [
                    {"src": "/assets/icons/optigsrigg_32.png?v=3", "sizes": "32x32", "type": "image/png", "purpose": "any"},
                    {"src": "/assets/icons/optigsrigg_64.png?v=3", "sizes": "64x64", "type": "image/png", "purpose": "any"},
                    {"src": "/assets/icons/optigsrigg_192.png?v=3", "sizes": "192x192", "type": "image/png", "purpose": "any maskable"},
                    {"src": "/assets/icons/optigsrigg_512.png?v=3", "sizes": "512x512", "type": "image/png", "purpose": "any maskable"},
                    {"src": "/favicon.ico?v=3", "sizes": "16x16 32x32 48x48 64x64 128x128 256x256", "type": "image/x-icon"},
                ],
            }
            data = json.dumps(manifest, ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/manifest+json; charset=utf-8")
            self.send_header("Cache-Control", "public, max-age=86400")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        elif self.path.startswith("/frame?"):
            qs = parse_qs(urlparse(self.path).query)
            proj_dir = Path(qs.get("proj_dir", [""])[0])
            cam = qs.get("cam", [""])[0]
            fname = qs.get("file", [""])[0]
            if cam in ("cam1_left", "cam2_center", "cam3_right") and fname and ".." not in fname:
                img_path = proj_dir / "images" / cam / fname
                if img_path.exists() and img_path.is_file():
                    data = img_path.read_bytes()
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Cache-Control", "max-age=3600")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                    return
            self.send_response(404)
            self.end_headers()

        elif self.path.startswith("/video_preview?"):
            # Extraherar bildruta i full 4K-originalupplösning (2880x3840 / 2160x3840)
            # så att 100% 1:1 pixel-zoom alltid visar den sanna bildkvaliteten.
            qs = parse_qs(urlparse(self.path).query)
            video_path = Path(unquote(qs.get("path", [""])[0]))
            t_param = qs.get("t", ["cover"])[0]
            if video_path.exists() and video_path.is_file() and video_path.suffix.lower() in (".mp4", ".mov"):
                try:
                    sec = 0.5 if t_param == "cover" else max(0.0, float(t_param))
                    cmd = [
                        "ffmpeg",
                        "-v",
                        "error",
                        "-ss",
                        f"{sec:.2f}",
                        "-i",
                        str(video_path),
                        "-frames:v",
                        "1",
                        "-pix_fmt",
                        "yuvj420p",
                        "-q:v",
                        "2",
                        "-f",
                        "image2pipe",
                        "-vcodec",
                        "mjpeg",
                        "-",
                    ]
                    jpg_bytes = subprocess.check_output(cmd, timeout=6.0)

                    if jpg_bytes:
                        self.send_response(200)
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Cache-Control", "max-age=600")
                        self.send_header("Content-Length", str(len(jpg_bytes)))
                        self.end_headers()
                        self.wfile.write(jpg_bytes)
                        return
                except Exception:
                    pass
            self.send_response(404)
            self.end_headers()

        elif self.path.startswith("/video_stream?"):
            # Stöder HTTP 206 Partial Content (Range) för direkt uppspelning & spolning av MP4 med ljud
            qs = parse_qs(urlparse(self.path).query)
            video_path = Path(unquote(qs.get("path", [""])[0]))
            if not (video_path.exists() and video_path.is_file() and video_path.suffix.lower() in (".mp4", ".mov")):
                self.send_response(404)
                self.end_headers()
                return

            file_size = video_path.stat().st_size
            range_hdr = self.headers.get("Range")
            start = 0
            end = file_size - 1

            if range_hdr and range_hdr.startswith("bytes="):
                parts = range_hdr[6:].split("-", 1)
                if parts[0]:
                    start = int(parts[0])
                if len(parts) > 1 and parts[1]:
                    end = int(parts[1])
                end = min(end, file_size - 1)
                chunk_len = (end - start) + 1
                self.send_response(206)
                self.send_header("Content-Range", f"bytes {start}-{end}/{file_size}")
            else:
                chunk_len = file_size
                self.send_response(200)

            self.send_header("Content-Type", "video/mp4")
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("Content-Length", str(chunk_len))
            self.end_headers()

            try:
                with open(video_path, "rb") as f:
                    f.seek(start)
                    remaining = chunk_len
                    while remaining > 0:
                        buf = f.read(min(262144, remaining))
                        if not buf:
                            break
                        self.wfile.write(buf)
                        remaining -= len(buf)
            except (ConnectionResetError, BrokenPipeError):
                pass

        elif self.path.startswith("/calib_preview?"):
            qs = parse_qs(urlparse(self.path).query)
            kind = qs.get("kind", [""])[0]
            cam = qs.get("cam", [""])[0]
            if kind in ("reproj", "coverage", "epipolar") and cam in ("cam1_left", "cam2_center", "cam3_right"):
                img_path = CALIB_PREVIEW_DIR / f"{kind}_{cam}.jpg"
                if img_path.exists() and img_path.is_file():
                    data = img_path.read_bytes()
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                    return
            self.send_response(404)
            self.end_headers()

        elif self.path.startswith("/calib_board?"):
            qs = parse_qs(urlparse(self.path).query)
            board = qs.get("board", [""])[0]
            if board in ("board_0_left", "board_1_center", "board_2_right"):
                img_path = BOARDS_DIR / f"{board}.png"
                if img_path.exists() and img_path.is_file():
                    data = img_path.read_bytes()
                    self.send_response(200)
                    self.send_header("Content-Type", "image/png")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                    return
            self.send_response(404)
            self.end_headers()

        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path.startswith("/api/"):
            action = self.path[len("/api/") :]
            length = int(self.headers.get("Content-Length") or 0)
            raw_body = self.rfile.read(length) if length > 0 else b"{}"
            try:
                payload = json.loads(raw_body.decode("utf-8"))
            except Exception:
                payload = {}

            fn = getattr(get_backend(), action, None)
            if callable(fn):
                try:
                    result = fn(payload)
                    resp_bytes = json.dumps(result).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Content-Length", str(len(resp_bytes)))
                    self.end_headers()
                    self.wfile.write(resp_bytes)
                    return
                except Exception as e:
                    err_bytes = json.dumps({"error": str(e)}).encode("utf-8")
                    self.send_response(500)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.end_headers()
                    self.wfile.write(err_bytes)
                    return

        self.send_response(404)
        self.end_headers()


def find_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _apply_win32_window_icon_async(window_title_prefix: str, ico_path: Path) -> None:
    """Sätter Win32 HICON (titelrad + aktivitetsfält) med 64-bitars typning på Edge/Chrome-fönstret."""
    if platform.system() != "Windows" or not ico_path.is_file():
        return

    def _worker() -> None:
        try:
            from ctypes import wintypes

            user32 = ctypes.windll.user32
            WNDENUMPROC = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
            user32.EnumWindows.argtypes = [WNDENUMPROC, wintypes.LPARAM]
            user32.EnumWindows.restype = wintypes.BOOL
            user32.IsWindowVisible.argtypes = [wintypes.HWND]
            user32.IsWindowVisible.restype = wintypes.BOOL
            user32.GetWindowTextLengthW.argtypes = [wintypes.HWND]
            user32.GetWindowTextLengthW.restype = ctypes.c_int
            user32.GetWindowTextW.argtypes = [wintypes.HWND, wintypes.LPWSTR, ctypes.c_int]
            user32.GetWindowTextW.restype = ctypes.c_int
            user32.GetSystemMetrics.argtypes = [ctypes.c_int]
            user32.GetSystemMetrics.restype = ctypes.c_int
            user32.LoadImageW.argtypes = [
                wintypes.HINSTANCE,
                wintypes.LPCWSTR,
                wintypes.UINT,
                ctypes.c_int,
                ctypes.c_int,
                wintypes.UINT,
            ]
            user32.LoadImageW.restype = wintypes.HANDLE
            user32.SendMessageW.argtypes = [wintypes.HWND, wintypes.UINT, wintypes.WPARAM, wintypes.LPARAM]
            user32.SendMessageW.restype = ctypes.c_ssize_t

            sm_cxsm = user32.GetSystemMetrics(49) or 16
            sm_cysm = user32.GetSystemMetrics(50) or 16
            sm_cx = user32.GetSystemMetrics(11) or 32
            sm_cy = user32.GetSystemMetrics(12) or 32
            hicon_small = user32.LoadImageW(None, str(ico_path), 1, sm_cxsm, sm_cysm, 0x0010)
            hicon_big = user32.LoadImageW(None, str(ico_path), 1, sm_cx, sm_cy, 0x0010)
            if not hicon_small and not hicon_big:
                return

            WM_SETICON = 0x0080
            applied_ticks = 0
            for _ in range(25):
                matched = []

                @WNDENUMPROC
                def _cb(hwnd, _lp):
                    if user32.IsWindowVisible(hwnd):
                        n_len = user32.GetWindowTextLengthW(hwnd)
                        if n_len > 0:
                            buf = ctypes.create_unicode_buffer(n_len + 1)
                            user32.GetWindowTextW(hwnd, buf, n_len + 1)
                            if buf.value.startswith(window_title_prefix):
                                matched.append(hwnd)
                    return True

                user32.EnumWindows(_cb, 0)
                if matched:
                    for hwnd in matched:
                        if hicon_small:
                            user32.SendMessageW(hwnd, WM_SETICON, 0, int(hicon_small))
                            user32.SendMessageW(hwnd, WM_SETICON, 2, int(hicon_small))
                        if hicon_big:
                            user32.SendMessageW(hwnd, WM_SETICON, 1, int(hicon_big))
                    applied_ticks += 1
                    if applied_ticks >= 5:
                        break
                time.sleep(0.3)
        except Exception:
            pass

    threading.Thread(target=_worker, daemon=True).start()


def launch_native_app_window(url: str) -> bool:
    sys_name = platform.system()
    if sys_name == "Windows":
        candidates = [
            shutil.which("msedge"),
            r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
            r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
            shutil.which("chrome"),
            r"C:\Program Files\Google\Chrome\Application\chrome.exe",
        ]
        profile_dir = APP_ROOT / "config" / "app_window_profile"
        profile_dir.mkdir(parents=True, exist_ok=True)
        for exe in candidates:
            if exe and Path(exe).exists():
                si = subprocess.STARTUPINFO()
                si.dwFlags |= subprocess.STARTF_USESHOWWINDOW
                si.wShowWindow = 1  # SW_SHOWNORMAL
                subprocess.Popen(
                    [
                        exe,
                        f"--app={url}",
                        f"--user-data-dir={profile_dir}",
                        "--window-size=1420,900",
                        "--no-first-run",
                        "--no-default-browser-check",
                        "--disable-sync",
                        "--disable-extensions",
                    ],
                    startupinfo=si,
                )
                _apply_win32_window_icon_async("OptiGSRigg Control Center", ICONS_DIR / "optigsrigg.ico")
                return True
    elif sys_name == "Darwin":
        mac_browsers = [
            "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
            "/Applications/Brave Browser.app/Contents/MacOS/Brave Browser",
            "/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge",
        ]
        for exe in mac_browsers:
            if Path(exe).exists():
                subprocess.Popen([exe, f"--app={url}", "--window-size=1420,900"])
                return True

    webbrowser.open(url)
    return False


def _terminate_python_pid_win32(pid: int) -> bool:
    """Avslutar en tidigare Python-process via Win32 API utan att starta PowerShell."""
    if platform.system() != "Windows" or pid <= 0 or pid == os.getpid():
        return False
    try:
        from ctypes import wintypes

        kernel32 = ctypes.windll.kernel32
        kernel32.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
        kernel32.OpenProcess.restype = wintypes.HANDLE
        kernel32.QueryFullProcessImageNameW.argtypes = [
            wintypes.HANDLE,
            wintypes.DWORD,
            wintypes.LPWSTR,
            ctypes.POINTER(wintypes.DWORD),
        ]
        kernel32.QueryFullProcessImageNameW.restype = wintypes.BOOL
        kernel32.TerminateProcess.argtypes = [wintypes.HANDLE, wintypes.UINT]
        kernel32.TerminateProcess.restype = wintypes.BOOL
        kernel32.WaitForSingleObject.argtypes = [wintypes.HANDLE, wintypes.DWORD]
        kernel32.WaitForSingleObject.restype = wintypes.DWORD
        kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
        kernel32.CloseHandle.restype = wintypes.BOOL

        access = 0x0001 | 0x1000 | 0x00100000  # PROCESS_TERMINATE | PROCESS_QUERY_LIMITED_INFORMATION | SYNCHRONIZE
        hproc = kernel32.OpenProcess(access, False, int(pid))
        if not hproc:
            return False
        try:
            buf = ctypes.create_unicode_buffer(512)
            size = wintypes.DWORD(len(buf))
            if kernel32.QueryFullProcessImageNameW(hproc, 0, buf, ctypes.byref(size)):
                exe_name = Path(buf.value).name.lower()
                if "python" in exe_name:
                    if kernel32.TerminateProcess(hproc, 0):
                        kernel32.WaitForSingleObject(hproc, 2000)
                        return True
        finally:
            kernel32.CloseHandle(hproc)
    except Exception:
        pass
    return False


def replace_previous_instance() -> tuple[int, bool]:
    """
    Stänger eventuella äldre python-processer som kör app.py och återanvänder
    exakt samma port (utan PowerShell/WMI så nätverksdiskar inte fördröjer start).
    """
    reused_port = 0
    my_pid = os.getpid()
    used_psutil = False
    try:
        import psutil

        used_psutil = True
        for proc in psutil.process_iter(["pid", "name", "cmdline"]):
            try:
                if proc.info["pid"] == my_pid:
                    continue
                name = (proc.info["name"] or "").lower()
                cmdline = " ".join(proc.info["cmdline"] or [])
                if "python" in name and "app.py" in cmdline and "-c" not in cmdline:
                    for conn in proc.net_connections(kind="inet"):
                        if conn.status == "LISTEN" and conn.laddr.ip == "127.0.0.1":
                            reused_port = conn.laddr.port
                            break
                    proc.terminate()
                    proc.wait(timeout=3.0)
            except Exception:
                pass
    except Exception:
        pass

    if not used_psutil and platform.system() == "Windows":
        if _RUNNING_INSTANCE_FILE.is_file():
            try:
                inst = json.loads(_RUNNING_INSTANCE_FILE.read_text(encoding="utf-8"))
                if isinstance(inst, dict):
                    old_pid = int(inst.get("pid") or 0)
                    old_port = int(inst.get("port") or 0)
                    if old_port > 0:
                        reused_port = old_port
                    if old_pid and old_pid != my_pid:
                        _terminate_python_pid_win32(old_pid)
            except Exception:
                pass

        check_port = reused_port or 18450
        port_busy = False
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind(("127.0.0.1", check_port))
        except OSError:
            port_busy = True

        if port_busy:
            try:
                out = subprocess.check_output(
                    ["netstat", "-ano", "-p", "tcp"],
                    text=True,
                    stderr=subprocess.DEVNULL,
                    timeout=3.0,
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
                )
                target_addr = f"127.0.0.1:{check_port}"
                for line in out.splitlines():
                    parts = line.split()
                    if len(parts) >= 4 and parts[1] == target_addr and parts[-1].isdigit():
                        state_tok = parts[-2].upper() if len(parts) >= 5 else ""
                        if state_tok in ("LISTENING", "LYSSNAR") or parts[2] in ("0.0.0.0:0", "[::]:0"):
                            owner_pid = int(parts[-1])
                            if owner_pid != my_pid and _terminate_python_pid_win32(owner_pid):
                                time.sleep(0.2)
            except Exception:
                pass

    return (reused_port or 18450), False


def main():
    _ensure_valid_std_streams()
    if sys.prefix == sys.base_prefix and platform.system() == "Windows":
        exe_name = Path(sys.executable).name.lower()
        target_py = APP_ROOT / ".venv" / "Scripts" / ("pythonw.exe" if exe_name.startswith("pythonw") else "python.exe")
        if target_py.is_file() and Path(sys.executable).resolve() != target_py.resolve():
            if exe_name.startswith("pythonw"):
                subprocess.Popen([str(target_py), str(Path(__file__).resolve()), *sys.argv[1:]], cwd=str(APP_ROOT))
            else:
                res = subprocess.run([str(target_py), str(Path(__file__).resolve()), *sys.argv[1:]], cwd=str(APP_ROOT))
                raise SystemExit(res.returncode)
            return

    if platform.system() == "Windows":
        try:
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("OptiGSRigg.Studio.2.0")
        except Exception:
            pass

    parser = argparse.ArgumentParser(description="OptiGSRigg Studio v2.0 (Windows & macOS)")
    parser.add_argument("--classic", action="store_true", help="Starta det äldre Tkinter-gränssnittet")
    parser.add_argument("--headless", action="store_true", help="Starta endast den lokala UI-servern utan att öppna fönster")
    parser.add_argument("--port", type=int, default=0, help="Fast port (0 = återanvänd eller välj automatiskt)")
    args = parser.parse_args()

    if args.classic:
        import app_tkinter

        classic_app = app_tkinter.OptiGSRiggApp()
        classic_app.mainloop()
        return

    target_port, had_existing_window = replace_previous_instance()
    if args.port:
        target_port = args.port

    get_backend()

    try:
        server = ThreadingHTTPServer(("127.0.0.1", target_port), StudioRequestHandler)
        port = target_port
    except OSError:
        port = find_free_port()
        server = ThreadingHTTPServer(("127.0.0.1", port), StudioRequestHandler)
        had_existing_window = False

    try:
        _RUNNING_INSTANCE_FILE.parent.mkdir(parents=True, exist_ok=True)
        _RUNNING_INSTANCE_FILE.write_text(json.dumps({"pid": os.getpid(), "port": port}), encoding="utf-8")
    except OSError:
        pass

    url = f"http://127.0.0.1:{port}"

    print("============================================================")
    print("  OptiGSRigg Studio v2.0 (HTML/CSS/JS + Python Desktop)")
    print(f"  Plattform : {platform.system()} ({platform.release()})")
    print(f"  Lokalt UI : {url}", "(Återanvände öppet fönster)" if had_existing_window else "")
    print("============================================================", flush=True)

    if not args.headless and not had_existing_window:
        try:
            import webview

            threading.Thread(target=server.serve_forever, daemon=True).start()
            webview.create_window(
                "OptiGSRigg Control Center",
                url,
                width=1420,
                height=900,
                min_size=(1150, 760),
                background_color="#141517",
            )
            webview.start()
            return
        except ImportError:
            pass

        launch_native_app_window(url)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStänger OptiGSRigg Studio...")
        server.server_close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback

        try:
            _STARTUP_LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
            with open(_STARTUP_LOG_PATH, "a", encoding="utf-8") as _lf:
                _lf.write(f"\n[{datetime.datetime.now().isoformat()}] Startup error:\n{traceback.format_exc()}\n")
        except OSError:
            pass
        raise
w