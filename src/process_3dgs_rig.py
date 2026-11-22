"""
AI-AGENT ARCHITECTURAL OVERVIEW:
================================
Module: process_3dgs_rig.py
Purpose: End-to-end preprocessing, freestyle rig self-calibration, and training orchestrator
         for the 3x DJI Osmo Action 4 3DGS Rig.
Hardware Spec:
  - 3x DJI Osmo Action 4 in 90° Portrait Mode (2880x3840 @ 50fps, 10-bit D-Log M, Wide FOV, RockSteady OFF).
  - Baseline: 300mm nominal (-0.150m Left, 0.000m Center, +0.150m Right along X-axis), or user-specified --baseline_mm.
  - Orientation Modes:
      * auto_calibrate (Default / Freestyle): Automatically measures exact relative Yaw/Pitch/Roll & translation
        between cameras directly from the reconstructed synchronized triplets, scales to 1:1 metric baseline,
        and saves `config/calibrated_rig.json`.
      * mode_a (CAD Standard): Cam1(-45° Yaw, +12° Pitch), Cam2(0° Yaw, -8° Pitch), Cam3(+45° Yaw, +12° Pitch).
      * mode_b (CAD Flipped 180°): Cam1(-45° Yaw, -12° Pitch), Cam2(0° Yaw, +8° Pitch), Cam3(+45° Yaw, -12° Pitch).
      * calibrated (Load saved): Reuses previously calibrated geometry from `config/calibrated_rig.json`.
Installed Binaries:
  - LichtFeld Studio v0.5.4.1: D:/LichtFeld-Studio-windows-v0.5.4.1/bin/LichtFeld-Studio.exe
  - Spirula Studio 2026.10.30: D:/spirula-2026.10.30-windows-vulkan-x86_64/spirula.exe (includes Vulkan `spirula sfm`)
"""

import os
import sys
import csv
import json
import math
import time
import struct
import shutil
import threading
import subprocess
import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any, Dict, List, Tuple, Optional
try:
    import numpy as np
    from scipy import signal
    import soundfile as sf
    import cv2
except ImportError:
    np = None
    signal = None
    sf = None
    cv2 = None


# =====================================================================
# 1. SÖKVÄGAR TILL INSTALLERADE MOTORER & OPTIK-KONSTANTER
# =====================================================================

_APP_ROOT_CFG = Path(__file__).resolve().parent.parent / "config"
CALIBRATED_RIG_PATH = _APP_ROOT_CFG / "calibrated_rig.json"
DJI_RIG_PROFILE_PATH = _APP_ROOT_CFG / "dji_rig_profile.json"
SETTINGS_JSON_PATH = _APP_ROOT_CFG / "settings.json"

DEFAULT_LICHTFELD_BIN = r"D:\LichtFeld-Studio-windows-v0.5.4.1\bin\LichtFeld-Studio.exe"
DEFAULT_SPIRULA_BIN = r"D:\spirula-2026.10.30-windows-vulkan-x86_64\spirula.exe"
DEFAULT_COLMAP_BIN = "colmap"
DEFAULT_CAPTURES_ROOT = Path(r"D:\3DGS_Captures")

if SETTINGS_JSON_PATH.exists():
    try:
        _cfg_s = json.loads(SETTINGS_JSON_PATH.read_text(encoding="utf-8"))
        if isinstance(_cfg_s, dict):
            DEFAULT_LICHTFELD_BIN = str(_cfg_s.get("lichtfeld_exe") or DEFAULT_LICHTFELD_BIN)
            DEFAULT_SPIRULA_BIN = str(_cfg_s.get("spirula_exe") or DEFAULT_SPIRULA_BIN)
            DEFAULT_COLMAP_BIN = str(_cfg_s.get("colmap_exe") or DEFAULT_COLMAP_BIN)
            if _cfg_s.get("captures_root"):
                DEFAULT_CAPTURES_ROOT = Path(str(_cfg_s["captures_root"]))
    except Exception:
        pass

# Låsta lins-intrinsics för DJI Osmo Action 4 i 4K 4:3 (2880x3840 stående, Wide FOV, RockSteady OFF)
# Modell i COLMAP: OPENCV_FISHEYE -> [fx, fy, cx, cy, k1, k2, k3, k4]
ACTION4_PORTRAIT_FISHEYE_PARAMS = "1585.0, 1585.0, 1440.0, 1920.0, -0.042, 0.008, -0.002, 0.0005"
ACTION4_PORTRAIT_FISHEYE_ARRAY = [1585.0, 1585.0, 1440.0, 1920.0, -0.042, 0.008, -0.002, 0.0005]
ACTION4_FOCAL_PX = "1585.0"
ACTION4_FISHEYE_DISTORTION = "-0.042,0.008,-0.002,0.0005"

# Nominella CAD-lägen (EXAKTA passiva cam_from_rig i meter och grader relativt Cam 2 Center)
# CAD-fysisk specifikation: Mitten +8° uppåt, Sidor ±150 mm utåt, vridna ±45° utåt, tiltade -12° nedåt.
RIG_MODES = {
    "mode_a": {
        "cam1_left":   {"tx":  0.106066, "ty": 0.022052, "tz": -0.103748, "yaw":  46.516, "pitch":  17.587, "roll": -5.926},
        "cam2_center": {"tx":  0.000000, "ty": 0.000000, "tz":  0.000000, "yaw":   0.000, "pitch":   0.000, "roll":  0.000},
        "cam3_right":  {"tx": -0.106066, "ty": 0.022052, "tz": -0.103748, "yaw": -46.516, "pitch":  17.587, "roll":  5.926},
    },
    "mode_b": {  # När hela 2020-skenan vänds 180° upp-och-ner (Flip-Rail: Mitten -8° ned, Sidor +12° upp)
        "cam1_left":   {"tx":  0.106066, "ty": 0.022052, "tz":  0.103748, "yaw": -46.516, "pitch": -17.587, "roll": -5.926},
        "cam2_center": {"tx":  0.000000, "ty": 0.000000, "tz":  0.000000, "yaw":   0.000, "pitch":   0.000, "roll":  0.000},
        "cam3_right":  {"tx": -0.106066, "ty": 0.022052, "tz":  0.103748, "yaw":  46.516, "pitch": -17.587, "roll":  5.926},
    }
}

# Mappning mellan ID-filer i roten på MicroSD-korten och rätt målkamera i pipelinen
SD_CARD_ID_FILES = {
    "CAM0_LEFT.id":  "cam1_left",
    "CAM1_MID.id":   "cam2_center",
    "CAM2_RIGHT.id": "cam3_right",
    # Extra alias om man råkar döpa dem 1/2/3:
    "CAM1_LEFT.id":   "cam1_left",
    "CAM2_CENTER.id": "cam2_center",
    "CAM3_RIGHT.id":  "cam3_right",
}


# =====================================================================
# 1B. AUTOMATISK IMPORT FRÅN MICROSD-KORT MED ID-FILER (CAM0/1/2.id)
# =====================================================================

def discover_sd_card_drives() -> Tuple[Dict[str, Tuple[Path, str]], List[Path]]:
    """
    AI-AGENT NOTE: Skannar alla monterade enheter i både Windows (D: till Z:)
    och macOS (/Volumes/*) / Linux (/media/*, /mnt/*)
    efter ID-filerna `CAM0_LEFT.id`, `CAM1_MID.id` och `CAM2_RIGHT.id` i kortets rot.
    Returnerar:
      - found_drives: `{ 'cam1_left': (Path('E:/'), 'CAM0_LEFT.id'), ... }`
      - unmarked_dcim_drives: Lista med enheter som har en `DCIM/`-mapp men SAKNAR `.id`-fil!
    """
    found_drives: Dict[str, Tuple[Path, str]] = {}
    unmarked_dcim_drives: List[Path] = []

    candidate_roots: List[Path] = []
    if os.name == "nt":
        try:
            import ctypes

            k32 = ctypes.windll.kernel32
            bitmask = k32.GetLogicalDrives()
            for idx, letter in enumerate("ABCDEFGHIJKLMNOPQRSTUVWXYZ"):
                if idx < 3:
                    continue
                if bitmask & (1 << idx):
                    if k32.GetDriveTypeW(f"{letter}:\\") in (2, 3):
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
                    print(f"[SD-Ingest] Hittade {id_filename} på enhet {drive_root} -> {target_cam}")
                    break
            if not matched_id and (drive_root / "DCIM").exists():
                unmarked_dcim_drives.append(drive_root)
        except OSError:
            continue
    return found_drives, unmarked_dcim_drives


def scan_sd_cards() -> Dict[str, dict]:
    """
    Returnerar strukturerad status för alla 3 kamerors MicroSD-kort (Windows & Mac)
    för direkt visning i GUI.
    """
    import datetime

    drives, unmarked = discover_sd_card_drives()
    status: Dict[str, dict] = {}
    for cam_key, default_id in [
        ("cam1_left", "CAM0_LEFT.id"),
        ("cam2_center", "CAM1_MID.id"),
        ("cam3_right", "CAM2_RIGHT.id"),
    ]:
        if cam_key in drives:
            drive_root, id_file = drives[cam_key]
            mp4s = find_mp4_takes_on_card(drive_root)
            clips = []
            # Nyast först i listan
            for p in reversed(mp4s):
                try:
                    st = p.stat()
                    clips.append(
                        {
                            "name": p.name,
                            "path": str(p),
                            "size_mb": round(st.st_size / (1024 * 1024), 1),
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
    status["_unmarked_dcim"] = [str(u) for u in unmarked]
    return status


def find_mp4_takes_on_card(drive_root: Path) -> List[Path]:
    """
    Hittar alla riktiga MP4-videofiler under `DCIM/` (eller i kortets rot om DCIM saknas),
    sorterade kronologiskt efter ändringstid (äldst först, nyast sist).
    Ignorerar DJI:s lågupplösta `.LRF`-filer och små skräpfiler.
    """
    dcim_dir = drive_root / "DCIM"
    search_root = dcim_dir if dcim_dir.exists() else drive_root
    mp4_files = [
        p for p in search_root.rglob("*")
        if p.is_file() and p.suffix.lower() == ".mp4" and not p.name.startswith("._")
    ]
    mp4_files.sort(key=lambda p: p.stat().st_mtime)
    return mp4_files


def import_from_sd_cards(
    raw_dir: Path,
    take_from_end: int = 1
) -> Dict[str, Path]:
    """
    AI-AGENT NOTE: Kopierar in rätt videofil från varje inkopplat MicroSD-kort
    (identifierat via `CAM0_LEFT.id`, `CAM1_MID.id`, `CAM2_RIGHT.id`) till `raw_dir`
    och döper om dem automatiskt till `cam1_left.mp4`, `cam2_center.mp4`, `cam3_right.mp4`.

    Ger tydlig diagnostik om:
      - Inga SD-kort med .id-filer hittas (och varnar om det finns ett SD-kort med DCIM utan .id-fil)
      - Ett SD-kort hittas men är tomt på .MP4-filer
      - Valt klippnummer (take_from_end) inte finns på kortet
    """
    raw_dir.mkdir(parents=True, exist_ok=True)
    drives, unmarked_dcim = discover_sd_card_drives()

    expected_labels = {
        "cam1_left": "Vänster kamera (CAM0_LEFT.id)",
        "cam2_center": "Mittenkamera (CAM1_MID.id)",
        "cam3_right": "Höger kamera (CAM2_RIGHT.id)",
    }

    if not drives:
        msg_lines = [
            "[FEL] Hittade inget MicroSD-kort med CAM0_LEFT.id, CAM1_MID.id eller CAM2_RIGHT.id!",
        ]
        if unmarked_dcim:
            unmarked_str = ", ".join(str(d) for d in unmarked_dcim)
            msg_lines.append(
                f"[TIPS] Hittade minneskort med DCIM-mapp på ({unmarked_str}), men det saknar .id-fil i roten! "
                "Skapa CAM0_LEFT.id, CAM1_MID.id eller CAM2_RIGHT.id i kortets rot."
            )
        else:
            msg_lines.append(
                "[TIPS] Kontrollera att minneskortläsaren/kameran är inkopplad och syns som en enhetsbokstav i Windows."
            )
        full_msg = "\n".join(msg_lines)
        print(full_msg)
        raise FileNotFoundError(full_msg)

    copied: Dict[str, Path] = {}
    empty_cards: List[str] = []

    for cam_id, (drive_root, id_filename) in sorted(drives.items()):
        takes = find_mp4_takes_on_card(drive_root)
        if not takes:
            warn_msg = f"[VARNING] Kortet {drive_root} ({id_filename} -> {cam_id}) hittades, men innehåller INGA .MP4-filer!"
            print(warn_msg)
            empty_cards.append(f"{cam_id} ({drive_root})")
            continue

        if take_from_end > len(takes):
            print(
                f"[VARNING] Du valde klipp #{take_from_end} från slutet, men {drive_root} ({id_filename}) "
                f"har bara {len(takes)} st .MP4-filer! Väljer det äldsta klippet på kortet istället."
            )
            idx = 0
        else:
            idx = len(takes) - max(1, take_from_end)

        src_mp4 = takes[idx]
        dst_mp4 = raw_dir / f"{cam_id}.mp4"
        src_size = src_mp4.stat().st_size
        size_mb = src_size / (1024 * 1024)
        if dst_mp4.exists() and dst_mp4.stat().st_size == src_size:
            print(
                f"[SD-Ingest] Hoppar över {src_mp4.name} -> {dst_mp4.name} (redan kopierad, {size_mb:.1f} MB).",
                flush=True,
            )
        else:
            print(
                f"[SD-Ingest] Kopierar {src_mp4.name} ({size_mb:.1f} MB, tagning {idx+1}/{len(takes)} på {drive_root}) "
                f"-> {dst_mp4.name} ...",
                flush=True,
            )
            shutil.copy2(src_mp4, dst_mp4)
            print(f"[SD-Ingest] Klar med {dst_mp4.name} ({size_mb:.1f} MB)!", flush=True)
        copied[cam_id] = dst_mp4

    # Sammanställ tydlig status för alla 3 kameror i raw_dir
    all_expected = ["cam1_left", "cam2_center", "cam3_right"]
    present = [c for c in all_expected if (raw_dir / f"{c}.mp4").exists()]
    missing = [f"{c} ({expected_labels[c]})" for c in all_expected if c not in present]

    if empty_cards:
        raise FileNotFoundError(
            f"Kunde inte kopiera video eftersom följande SD-kort saknade .MP4-filer: {', '.join(empty_cards)}"
        )

    if missing:
        print(
            f"[SD-Ingest] Delvis klart! Klara i {raw_dir.name}: {', '.join(present)}.\n"
            f"[SD-Ingest] Sätt nu i kortet för: {', '.join(missing)} och klicka igen.",
            flush=True,
        )
    else:
        print(
            f"[SD-Ingest] [OK] ALLA 3 KAMEROR (cam1_left, cam2_center, cam3_right) KOPIERADE OCH KLARA I {raw_dir}!",
            flush=True,
        )

    return copied


def import_all_takes_from_sd_cards(captures_root: Path) -> List[Path]:
    """
    AI-AGENT NOTE: Kopierar ALLA tagningar (takes) från de inkopplade MicroSD-korten
    till separata, tydligt namngivna sessionsmappar under `captures_root`, t.ex.:
      D:\\3DGS_Captures\\Take_01_20261105_1847\\raw\\cam1_left.mp4 ...
      D:\\3DGS_Captures\\Take_02_20261108_1150\\raw\\cam1_left.mp4 ...
    Hoppar automatiskt över filer som redan är kopierade med exakt samma filstorlek.
    """
    import datetime

    captures_root.mkdir(parents=True, exist_ok=True)
    drives, unmarked_dcim = discover_sd_card_drives()
    if not drives:
        raise FileNotFoundError("Hittade inget MicroSD-kort med CAM0_LEFT.id, CAM1_MID.id eller CAM2_RIGHT.id!")

    takes_per_cam: Dict[str, List[Path]] = {}
    for cam_id, (drive_root, _id_file) in sorted(drives.items()):
        takes_per_cam[cam_id] = find_mp4_takes_on_card(drive_root)

    max_takes = max((len(v) for v in takes_per_cam.values()), default=0)
    if max_takes == 0:
        raise FileNotFoundError("Inga .MP4-filer hittades på de inkopplade SD-korten!")

    created_sessions: List[Path] = []
    print(f"[SD-Ingest] Importerar samtliga {max_takes} tagningar från SD-korten till {captures_root} ...", flush=True)

    for take_idx in range(max_takes):
        # Bestäm tidsstämpel från första tillgängliga kamera för denna tagning
        sample_mp4 = None
        for cam_id in ("cam2_center", "cam1_left", "cam3_right"):
            cam_list = takes_per_cam.get(cam_id, [])
            if take_idx < len(cam_list):
                sample_mp4 = cam_list[take_idx]
                break
        if sample_mp4 is None:
            continue

        dt_str = datetime.datetime.fromtimestamp(sample_mp4.stat().st_mtime).strftime("%Y%m%d_%H%M%S")
        session_dir = captures_root / f"Take_{take_idx + 1:02d}_{dt_str}"
        raw_dir = session_dir / "raw"
        proj_dir = session_dir / "processed"
        raw_dir.mkdir(parents=True, exist_ok=True)
        proj_dir.mkdir(parents=True, exist_ok=True)

        print(f"\n--- [Tagning #{take_idx + 1}/{max_takes}] -> {session_dir.name} ---", flush=True)
        for cam_id, cam_list in sorted(takes_per_cam.items()):
            if take_idx >= len(cam_list):
                print(f"  [VARNING] {cam_id} saknar klipp #{take_idx + 1}!", flush=True)
                continue
            src_mp4 = cam_list[take_idx]
            dst_mp4 = raw_dir / f"{cam_id}.mp4"
            src_size = src_mp4.stat().st_size
            size_mb = src_size / (1024 * 1024)
            if dst_mp4.exists() and dst_mp4.stat().st_size == src_size:
                print(f"  [Redan klar] {src_mp4.name} -> {dst_mp4.name} ({size_mb:.1f} MB)", flush=True)
            else:
                print(f"  [Kopierar] {src_mp4.name} ({size_mb:.1f} MB) -> {dst_mp4.name} ...", flush=True)
                shutil.copy2(src_mp4, dst_mp4)
                print(f"  [OK] {dst_mp4.name} klar!", flush=True)
        created_sessions.append(session_dir)

    print(
        f"\n[SD-Ingest] [OK] Samtliga {len(created_sessions)} tagningar är nu importerade till {captures_root}!",
        flush=True,
    )
    return created_sessions


# =====================================================================
# 2. LJUDSYNKRONISERING VIA KORSKORRELATION (KLAPP-DETEKTERING)
# =====================================================================

def extract_audio_wav(video_path: Path, temp_wav: Path, sample_rate: int = 48000) -> Path:
    """
    AI-AGENT NOTE: Extraherar de första 15 sekunderna av ljudspåret till mono-WAV
    för snabb korskorrelation av startklappen.
    """
    cmd = [
        "ffmpeg", "-y", "-i", str(video_path),
        "-t", "15", "-vn", "-ac", "1", "-ar", str(sample_rate),
        "-loglevel", "error", str(temp_wav)
    ]
    subprocess.run(cmd, check=True)
    return temp_wav


def compute_frame_offsets(
    videos: Dict[str, Path],
    work_dir: Path,
    fps: float = 50.0
) -> Dict[str, int]:
    """
    AI-AGENT NOTE: Beräknar exakt bildrute-offset (i heltalsrutor vid 50fps) för
    cam1_left och cam3_right relativt cam2_center med hjälp av FFT-korskorrelation.
    Returnerar en dict med frame_offset per kamera så att index N motsvarar samma tidsögonblick.
    """
    sample_rate = 48000
    wav_dir = work_dir / "temp_audio"
    wav_dir.mkdir(parents=True, exist_ok=True)

    print("[Steg 1/3 - Ljudsynk] Extraherar ljudspår (första 15 sek) från alla 3 kameror...", flush=True)
    audio_signals = {}
    for cam_id, v_path in videos.items():
        print(f"[Steg 1/3 - Ljudsynk] Läser ljud från {v_path.name} ({cam_id})...", flush=True)
        wav_path = extract_audio_wav(v_path, wav_dir / f"{cam_id}.wav", sample_rate)
        data, _ = sf.read(str(wav_path))
        audio_signals[cam_id] = np.abs(data)

    ref_sig = audio_signals["cam2_center"]
    offsets_frames = {"cam2_center": 0}
    lags_ms = {"cam2_center": 0.0}
    clap_peak_sec = round(int(np.argmax(ref_sig)) / float(sample_rate), 3) if len(ref_sig) > 0 else 1.5

    for cam_id in ["cam1_left", "cam3_right"]:
        target_sig = audio_signals[cam_id]
        corr = signal.correlate(target_sig, ref_sig, mode="full", method="fft")
        lag_samples = int(np.argmax(corr)) - (len(ref_sig) - 1)
        lag_seconds = lag_samples / float(sample_rate)
        frame_offset = int(round(lag_seconds * fps))
        offsets_frames[cam_id] = frame_offset
        lags_ms[cam_id] = round(lag_seconds * 1000.0, 2)
        print(
            f"[AudioSync] {cam_id} relativt center: {lag_seconds*1000:+.1f} ms ({frame_offset:+d} bildrutor @ {fps} fps)",
            flush=True,
        )

    min_offset = min(offsets_frames.values())
    normalized_offsets = {k: v - min_offset for k, v in offsets_frames.items()}
    try:
        sync_json = {
            "fps": fps,
            "clap_peak_sec": clap_peak_sec,
            "lags_ms": lags_ms,
            "raw_frame_offsets": offsets_frames,
            "normalized_offsets": normalized_offsets,
        }
        (work_dir / "sync_offsets.json").write_text(json.dumps(sync_json, indent=2), encoding="utf-8")
    except Exception:
        pass
    print(f"[Steg 1/3 - Ljudsynk] KLAR! Frame-offsets: {normalized_offsets}", flush=True)
    return normalized_offsets


# =====================================================================
# 3. LAPLACIAN SKÄRPEFILTER & LUT-EXTRAHERING (D-LOG M -> REC.709)
# =====================================================================

def measure_sharpness_laplacian(frame_bgr: np.ndarray) -> float:
    """
    AI-AGENT NOTE: Beräknar variansen av Laplacian på en nedskalad gråskalebild
    för att snabbt mäta rörelseoskärpa. Högre värde = skarpare bildruta.
    """
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    small = cv2.resize(gray, (720, 960), interpolation=cv2.INTER_AREA)
    return float(cv2.Laplacian(small, cv2.CV_64F).var())


def _clear_existing_jpgs(output_images_dir: Path, cam_ids) -> None:
    """Skapar kamerakataloger och rensar eventuella gamla *.jpg innan ny extraktion."""
    for cam_id in cam_ids:
        cam_dir = output_images_dir / cam_id
        cam_dir.mkdir(parents=True, exist_ok=True)
        for old_jpg in cam_dir.glob("*.jpg"):
            try:
                old_jpg.unlink()
            except OSError:
                pass


_CUBE_LUT_CACHE: Dict[Tuple[str, float], object] = {}
_CUBE_LUT_LOCK = threading.Lock()


def _load_cube_lut256_bgr(lut_path: Path) -> "np.ndarray":
    """
    Läser in en .cube 3D-LUT (godtycklig storlek N^3, t.ex. 33x33x33) och
    för-interpolerar den separabelt till en snabb 256x256x256x3 uint8 BGR-tabell.
    """
    if np is None:
        raise RuntimeError("numpy krävs för 3D-LUT-interpolering")
    resolved = str(lut_path.resolve())
    mtime = float(lut_path.stat().st_mtime)
    cache_key = (resolved, mtime)
    with _CUBE_LUT_LOCK:
        cached = _CUBE_LUT_CACHE.get(cache_key)
        if cached is not None:
            return cached

    n_size: Optional[int] = None
    dmin = [0.0, 0.0, 0.0]
    dmax = [1.0, 1.0, 1.0]
    rows: List[Tuple[float, float, float]] = []

    with open(lut_path, "r", encoding="utf-8", errors="replace") as f:
        for raw_line in f:
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.split()
            kw = parts[0].upper()
            if kw in ("TITLE", "LUT_1D_SIZE", "LUT_3D_INPUT_RANGE"):
                if kw == "LUT_1D_SIZE":
                    raise ValueError(f"1D-LUT stöds inte av 3D-interpolerare: {lut_path.name}")
                continue
            if kw == "LUT_3D_SIZE":
                n_size = int(parts[1])
                continue
            if kw == "DOMAIN_MIN" and len(parts) >= 4:
                dmin = [float(parts[1]), float(parts[2]), float(parts[3])]
                continue
            if kw == "DOMAIN_MAX" and len(parts) >= 4:
                dmax = [float(parts[1]), float(parts[2]), float(parts[3])]
                continue
            if len(parts) >= 3:
                rows.append((float(parts[0]), float(parts[1]), float(parts[2])))

    if n_size is None:
        n_size = round(len(rows) ** (1.0 / 3.0))
    if n_size < 2 or len(rows) != n_size ** 3:
        raise ValueError(f"Ogiltig .cube 3D-LUT ({len(rows)} rader för N={n_size}): {lut_path}")

    # Adobe .cube har R snabbast (innerst), G i mitten, B långsammast (ytterst) -> shape (B, G, R, RGB_out)
    table_rgb = np.array(rows, dtype=np.float32).reshape((n_size, n_size, n_size, 3))
    # Vänd utgående kanaler från RGB till OpenCV BGR så indexering med [B, G, R] ger BGR direkt
    arr = np.ascontiguousarray(table_rgb[..., ::-1])

    ramp = np.arange(256, dtype=np.float32) / 255.0

    def _coords(ch: int) -> "np.ndarray":
        span = max(1e-6, dmax[ch] - dmin[ch])
        return np.clip((ramp - dmin[ch]) / span, 0.0, 1.0) * (n_size - 1)

    tb = _coords(2)
    i0 = np.floor(tb).astype(np.int32)
    i1 = np.minimum(i0 + 1, n_size - 1)
    w = (tb - i0)[:, None, None, None]
    arr = arr[i0] * (1.0 - w) + arr[i1] * w

    tg = _coords(1)
    i0 = np.floor(tg).astype(np.int32)
    i1 = np.minimum(i0 + 1, n_size - 1)
    w = (tg - i0)[None, :, None, None]
    arr = arr[:, i0] * (1.0 - w) + arr[:, i1] * w

    tr = _coords(0)
    i0 = np.floor(tr).astype(np.int32)
    i1 = np.minimum(i0 + 1, n_size - 1)
    w = (tr - i0)[None, None, :, None]
    arr = arr[:, :, i0] * (1.0 - w) + arr[:, :, i1] * w

    lut256_bgr = np.ascontiguousarray(np.clip(np.rint(arr * 255.0), 0.0, 255.0).astype(np.uint8))
    with _CUBE_LUT_LOCK:
        _CUBE_LUT_CACHE[cache_key] = lut256_bgr
    return lut256_bgr


def _extract_camera_frames_parallel_cv2(
    video_path: Path,
    target_frames: List[Tuple[int, int]],
    out_dir: Path,
    lut_path: Optional[Path] = None,
    fps: float = 50.0,
    progress_cb=None,
) -> int:
    """
    Extraherar exakta 0-baserade bildrutenummer ur en videofil med flertrådad
    linjär GOP-läsning (varje tråd söker EN gång till sin slice-start och stegar framåt).
    """
    if not target_frames:
        return 0
    out_dir.mkdir(parents=True, exist_ok=True)

    lut256_bgr: Optional["np.ndarray"] = None
    if lut_path and lut_path.exists():
        try:
            lut256_bgr = _load_cube_lut256_bgr(lut_path)
        except Exception as exc:
            print(f"[Steg 2/3] Varning: kunde inte bygga 3D-LUT-tabell ({exc}), använder FFmpeg-fallback.", flush=True)
            lut_escaped = lut_path.resolve().as_posix().replace(":", r"\:")
            written_lock = threading.Lock()
            written_count = 0

            def _lut_job(item: Tuple[int, int]):
                nonlocal written_count
                t_idx, frame_no = item
                out_file = out_dir / f"frame_{t_idx:04d}.jpg"
                ts = frame_no / max(1.0, fps)
                cmd = [
                    "ffmpeg", "-y", "-ss", f"{ts:.4f}",
                    "-i", str(video_path),
                    "-vf", f"lut3d='{lut_escaped}'",
                    "-frames:v", "1", "-q:v", "1",
                    "-loglevel", "error", str(out_file),
                ]
                subprocess.run(cmd, check=True)
                with written_lock:
                    written_count += 1
                if progress_cb:
                    progress_cb(t_idx)

            with ThreadPoolExecutor(max_workers=6) as ex:
                list(ex.map(_lut_job, target_frames))
            return written_count

    n_workers = min(12, max(4, (os.cpu_count() or 8) // 2), max(1, (len(target_frames) + 3) // 4))
    chunk_sz = (len(target_frames) + n_workers - 1) // n_workers
    chunks = [target_frames[i : i + chunk_sz] for i in range(0, len(target_frames), chunk_sz)]

    written_lock = threading.Lock()
    written_count = 0

    def _slice_worker(chunk_items: List[Tuple[int, int]]) -> None:
        nonlocal written_count
        if not chunk_items:
            return
        cap = cv2.VideoCapture(str(video_path))
        if not cap.isOpened():
            return
        try:
            first_f = max(0, int(chunk_items[0][1]))
            cap.set(cv2.CAP_PROP_POS_FRAMES, first_f)
            cur_f = first_f
            for t_idx, target_f in chunk_items:
                tf = max(0, int(target_f))
                if tf < cur_f:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, tf)
                    cur_f = tf
                while cur_f < tf:
                    if not cap.grab():
                        break
                    cur_f += 1
                ret, img = cap.read()
                cur_f += 1
                if ret and img is not None:
                    if lut256_bgr is not None:
                        img = lut256_bgr[img[..., 0], img[..., 1], img[..., 2]]
                    out_file = out_dir / f"frame_{t_idx:04d}.jpg"
                    cv2.imwrite(str(out_file), img, [cv2.IMWRITE_JPEG_QUALITY, 98])
                    with written_lock:
                        written_count += 1
                    if progress_cb:
                        progress_cb(t_idx)
        finally:
            cap.release()

    with ThreadPoolExecutor(max_workers=len(chunks)) as ex:
        list(ex.map(_slice_worker, chunks))
    return written_count


def _extract_with_spirula_gpu(
    videos: Dict[str, Path],
    offsets: Dict[str, int],
    output_images_dir: Path,
    lut_path: Optional[Path],
    fps: float,
    window_frames: int,
    start_base_frame: int,
    max_sync_frame: int,
    spirula_bin: str,
) -> int:
    """
    Hybrid Vulkan GPU + parallell flertrådad bildextraktion med exakt 1-frame klapp-synk:
      1) Kameror med samma offset som cam2_center muxas utan omkodning (-c copy) och avkodas
         via Spirula Vulkan Video Decode (`spirula sam extract --sync`) med skärpeval i VRAM.
      2) Eventuell sidokamera med annat offset (t.ex. ±1 bildruta) extraheras exakt på de
         vinnande bildrutorna med skivad linjär flertrådad OpenCV.
    """
    t0 = time.time()
    est_total = max(1, (max_sync_frame - start_base_frame) // max(1, window_frames))
    center_off = offsets.get("cam2_center", 0)
    aligned_cams = [c for c in ("cam1_left", "cam2_center", "cam3_right") if offsets.get(c, 0) == center_off]
    if "cam2_center" not in aligned_cams:
        aligned_cams.append("cam2_center")
    unaligned_cams = [c for c in ("cam1_left", "cam2_center", "cam3_right") if c not in aligned_cams]

    lut_desc = f"med LUT ({lut_path.name})" if (lut_path and lut_path.exists()) else "utan LUT (rå färgprofil)"
    print(
        f"[Steg 2/3 - Bildextrahering] Startar Vulkan GPU-extraktion (Spirula) {lut_desc} för {', '.join(aligned_cams)} "
        + (f"+ parallell exakt-offset ({unaligned_cams}) " if unaligned_cams else "")
        + f"(ca {est_total} bild-trior till {output_images_dir})...",
        flush=True,
    )

    temp_sp_dir = output_images_dir.parent / "_temp_spirula_extract"
    if temp_sp_dir.exists():
        shutil.rmtree(temp_sp_dir, ignore_errors=True)
    temp_sp_dir.mkdir(parents=True, exist_ok=True)

    sp_out = temp_sp_dir / "out"
    sp_out.mkdir(parents=True, exist_ok=True)
    report_csv = temp_sp_dir / "report.csv"

    try:
        start_sec = max(0.0, (start_base_frame + center_off) / max(1.0, fps))
        if len(aligned_cams) >= 2:
            mux_mp4 = temp_sp_dir / "sync_mux.mp4"
            mux_cmd = ["ffmpeg", "-y", "-v", "error"]
            for ck in aligned_cams:
                mux_cmd.extend(["-i", str(videos[ck])])
            for track_idx in range(len(aligned_cams)):
                mux_cmd.extend(["-map", f"{track_idx}:v:0"])
            mux_cmd.extend(["-c", "copy", str(mux_mp4)])
            mux_res = subprocess.run(mux_cmd, capture_output=True, text=True)
            if mux_res.returncode != 0 or not mux_mp4.exists():
                raise RuntimeError(f"FFmpeg mux misslyckades (kod {mux_res.returncode}): {mux_res.stderr[:250]}")
            sp_input = mux_mp4
            is_multi = True
        else:
            sp_input = videos["cam2_center"]
            is_multi = False

        sp_cmd = [
            spirula_bin,
            "sam",
            "extract",
            str(sp_input),
        ]
        if is_multi:
            sp_cmd.append("--sync")
        sp_cmd.extend(
            [
                "--start",
                f"{start_sec:.4f}",
                "-o",
                str(sp_out),
                "-s",
                str(window_frames),
                "-k",
                str(window_frames),
                "-q",
                "98",
                "--report",
                str(report_csv),
            ]
        )
        if lut_path and lut_path.exists():
            sp_cmd.extend(["--lut", str(lut_path)])

        watch_dir = (sp_out / "cam0") if is_multi else sp_out
        proc = subprocess.Popen(sp_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace")

        last_poll_n = 0
        sp_scale = 0.95 if not unaligned_cams else 0.58
        while proc.poll() is None:
            time.sleep(0.25)
            try:
                cur_files = sorted(watch_dir.glob("*.jpg")) if watch_dir.exists() else []
                n_cur = len(cur_files)
                if n_cur > last_poll_n:
                    last_poll_n = n_cur
                    virt_idx = max(1, min(est_total - 1, int(round(n_cur * sp_scale))))
                    latest_name = cur_files[-1].name
                    print(
                        f"[ExtractProgress] {virt_idx}/{est_total}|{n_cur * len(aligned_cams)}|{latest_name}|0.0",
                        flush=True,
                    )
            except OSError:
                pass

        sp_out_log = proc.stdout.read() if proc.stdout else ""
        rc = proc.wait()
        if rc != 0:
            raise RuntimeError(f"Spirula GPU-extraktion avslutades med kod {rc}: {sp_out_log[:300]}")

        # Läs exakta bildrutenummer för cam2_center från report.csv
        chosen_center_frames: List[int] = []
        if report_csv.exists():
            with open(report_csv, "r", encoding="utf-8", errors="replace") as rf:
                reader = csv.DictReader(rf)
                for row in reader:
                    verdict = (row.get("verdict") or "").strip().lower()
                    if not verdict or verdict == "kept":
                        try:
                            chosen_center_frames.append(int(float(row["frame"])))
                        except (KeyError, ValueError, TypeError):
                            pass

        # Bygg mappning {frame_int: Path} för varje aligned-kamera
        cam_file_maps: Dict[str, Dict[int, Path]] = {}
        cam_sorted_lists: Dict[str, List[Path]] = {}
        for t_idx, ck in enumerate(aligned_cams):
            src_dir = (sp_out / f"cam{t_idx}") if is_multi else sp_out
            files = sorted([p for p in src_dir.glob("*.jpg")]) if src_dir.exists() else []
            cam_sorted_lists[ck] = files
            fmap: Dict[int, Path] = {}
            for p in files:
                if p.stem.isdigit():
                    fmap[int(p.stem)] = p
            cam_file_maps[ck] = fmap

        if not chosen_center_frames and cam_file_maps.get("cam2_center"):
            chosen_center_frames = sorted(cam_file_maps["cam2_center"].keys())

        if not chosen_center_frames:
            raise RuntimeError("Spirula GPU-extraktion gav inga utdatabilder i report.csv.")

        # Säkerställ att vi bara behåller bildrutor som finns i ALLA aligned-kameror
        common_frames = [
            cf for cf in chosen_center_frames
            if all(cf in cam_file_maps[ck] for ck in aligned_cams)
        ]
        if not common_frames and len(cam_sorted_lists["cam2_center"]) > 0:
            # Fallback om filnamnsstammarna inte matchade direkt siffermässigt
            min_len = min([len(cam_sorted_lists[ck]) for ck in aligned_cams] + [len(chosen_center_frames)])
            common_frames = chosen_center_frames[:min_len]
            for ck in aligned_cams:
                files = cam_sorted_lists[ck][:min_len]
                for out_idx, src_p in enumerate(files, 1):
                    dst_p = output_images_dir / ck / f"frame_{out_idx:04d}.jpg"
                    shutil.move(str(src_p), str(dst_p))
        else:
            for out_idx, cf in enumerate(common_frames, 1):
                dst_name = f"frame_{out_idx:04d}.jpg"
                for ck in aligned_cams:
                    shutil.move(str(cam_file_maps[ck][cf]), str(output_images_dir / ck / dst_name))

        saved_count = len(common_frames)
        if saved_count == 0:
            raise RuntimeError("Kunde inte matcha synkade GPU-extraherade bildfiler.")

        # Ta bort tillfällig mux/temp-mapp direkt
        shutil.rmtree(temp_sp_dir, ignore_errors=True)

        # Extrahera kvarvarande sidokameror (t.ex. cam3_right med +1/-1 frame offset) parallellt
        if unaligned_cams:
            done_lock = threading.Lock()
            side_done_total = 0
            total_side_imgs = saved_count * len(unaligned_cams)
            base_imgs = saved_count * len(aligned_cams)
            last_emit_pct = -1

            def _on_side_frame(t_idx: int):
                nonlocal side_done_total, last_emit_pct
                with done_lock:
                    side_done_total += 1
                    cur_side = side_done_total
                    frac = 0.58 + 0.40 * (cur_side / max(1, total_side_imgs))
                    virt_tri = max(1, min(saved_count - 1, int(round(frac * saved_count))))
                    pct_now = int(round(100.0 * virt_tri / saved_count))
                    emit = (pct_now != last_emit_pct and (cur_side % 6 == 0 or cur_side == total_side_imgs))
                    if emit:
                        last_emit_pct = pct_now
                if emit:
                    print(
                        f"[ExtractProgress] {virt_tri}/{saved_count}|{base_imgs + cur_side}|frame_{t_idx:04d}.jpg|0.0",
                        flush=True,
                    )

            for side_cam in unaligned_cams:
                delta_f = offsets[side_cam] - center_off
                targets = [(i, max(0, cf + delta_f)) for i, cf in enumerate(common_frames, 1)]
                print(
                    f"[Steg 2/3 - Bildextrahering] Extraherar exakt-offset ({delta_f:+d} f) för {side_cam} "
                    f"({len(targets)} bildrutor med flertrådad dekodning)...",
                    flush=True,
                )
                _extract_camera_frames_parallel_cv2(
                    videos[side_cam],
                    targets,
                    output_images_dir / side_cam,
                    lut_path=lut_path,
                    fps=fps,
                    progress_cb=_on_side_frame,
                )

        last_name = f"frame_{saved_count:04d}.jpg"
        dt = time.time() - t0
        print(f"[ExtractProgress] {saved_count}/{saved_count}|{saved_count * 3}|{last_name}|0.0", flush=True)
        print(
            f"[Steg 2/3 - Bildextrahering] KLAR på {dt:.1f} s! Extraherade {saved_count} synkroniserade bild-trior "
            f"({saved_count * 3} bilder totalt, GPU+parallell).",
            flush=True,
        )
        return saved_count
    finally:
        shutil.rmtree(temp_sp_dir, ignore_errors=True)


def _extract_with_opencv_parallel(
    videos: Dict[str, Path],
    offsets: Dict[str, int],
    output_images_dir: Path,
    lut_path: Optional[Path],
    fps: float,
    window_frames: int,
    start_base_frame: int,
    max_sync_frame: int,
) -> int:
    """
    Flertrådad skivad OpenCV-reserv (används när Spirula GPU inte är valt/tillgängligt).
    """
    t0 = time.time()
    window_starts = list(range(start_base_frame, max_sync_frame - window_frames, window_frames))
    total_windows = len(window_starts)
    if total_windows <= 0:
        return 0

    lut_desc = f"med LUT ({lut_path.name})" if (lut_path and lut_path.exists()) else "utan LUT (rå färgprofil)"
    print(
        f"[Steg 2/3 - Bildextrahering] Startar flertrådad Laplacian-skärpefilter {lut_desc} över {total_windows} tidsfönster "
        f"(ca {total_windows * 3} bilder till {output_images_dir})...",
        flush=True,
    )

    eval_stride = 2 if window_frames >= 8 else 1
    center_offset = offsets.get("cam2_center", 0)
    indexed_windows = list(enumerate(window_starts, 1))
    n_scan_workers = min(10, max(2, (os.cpu_count() or 8) // 2), max(1, (total_windows + 7) // 8))
    chunk_sz = (len(indexed_windows) + n_scan_workers - 1) // n_scan_workers
    scan_chunks = [indexed_windows[i : i + chunk_sz] for i in range(0, len(indexed_windows), chunk_sz)]

    results: List[Optional[Tuple[int, int, float]]] = [None] * total_windows
    scan_lock = threading.Lock()
    scan_done = 0

    def _scan_worker(chunk: List[Tuple[int, int]]):
        nonlocal scan_done
        cap_c = cv2.VideoCapture(str(videos["cam2_center"]))
        if not cap_c.isOpened():
            return
        try:
            first_f = max(0, chunk[0][1] + center_offset)
            cap_c.set(cv2.CAP_PROP_POS_FRAMES, first_f)
            cur_f = first_f
            for win_idx, base_idx in chunk:
                start_f = max(0, base_idx + center_offset)
                if start_f < cur_f:
                    cap_c.set(cv2.CAP_PROP_POS_FRAMES, start_f)
                    cur_f = start_f
                while cur_f < start_f:
                    if not cap_c.grab():
                        break
                    cur_f += 1

                best_score = -1.0
                best_sub = 0
                for w in range(window_frames):
                    if w % eval_stride != 0:
                        if not cap_c.grab():
                            break
                        cur_f += 1
                        continue
                    ret, frame = cap_c.read()
                    cur_f += 1
                    if not ret or frame is None:
                        break
                    score = measure_sharpness_laplacian(frame)
                    if score > best_score:
                        best_score = score
                        best_sub = w

                results[win_idx - 1] = (win_idx, base_idx + best_sub, max(0.0, best_score))
                with scan_lock:
                    scan_done += 1
                    done_now = scan_done
                if done_now % 8 == 0 or done_now == total_windows:
                    virt = max(1, int(round(0.40 * done_now)))
                    print(
                        f"[ExtractProgress] {virt}/{total_windows}|0|frame_{win_idx:04d}.jpg|{best_score:.1f}",
                        flush=True,
                    )
        finally:
            cap_c.release()

    with ThreadPoolExecutor(max_workers=len(scan_chunks)) as ex:
        list(ex.map(_scan_worker, scan_chunks))

    chosen_list = [r for r in results if r is not None]
    if not chosen_list:
        return 0

    total_triplets = len(chosen_list)
    score_map = {win_idx: score for win_idx, _, score in chosen_list}
    cam_keys = ["cam1_left", "cam2_center", "cam3_right"]

    extract_lock = threading.Lock()
    saved_imgs_total = 0

    def _on_img_saved(t_idx: int):
        nonlocal saved_imgs_total
        with extract_lock:
            saved_imgs_total += 1
            cur_imgs = saved_imgs_total
        if cur_imgs % 9 == 0 or cur_imgs == total_triplets * 3:
            frac = 0.40 + 0.60 * (cur_imgs / max(1, total_triplets * 3))
            virt = max(1, min(total_triplets, int(round(frac * total_triplets))))
            sc = score_map.get(t_idx, 0.0)
            print(
                f"[ExtractProgress] {virt}/{total_triplets}|{cur_imgs}|frame_{t_idx:04d}.jpg|{sc:.1f}",
                flush=True,
            )

    def _extract_cam(cam_id: str):
        off = offsets.get(cam_id, 0)
        targets = [(i, max(0, sync_idx + off)) for i, (_, sync_idx, _) in enumerate(chosen_list, 1)]
        _extract_camera_frames_parallel_cv2(
            videos[cam_id],
            targets,
            output_images_dir / cam_id,
            lut_path=lut_path,
            fps=fps,
            progress_cb=_on_img_saved,
        )

    with ThreadPoolExecutor(max_workers=3) as cam_ex:
        list(cam_ex.map(_extract_cam, cam_keys))

    last_name = f"frame_{total_triplets:04d}.jpg"
    dt = time.time() - t0
    print(f"[ExtractProgress] {total_triplets}/{total_triplets}|{total_triplets * 3}|{last_name}|{score_map.get(total_triplets, 0.0):.1f}", flush=True)
    print(
        f"[Steg 2/3 - Bildextrahering] KLAR på {dt:.1f} s! Extraherade {total_triplets} synkroniserade bild-trior ({total_triplets * 3} bilder totalt).",
        flush=True,
    )
    return total_triplets


def extract_sharpest_synced_triplets(
    videos: Dict[str, Path],
    offsets: Dict[str, int],
    output_images_dir: Path,
    lut_path: Optional[Path] = None,
    fps: float = 50.0,
    window_sec: float = 0.4,
    skip_start_sec: float = 3.0,
    spirula_bin: str = DEFAULT_SPIRULA_BIN,
    extract_backend: str = "auto",
) -> int:
    """
    AI-AGENT NOTE: Stegar igenom videofilerna i synkroniserade tidsfönster (t.ex. 0.4s = 20 rutor vid 50fps).
    Använder i första hand Spirula Vulkan GPU-avkodning + parallell skivning för maximal fart
    med exakt 1-frame klapp-synk, och faller automatiskt tillbaka på flertrådad OpenCV.
    """
    window_frames = max(1, int(round(window_sec * fps)))
    start_base_frame = int(round(skip_start_sec * fps))

    _clear_existing_jpgs(output_images_dir, videos.keys())

    total_frames_list = []
    for cam_id, path in videos.items():
        cap = cv2.VideoCapture(str(path))
        total_frames_list.append(int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) - offsets.get(cam_id, 0))
        cap.release()
    max_sync_frame = min(total_frames_list) if total_frames_list else 0

    spirula_resolved = shutil.which(spirula_bin) or (spirula_bin if Path(spirula_bin).is_file() else None)
    use_spirula = extract_backend in ("auto", "spirula_gpu") and spirula_resolved is not None

    if use_spirula:
        try:
            return _extract_with_spirula_gpu(
                videos=videos,
                offsets=offsets,
                output_images_dir=output_images_dir,
                lut_path=lut_path,
                fps=fps,
                window_frames=window_frames,
                start_base_frame=start_base_frame,
                max_sync_frame=max_sync_frame,
                spirula_bin=spirula_resolved,
            )
        except Exception as e:
            if extract_backend == "spirula_gpu":
                raise
            print(f"[VARNING] Spirula GPU-extraktion misslyckades ({e}), växlar till flertrådad OpenCV...", flush=True)

    return _extract_with_opencv_parallel(
        videos=videos,
        offsets=offsets,
        output_images_dir=output_images_dir,
        lut_path=lut_path,
        fps=fps,
        window_frames=window_frames,
        start_base_frame=start_base_frame,
        max_sync_frame=max_sync_frame,
    )


# =====================================================================
# 4. ROTATIONSMATEMATIK & AUTOMATISK RIGG-KALIBRERING (FREESTYLE-LÄGE)
# =====================================================================

def euler_to_rotmat(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    """Bygger rotationsmatris R = R_roll @ R_pitch @ R_yaw i COLMAP:s kameraram (X=höger, Y=ner, Z=framåt)."""
    yaw = np.radians(yaw_deg)
    pitch = np.radians(pitch_deg)
    roll = np.radians(roll_deg)

    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)

    R_yaw = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    R_pitch = np.array([[1.0, 0.0, 0.0], [0.0, cp, -sp], [0.0, sp, cp]])
    R_roll = np.array([[cr, -sr, 0.0], [sr, cr, 0.0], [0.0, 0.0, 1.0]])
    return R_roll @ R_pitch @ R_yaw


def rotmat_to_euler_deg(R: np.ndarray) -> Tuple[float, float, float]:
    """Extraherar (yaw_deg, pitch_deg, roll_deg) från R = R_roll @ R_pitch @ R_yaw."""
    pitch = np.arcsin(np.clip(R[2, 1], -1.0, 1.0))
    if np.abs(np.cos(pitch)) > 1e-6:
        yaw = np.arctan2(-R[2, 0], R[2, 2])
        roll = np.arctan2(-R[0, 1], R[1, 1])
    else:
        yaw = np.arctan2(R[0, 2], R[0, 0])
        roll = 0.0
    return float(np.degrees(yaw)), float(np.degrees(pitch)), float(np.degrees(roll))


def rotmat_to_quaternion_wxyz(R: np.ndarray) -> List[float]:
    """Konverterar en 3x3 rotationsmatris till enhetskvaternion [qw, qx, qy, qz]."""
    trace = float(R[0, 0] + R[1, 1] + R[2, 2])
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    q = np.array([qw, qx, qy, qz], dtype=np.float64)
    q /= np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    return [float(x) for x in q]


def quaternion_wxyz_to_rotmat(q: List[float]) -> np.ndarray:
    """Konverterar kvaternion [qw, qx, qy, qz] till 3x3 rotationsmatris."""
    qw, qx, qy, qz = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)]
    ], dtype=np.float64)


def euler_to_quaternion_wxyz(yaw_deg: float, pitch_deg: float, roll_deg: float) -> List[float]:
    """Konverterar riggens relativa vinklar till COLMAP-kvaternion [qw, qx, qy, qz]."""
    return rotmat_to_quaternion_wxyz(euler_to_rotmat(yaw_deg, pitch_deg, roll_deg))


def read_colmap_images_bin(images_bin_path: Path) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    """
    Läser COLMAP:s `images.bin` och returnerar en dict:
      { "cam1_left/frame_0001.jpg": (R_cam_from_world, t_cam_from_world) }
    """
    poses = {}
    data = images_bin_path.read_bytes()
    offset = 0
    num_images = struct.unpack_from("<Q", data, offset)[0]
    offset += 8

    for _ in range(num_images):
        _image_id = struct.unpack_from("<I", data, offset)[0]
        qw, qx, qy, qz, tx, ty, tz = struct.unpack_from("<7d", data, offset + 4)
        _camera_id = struct.unpack_from("<I", data, offset + 60)[0]
        offset += 64

        name_end = data.index(b"\x00", offset)
        name = data[offset:name_end].decode("utf-8", errors="replace").replace("\\", "/")
        offset = name_end + 1

        num_points2D = struct.unpack_from("<Q", data, offset)[0]
        offset += 8 + num_points2D * 24  # 2x double (x, y) + 1x uint64 (point3D_id)

        R_cw = quaternion_wxyz_to_rotmat([qw, qx, qy, qz])
        t_cw = np.array([tx, ty, tz], dtype=np.float64)
        poses[name] = (R_cw, t_cw)

    return poses


def scale_colmap_sparse_model(sparse_dir: Path, scale_factor: float):
    """
    Skalar om alla kameratranslationer i `images.bin` och alla 3D-punkter i `points3D.bin`
    med `scale_factor` så att modellen hamnar i exakt 1:1 meterskala!
    """
    if abs(scale_factor - 1.0) < 1e-6:
        return

    images_bin = sparse_dir / "images.bin"
    if images_bin.exists():
        data = bytearray(images_bin.read_bytes())
        offset = 0
        num_images = struct.unpack_from("<Q", data, offset)[0]
        offset += 8
        for _ in range(num_images):
            tx, ty, tz = struct.unpack_from("<3d", data, offset + 4 + 32)
            struct.pack_into("<3d", data, offset + 4 + 32, tx * scale_factor, ty * scale_factor, tz * scale_factor)
            offset += 64
            name_end = data.index(b"\x00", offset)
            offset = name_end + 1
            num_pts2d = struct.unpack_from("<Q", data, offset)[0]
            offset += 8 + num_pts2d * 24
        images_bin.write_bytes(bytes(data))

    points_bin = sparse_dir / "points3D.bin"
    if points_bin.exists():
        data = bytearray(points_bin.read_bytes())
        offset = 0
        num_pts = struct.unpack_from("<Q", data, offset)[0]
        offset += 8
        for _ in range(num_pts):
            x, y, z = struct.unpack_from("<3d", data, offset + 8)
            struct.pack_into("<3d", data, offset + 8, x * scale_factor, y * scale_factor, z * scale_factor)
            offset += 8 + 24 + 3 + 8  # point3D_id(8) + xyz(24) + rgb(3) + error(8)
            track_len = struct.unpack_from("<Q", data, offset)[0]
            offset += 8 + track_len * 8
        points_bin.write_bytes(bytes(data))


def calibrate_rig_from_sparse_model(
    sparse_dir: Path,
    baseline_mm: float = 300.0,
    save_calibrated_path: Path = CALIBRATED_RIG_PATH
) -> Dict[str, Dict[str, float]]:
    """
    AI-AGENT NOTE: "Freestyle Rigg-Kalibrering"!
    Läser en rekonstruerad `sparse/0/images.bin` (från antingen Spirula SfM eller COLMAP mapper),
    och beräknar det EXAKTA relativa förhållandet (Yaw, Pitch, Roll + translation) mellan
    `cam1_left`, `cam2_center` (referens) och `cam3_right` över alla synkroniserade bild-trior.

    Skalar dessutom automatiskt baslinjen till `baseline_mm` (default 300 mm mellan ytterkamerorna,
    dvs. 150 mm per sida) och sparar resultatet till `config/calibrated_rig.json`.
    """
    images_bin = sparse_dir / "images.bin"
    if not images_bin.exists():
        raise FileNotFoundError(f"Hittade ingen {images_bin} att kalibrera riggen från.")

    poses = read_colmap_images_bin(images_bin)

    # Gruppera per frame_XXXX.jpg
    frames: Dict[str, Dict[str, Tuple[np.ndarray, np.ndarray]]] = {}
    for img_name, pose in poses.items():
        parts = img_name.split("/")
        if len(parts) >= 2:
            cam_folder, frame_file = parts[-2], parts[-1]
            frames.setdefault(frame_file, {})[cam_folder] = pose

    rel_rotations: Dict[str, List[np.ndarray]] = {"cam1_left": [], "cam3_right": []}
    rel_translations_in_ref: Dict[str, List[np.ndarray]] = {"cam1_left": [], "cam3_right": []}

    for frame_file, cam_dict in sorted(frames.items()):
        if "cam2_center" not in cam_dict:
            continue
        R_ref, t_ref = cam_dict["cam2_center"]
        # Kamerans centrum i världskoordinater: C = -R^T @ t
        C_ref_w = -R_ref.T @ t_ref

        for side_cam in ("cam1_left", "cam3_right"):
            if side_cam not in cam_dict:
                continue
            R_side, t_side = cam_dict[side_cam]
            C_side_w = -R_side.T @ t_side

            # Relativ rotation som tar en vektor från ref (cam2_center) till side_cam:
            # p_side = R_rel @ p_ref + t_rel
            R_rel = R_side @ R_ref.T
            # Sidokamerans optiska centrum uttryckt i mittenkamerans koordinatsystem:
            pos_in_ref = R_ref @ (C_side_w - C_ref_w)

            rel_rotations[side_cam].append(R_rel)
            rel_translations_in_ref[side_cam].append(pos_in_ref)

    if not rel_rotations["cam1_left"] or not rel_rotations["cam3_right"]:
        raise RuntimeError(
            "För få gemensamt registrerade bild-trior i sparse/0/images.bin för att kalibrera riggen! "
            "Kontrollera att alla 3 kameror ser struktur i rummet."
        )

    # Robust medelrotation via SVD-projicering (Chords/Frobenius-medel på SO(3))
    avg_R: Dict[str, np.ndarray] = {}
    spread_deg: Dict[str, float] = {}
    avg_pos_in_ref: Dict[str, np.ndarray] = {}

    for side_cam in ("cam1_left", "cam3_right"):
        raw_R_list = rel_rotations[side_cam]
        raw_pos_list = rel_translations_in_ref[side_cam]

        # Filtrera bort uppenbart felvända frames (> 30° från både mode_a och mode_b) om minst 2 giltiga finns
        g_a = RIG_MODES["mode_a"][side_cam]
        g_b = RIG_MODES["mode_b"][side_cam]
        R_nom_a = euler_to_rotmat(g_a["yaw"], g_a["pitch"], g_a["roll"])
        R_nom_b = euler_to_rotmat(g_b["yaw"], g_b["pitch"], g_b["roll"])
        inlier_idx: List[int] = []
        for idx_f, R_i in enumerate(raw_R_list):
            ang_a = math.degrees(math.acos(float(np.clip((np.trace(R_i @ R_nom_a.T) - 1.0) * 0.5, -1.0, 1.0))))
            ang_b = math.degrees(math.acos(float(np.clip((np.trace(R_i @ R_nom_b.T) - 1.0) * 0.5, -1.0, 1.0))))
            if min(ang_a, ang_b) <= 30.0:
                inlier_idx.append(idx_f)

        if len(inlier_idx) >= 2:
            R_filt = [raw_R_list[i] for i in inlier_idx]
            pos_filt = [raw_pos_list[i] for i in inlier_idx]
        else:
            R_filt = raw_R_list
            pos_filt = raw_pos_list

        R_stack = np.stack(R_filt, axis=0)
        # Använd median per element följt av SVD-ortogonalisering för att ignorera ev. outliers
        M = np.median(R_stack, axis=0)
        U, _, Vt = np.linalg.svd(M)
        R_mean = U @ Vt
        if np.linalg.det(R_mean) < 0:
            U[:, -1] *= -1
            R_mean = U @ Vt

        # Andra stegets trimning (behåll rutor inom 15° från medianen om >= 2 finns)
        angles_pre = [
            float(np.degrees(np.arccos(np.clip((np.trace(R_i @ R_mean.T) - 1.0) / 2.0, -1.0, 1.0))))
            for R_i in R_stack
        ]
        keep2 = [j for j, a in enumerate(angles_pre) if a <= 15.0]
        if len(keep2) >= 2 and len(keep2) < len(R_filt):
            R_filt = [R_filt[j] for j in keep2]
            pos_filt = [pos_filt[j] for j in keep2]
            R_stack = np.stack(R_filt, axis=0)
            M2 = np.median(R_stack, axis=0)
            U2, _, Vt2 = np.linalg.svd(M2)
            R_mean = U2 @ Vt2
            if np.linalg.det(R_mean) < 0:
                U2[:, -1] *= -1
                R_mean = U2 @ Vt2

        avg_R[side_cam] = R_mean
        rel_rotations[side_cam] = R_filt

        # Beräkna vinkelspridning (stabilitetsmått i grader över sekvensen)
        angles = []
        for R_i in R_stack:
            dR = R_i @ R_mean.T
            cos_theta = np.clip((np.trace(dR) - 1.0) / 2.0, -1.0, 1.0)
            angles.append(np.degrees(np.arccos(cos_theta)))
        spread_deg[side_cam] = float(np.median(angles))

        pos_stack = np.stack(pos_filt, axis=0)
        avg_pos_in_ref[side_cam] = np.median(pos_stack, axis=0)

    # Beräkna skalfaktor så att totala avståndet mellan cam1_left och cam3_right blir exakt baseline_mm
    raw_total_baseline = float(np.linalg.norm(avg_pos_in_ref["cam3_right"] - avg_pos_in_ref["cam1_left"]))
    target_total_m = baseline_mm / 1000.0
    scale_factor = (target_total_m / raw_total_baseline) if raw_total_baseline > 1e-6 else 1.0

    # Skala om sparse-modellen på disk till 1:1 meter
    scale_colmap_sparse_model(sparse_dir, scale_factor)

    # Mittenkamerans fysiska lutning på bommen (+8.0° uppåt => -8.0° i COLMAP Y-ned)
    center_tilt_up_deg = 8.0
    R_center_rig = euler_to_rotmat(0.0, -center_tilt_up_deg, 0.0)

    calibrated_geom: Dict[str, Dict[str, float]] = {
        "cam2_center": {
            "tx": 0.0,
            "ty": 0.0,
            "tz": 0.0,
            "pos_x_m": 0.0,
            "pos_y_m": 0.0,
            "pos_z_m": 0.0,
            "yaw": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
            "phys_yaw": 0.0,
            "phys_tilt": +center_tilt_up_deg,
            "phys_roll": 0.0,
            "spread_deg": 0.0,
        }
    }

    print("\n" + "=" * 78)
    print(f"[RigCalib] AUTOMATISK FREESTYLE-KALIBRERING KLAR (Mål-baslinje: {baseline_mm:.1f} mm)")
    print("=" * 78)

    for side_cam, target_yaw, target_pitch in [("cam1_left", -45.0, 20.0), ("cam3_right", 45.0, 20.0)]:
        R_rel = avg_R[side_cam]
        pos_metric = avg_pos_in_ref[side_cam] * scale_factor
        # COLMAP:s cam_from_rig_translation (t_rel = -R_rel @ pos_in_ref)
        t_cam_from_rig = -R_rel @ pos_metric
        yaw_deg, pitch_deg, roll_deg = rotmat_to_euler_deg(R_rel)

        # Omvandla även till riggens horisontalplan (där cam2_center lutar +8° uppåt)
        R_side_rig = R_rel @ R_center_rig
        rig_y, rig_p, rig_r = rotmat_to_euler_deg(R_side_rig)
        phys_yaw = -rig_y
        phys_tilt = -rig_p
        phys_roll = rig_r

        calibrated_geom[side_cam] = {
            "tx": float(t_cam_from_rig[0]),
            "ty": float(t_cam_from_rig[1]),
            "tz": float(t_cam_from_rig[2]),
            "pos_x_m": float(pos_metric[0]),
            "pos_y_m": float(pos_metric[1]),
            "pos_z_m": float(pos_metric[2]),
            "yaw": yaw_deg,
            "pitch": pitch_deg,
            "roll": roll_deg,
            "phys_yaw": round(phys_yaw, 2),
            "phys_tilt": round(phys_tilt, 2),
            "phys_roll": round(phys_roll, 2),
            "spread_deg": spread_deg[side_cam],
        }
        dist_mm = float(np.linalg.norm(pos_metric) * 1000.0)
        n_used = len(rel_rotations[side_cam])
        print(
            f"  {side_cam:11s}: Fysisk Yaw={phys_yaw:+6.1f}° (mål {target_yaw:+.0f}°) | "
            f"Lutning={phys_tilt:+5.1f}° (mål -12°, ΔPitch={pitch_deg:+.1f}°) | "
            f"Fästes-Roll={phys_roll:+4.1f}° | Bas={dist_mm:5.1f} mm | ±{spread_deg[side_cam]:.2f}° ({n_used} trior)"
        )
        if spread_deg[side_cam] > 1.5:
            print(f"  [!] VARNING: Hög vinkelspridning (±{spread_deg[side_cam]:.2f}°) på {side_cam} – kontrollera att fästet sitter stumt!")

    print("=" * 78 + "\n")

    keep_global_charuco = (
        DJI_RIG_PROFILE_PATH.exists()
        and Path(save_calibrated_path).resolve() == CALIBRATED_RIG_PATH.resolve()
    )
    if not keep_global_charuco:
        save_calibrated_path.parent.mkdir(parents=True, exist_ok=True)
        save_calibrated_path.write_text(json.dumps(calibrated_geom, indent=2), encoding="utf-8")
        print(f"[RigCalib] Sparade uppmätt rigg-profil -> {save_calibrated_path}")
    else:
        print(f"[RigCalib] Behåller global ChArUco-profil i config/ (skriver endast lokalt i projektet).")
    # Spara även direkt i projektets processed-mapp så varje projekt bär sin egen uppmätta kalibrering
    proj_root = sparse_dir.parent.parent
    if proj_root.exists() and proj_root.name != "_temp_precalib":
        try:
            (proj_root / "calibrated_rig.json").write_text(json.dumps(calibrated_geom, indent=2), encoding="utf-8")
        except Exception:
            pass
    return calibrated_geom


def fast_calibrate_rig_from_images(
    project_dir: Path,
    baseline_mm: float = 300.0,
    num_samples: int = 12,
    save_calibrated_path: Path = CALIBRATED_RIG_PATH,
) -> Optional[Dict[str, Dict[str, float]]]:
    """
    AI-AGENT NOTE: Blixtsnabb (~4 sek) direkt Fisheye-Rigg-Kalibrering på extraherade bild-trior!
    Använder OpenCV SIFT + Fisheye-avdistorsion (`cv2.fisheye.undistortPoints`) + 3D-strålrotation (Kabsch/SVD RANSAC)
    med kompensering för den kända baslinjen (`±150 mm`) så att horisontell parallax inte förväxlas med Yaw.
    Beräknar både:
      1) Fysiska vinklar på riggbommen (där Mitten lutar +8° upp, Sidorna ±45° utåt och -12° nedåt)
      2) Relativa COLMAP-vinklar (`R_rel`, där ΔPitch = 12° + 8° = 20°).
    """
    images_dir = project_dir / "images"
    center_imgs = sorted((images_dir / "cam2_center").glob("*.jpg"))
    if not center_imgs:
        print("[RigCalib] Inga extraherade bilder hittades i processed/images/cam2_center.", flush=True)
        return None

    step = max(1, len(center_imgs) // num_samples)
    sample_names = [p.name for p in center_imgs[::step][:num_samples]]
    print(
        f"[Steg 3/3 - Snabb Rigg-Kalibrering] Mäter exakta kameravinklar (3D-Fisheye SIFT + Kabsch RANSAC) "
        f"över {len(sample_names)} bild-trior...",
        flush=True,
    )

    scale = 0.35
    f_full = 1600.0
    d_fisheye = np.array([-0.045, 0.012, -0.003, 0.0005], dtype=np.float64)
    sift = cv2.SIFT_create(nfeatures=1800)
    flann = cv2.FlannBasedMatcher(dict(algorithm=1, trees=5), dict(checks=40))

    half_base_m = (baseline_mm / 1000.0) / 2.0
    side_pos_x = {"cam1_left": -half_base_m, "cam3_right": +half_base_m}

    rel_rotations: Dict[str, List[np.ndarray]] = {"cam1_left": [], "cam3_right": []}
    inlier_counts: Dict[str, List[int]] = {"cam1_left": [], "cam3_right": []}
    rng = np.random.default_rng(42)
    cos_thresh = float(np.cos(np.radians(1.5)))

    for idx, fname in enumerate(sample_names, 1):
        im_ref_full = cv2.imread(str(images_dir / "cam2_center" / fname), cv2.IMREAD_GRAYSCALE)
        if im_ref_full is None:
            continue
        im_ref = cv2.resize(im_ref_full, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        f_scaled = f_full * scale
        K_ref = np.array(
            [[f_scaled, 0.0, im_ref.shape[1] / 2.0], [0.0, f_scaled, im_ref.shape[0] / 2.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        kp_ref, des_ref = sift.detectAndCompute(im_ref, None)
        if des_ref is None or len(kp_ref) < 20:
            continue

        for side_cam in ("cam1_left", "cam3_right"):
            im_side_full = cv2.imread(str(images_dir / side_cam / fname), cv2.IMREAD_GRAYSCALE)
            if im_side_full is None:
                continue
            im_side = cv2.resize(im_side_full, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            K_side = np.array(
                [[f_scaled, 0.0, im_side.shape[1] / 2.0], [0.0, f_scaled, im_side.shape[0] / 2.0], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            )
            kp_side, des_side = sift.detectAndCompute(im_side, None)
            if des_side is None or len(kp_side) < 20:
                continue

            matches = flann.knnMatch(des_ref, des_side, k=2)
            good = [m for m, n in matches if m.distance < 0.75 * n.distance]
            if len(good) < 15:
                continue

            pts_ref = np.float64([kp_ref[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            pts_side = np.float64([kp_side[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

            n1 = cv2.fisheye.undistortPoints(pts_ref, K_ref, d_fisheye).reshape(-1, 2)
            n2 = cv2.fisheye.undistortPoints(pts_side, K_side, d_fisheye).reshape(-1, 2)

            # Kompensera för känd baslinje (pos_x = ±0.15m) vid normal rumslig djupdistans (~2.5 m)
            z_est = 2.5
            px = side_pos_x[side_cam]
            P_ref = np.column_stack([n1[:, 0] * z_est - px, n1[:, 1] * z_est, np.full(len(n1), z_est)])
            v1 = P_ref / np.linalg.norm(P_ref, axis=1, keepdims=True)
            v2 = np.column_stack([n2[:, 0], n2[:, 1], np.ones(len(n2))])
            v2 = v2 / np.linalg.norm(v2, axis=1, keepdims=True)

            best_mask = None
            best_cnt = 0
            n_pts = len(v1)
            for _ in range(250):
                sample_idx = rng.choice(n_pts, size=3, replace=False)
                H = v2[sample_idx].T @ v1[sample_idx]
                U, _, Vt = np.linalg.svd(H)
                R_try = U @ Vt
                if np.linalg.det(R_try) < 0:
                    U[:, -1] *= -1
                    R_try = U @ Vt
                dots = np.sum(v2 * (v1 @ R_try.T), axis=1)
                mask = dots > cos_thresh
                cnt = int(np.sum(mask))
                if cnt > best_cnt:
                    best_cnt = cnt
                    best_mask = mask

            if best_mask is not None and best_cnt >= 10:
                H = v2[best_mask].T @ v1[best_mask]
                U, _, Vt = np.linalg.svd(H)
                R_opt = U @ Vt
                if np.linalg.det(R_opt) < 0:
                    U[:, -1] *= -1
                    R_opt = U @ Vt
                rel_rotations[side_cam].append(R_opt)
                inlier_counts[side_cam].append(best_cnt)

    if not rel_rotations["cam1_left"] or not rel_rotations["cam3_right"]:
        print(
            "[VARNING] [RigCalib] För få gemensamma SIFT-träffar mellan kamerorna för att kalibrera automatiskt.",
            flush=True,
        )
        return None

    # Mittenkamerans fysiska lutning på riggbommen (+8.0° uppåt => pitch = -8.0° i COLMAP Y-ned)
    center_tilt_up_deg = 8.0
    R_center_rig = euler_to_rotmat(0.0, -center_tilt_up_deg, 0.0)

    calibrated_geom: Dict[str, Dict[str, float]] = {
        "cam2_center": {
            "tx": 0.0,
            "ty": 0.0,
            "tz": 0.0,
            "pos_x_m": 0.0,
            "pos_y_m": 0.0,
            "pos_z_m": 0.0,
            "yaw": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
            "phys_yaw": 0.0,
            "phys_tilt": +center_tilt_up_deg,
            "phys_roll": 0.0,
            "spread_deg": 0.0,
            "avg_inliers": 0.0,
        }
    }

    print("\n" + "=" * 86, flush=True)
    print(
        f"[RigCalib] KAMERAKALIBRERING KLAR UR BILD-TRIOR (Baslinje: {baseline_mm:.0f} mm | Mitten: +{center_tilt_up_deg:.0f}° upp)",
        flush=True,
    )
    print("=" * 86, flush=True)

    for side_cam, default_x, target_phys_yaw in [
        ("cam1_left", -half_base_m, -45.0),
        ("cam3_right", +half_base_m, +45.0),
    ]:
        R_stack = np.stack(rel_rotations[side_cam], axis=0)
        M = np.median(R_stack, axis=0)
        U, _, Vt = np.linalg.svd(M)
        R_mean = U @ Vt
        if np.linalg.det(R_mean) < 0:
            U[:, -1] *= -1
            R_mean = U @ Vt

        angles = []
        for R_i in R_stack:
            dR = R_i @ R_mean.T
            cos_theta = np.clip((np.trace(dR) - 1.0) / 2.0, -1.0, 1.0)
            angles.append(np.degrees(np.arccos(cos_theta)))
        spread = float(np.median(angles))
        avg_inl = float(np.mean(inlier_counts[side_cam]))

        pos_metric = np.array([default_x, 0.0, 0.0], dtype=np.float64)
        t_cam_from_rig = -R_mean @ pos_metric
        yaw_deg, pitch_deg, roll_deg = rotmat_to_euler_deg(R_mean)

        # Omvandla från lutande mittenkamera (+8° upp) till Riggens fysiska horisontalplan:
        R_side_rig = R_mean @ R_center_rig
        rig_y, rig_p, rig_r = rotmat_to_euler_deg(R_side_rig)
        phys_yaw = -rig_y
        phys_tilt = -rig_p
        phys_roll = rig_r

        calibrated_geom[side_cam] = {
            "tx": float(t_cam_from_rig[0]),
            "ty": float(t_cam_from_rig[1]),
            "tz": float(t_cam_from_rig[2]),
            "pos_x_m": float(pos_metric[0]),
            "pos_y_m": 0.0,
            "pos_z_m": 0.0,
            "yaw": round(yaw_deg, 2),
            "pitch": round(pitch_deg, 2),
            "roll": round(roll_deg, 2),
            "phys_yaw": round(phys_yaw, 2),
            "phys_tilt": round(phys_tilt, 2),
            "phys_roll": round(phys_roll, 2),
            "spread_deg": round(spread, 2),
            "avg_inliers": round(avg_inl, 1),
        }
        print(
            f"  [OK] {side_cam:10s}: Fysisk Yaw={phys_yaw:+5.1f}° (CAD {target_phys_yaw:+.0f}°) | "
            f"Lutning={phys_tilt:+5.1f}° ned (CAD -12.0°, ΔPitch={pitch_deg:+.1f}°) | "
            f"Roll={phys_roll:+4.1f}° | ±{spread:.2f}° ({avg_inl:.0f} pt)",
            flush=True,
        )

    print("=" * 86 + "\n", flush=True)

    try:
        (project_dir / "calibrated_rig.json").write_text(json.dumps(calibrated_geom, indent=2), encoding="utf-8")
    except Exception as e:
        print(f"[VARNING] Kunde inte spara {project_dir / 'calibrated_rig.json'}: {e}", flush=True)

    keep_global_charuco = (
        DJI_RIG_PROFILE_PATH.exists()
        and Path(save_calibrated_path).resolve() == CALIBRATED_RIG_PATH.resolve()
    )
    if not keep_global_charuco:
        try:
            save_calibrated_path.parent.mkdir(parents=True, exist_ok=True)
            save_calibrated_path.write_text(json.dumps(calibrated_geom, indent=2), encoding="utf-8")
        except Exception:
            pass

    return calibrated_geom


# =====================================================================
# 5. COLMAP & SPIRULA RIGG-SFM (STÖDER BÅDE FREESTYLE & FASTA LÄGEN)
# =====================================================================

def _normalize_geom_dict(raw_geom: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    """Normaliserar både platt calibrated_rig.json och v2 dji_rig_profile.json till enhetlig rigg-dict."""
    if "per_camera" in raw_geom and isinstance(raw_geom["per_camera"], dict):
        img_sz = raw_geom.get("image_size") or [2880, 3840]
        cw, ch = int(img_sz[0]), int(img_sz[1])
        out: Dict[str, Dict[str, Any]] = {}
        for ck in ("cam1_left", "cam2_center", "cam3_right"):
            pc = raw_geom["per_camera"][ck]
            eul = pc.get("rel_euler_deg") or [0.0, 0.0, 0.0]
            tv = pc.get("cam_from_rig_tvec_m") or [0.0, 0.0, 0.0]
            pos = pc.get("pos_in_ref_m") or [0.0, 0.0, 0.0]
            out[ck] = {
                "tx": float(tv[0]),
                "ty": float(tv[1]),
                "tz": float(tv[2]),
                "pos_x_m": float(pos[0]),
                "pos_y_m": float(pos[1]),
                "pos_z_m": float(pos[2]),
                "yaw": float(eul[0]),
                "pitch": float(eul[1]),
                "roll": float(eul[2]),
                "phys_yaw": float(pc.get("phys_yaw", 0.0)),
                "phys_tilt": float(pc.get("phys_tilt", 0.0)),
                "phys_roll": float(pc.get("phys_roll", 0.0)),
                "spread_deg": float(pc.get("spread_deg", 0.0)),
                "qvec_wxyz": list(pc.get("cam_from_rig_qvec_wxyz") or [1.0, 0.0, 0.0, 0.0]),
                "intrinsics": list(pc.get("intrinsics") or ACTION4_PORTRAIT_FISHEYE_ARRAY),
                "width": cw,
                "height": ch,
            }
        return out
    return raw_geom


def get_rig_geometry(mode: str) -> Dict[str, Dict[str, Any]]:
    """Hämtar antingen ett fast CAD-läge (mode_a/mode_b) eller senast uppmätta ChArUco/calibrated-profil."""
    if mode in RIG_MODES:
        return RIG_MODES[mode]
    if DJI_RIG_PROFILE_PATH.exists():
        try:
            return _normalize_geom_dict(json.loads(DJI_RIG_PROFILE_PATH.read_text(encoding="utf-8")))
        except Exception:
            pass
    if CALIBRATED_RIG_PATH.exists():
        return _normalize_geom_dict(json.loads(CALIBRATED_RIG_PATH.read_text(encoding="utf-8")))
    return RIG_MODES["mode_a"]


def generate_colmap_rig_config(
    rig_json_path: Path,
    mode: str = "mode_a",
    custom_geom: Optional[Dict[str, Any]] = None
) -> Path:
    """
    AI-AGENT NOTE: Skapar både:
      1) rig_config.json (COLMAP 3.12+ nytt format med cam_from_rig_rotation / cam_from_rig_translation)
      2) rig_config_legacy.json (COLMAP <= 3.11 format med ref_camera_id / rel_qvec / rel_tvec)
    Om `mode == 'auto_calibrate'` och `custom_geom is None` skapas även `rig_config_uncalibrated.json`
    (utan förvalda vinklar) så att COLMAP 3.12 `rig_configurator` kan estimera vinklarna fritt.
    """
    geom = _normalize_geom_dict(custom_geom) if custom_geom is not None else get_rig_geometry(mode)
    cameras_modern = []
    cameras_legacy = []
    cameras_uncalibrated = []

    cam_id_map = {"cam1_left": 1, "cam2_center": 2, "cam3_right": 3}
    images_base = rig_json_path.parent / "images"
    for name, cid in cam_id_map.items():
        g = geom[name]
        q_wxyz = list(g.get("qvec_wxyz") or euler_to_quaternion_wxyz(g["yaw"], g["pitch"], g["roll"]))

        cam_params = list(g.get("intrinsics") or ACTION4_PORTRAIT_FISHEYE_ARRAY)
        cal_w = int(g.get("width") or 2880)
        cal_h = int(g.get("height") or 3840)
        cam_img_dir = images_base / name
        if cam_img_dir.exists() and cv2 is not None:
            sample_imgs = sorted(cam_img_dir.glob("*.jpg"))
            if sample_imgs:
                im = cv2.imread(str(sample_imgs[0]))
                if im is not None:
                    h_px, w_px = im.shape[:2]
                    if w_px != cal_w or h_px != cal_h:
                        sx = float(w_px) / max(1, cal_w)
                        sy = float(h_px) / max(1, cal_h)
                        cam_params[0] *= sx
                        cam_params[1] *= sy
                        cam_params[2] *= sx
                        cam_params[3] *= sy

        # COLMAP 3.12+ schema (med kända/kalibrerade extrinsics)
        modern_entry = {
            "image_prefix": f"{name}/",
            "camera_model_name": "OPENCV_FISHEYE",
            "camera_params": cam_params,
        }
        uncal_entry = {
            "image_prefix": f"{name}/",
            "camera_model_name": "OPENCV_FISHEYE",
            "camera_params": cam_params,
        }
        if name == "cam2_center":
            modern_entry["ref_sensor"] = True
            uncal_entry["ref_sensor"] = True
        else:
            modern_entry["cam_from_rig_rotation"] = q_wxyz
            modern_entry["cam_from_rig_translation"] = [g["tx"], g["ty"], g["tz"]]

        cameras_modern.append(modern_entry)
        cameras_uncalibrated.append(uncal_entry)

        # COLMAP <= 3.11 schema (rig_bundle_adjuster)
        legacy_entry = {
            "camera_id": cid,
            "image_prefix": f"{name}/",
            "rel_tvec": [g["tx"], g["ty"], g["tz"]],
            "rel_qvec": q_wxyz,
        }
        if name == "cam2_center":
            legacy_entry["ref_camera"] = True
        cameras_legacy.append(legacy_entry)

    rig_json_path.parent.mkdir(parents=True, exist_ok=True)
    rig_json_path.write_text(json.dumps([{"cameras": cameras_modern}], indent=2), encoding="utf-8")

    legacy_path = rig_json_path.with_name("rig_config_legacy.json")
    legacy_path.write_text(
        json.dumps([{"ref_camera_id": 2, "cameras": cameras_legacy}], indent=2),
        encoding="utf-8",
    )

    uncal_path = rig_json_path.with_name("rig_config_uncalibrated.json")
    uncal_path.write_text(json.dumps([{"cameras": cameras_uncalibrated}], indent=2), encoding="utf-8")
    return rig_json_path


def run_spirula_rig_sfm(
    project_dir: Path,
    mode: str = "auto_calibrate",
    baseline_mm: float = 300.0,
    spirula_bin: str = DEFAULT_SPIRULA_BIN
) -> Path:
    """
    AI-AGENT NOTE: Kör Spirula Studios inbyggda Vulkan-accelererade SfM (`spirula.exe sfm auto`)
    med separata sekvenser per kamera (`--sequence cam1_left --sequence cam2_center --sequence cam3_right`)
    och låsta OPENCV_FISHEYE-linsparametrar per kamera.
    För låsta lägen (`calibrated`, `mode_a`, `mode_b`) körs sedan `enforce_calibrated_rig_on_sparse_model`
    som skriver exakta kalibrerade kameror till `cameras.bin`, skalar till 1:1 meter, räddar
    borttappade bilder och optimerar gemensamma 6-DoF rigg-poser + 3D-punkter.
    """
    images_dir = project_dir / "images"
    cam_order = ("cam1_left", "cam2_center", "cam3_right")

    # Välj rigg-specifikation för låsta lägen + lins-intrinsics
    proj_profile_p = project_dir / "dji_rig_profile.json"
    rig_spec_raw: Optional[Dict[str, Any]] = None
    if mode == "calibrated":
        if proj_profile_p.exists():
            rig_spec_raw = json.loads(proj_profile_p.read_text(encoding="utf-8"))
        elif DJI_RIG_PROFILE_PATH.exists():
            rig_spec_raw = json.loads(DJI_RIG_PROFILE_PATH.read_text(encoding="utf-8"))
        elif CALIBRATED_RIG_PATH.exists():
            rig_spec_raw = json.loads(CALIBRATED_RIG_PATH.read_text(encoding="utf-8"))
        else:
            rig_spec_raw = RIG_MODES["mode_a"]
    elif mode in RIG_MODES:
        rig_spec_raw = RIG_MODES[mode]
    elif DJI_RIG_PROFILE_PATH.exists():
        # Även i Freestyle kan vi ge Spirula kända startvärden på linserna från ChArUco
        rig_spec_raw = json.loads(DJI_RIG_PROFILE_PATH.read_text(encoding="utf-8"))

    geom_norm = _normalize_geom_dict(rig_spec_raw) if rig_spec_raw is not None else RIG_MODES["mode_a"]

    total_imgs = 0
    focal_args: List[str] = []
    dist_args: List[str] = []
    for ck in cam_order:
        cdir = images_dir / ck
        jpgs = sorted(cdir.glob("*.jpg")) if cdir.exists() else []
        total_imgs += len(jpgs)

        g = geom_norm[ck]
        params = list(g.get("intrinsics") or ACTION4_PORTRAIT_FISHEYE_ARRAY)
        cal_w = int(g.get("width") or 2880)
        cal_h = int(g.get("height") or 3840)
        if jpgs and cv2 is not None:
            im0 = cv2.imread(str(jpgs[0]))
            if im0 is not None:
                h_px, w_px = im0.shape[:2]
                if w_px != cal_w or h_px != cal_h:
                    sx = float(w_px) / max(1, cal_w)
                    sy = float(h_px) / max(1, cal_h)
                    params[0] *= sx
                    params[1] *= sy
                    params[2] *= sx
                    params[3] *= sy
        f_avg = 0.5 * (float(params[0]) + float(params[1]))
        focal_args.extend(["--focal", f"{ck}={f_avg:.4f}"])
        dist_args.extend(["--distortion", f"{ck}={params[4]:.6f},{params[5]:.6f},{params[6]:.6f},{params[7]:.6f}"])

    cmd = [
        spirula_bin, "sfm", "auto",
        str(images_dir),
        "-o", str(project_dir),
        "--device", "auto",
        "--data-type", "video",
        "--camera-mode", "folder",
        "--camera-model", "opencv-fisheye",
        *focal_args,
        *dist_args,
        "--no-exif-focal",
        "--focal-trials", "0",
        "--no-refine-extra-params",
        "--no-final-extra-params",
        "--no-final-principal-point",
        "--rig", "cam1_left,cam2_center,cam3_right",
        "--sequence", "cam1_left",
        "--sequence", "cam2_center",
        "--sequence", "cam3_right",
    ]
    if total_imgs <= 270:
        cmd.extend(["--pairs", "exhaustive"])

    print(
        f"[Steg 3/3 - Spirula SfM] Kör Vulkan GPU Rigg-SfM ({total_imgs} bilder, läge={mode}, "
        f"pairs={'exhaustive' if total_imgs <= 270 else 'auto'}) -> {project_dir / 'sparse' / '0'} ...",
        flush=True,
    )
    res = subprocess.run(cmd, check=False)

    sparse_dir = project_dir / "sparse" / "0"
    if not (sparse_dir / "images.bin").exists():
        raise RuntimeError(
            f"Spirula SfM avslutades med kod {res.returncode} och skapade ingen sparse/0/images.bin. Se loggen ovan."
        )

    if mode in ("calibrated", "mode_a", "mode_b") and rig_spec_raw is not None:
        print(f"[Steg 3/3 - RigLock] Låser sparse/0 exakt till uppmätt/vald rigg & 1:1 meterskala...", flush=True)
        src_dir = str(Path(__file__).resolve().parent)
        if src_dir not in sys.path:
            sys.path.insert(0, src_dir)
        from charuco_rig_calibration import enforce_calibrated_rig_on_sparse_model

        enforce_calibrated_rig_on_sparse_model(sparse_dir, project_dir, rig_spec_raw, progress_cb=print)
        try:
            (project_dir / "calibrated_rig.json").write_text(json.dumps(geom_norm, indent=2), encoding="utf-8")
        except Exception:
            pass
    else:
        print(f"[Steg 3/3 - Spirula SfM] 3D-rekonstruktion skapad i {sparse_dir}! Mäter kameravinklar & skalar till {baseline_mm:.0f} mm...", flush=True)
        calibrated_geom = calibrate_rig_from_sparse_model(sparse_dir, baseline_mm=baseline_mm)
        generate_colmap_rig_config(project_dir / "rig_config.json", mode=mode, custom_geom=calibrated_geom)
    return project_dir


def _run_colmap_fast_subset_calibration(
    project_dir: Path,
    baseline_mm: float,
    colmap_bin: str,
    max_calib_triplets: int = 18
) -> Optional[Dict[str, Dict[str, float]]]:
    """
    AI-AGENT NOTE: Plockar ut de första `max_calib_triplets` (t.ex. 18 st = 54 bilder) direkt ur
    DET RIKTIGA VIDEOKLIPPET, kör en blixtsnabb (~15 s) för-kalibrering för att låsa exakt
    Yaw/Pitch/Roll + baslinje mellan kamerorna, och raderar sedan temp-mappen.
    Då behövs INGEN separat provfilmning innan den riktiga skanningen!
    """
    images_dir = project_dir / "images"
    center_imgs = sorted((images_dir / "cam2_center").glob("*.jpg"))
    if len(center_imgs) <= max_calib_triplets:
        return None  # Datasetet är redan litet nog att kalibreras direkt i huvudkörningen

    print(f"[RigCalib] Kör snabb för-kalibrering på de första {max_calib_triplets} bild-triorna ur ditt riktiga klipp...")
    calib_dir = project_dir / "_temp_precalib"
    if calib_dir.exists():
        shutil.rmtree(calib_dir)

    calib_imgs = calib_dir / "images"
    for cam_id in ("cam1_left", "cam2_center", "cam3_right"):
        (calib_imgs / cam_id).mkdir(parents=True, exist_ok=True)

    for f_path in center_imgs[:max_calib_triplets]:
        fname = f_path.name
        for cam_id in ("cam1_left", "cam2_center", "cam3_right"):
            src = images_dir / cam_id / fname
            if src.exists():
                shutil.copy2(src, calib_imgs / cam_id / fname)

    db_path = calib_dir / "database.db"
    sparse_zero = calib_dir / "sparse" / "0"
    sparse_zero.mkdir(parents=True, exist_ok=True)

    subprocess.run([
        colmap_bin, "feature_extractor",
        "--database_path", str(db_path),
        "--image_path", str(calib_imgs),
        "--ImageReader.camera_model", "OPENCV_FISHEYE",
        "--ImageReader.single_camera_per_folder", "1",
        "--ImageReader.camera_params", ACTION4_PORTRAIT_FISHEYE_PARAMS,
        "--SiftExtraction.max_num_features", "4096"
    ], check=True)

    subprocess.run([colmap_bin, "exhaustive_matcher", "--database_path", str(db_path)], check=True)
    subprocess.run([
        colmap_bin, "mapper",
        "--database_path", str(db_path),
        "--image_path", str(calib_imgs),
        "--output_path", str(calib_dir / "sparse"),
        "--Mapper.ba_refine_focal_length", "0",
        "--Mapper.ba_refine_extra_params", "0"
    ], check=True)

    calibrated_geom = None
    if (sparse_zero / "images.bin").exists():
        calibrated_geom = calibrate_rig_from_sparse_model(sparse_zero, baseline_mm=baseline_mm)

    shutil.rmtree(calib_dir, ignore_errors=True)
    return calibrated_geom


def run_colmap_rig_sfm(
    project_dir: Path,
    mode: str = "auto_calibrate",
    baseline_mm: float = 300.0,
    colmap_bin: str = DEFAULT_COLMAP_BIN,
    spirula_bin: str = DEFAULT_SPIRULA_BIN,
    sfm_backend: str = "auto"
) -> Path:
    """
    AI-AGENT NOTE: Kör Rigg-SfM med OPENCV_FISHEYE direkt på det riktiga filmmaterialet (ingen separat provfilmning krävs!):
    - Med Spirula SfM (`spirula.exe sfm auto`): Kalibrerar riggens vinklar automatiskt på de första
      bildrutorna i det riktiga klippet (`--rig-min-frames 3`), rekonstruerar hela scenen låst till riggen,
      och skalar sedan hela resultatet till exakt 1:1 meter (`baseline_mm`).
    - Med COLMAP (`colmap.exe`): Om klippet är långt körs först `_run_colmap_fast_subset_calibration`
      på de första 18 bild-triorna ur det riktiga klippet, varpå hela klippet rekonstrueras och låses
      med de kalibrerade vinklarna!
    """
    generate_colmap_rig_config(project_dir / "rig_config.json", mode=mode)

    colmap_resolved = shutil.which(colmap_bin)
    if sfm_backend == "spirula" or (sfm_backend == "auto" and not colmap_resolved):
        if Path(spirula_bin).exists() or shutil.which(spirula_bin):
            print("[SfM] Använder Spirula Studios inbyggda Vulkan-SfM (spirula.exe sfm auto)...")
            return run_spirula_rig_sfm(project_dir, mode=mode, baseline_mm=baseline_mm, spirula_bin=spirula_bin)
        raise FileNotFoundError(
            f"Varken '{colmap_bin}' eller '{spirula_bin}' hittades. "
            "Ladda ner COLMAP (colmap-x64-windows-cuda.zip) eller kontrollera sökvägen till spirula.exe."
        )

    # Om vi har ett långt riktigt klipp i auto_calibrate-läge: för-kalibrera snabbt på de första 18 triorna!
    precalibrated_geom = None
    if mode == "auto_calibrate":
        precalibrated_geom = _run_colmap_fast_subset_calibration(project_dir, baseline_mm=baseline_mm, colmap_bin=colmap_bin)
        if precalibrated_geom is not None:
            try:
                (project_dir / "calibrated_rig.json").write_text(json.dumps(precalibrated_geom, indent=2), encoding="utf-8")
            except Exception:
                pass
            generate_colmap_rig_config(project_dir / "rig_config.json", mode=mode, custom_geom=precalibrated_geom)

    db_path = project_dir / "database.db"
    images_dir = project_dir / "images"
    sparse_dir = project_dir / "sparse" / "0"
    rig_locked_dir = project_dir / "sparse_rig_locked" / "0"
    sparse_dir.mkdir(parents=True, exist_ok=True)
    rig_locked_dir.mkdir(parents=True, exist_ok=True)

    # 1. Feature Extraction
    subprocess.run([
        colmap_bin, "feature_extractor",
        "--database_path", str(db_path),
        "--image_path", str(images_dir),
        "--ImageReader.camera_model", "OPENCV_FISHEYE",
        "--ImageReader.single_camera_per_folder", "1",
        "--ImageReader.camera_params", ACTION4_PORTRAIT_FISHEYE_PARAMS,
        "--SiftExtraction.max_num_features", "8192"
    ], check=True)

    # 2. Matchning
    subprocess.run([
        colmap_bin, "exhaustive_matcher",
        "--database_path", str(db_path)
    ], check=True)

    # 3. Initial rekonstruktion (fri kamera-pose per bildruta)
    subprocess.run([
        colmap_bin, "mapper",
        "--database_path", str(db_path),
        "--image_path", str(images_dir),
        "--output_path", str(project_dir / "sparse"),
        "--Mapper.ba_refine_focal_length", "0",
        "--Mapper.ba_refine_extra_params", "0"
    ], check=True)

    # 3b. Om klippet var kort (<=18 trior) så att _run_colmap_fast_subset_calibration hoppades över:
    # mät vinklarna direkt från sparse/0 nu innan rig_bundle_adjuster låser riggen!
    if mode == "auto_calibrate" and precalibrated_geom is None and (sparse_dir / "images.bin").exists():
        calibrated_geom = calibrate_rig_from_sparse_model(sparse_dir, baseline_mm=baseline_mm)
        generate_colmap_rig_config(project_dir / "rig_config.json", mode=mode, custom_geom=calibrated_geom)
    elif precalibrated_geom is not None and (sparse_dir / "images.bin").exists():
        # Skala även huvudmodellen till 1:1 meter innan rig_bundle_adjuster
        calibrate_rig_from_sparse_model(sparse_dir, baseline_mm=baseline_mm)

    # 4. Lås rigg-geometrin och 1:1 meterskalan
    rig_modern_json = project_dir / "rig_config.json"
    rig_legacy_json = project_dir / "rig_config_legacy.json"

    res_modern = subprocess.run([
        colmap_bin, "rig_configurator",
        "--database_path", str(db_path),
        "--input_path", str(sparse_dir),
        "--output_path", str(rig_locked_dir),
        "--rig_config_path", str(rig_modern_json)
    ], check=False)

    if res_modern.returncode == 0:
        subprocess.run([
            colmap_bin, "bundle_adjuster",
            "--input_path", str(rig_locked_dir),
            "--output_path", str(rig_locked_dir),
            "--BundleAdjustment.refine_focal_length", "0",
            "--BundleAdjustment.refine_extra_params", "0"
        ], check=True)
    else:
        subprocess.run([
            colmap_bin, "rig_bundle_adjuster",
            "--input_path", str(sparse_dir),
            "--output_path", str(rig_locked_dir),
            "--rig_config_path", str(rig_legacy_json),
            "--BundleAdjustment.refine_focal_length", "0",
            "--BundleAdjustment.refine_extra_params", "0"
        ], check=True)

    for f in rig_locked_dir.glob("*"):
        (sparse_dir / f.name).write_bytes(f.read_bytes())

    print(f"[COLMAP Rig] Rigg-låst SfM klar i 1:1 meterskala -> {sparse_dir}")
    return project_dir


# =====================================================================
# 6. STARTA TRÄNING I LICHTFELD STUDIO 0.5.4.1 ELLER SPIRULA 2026.10.30
# =====================================================================

def launch_training_engine(
    project_dir: Path,
    engine: str = "lichtfeld",
    lichtfeld_bin: str = DEFAULT_LICHTFELD_BIN,
    spirula_bin: str = DEFAULT_SPIRULA_BIN,
    target_splats: int = 1500000
):
    """
    AI-AGENT NOTE: Skickar den färdiga COLMAP-mappen direkt till:
      - LichtFeld Studio v0.5.4.1 (`--strategy mrnf --gut --use-normal-loss --use-depth-loss --enable-sparsity`)
      - Spirula Studio 2026.10.30 (`spirula geometry <dir> --model moge3-vitl --depth` + `spirula train 3dgs --primitive 3dgut`)
    """
    output_dir = project_dir / "trained_models"
    output_dir.mkdir(exist_ok=True)

    if engine in ("lichtfeld", "both"):
        licht_out = output_dir / "lichtfeld"
        licht_out.mkdir(exist_ok=True)

        cmd = [
            lichtfeld_bin,
            "--headless",
            "-d", str(project_dir),
            "-o", str(licht_out),
            "--output-name", "model_lichtfeld",
            "--strategy", "mrnf",
            "--gut",
            "--use-normal-loss",
            "--use-depth-loss",
            "--enable-sparsity",
            "--max-cap", str(target_splats),
            "--export", "ply,spz"
        ]
        print(f"[LichtFeld 0.5.4.1] Startar 3DGUT-träning -> {licht_out}")
        subprocess.run(cmd, check=False)

    if engine in ("spirula", "both"):
        spirula_out = output_dir / "spirula"
        spirula_out.mkdir(exist_ok=True)

        subprocess.run([
            spirula_bin, "geometry",
            str(project_dir),
            "--model", "moge3-vitl",
            "--depth"
        ], check=False)

        subprocess.run([
            spirula_bin, "train", "3dgs",
            "--data", str(project_dir),
            "--primitive", "3dgut",
            "--cap-max", str(target_splats),
            "--output-dir-prefix", str(spirula_out),
            "--output-dir-name", "model_spirula"
        ], check=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="OptiGSRigg Processor (LichtFeld 0.5.4.1 & Spirula 2026.10.30)")
    parser.add_argument("--raw_dir", type=Path, default=None, help="Mapp med cam1_left.mp4, cam2_center.mp4, cam3_right.mp4")
    parser.add_argument("--project_dir", type=Path, default=None, help="Utdata-mapp för COLMAP & 3DGS-projekt")
    parser.add_argument("--ingest_sd", "--import_sd", dest="ingest_sd", action="store_true", help="Kopiera automatiskt MP4 från inkopplade SD-kort (CAM0_LEFT.id, CAM1_MID.id, CAM2_RIGHT.id) till --raw_dir")
    parser.add_argument("--ingest_only", "--import_only", dest="ingest_only", action="store_true", help="Kör endast SD-kortsimporten och avsluta sedan")
    parser.add_argument("--import_all_takes", action="store_true", help="Kopiera ALLA tagningar på SD-korten till egna sessionsmappar under --captures_root")
    parser.add_argument("--captures_root", type=Path, default=DEFAULT_CAPTURES_ROOT, help="Rotmapp för sessioner vid --import_all_takes")
    parser.add_argument("--take_from_end", type=int, default=1, help="Vilket klipp från slutet på SD-kortet som ska hämtas (1 = senaste, 2 = näst senaste)")
    parser.add_argument(
        "--mode",
        choices=["auto_calibrate", "calibrated", "mode_a", "mode_b"],
        default="auto_calibrate",
        help="auto_calibrate (Freestyle: mät exakta kameravinklar automatiskt), calibrated (ladda sparad kalibrering), mode_a (CAD Standard), mode_b (CAD 180° Flip)",
    )
    parser.add_argument("--baseline_mm", type=float, default=300.0, help="Fysiskt avstånd i mm mellan ytterkamerorna Cam1 och Cam3 (default: 300.0 mm = ±150 mm)")
    parser.add_argument("--fps", type=float, default=50.0, help="Videons bildfrekvens (default: 50.0)")
    parser.add_argument("--lut", type=Path, default=None, help="Sökväg till D-Log M .cube LUT")
    parser.add_argument("--window_sec", type=float, default=0.4, help="Tidsfönster i sekunder för Laplacian-skärpeval (default: 0.4)")
    parser.add_argument("--skip_start_sec", type=float, default=3.0, help="Sekunder att hoppa över i början efter klappen (default: 3.0)")
    parser.add_argument("--extract_backend", choices=["auto", "spirula_gpu", "opencv_parallel"], default="auto", help="Motor för bildextraktion (auto = Spirula Vulkan GPU + parallell)")
    parser.add_argument("--sfm_backend", choices=["auto", "spirula", "colmap", "none"], default="auto", help="Vilken SfM-motor som ska köras")
    parser.add_argument("--skip_sfm", action="store_true", help="Hoppa över SfM-steget (alias för --sfm_backend none)")
    parser.add_argument("--skip_extract", action="store_true", help="Hoppa över ljudsynk & bildextraktion om bild-trior redan finns i processed/images")
    parser.add_argument("--calibrate_only", action="store_true", help="Kör endast snabb kamerakalibrering på befintliga bild-trior i processed/images")
    parser.add_argument("--engine", choices=["none", "lichtfeld", "spirula", "both"], default="none")
    parser.add_argument("--lichtfeld_bin", type=str, default=DEFAULT_LICHTFELD_BIN)
    parser.add_argument("--spirula_bin", type=str, default=DEFAULT_SPIRULA_BIN)
    parser.add_argument("--colmap_bin", type=str, default=DEFAULT_COLMAP_BIN)
    args = parser.parse_args()
    if args.skip_sfm:
        args.sfm_backend = "none"

    if args.import_all_takes:
        import_all_takes_from_sd_cards(args.captures_root)
        raise SystemExit(0)

    if args.calibrate_only:
        if args.project_dir is None:
            raise ValueError("--project_dir krävs vid --calibrate_only.")
        calib = fast_calibrate_rig_from_images(args.project_dir, baseline_mm=args.baseline_mm)
        generate_colmap_rig_config(args.project_dir / "rig_config.json", mode=args.mode, custom_geom=calib)
        raise SystemExit(0)

    if args.raw_dir is None and not args.skip_extract:
        raise ValueError("--raw_dir krävs.")

    if args.ingest_sd or args.ingest_only:
        import_from_sd_cards(args.raw_dir, take_from_end=args.take_from_end)
        if args.ingest_only:
            raise SystemExit(0)

    if args.project_dir is None:
        raise ValueError("--project_dir krävs när hela pipelinen körs.")

    existing_imgs = sorted((args.project_dir / "images" / "cam2_center").glob("*.jpg"))
    if args.skip_extract and existing_imgs:
        print(
            f"[Steg 2/3] Hoppar över bildextraktion – använder {len(existing_imgs)} befintliga bild-trior i {args.project_dir / 'images'}.",
            flush=True,
        )
    else:
        videos = {
            "cam1_left": args.raw_dir / "cam1_left.mp4",
            "cam2_center": args.raw_dir / "cam2_center.mp4",
            "cam3_right": args.raw_dir / "cam3_right.mp4",
        }

        missing_videos = [f"{k} ({p})" for k, p in videos.items() if not p.exists() or p.stat().st_size == 0]
        if missing_videos:
            print(f"[FEL] Saknar videofiler i {args.raw_dir}:")
            for mv in missing_videos:
                print(f"  - {mv}")
            raise FileNotFoundError(f"Saknar {len(missing_videos)} av 3 kameravideor i {args.raw_dir}")

        offsets = compute_frame_offsets(videos, args.project_dir, fps=args.fps)
        extract_sharpest_synced_triplets(
            videos,
            offsets,
            args.project_dir / "images",
            lut_path=args.lut,
            fps=args.fps,
            window_sec=args.window_sec,
            skip_start_sec=args.skip_start_sec,
            spirula_bin=args.spirula_bin,
            extract_backend=args.extract_backend,
        )

    # Kör alltid snabb fisheye-kamerakalibrering direkt på bild-triorna om mode == auto_calibrate!
    measured_calib = None
    if args.mode == "auto_calibrate":
        measured_calib = fast_calibrate_rig_from_images(args.project_dir, baseline_mm=args.baseline_mm)
    elif args.mode == "calibrated" and DJI_RIG_PROFILE_PATH.exists():
        try:
            args.project_dir.mkdir(parents=True, exist_ok=True)
            (args.project_dir / "dji_rig_profile.json").write_text(
                DJI_RIG_PROFILE_PATH.read_text(encoding="utf-8"), encoding="utf-8"
            )
        except Exception:
            pass

    generate_colmap_rig_config(args.project_dir / "rig_config.json", mode=args.mode, custom_geom=measured_calib)

    if args.sfm_backend != "none":
        run_colmap_rig_sfm(
            args.project_dir,
            mode=args.mode,
            baseline_mm=args.baseline_mm,
            colmap_bin=args.colmap_bin,
            spirula_bin=args.spirula_bin,
            sfm_backend=args.sfm_backend,
        )

    if args.engine != "none":
        launch_training_engine(
            args.project_dir,
            engine=args.engine,
            lichtfeld_bin=args.lichtfeld_bin,
            spirula_bin=args.spirula_bin,
        )
