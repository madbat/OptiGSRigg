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
import json
import struct
import shutil
import subprocess
import argparse
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import numpy as np
from scipy import signal
import soundfile as sf
import cv2


# =====================================================================
# 1. SÖKVÄGAR TILL INSTALLERADE MOTORER & OPTIK-KONSTANTER
# =====================================================================

DEFAULT_LICHTFELD_BIN = r"D:\LichtFeld-Studio-windows-v0.5.4.1\bin\LichtFeld-Studio.exe"
DEFAULT_SPIRULA_BIN = r"D:\spirula-2026.10.30-windows-vulkan-x86_64\spirula.exe"
DEFAULT_COLMAP_BIN = "colmap"
CALIBRATED_RIG_PATH = Path(__file__).resolve().parent.parent / "config" / "calibrated_rig.json"

# Låsta lins-intrinsics för DJI Osmo Action 4 i 4K 4:3 (2880x3840 stående, Wide FOV, RockSteady OFF)
# Modell i COLMAP: OPENCV_FISHEYE -> [fx, fy, cx, cy, k1, k2, k3, k4]
ACTION4_PORTRAIT_FISHEYE_PARAMS = "1585.0, 1585.0, 1440.0, 1920.0, -0.042, 0.008, -0.002, 0.0005"
ACTION4_PORTRAIT_FISHEYE_ARRAY = [1585.0, 1585.0, 1440.0, 1920.0, -0.042, 0.008, -0.002, 0.0005]
ACTION4_FOCAL_PX = "1585.0"
ACTION4_FISHEYE_DISTORTION = "-0.042,0.008,-0.002,0.0005"

# Nominella CAD-lägen (i meter och grader relativt Cam 2 Center)
RIG_MODES = {
    "mode_a": {
        "cam1_left":   {"tx": -0.150, "ty": 0.0, "tz": 0.0, "yaw": -45.0, "pitch":  12.0, "roll": 0.0},
        "cam2_center": {"tx":  0.000, "ty": 0.0, "tz": 0.0, "yaw":   0.0, "pitch":  -8.0, "roll": 0.0},
        "cam3_right":  {"tx":  0.150, "ty": 0.0, "tz": 0.0, "yaw":  45.0, "pitch":  12.0, "roll": 0.0},
    },
    "mode_b": {  # När hela 2020-skenan vänds 180° upp-och-ner (Flip-Rail)
        "cam1_left":   {"tx": -0.150, "ty": 0.0, "tz": 0.0, "yaw": -45.0, "pitch": -12.0, "roll": 0.0},
        "cam2_center": {"tx":  0.000, "ty": 0.0, "tz": 0.0, "yaw":   0.0, "pitch":   8.0, "roll": 0.0},
        "cam3_right":  {"tx":  0.150, "ty": 0.0, "tz": 0.0, "yaw":  45.0, "pitch": -12.0, "roll": 0.0},
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
    AI-AGENT NOTE: Skannar alla monterade enhetsbokstäver i Windows (D: till Z:)
    efter ID-filerna `CAM0_LEFT.id`, `CAM1_MID.id` och `CAM2_RIGHT.id` i kortets rot.
    Returnerar:
      - found_drives: `{ 'cam1_left': (Path('E:/'), 'CAM0_LEFT.id'), ... }`
      - unmarked_dcim_drives: Lista med enheter som har en `DCIM/`-mapp men SAKNAR `.id`-fil!
    """
    found_drives: Dict[str, Tuple[Path, str]] = {}
    unmarked_dcim_drives: List[Path] = []
    for letter in "DEFGHIJKLMNOPQRSTUVWXYZ":
        drive_root = Path(f"{letter}:/")
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

    for cam_id in ["cam1_left", "cam3_right"]:
        target_sig = audio_signals[cam_id]
        corr = signal.correlate(target_sig, ref_sig, mode="full", method="fft")
        lag_samples = int(np.argmax(corr)) - (len(ref_sig) - 1)
        lag_seconds = lag_samples / float(sample_rate)
        frame_offset = int(round(lag_seconds * fps))
        offsets_frames[cam_id] = frame_offset
        print(
            f"[AudioSync] {cam_id} relativt center: {lag_seconds*1000:+.1f} ms ({frame_offset:+d} bildrutor @ {fps} fps)",
            flush=True,
        )

    min_offset = min(offsets_frames.values())
    normalized_offsets = {k: v - min_offset for k, v in offsets_frames.items()}
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


def extract_sharpest_synced_triplets(
    videos: Dict[str, Path],
    offsets: Dict[str, int],
    output_images_dir: Path,
    lut_path: Optional[Path] = None,
    fps: float = 50.0,
    window_sec: float = 0.4,
    skip_start_sec: float = 3.0
) -> int:
    """
    AI-AGENT NOTE: Stegar igenom videofilerna i synkroniserade tidsfönster (t.ex. 0.4s = 20 rutor vid 50fps).
    För varje tidsfönster utvärderas skärpan i mittenkameran, och den skarpaste bildrutetrion
    sparas i respektive undermapp (images/cam1_left, cam2_center, cam3_right).
    """
    caps = {cam_id: cv2.VideoCapture(str(path)) for cam_id, path in videos.items()}
    window_frames = max(1, int(round(window_sec * fps)))
    start_base_frame = int(round(skip_start_sec * fps))

    for cam_id in videos.keys():
        (output_images_dir / cam_id).mkdir(parents=True, exist_ok=True)

    total_frames_list = [
        int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) - offsets[cam_id]
        for cam_id, cap in caps.items()
    ]
    max_sync_frame = min(total_frames_list)
    window_starts = list(range(start_base_frame, max_sync_frame - window_frames, window_frames))
    total_windows = len(window_starts)
    print(
        f"[Steg 2/3 - Bildextrahering] Startar Laplacian-skärpefilter över {total_windows} tidsfönster "
        f"(ca {total_windows * 3} bilder kommer sparas till {output_images_dir})...",
        flush=True,
    )

    saved_count = 0
    # Stega igenom fönstret med steg=2 bildrutor vid >=40 fps (halverar avkodningstiden utan att missa stegets viloläge)
    eval_stride = 2 if window_frames >= 8 else 1

    for win_idx, base_idx in enumerate(window_starts, 1):
        best_score = -1.0
        best_sub_idx = 0

        cap_center = caps["cam2_center"]
        cap_center.set(cv2.CAP_PROP_POS_FRAMES, base_idx + offsets["cam2_center"])

        for w in range(window_frames):
            if w % eval_stride != 0:
                if not cap_center.grab():
                    break
                continue
            ret, frame = cap_center.read()
            if not ret:
                break
            score = measure_sharpness_laplacian(frame)
            if score > best_score:
                best_score = score
                best_sub_idx = w

        chosen_sync_idx = base_idx + best_sub_idx
        saved_count += 1
        frame_name = f"frame_{saved_count:04d}.jpg"

        for cam_id, cap in caps.items():
            exact_frame_no = chosen_sync_idx + offsets[cam_id]
            out_file = output_images_dir / cam_id / frame_name

            if lut_path and lut_path.exists():
                timestamp = exact_frame_no / fps
                lut_escaped = lut_path.resolve().as_posix().replace(":", r"\:")
                cmd = [
                    "ffmpeg", "-y", "-ss", f"{timestamp:.4f}",
                    "-i", str(videos[cam_id]),
                    "-vf", f"lut3d='{lut_escaped}'",
                    "-frames:v", "1", "-q:v", "1",
                    "-loglevel", "error", str(out_file)
                ]
                subprocess.run(cmd, check=True)
            else:
                cap.set(cv2.CAP_PROP_POS_FRAMES, exact_frame_no)
                ret, img = cap.read()
                if ret:
                    cv2.imwrite(str(out_file), img, [cv2.IMWRITE_JPEG_QUALITY, 98])

        if win_idx == 1 or win_idx % 3 == 0 or win_idx == total_windows:
            pct = int(round(100.0 * win_idx / max(1, total_windows)))
            print(
                f"[Steg 2/3 - Bildextrahering] Trio {win_idx}/{total_windows} ({pct}%) "
                f"-> {frame_name} ({saved_count * 3} bilder i processed/images, skärpa={best_score:.1f})",
                flush=True,
            )

    for cap in caps.values():
        cap.release()

    print(
        f"[Steg 2/3 - Bildextrahering] KLAR! Extraherade {saved_count} synkroniserade bild-trior ({saved_count*3} bilder totalt).",
        flush=True,
    )
    return saved_count


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
        R_stack = np.stack(rel_rotations[side_cam], axis=0)
        # Använd median per element följt av SVD-ortogonalisering för att ignorera ev. outliers
        M = np.median(R_stack, axis=0)
        U, _, Vt = np.linalg.svd(M)
        R_mean = U @ Vt
        if np.linalg.det(R_mean) < 0:
            U[:, -1] *= -1
            R_mean = U @ Vt
        avg_R[side_cam] = R_mean

        # Beräkna vinkelspridning (stabilitetsmått i grader över sekvensen)
        angles = []
        for R_i in R_stack:
            dR = R_i @ R_mean.T
            cos_theta = np.clip((np.trace(dR) - 1.0) / 2.0, -1.0, 1.0)
            angles.append(np.degrees(np.arccos(cos_theta)))
        spread_deg[side_cam] = float(np.median(angles))

        pos_stack = np.stack(rel_translations_in_ref[side_cam], axis=0)
        avg_pos_in_ref[side_cam] = np.median(pos_stack, axis=0)

    # Beräkna skalfaktor så att totala avståndet mellan cam1_left och cam3_right blir exakt baseline_mm
    raw_total_baseline = float(np.linalg.norm(avg_pos_in_ref["cam3_right"] - avg_pos_in_ref["cam1_left"]))
    target_total_m = baseline_mm / 1000.0
    scale_factor = (target_total_m / raw_total_baseline) if raw_total_baseline > 1e-6 else 1.0

    # Skala om sparse-modellen på disk till 1:1 meter
    scale_colmap_sparse_model(sparse_dir, scale_factor)

    calibrated_geom: Dict[str, Dict[str, float]] = {
        "cam2_center": {"tx": 0.0, "ty": 0.0, "tz": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}
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
            "spread_deg": spread_deg[side_cam],
        }
        dist_mm = float(np.linalg.norm(pos_metric) * 1000.0)
        n_used = len(rel_rotations[side_cam])
        print(
            f"  {side_cam:11s}: Yaw={yaw_deg:+6.2f}° | Rel.Pitch={pitch_deg:+6.2f}° | Roll={roll_deg:+6.2f}° "
            f"| Avstånd={dist_mm:5.1f} mm | Stabilitet=±{spread_deg[side_cam]:.2f}° ({n_used} trior)"
        )
        if spread_deg[side_cam] > 1.0:
            print(f"  [!] VARNING: Hög vinkelspridning (±{spread_deg[side_cam]:.2f}°) på {side_cam} – kontrollera att fästet sitter stumt!")

    print("=" * 78 + "\n")

    save_calibrated_path.parent.mkdir(parents=True, exist_ok=True)
    save_calibrated_path.write_text(json.dumps(calibrated_geom, indent=2), encoding="utf-8")
    print(f"[RigCalib] Sparade uppmätt rigg-profil -> {save_calibrated_path}")
    return calibrated_geom


# =====================================================================
# 5. COLMAP & SPIRULA RIGG-SFM (STÖDER BÅDE FREESTYLE & FASTA LÄGEN)
# =====================================================================

def get_rig_geometry(mode: str) -> Dict[str, Dict[str, float]]:
    """Hämtar antingen ett fast CAD-läge (mode_a/mode_b) eller senast uppmätta calibrated_rig.json."""
    if mode in RIG_MODES:
        return RIG_MODES[mode]
    if CALIBRATED_RIG_PATH.exists():
        return json.loads(CALIBRATED_RIG_PATH.read_text(encoding="utf-8"))
    return RIG_MODES["mode_a"]


def generate_colmap_rig_config(
    rig_json_path: Path,
    mode: str = "mode_a",
    custom_geom: Optional[Dict[str, Dict[str, float]]] = None
) -> Path:
    """
    AI-AGENT NOTE: Skapar både:
      1) rig_config.json (COLMAP 3.12+ nytt format med cam_from_rig_rotation / cam_from_rig_translation)
      2) rig_config_legacy.json (COLMAP <= 3.11 format med ref_camera_id / rel_qvec / rel_tvec)
    Om `mode == 'auto_calibrate'` och `custom_geom is None` skapas även `rig_config_uncalibrated.json`
    (utan förvalda vinklar) så att COLMAP 3.12 `rig_configurator` kan estimera vinklarna fritt.
    """
    geom = custom_geom if custom_geom is not None else get_rig_geometry(mode)
    cameras_modern = []
    cameras_legacy = []
    cameras_uncalibrated = []

    cam_id_map = {"cam1_left": 1, "cam2_center": 2, "cam3_right": 3}
    images_base = rig_json_path.parent / "images"
    for name, cid in cam_id_map.items():
        g = geom[name]
        q_wxyz = euler_to_quaternion_wxyz(g["yaw"], g["pitch"], g["roll"])

        # Dynamisk avläsning av upplösning per kamera (ifall t.ex. en kamera är 2160x3840 16:9 och övriga 2880x3840 4:3)
        cam_params = list(ACTION4_PORTRAIT_FISHEYE_ARRAY)
        cam_img_dir = images_base / name
        if cam_img_dir.exists():
            sample_imgs = sorted(cam_img_dir.glob("*.jpg"))
            if sample_imgs:
                im = cv2.imread(str(sample_imgs[0]))
                if im is not None:
                    h_px, w_px = im.shape[:2]
                    cam_params[2] = float(w_px) / 2.0
                    cam_params[3] = float(h_px) / 2.0

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
    AI-AGENT NOTE: Kör Spirula Studios inbyggda Vulkan-accelererade SfM (`spirula.exe sfm auto`).
    Spirulas `--rig cam1_left,cam2_center,cam3_right` estimerar och låser automatiskt de exakta
    vinklarna mellan kamerorna även när du "freestylar" riggen!
    Efteråt kör vi `calibrate_rig_from_sparse_model` för att skala modellen till exakt `baseline_mm`
    och skriva ut de uppmätta vinklarna.
    """
    images_dir = project_dir / "images"
    cmd = [
        spirula_bin, "sfm", "auto",
        str(images_dir),
        "-o", str(project_dir),
        "--device", "auto",
        "--data-type", "video",
        "--camera-mode", "folder",
        "--camera-model", "opencv-fisheye",
        "--focal", ACTION4_FOCAL_PX,
        "--distortion", ACTION4_FISHEYE_DISTORTION,
        "--rig", "cam1_left,cam2_center,cam3_right",
        "--sequence", "cam1_left,cam2_center,cam3_right",
    ]
    print(f"[Steg 3/3 - Spirula SfM] Kör Vulkan GPU Rigg-SfM -> {project_dir / 'sparse' / '0'} ...", flush=True)
    res = subprocess.run(cmd, check=False)

    sparse_dir = project_dir / "sparse" / "0"
    if not (sparse_dir / "images.bin").exists():
        raise RuntimeError(
            f"Spirula SfM avslutades med kod {res.returncode} och skapade ingen sparse/0/images.bin. Se loggen ovan."
        )
    print(f"[Steg 3/3 - Spirula SfM] 3D-rekonstruktion skapad i {sparse_dir}! Mäter kameravinklar & skalar till {baseline_mm:.0f} mm...", flush=True)
    calibrated_geom = calibrate_rig_from_sparse_model(sparse_dir, baseline_mm=baseline_mm)
    if mode == "auto_calibrate":
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
    parser.add_argument("--ingest_sd", action="store_true", help="Kopiera automatiskt MP4 från inkopplade SD-kort (CAM0_LEFT.id, CAM1_MID.id, CAM2_RIGHT.id) till --raw_dir")
    parser.add_argument("--ingest_only", action="store_true", help="Kör endast SD-kortsimporten och avsluta sedan")
    parser.add_argument("--import_all_takes", action="store_true", help="Kopiera ALLA tagningar på SD-korten till egna sessionsmappar under --captures_root")
    parser.add_argument("--captures_root", type=Path, default=Path(r"D:\3DGS_Captures"), help="Rotmapp för sessioner vid --import_all_takes")
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
    parser.add_argument("--sfm_backend", choices=["auto", "spirula", "colmap", "none"], default="auto", help="Vilken SfM-motor som ska köras")
    parser.add_argument("--engine", choices=["none", "lichtfeld", "spirula", "both"], default="none")
    parser.add_argument("--lichtfeld_bin", type=str, default=DEFAULT_LICHTFELD_BIN)
    parser.add_argument("--spirula_bin", type=str, default=DEFAULT_SPIRULA_BIN)
    parser.add_argument("--colmap_bin", type=str, default=DEFAULT_COLMAP_BIN)
    args = parser.parse_args()

    if args.import_all_takes:
        import_all_takes_from_sd_cards(args.captures_root)
        raise SystemExit(0)

    if args.raw_dir is None:
        raise ValueError("--raw_dir krävs.")

    if args.ingest_sd or args.ingest_only:
        import_from_sd_cards(args.raw_dir, take_from_end=args.take_from_end)
        if args.ingest_only:
            raise SystemExit(0)

    if args.project_dir is None:
        raise ValueError("--project_dir krävs när hela pipelinen körs.")

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
    )
    generate_colmap_rig_config(args.project_dir / "rig_config.json", mode=args.mode)

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
