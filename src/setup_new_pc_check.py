#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Post-installationskontroll och automatisk anpassning för flytt till ny dator.
Körs automatiskt av setup_ny_dator.bat efter att .venv och paket installerats.
"""

from __future__ import annotations

import argparse
import importlib
import json
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Tuple

APP_ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = APP_ROOT / "src"
if str(APP_ROOT) not in sys.path:
    sys.path.insert(0, str(APP_ROOT))
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))


def _is_valid_exe_path(path_str: str) -> bool:
    s = (path_str or "").strip()
    if not s:
        return False
    p = Path(s)
    if p.is_file():
        return True
    return shutil.which(s) is not None


def heal_settings_dict(saved: Dict[str, object], auto_detected: Dict[str, object]) -> Tuple[Dict[str, object], List[str]]:
    """
    Slår samman sparad settings.json med nyupptäckta sökvägar på den nya datorn.
    Giltiga befintliga sökvägar bevaras orörda; trasiga/gamla datorsökvägar ersätts.
    """
    merged: Dict[str, object] = dict(auto_detected)
    for k in ("spirula_exe", "lichtfeld_exe", "colmap_exe", "captures_root", "mcp_url", "extract_backend"):
        val = str((saved or {}).get(k) or "").strip()
        if val:
            merged[k] = val

    changes: List[str] = []
    for bin_key in ("spirula_exe", "lichtfeld_exe", "colmap_exe"):
        cur_val = str(merged.get(bin_key) or "").strip()
        auto_val = str(auto_detected.get(bin_key) or "").strip()
        if not _is_valid_exe_path(cur_val) and auto_val and cur_val != auto_val:
            if _is_valid_exe_path(auto_val) or (bin_key in (saved or {}) and str(saved.get(bin_key) or "").strip()):
                merged[bin_key] = auto_val
                changes.append(f"{bin_key}: '{cur_val}' -> '{auto_val}'")

    cap_val = str(merged.get("captures_root") or "").strip()
    auto_cap = str(auto_detected.get("captures_root") or "").strip()
    cap_invalid = not cap_val
    if cap_val:
        cap_path = Path(cap_val)
        if cap_path.anchor and not Path(cap_path.anchor).exists():
            cap_invalid = True
    if cap_invalid and auto_cap and cap_val != auto_cap:
        merged["captures_root"] = auto_cap
        changes.append(f"captures_root: '{cap_val}' -> '{auto_cap}'")

    return merged, changes


def check_python_packages() -> bool:
    """Verifierar att alla nödvändiga Python-bibliotek + OpenCV ArUco laddas korrekt."""
    ok = True
    for mod_name in ("numpy", "scipy", "cv2", "soundfile", "tqdm"):
        try:
            mod = importlib.import_module(mod_name)
            ver = getattr(mod, "__version__", "OK")
            print(f"  [OK] Python-paket '{mod_name}' ({ver})")
        except Exception as e:
            print(f"  [FEL] Saknar eller trasigt paket '{mod_name}': {e}")
            ok = False

    if ok:
        try:
            import cv2  # type: ignore

            _ = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
            print("  [OK] OpenCV cv2.aruco (ChArUco-kalibrering) fungerar")
        except Exception as e:
            print(f"  [FEL] OpenCV ArUco/ChArUco saknas ({e}) — installera opencv-contrib-python eller headless-hjul med aruco")
            ok = False

    if platform.system() == "Windows" and Path(sys.prefix).resolve() == (APP_ROOT / ".venv").resolve():
        pyw_exe = APP_ROOT / ".venv" / "Scripts" / "pythonw.exe"
        if pyw_exe.is_file():
            print("  [OK] GUI-startare (.venv\\Scripts\\pythonw.exe) redo")
        else:
            print(f"  [FEL] Saknar {pyw_exe} för tyst start")
            ok = False
    return ok


def sync_edge_app_profile_host() -> None:
    """
    Rensar kopierad Edge-appprofil automatiskt om datornamnet (hostname) ändrats,
    så att ny dator startar utan låsta profiler eller gammal ikon-cache.
    """
    prof_dir = APP_ROOT / "config" / "app_window_profile"
    host_stamp = prof_dir / ".host_machine"
    cur_host = platform.node() or "unknown"

    if prof_dir.exists() and host_stamp.is_file():
        prev_host = host_stamp.read_text(encoding="utf-8", errors="ignore").strip()
        if prev_host and prev_host != cur_host:
            shutil.rmtree(prof_dir, ignore_errors=True)
            print(f"  [INFO] Ny dator upptäckt ({prev_host} -> {cur_host}) — rensade gammal app-fönsterprofil.")

    prof_dir.mkdir(parents=True, exist_ok=True)
    host_stamp.write_text(cur_host, encoding="utf-8")
    print(f"  [OK] App-fönsterprofil kopplad till datorn '{cur_host}'")


def heal_and_verify_environment() -> dict:
    """Anpassar config/settings.json till nya datorn, skapar captures_root och kontrollerar motorer."""
    import app as studio_app

    settings_path = studio_app.SETTINGS_JSON_PATH
    raw_saved: Dict[str, object] = {}
    had_file = settings_path.exists()
    if had_file:
        try:
            loaded = json.loads(settings_path.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                raw_saved = loaded
        except Exception:
            raw_saved = {}

    auto_det = studio_app.auto_find_engine_paths()
    merged, changes = heal_settings_dict(raw_saved, auto_det)

    if (not had_file) or changes:
        studio_app.save_app_settings(merged)
        if changes:
            for ch in changes:
                print(f"  [UPPDATERAD] settings.json -> {ch}")
        else:
            print(f"  [OK] Skapade ny config/settings.json för denna dator")
    else:
        print("  [OK] config/settings.json matchar redan denna dator")

    cap_dir = Path(str(merged.get("captures_root") or ""))
    try:
        if not cap_dir.anchor or Path(cap_dir.anchor).exists():
            cap_dir.mkdir(parents=True, exist_ok=True)
            print(f"  [OK] Inspelningsmapp klar: {cap_dir}")
    except Exception as e:
        print(f"  [VARNING] Kunde inte skapa inspelningsmapp '{cap_dir}': {e}")

    # Kontrollera externa verktyg
    ffmpeg_bin = shutil.which("ffmpeg")
    ffprobe_bin = shutil.which("ffprobe")
    if ffmpeg_bin and ffprobe_bin:
        print(f"  [OK] FFmpeg & FFprobe hittade: {ffmpeg_bin}")
    else:
        print("  [VARNING] FFmpeg / FFprobe saknas i PATH (behövs för ljudklapp-synk, miniatyrer & EXIF).")

    for label, key in (
        ("Spirula Studio", "spirula_exe"),
        ("LichtFeld Studio", "lichtfeld_exe"),
        ("COLMAP", "colmap_exe"),
    ):
        val = str(merged.get(key) or "")
        if _is_valid_exe_path(val):
            print(f"  [OK] {label}: {val}")
        else:
            print(f"  [INFO] {label} ej hittad på '{val}' (valfritt om bara Steg 1/kalibrering körs här; ange i Inställningar senare).")

    # Kontrollera riggkalibrering
    for cfg_name in ("dji_rig_profile.json", "calibrated_rig.json"):
        cp = APP_ROOT / "config" / cfg_name
        if cp.is_file():
            print(f"  [OK] Riggkalibrering medföljer: config/{cfg_name}")

    return merged


def ensure_icons_and_shortcuts(skip_desktop: bool = False) -> None:
    ico_path = APP_ROOT / "assets" / "icons" / "optigsrigg.ico"
    if not ico_path.is_file():
        import build_app_icon

        build_app_icon.build_icon_pack(out_dir=ico_path.parent)
        print(f"  [OK] Byggde ikonpaket i {ico_path.parent}")
    else:
        print(f"  [OK] App-ikon redo: {ico_path}")

    if platform.system() != "Windows":
        return

    ps_script = APP_ROOT / "create_desktop_shortcut.ps1"
    if ps_script.is_file():
        cmd = ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(ps_script)]
        if skip_desktop:
            cmd.append("-SkipDesktop")
        subprocess.run(cmd, check=True)
        if skip_desktop:
            print("  [INFO] Skapade/uppdaterade lokal app-genväg; hoppade över skrivbordsgenväg (--no-desktop-shortcut).")
    else:
        print(f"  [VARNING] Hittade inte {ps_script}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Kontrollera och anpassa OptiGSRigg för ny dator.")
    parser.add_argument("--no-desktop-shortcut", action="store_true", help="Skapa inte .lnk-genväg på Skrivbordet")
    args, _unknown = parser.parse_known_args()

    print("\n--- Kontrollerar Python-miljö & beroenden ---")
    if not check_python_packages():
        return 1

    print("\n--- Kontrollerar profil, motorer & kalibrering ---")
    sync_edge_app_profile_host()
    heal_and_verify_environment()

    print("\n--- Kontrollerar app-ikon & genvägar ---")
    ensure_icons_and_shortcuts(skip_desktop=args.no_desktop_shortcut)
    print("\n[KLAR] Alla kontroller passerade!")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
