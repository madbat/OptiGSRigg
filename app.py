#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OptiGSRigg Standalone Control Center (GUI)
==========================================
Fristående kontrollprogram för 3x DJI Osmo Action 4 -> 3DGUT Pipeline.
Ger full kontroll och realtidsfeedback över alla 10 steg i pipelinen:
  1. Identifiera SD-kort (.id-filer: CAM0_LEFT, CAM1_MID, CAM2_RIGHT)
  2. Kopiera valda eller alla tagningar till D:\3DGS_Captures\Session_...
  3. Validera råvideofiler (upplösning, storlek, 3/3 kameror)
  4. Ljudsynkronisering på handklapp (FFT-korskorrelation)
  5. Laplacian Skärpefilter (skarpaste bild-trion var 0.40 s)
  6. Skapa OPENCV_FISHEYE rig_config.json (dynamisk upplösning per kamera)
  7. Kör Spirula Studio Vulkan Rigg-SfM (eller COLMAP)
  8. Freestyle Auto-Kalibrering av vinklar (Yaw/Pitch/Roll) & 1:1 Meterskalning (300 mm)
  9. Aktivera 3DGUT (Unscented Transform för fisheye) & ladda in i LichtFeld Studio
 10. Övervaka 3DGS-träning i realtid via LichtFeld MCP (Iteration, Loss, Antal Splats)
"""

import json
import os
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import urllib.request
from pathlib import Path

APP_ROOT = Path(__file__).resolve().parent
SRC_SCRIPT = APP_ROOT / "src" / "process_3dgs_rig.py"
CAPTURES_ROOT = Path(r"D:\3DGS_Captures")
LICHTFELD_EXE = Path(r"D:\LichtFeld-Studio-windows-v0.5.4.1\bin\LichtFeld-Studio.exe")
SPIRULA_EXE = Path(r"D:\spirula-2026.10.30-windows-vulkan-x86_64\spirula.exe")
MCP_URL = "http://127.0.0.1:45677/mcp"

# Lägg till src/ i sys.path så vi kan återanvända hjälpfunktioner direkt
if str(APP_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(APP_ROOT / "src"))

try:
    from process_3dgs_rig import scan_sd_cards
except Exception:
    scan_sd_cards = None


PIPELINE_STEPS = [
    ("1. SD-kort & ID", "Identifiera CAM0_LEFT, CAM1_MID, CAM2_RIGHT"),
    ("2. Kopiera MP4", "Kopiera och döp om till cam1_left / cam2_center / cam3_right"),
    ("3. Filkontroll", "Verifiera att alla 3 MP4-filer finns i raw/"),
    ("4. Ljud-synk (Klapp)", "FFT-korskorrelation på de första 15 s ljud"),
    ("5. Skärpefilter", "Laplacian-filter plockar skarpaste trion var 0.40 s"),
    ("6. Fisheye-konfig", "Skapa OPENCV_FISHEYE per kameras upplösning"),
    ("7. Rigg-SfM (3D)", "Spirula Vulkan SfM registrerar alla kamerapositioner"),
    ("8. Auto-Kalibrering", "Mät vinklar (Yaw/Pitch/Roll) & skala till 1:1 meter"),
    ("9. Ladda & 3DGUT", "Aktivera GUT=True för fisheye i LichtFeld Studio"),
    ("10. 3DGS-Träning", "Träna 1.5M splats och följ Loss i realtid"),
]


class OptiGSRiggApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("OptiGSRigg Control Center – 3x DJI Osmo Action 4 (3DGUT Pipeline)")
        self.geometry("1180x860")
        self.minsize(1024, 740)
        self.configure(bg="#11161d")

        self._log_queue = queue.Queue()
        self._proc = None
        self._is_running = False
        self._sd_status = {}
        self._sessions = []
        self._step_states = ["pending"] * len(PIPELINE_STEPS)

        self._setup_styles()
        self._build_ui()

        self.refresh_sd_and_sessions()
        self.after(200, self._poll_log_queue)
        self.after(1500, self._poll_lichtfeld_mcp)

    def _setup_styles(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure(".", background="#11161d", foreground="#e2e8f0", font=("Segoe UI", 10))
        style.configure("Card.TFrame", background="#19212c", relief="flat")
        style.configure("Header.TLabel", background="#11161d", foreground="#58a6ff", font=("Segoe UI", 15, "bold"))
        style.configure("SubHeader.TLabel", background="#11161d", foreground="#8b949e", font=("Segoe UI", 9))
        style.configure("Section.TLabel", background="#19212c", foreground="#58a6ff", font=("Segoe UI", 11, "bold"))
        style.configure("CardText.TLabel", background="#19212c", foreground="#e2e8f0", font=("Segoe UI", 10))
        style.configure("Action.TButton", background="#238636", foreground="#ffffff", font=("Segoe UI", 10, "bold"), padding=6)
        style.map("Action.TButton", background=[("active", "#2ea043"), ("disabled", "#21262d")])
        style.configure("Secondary.TButton", background="#21262d", foreground="#c9d1d9", font=("Segoe UI", 9, "bold"), padding=5)
        style.map("Secondary.TButton", background=[("active", "#30363d")])
        style.configure("Horizontal.TProgressbar", troughcolor="#0d1117", background="#2ea043", thickness=14)

    def _build_ui(self):
        # Top Header
        top = ttk.Frame(self, padding=(14, 10))
        top.pack(fill="x")
        ttk.Label(top, text="OptiGSRigg Control Center", style="Header.TLabel").pack(anchor="w")
        ttk.Label(
            top,
            text="Fristående 10-stegs pipeline med realtidsfeedback • SD-Import • 50 fps Klapp-Synk • Freestyle Rigg-SfM • LichtFeld MCP",
            style="SubHeader.TLabel",
        ).pack(anchor="w")

        # 10-Step Visual Pipeline Bar
        step_card = ttk.Frame(self, style="Card.TFrame", padding=10)
        step_card.pack(fill="x", padx=14, pady=(0, 8))
        ttk.Label(step_card, text="Pipeline-steg (1–10)", style="Section.TLabel").pack(anchor="w", pady=(0, 6))

        steps_grid = tk.Frame(step_card, bg="#19212c")
        steps_grid.pack(fill="x")
        self._step_badges = []
        for idx, (short_title, tooltip) in enumerate(PIPELINE_STEPS):
            col = idx % 5
            row = idx // 5
            steps_grid.columnconfigure(col, weight=1)
            f = tk.Frame(steps_grid, bg="#0d1117", highlightbackground="#30363d", highlightthickness=1, padx=8, pady=5)
            f.grid(row=row, column=col, sticky="ew", padx=3, pady=3)
            lbl_title = tk.Label(f, text=short_title, bg="#0d1117", fg="#8b949e", font=("Segoe UI", 9, "bold"), anchor="w")
            lbl_title.pack(fill="x")
            lbl_sub = tk.Label(f, text=tooltip, bg="#0d1117", fg="#6e7681", font=("Segoe UI", 8), anchor="w")
            lbl_sub.pack(fill="x")
            self._step_badges.append((f, lbl_title, lbl_sub))

        # Main Split Area (Left: Controls, Right: Live Status, MCP & Logs)
        main_pane = tk.PanedWindow(self, orient="horizontal", bg="#11161d", sashwidth=6, bd=0)
        main_pane.pack(fill="both", expand=True, padx=14, pady=(0, 12))

        left_col = ttk.Frame(main_pane)
        right_col = ttk.Frame(main_pane)
        main_pane.add(left_col, width=560)
        main_pane.add(right_col, width=580)

        # --- LEFT COLUMN: SD Cards, Sessions & Parameters ---
        sd_card = ttk.Frame(left_col, style="Card.TFrame", padding=10)
        sd_card.pack(fill="x", pady=(0, 8))
        hdr_sd = tk.Frame(sd_card, bg="#19212c")
        hdr_sd.pack(fill="x")
        ttk.Label(hdr_sd, text="Steg 1–2: MicroSD-kort & Tagningar (.id-filer)", style="Section.TLabel").pack(side="left")
        ttk.Button(hdr_sd, text="Skanna SD & Sessioner", style="Secondary.TButton", command=self.refresh_sd_and_sessions).pack(side="right")

        self._sd_labels = {}
        for cam_key, title in [
            ("cam1_left", "Cam 1 Vänster (CAM0_LEFT.id):"),
            ("cam2_center", "Cam 2 Mitten (CAM1_MID.id):"),
            ("cam3_right", "Cam 3 Höger (CAM2_RIGHT.id):"),
        ]:
            row_f = tk.Frame(sd_card, bg="#19212c")
            row_f.pack(fill="x", pady=2)
            tk.Label(row_f, text=title, width=28, anchor="w", bg="#19212c", fg="#c9d1d9", font=("Segoe UI", 9, "bold")).pack(side="left")
            val_lbl = tk.Label(row_f, text="Skannar...", anchor="w", bg="#19212c", fg="#8b949e", font=("Consolas", 9))
            val_lbl.pack(side="left", fill="x", expand=True)
            self._sd_labels[cam_key] = val_lbl

        take_row = tk.Frame(sd_card, bg="#19212c")
        take_row.pack(fill="x", pady=(6, 4))
        tk.Label(take_row, text="Välj tagning på korten:", bg="#19212c", fg="#c9d1d9", font=("Segoe UI", 9)).pack(side="left")
        self._take_combo = ttk.Combobox(take_row, state="readonly", width=48)
        self._take_combo.pack(side="left", padx=6, fill="x", expand=True)

        btn_sd_row = tk.Frame(sd_card, bg="#19212c")
        btn_sd_row.pack(fill="x", pady=(4, 0))
        ttk.Button(btn_sd_row, text="0A. Kopiera VALD tagning -> Ny Session", style="Secondary.TButton", command=self.import_selected_take).pack(side="left", fill="x", expand=True, padx=(0, 4))
        ttk.Button(btn_sd_row, text="0B. Kopiera ALLA tagningar på korten", style="Secondary.TButton", command=self.import_all_takes).pack(side="left", fill="x", expand=True, padx=(4, 0))

        # Session & Folder Card
        sess_card = ttk.Frame(left_col, style="Card.TFrame", padding=10)
        sess_card.pack(fill="x", pady=(0, 8))
        ttk.Label(sess_card, text="Steg 3: Sessionsmapp & Filkontroll (raw / processed)", style="Section.TLabel").pack(anchor="w", pady=(0, 6))

        sess_row = tk.Frame(sess_card, bg="#19212c")
        sess_row.pack(fill="x", pady=2)
        tk.Label(sess_row, text="Befintliga sessioner:", width=18, anchor="w", bg="#19212c", fg="#c9d1d9").pack(side="left")
        self._session_combo = ttk.Combobox(sess_row, state="readonly")
        self._session_combo.pack(side="left", fill="x", expand=True)
        self._session_combo.bind("<<ComboboxSelected>>", self._on_session_selected)

        self._raw_dir_var = tk.StringVar(value=r"D:\3DGS_Captures\Session_20261108_124238\raw")
        self._proj_dir_var = tk.StringVar(value=r"D:\3DGS_Captures\Session_20261108_124238\processed")

        for label_txt, var in [("Råvideo (raw):", self._raw_dir_var), ("Projekt (processed):", self._proj_dir_var)]:
            r = tk.Frame(sess_card, bg="#19212c")
            r.pack(fill="x", pady=2)
            tk.Label(r, text=label_txt, width=18, anchor="w", bg="#19212c", fg="#c9d1d9").pack(side="left")
            tk.Entry(r, textvariable=var, bg="#0d1117", fg="#e2e8f0", insertbackground="#e2e8f0", relief="flat").pack(side="left", fill="x", expand=True, padx=(0, 4))
            ttk.Button(r, text="...", width=3, style="Secondary.TButton", command=lambda v=var: self._browse_dir(v)).pack(side="right")

        self._file_check_lbl = tk.Label(sess_card, text="", justify="left", anchor="w", bg="#0d1117", fg="#58a6ff", font=("Consolas", 9), padx=8, pady=6)
        self._file_check_lbl.pack(fill="x", pady=(6, 0))

        # Pipeline Settings & Execution Buttons
        run_card = ttk.Frame(left_col, style="Card.TFrame", padding=10)
        run_card.pack(fill="both", expand=True)
        ttk.Label(run_card, text="Steg 4–10: Kör Pipeline & 3DGUT-Träning", style="Section.TLabel").pack(anchor="w", pady=(0, 6))

        params_f = tk.Frame(run_card, bg="#19212c")
        params_f.pack(fill="x", pady=(0, 8))
        tk.Label(params_f, text="Baslinje Cam1–Cam3 (mm):", bg="#19212c", fg="#c9d1d9").grid(row=0, column=0, sticky="w", pady=2)
        self._baseline_var = tk.StringVar(value="300.0")
        tk.Entry(params_f, textvariable=self._baseline_var, width=10, bg="#0d1117", fg="#e2e8f0", relief="flat").grid(row=0, column=1, sticky="w", padx=6)

        tk.Label(params_f, text="Skärpefönster (s):", bg="#19212c", fg="#c9d1d9").grid(row=0, column=2, sticky="w", padx=(12, 0))
        self._window_var = tk.StringVar(value="0.40")
        tk.Entry(params_f, textvariable=self._window_var, width=8, bg="#0d1117", fg="#e2e8f0", relief="flat").grid(row=0, column=3, sticky="w", padx=6)

        tk.Label(params_f, text="FPS:", bg="#19212c", fg="#c9d1d9").grid(row=0, column=4, sticky="w", padx=(12, 0))
        self._fps_var = tk.StringVar(value="50.0")
        tk.Entry(params_f, textvariable=self._fps_var, width=8, bg="#0d1117", fg="#e2e8f0", relief="flat").grid(row=0, column=5, sticky="w", padx=6)

        ttk.Button(
            run_card,
            text="▶ 1. KÖR STEG 4–8: Klapp-Synk + Skärpefilter + Spirula Rigg-SfM + Auto-Kalibrering",
            style="Action.TButton",
            command=self.run_step1_sfm,
        ).pack(fill="x", pady=3)

        ttk.Button(
            run_card,
            text="▶ 1B. Endast Steg 4–5: Klapp-Synk & Extrahera skarpaste bild-trior (Ingen SfM)",
            style="Secondary.TButton",
            command=self.run_sync_only,
        ).pack(fill="x", pady=3)

        ttk.Button(
            run_card,
            text="▶ 2A. Steg 9–10: Ladda in i LichtFeld Studio (via MCP) & Starta 3DGUT-Träning",
            style="Action.TButton",
            command=self.start_lichtfeld_mcp_training,
        ).pack(fill="x", pady=3)

        ttk.Button(
            run_card,
            text="▶ 2C. Öppna projektet i Spirula Studio GUI",
            style="Secondary.TButton",
            command=self.open_in_spirula_gui,
        ).pack(fill="x", pady=3)

        # --- RIGHT COLUMN: Live Progress, LichtFeld MCP Monitor & Real-time Log ---
        prog_card = ttk.Frame(right_col, style="Card.TFrame", padding=10)
        prog_card.pack(fill="x", pady=(0, 8))
        ttk.Label(prog_card, text="Realtidsstatus & Förlopp", style="Section.TLabel").pack(anchor="w")
        self._status_title_lbl = tk.Label(prog_card, text="REDO – Väntar på kommando", anchor="w", bg="#19212c", fg="#3fb950", font=("Segoe UI", 11, "bold"))
        self._status_title_lbl.pack(fill="x", pady=(4, 2))
        self._status_detail_lbl = tk.Label(prog_card, text="Välj en session eller kopiera från SD-kort för att börja.", anchor="w", bg="#19212c", fg="#c9d1d9", font=("Consolas", 9))
        self._status_detail_lbl.pack(fill="x", pady=(0, 6))
        self._progress_bar = ttk.Progressbar(prog_card, mode="determinate", maximum=100, style="Horizontal.TProgressbar")
        self._progress_bar.pack(fill="x")

        # LichtFeld Studio Live MCP Monitor Card
        mcp_card = ttk.Frame(right_col, style="Card.TFrame", padding=10)
        mcp_card.pack(fill="x", pady=(0, 8))
        mcp_hdr = tk.Frame(mcp_card, bg="#19212c")
        mcp_hdr.pack(fill="x")
        ttk.Label(mcp_hdr, text="LichtFeld Studio Live-Telemetri (MCP 127.0.0.1:45677)", style="Section.TLabel").pack(side="left")
        self._mcp_conn_lbl = tk.Label(mcp_hdr, text="● Kontrollerar...", bg="#19212c", fg="#8b949e", font=("Segoe UI", 9, "bold"))
        self._mcp_conn_lbl.pack(side="right")

        mcp_metrics = tk.Frame(mcp_card, bg="#0d1117", padx=10, pady=8)
        mcp_metrics.pack(fill="x", pady=(6, 0))
        self._mcp_state_lbl = tk.Label(mcp_metrics, text="State: -", bg="#0d1117", fg="#58a6ff", font=("Consolas", 10, "bold"))
        self._mcp_state_lbl.pack(side="left", padx=(0, 18))
        self._mcp_iter_lbl = tk.Label(mcp_metrics, text="Iteration: -", bg="#0d1117", fg="#e2e8f0", font=("Consolas", 10, "bold"))
        self._mcp_iter_lbl.pack(side="left", padx=(0, 18))
        self._mcp_loss_lbl = tk.Label(mcp_metrics, text="Loss: -", bg="#0d1117", fg="#3fb950", font=("Consolas", 10, "bold"))
        self._mcp_loss_lbl.pack(side="left", padx=(0, 18))
        self._mcp_splats_lbl = tk.Label(mcp_metrics, text="Splats: -", bg="#0d1117", fg="#d2a8ff", font=("Consolas", 10, "bold"))
        self._mcp_splats_lbl.pack(side="left")

        # Live Console Output
        log_card = ttk.Frame(right_col, style="Card.TFrame", padding=10)
        log_card.pack(fill="both", expand=True)
        ttk.Label(log_card, text="Fullständig Körningslogg (Live)", style="Section.TLabel").pack(anchor="w", pady=(0, 4))
        self._log_text = tk.Text(log_card, bg="#090d12", fg="#c9d1d9", font=("Consolas", 9), relief="flat", wrap="word")
        self._log_text.pack(fill="both", expand=True)

    def _set_step_state(self, step_idx: int, state: str):
        """state: 'pending' | 'active' | 'done' | 'error'"""
        if not (0 <= step_idx < len(self._step_badges)):
            return
        self._step_states[step_idx] = state
        frame, lbl_title, lbl_sub = self._step_badges[step_idx]
        colors = {
            "pending": ("#0d1117", "#30363d", "#8b949e"),
            "active": ("#12261e", "#2ea043", "#58a6ff"),
            "done": ("#0f2419", "#238636", "#3fb950"),
            "error": ("#2d1215", "#f85149", "#ff7b72"),
        }
        bg, border, fg = colors.get(state, colors["pending"])
        frame.configure(bg=bg, highlightbackground=border)
        lbl_title.configure(bg=bg, fg=fg)
        lbl_sub.configure(bg=bg)

    def _append_log(self, msg: str):
        self._log_text.insert("end", msg + "\n")
        self._log_text.see("end")

    def _browse_dir(self, var: tk.StringVar):
        chosen = filedialog.askdirectory(initialdir=var.get())
        if chosen:
            var.set(chosen)
            self._update_file_check()

    def refresh_sd_and_sessions(self):
        # 1. Scan SD cards
        if scan_sd_cards:
            try:
                self._sd_status = scan_sd_cards()
            except Exception as e:
                self._sd_status = {}
                self._append_log(f"[VARNING] Kunde inte skanna SD-kort: {e}")

        max_clips = 0
        all_connected = True
        for cam_key, lbl in self._sd_labels.items():
            info = self._sd_status.get(cam_key, {})
            if info.get("connected") and info.get("clip_count", 0) > 0:
                cnt = info["clip_count"]
                max_clips = max(max_clips, cnt)
                latest = info["clips"][0]
                lbl.configure(
                    text=f"[REDO] {info['drive']} ({cnt} klipp) Senaste: {latest['name']} ({latest['size_mb']} MB)",
                    fg="#3fb950",
                )
            elif info.get("connected"):
                all_connected = False
                lbl.configure(text=f"[TOMT KORT] {info['drive']} har 0 MP4 i DCIM", fg="#f85149")
            else:
                all_connected = False
                lbl.configure(text="[EJ ANSLUTET] Inget kort med .id hittades", fg="#8b949e")

        if all_connected and max_clips > 0:
            self._set_step_state(0, "done")
        else:
            self._set_step_state(0, "pending")

        # Populate take dropdown
        takes = []
        ref_clips = self._sd_status.get("cam2_center", {}).get("clips", []) or self._sd_status.get("cam1_left", {}).get("clips", [])
        for idx, c in enumerate(reversed(ref_clips), start=1):
            from_end = len(ref_clips) - idx + 1
            takes.append(f"Tagning #{idx}/{len(ref_clips)} (från slutet={from_end}): {c['name']} ({c['size_mb']} MB, {c['mtime_str']})")
        self._take_combo["values"] = takes if takes else ["Inga SD-kort anslutna"]
        if takes:
            self._take_combo.current(len(takes) - 1)

        # 2. Scan existing sessions in D:\3DGS_Captures
        self._sessions = []
        session_labels = []
        if CAPTURES_ROOT.exists():
            for d in sorted(CAPTURES_ROOT.iterdir(), key=lambda p: p.name, reverse=True):
                if d.is_dir() and d.name.startswith("Session_"):
                    raw_d = d / "raw"
                    proc_d = d / "processed"
                    mp4s = sum(1 for n in ("cam1_left.mp4", "cam2_center.mp4", "cam3_right.mp4") if (raw_d / n).exists())
                    has_sfm = (proc_d / "sparse" / "0" / "points3D.bin").exists()
                    tag = f"[{mp4s}/3 MP4" + (" + 3D-SfM KLAR]" if has_sfm else "]")
                    self._sessions.append(d)
                    session_labels.append(f"{d.name}  {tag}")

        self._session_combo["values"] = session_labels
        if session_labels:
            self._session_combo.current(0)

        self._update_file_check()

    def _on_session_selected(self, _event=None):
        idx = self._session_combo.current()
        if 0 <= idx < len(self._sessions):
            sess = self._sessions[idx]
            self._raw_dir_var.set(str(sess / "raw"))
            self._proj_dir_var.set(str(sess / "processed"))
            self._update_file_check()

    def _update_file_check(self):
        raw_d = Path(self._raw_dir_var.get())
        proc_d = Path(self._proj_dir_var.get())
        lines = []
        found = 0
        for name in ("cam1_left.mp4", "cam2_center.mp4", "cam3_right.mp4"):
            p = raw_d / name
            if p.exists():
                found += 1
                mb = p.stat().st_size / (1024 * 1024)
                lines.append(f"[OK] {name:<16} ({mb:.1f} MB)")
            else:
                lines.append(f"[SAKNAS] {name:<16}")

        sfm_bin = proc_d / "sparse" / "0" / "points3D.bin"
        img_dir = proc_d / "images" / "cam2_center"
        n_imgs = len(list(img_dir.glob("*.jpg"))) if img_dir.exists() else 0

        if sfm_bin.exists():
            lines.append(f"[3D-SfM KLAR] {n_imgs} bild-trior ({n_imgs*3} bilder) + sparse/0/points3D.bin redo för träning!")
            for s in (1, 2, 3, 4, 5, 6, 7):
                self._set_step_state(s, "done")
        elif found == 3:
            lines.append("[REDO FÖR STEG 1] Alla 3 MP4-filer finns i raw/. Klicka på '1. KÖR STEG 4–8' nedan.")
            self._set_step_state(1, "done")
            self._set_step_state(2, "done")
        else:
            self._set_step_state(2, "pending")

        self._file_check_lbl.configure(text="\n".join(lines))

    def _start_subprocess(self, args: list, title: str):
        if self._is_running:
            messagebox.showwarning("Upptagen", "En process körs redan. Vänta tills den är klar.")
            return
        self._is_running = True
        self._status_title_lbl.configure(text=f"[ARBETAR] {title}", fg="#58a6ff")
        self._progress_bar["value"] = 5
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
                for line in self._proc.stdout:
                    self._log_queue.put(("line", line.rstrip("\r\n")))
                rc = self._proc.wait()
                self._log_queue.put(("done", rc))
            except Exception as e:
                self._log_queue.put(("error", str(e)))

        threading.Thread(target=worker, daemon=True).start()

    def _poll_log_queue(self):
        try:
            while True:
                kind, payload = self._log_queue.get_nowait()
                if kind == "line":
                    self._append_log(payload)
                    self._parse_progress_line(payload)
                elif kind == "done":
                    self._is_running = False
                    if payload == 0:
                        self._status_title_lbl.configure(text="[KLAR] Processen slutfördes utan fel!", fg="#3fb950")
                        self._progress_bar["value"] = 100
                    else:
                        self._status_title_lbl.configure(text=f"[FEL] Processen avslutades med kod {payload}", fg="#f85149")
                    self.refresh_sd_and_sessions()
                elif kind == "error":
                    self._is_running = False
                    self._status_title_lbl.configure(text=f"[FEL] {payload}", fg="#f85149")
        except queue.Empty:
            pass
        self.after(150, self._poll_log_queue)

    def _parse_progress_line(self, line: str):
        low = line.lower()
        self._status_detail_lbl.configure(text=line[:115])
        if "kopierar" in low:
            self._set_step_state(1, "active")
            self._progress_bar["value"] = 20
        elif "steg 1/4" in low or "ljudspår" in low or "korskorrelation" in low:
            self._set_step_state(3, "active")
            self._progress_bar["value"] = 25
        elif "steg 2/4" in low or "laplacian" in low or "extraherar" in low:
            self._set_step_state(3, "done")
            self._set_step_state(4, "active")
            self._progress_bar["value"] = 45
        elif "rig_config.json" in low:
            self._set_step_state(4, "done")
            self._set_step_state(5, "done")
            self._progress_bar["value"] = 60
        elif "spirula" in low or "sfm" in low or "feature" in low or "bundle" in low:
            self._set_step_state(6, "active")
            self._progress_bar["value"] = 75
        elif "auto-kalibrer" in low or "calibrated_rig.json" in low or "baslinje" in low:
            self._set_step_state(6, "done")
            self._set_step_state(7, "active")
            self._progress_bar["value"] = 92
        elif "pipeline klar" in low:
            self._set_step_state(7, "done")
            self._progress_bar["value"] = 100

    def import_selected_take(self):
        python_exe = Path(r"D:\OptiGSRigg\.venv\Scripts\python.exe")
        if not python_exe.exists():
            python_exe = Path(sys.executable)
        ts = time.strftime("%Y%m%d_%H%M%S")
        sess_dir = CAPTURES_ROOT / f"Session_{ts}"
        raw_dir = sess_dir / "raw"
        proj_dir = sess_dir / "processed"
        self._raw_dir_var.set(str(raw_dir))
        self._proj_dir_var.set(str(proj_dir))

        args = [
            str(python_exe),
            str(SRC_SCRIPT),
            "--raw_dir", str(raw_dir),
            "--project_dir", str(proj_dir),
            "--import_sd",
            "--import_only",
        ]
        self._start_subprocess(args, "Steg 2: Kopierar vald tagning från SD-kort")

    def import_all_takes(self):
        python_exe = Path(r"D:\OptiGSRigg\.venv\Scripts\python.exe")
        if not python_exe.exists():
            python_exe = Path(sys.executable)
        args = [
            str(python_exe),
            str(SRC_SCRIPT),
            "--raw_dir", self._raw_dir_var.get(),
            "--project_dir", self._proj_dir_var.get(),
            "--import_all_takes",
            "--captures_root", str(CAPTURES_ROOT),
        ]
        self._start_subprocess(args, "Steg 2: Kopierar ALLA tagningar från SD-korten")

    def run_step1_sfm(self):
        python_exe = Path(r"D:\OptiGSRigg\.venv\Scripts\python.exe")
        if not python_exe.exists():
            python_exe = Path(sys.executable)
        args = [
            str(python_exe),
            str(SRC_SCRIPT),
            "--raw_dir", self._raw_dir_var.get(),
            "--project_dir", self._proj_dir_var.get(),
            "--mode", "auto_calibrate",
            "--baseline_mm", self._baseline_var.get(),
            "--fps", self._fps_var.get(),
            "--window_sec", self._window_var.get(),
            "--sfm_backend", "auto",
            "--engine", "none",
        ]
        self._start_subprocess(args, "Steg 4–8: Klapp-Synk + Skärpefilter + Rigg-SfM + Auto-Kalibrering")

    def run_sync_only(self):
        python_exe = Path(r"D:\OptiGSRigg\.venv\Scripts\python.exe")
        if not python_exe.exists():
            python_exe = Path(sys.executable)
        args = [
            str(python_exe),
            str(SRC_SCRIPT),
            "--raw_dir", self._raw_dir_var.get(),
            "--project_dir", self._proj_dir_var.get(),
            "--fps", self._fps_var.get(),
            "--window_sec", self._window_var.get(),
            "--skip_sfm",
            "--engine", "none",
        ]
        self._start_subprocess(args, "Steg 4–5: Endast Klapp-Synk & Skärpefilter")

    def open_in_spirula_gui(self):
        if not SPIRULA_EXE.exists():
            messagebox.showerror("Saknas", f"Hittade inte Spirula Studio på {SPIRULA_EXE}")
            return
        subprocess.Popen([str(SPIRULA_EXE), self._proj_dir_var.get()])
        self._append_log(f"Startade Spirula Studio GUI med: {self._proj_dir_var.get()}")

    def _mcp_call(self, tool_name: str, arguments: dict = None):
        payload = {
            "jsonrpc": "2.0",
            "id": 1,
            "method": "tools/call",
            "params": {"name": tool_name, "arguments": arguments or {}},
        }
        req = urllib.request.Request(
            MCP_URL,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=2.0) as resp:
            body = json.loads(resp.read().decode("utf-8"))
            txt = body.get("result", {}).get("content", [{}])[0].get("text", "{}")
            return json.loads(txt)

    def start_lichtfeld_mcp_training(self):
        proj = self._proj_dir_var.get()
        try:
            self._set_step_state(8, "active")
            self._append_log(f"Ansluter till LichtFeld Studio MCP ({MCP_URL})...")
            self._mcp_call("scene.load_dataset", {"path": proj})
            self._mcp_call("property.set", {"path": "training_params.gut", "value": True})
            self._mcp_call("property.set", {"path": "training_params.max_cap", "value": 1500000})
            self._mcp_call("training.start", {})
            self._set_step_state(8, "done")
            self._set_step_state(9, "active")
            self._append_log("[MCP] Laddade in projekt, satte GUT=True (3DGUT) och startade träning!")
            self._append_log("OBS: På RTX 2080 Ti tar Iteration 0 ca 4 minuter för CUDA PTX-kompilering innan iterationerna rullar i ~90 iter/s.")
        except Exception as e:
            self._append_log(f"[MCP-FEL] Kunde inte styra LichtFeld via MCP ({e}). Kontrollera att LichtFeld Studio är igång.")

    def _poll_lichtfeld_mcp(self):
        def check():
            try:
                st = self._mcp_call("training.status", {})
                state = st.get("trainer_state", "unknown")
                it = st.get("iteration", 0)
                loss = st.get("loss", 0.0)
                n_gauss = st.get("num_gaussians", 0)
                self.after(0, lambda: self._update_mcp_ui(True, state, it, loss, n_gauss))
            except Exception:
                self.after(0, lambda: self._update_mcp_ui(False, "-", 0, 0.0, 0))

        threading.Thread(target=check, daemon=True).start()
        self.after(2000, self._poll_lichtfeld_mcp)

    def _update_mcp_ui(self, connected: bool, state: str, it: int, loss: float, n_gauss: int):
        if connected:
            self._mcp_conn_lbl.configure(text="● Ansluten till LichtFeld Studio", fg="#3fb950")
            self._mcp_state_lbl.configure(text=f"State: {state}")
            self._mcp_iter_lbl.configure(text=f"Iteration: {it:,} / 36,000")
            self._mcp_loss_lbl.configure(text=f"Loss: {loss:.5f}")
            self._mcp_splats_lbl.configure(text=f"Splats: {n_gauss:,}")
            if state == "running":
                self._set_step_state(8, "done")
                self._set_step_state(9, "active")
            elif state == "completed":
                self._set_step_state(8, "done")
                self._set_step_state(9, "done")
        else:
            self._mcp_conn_lbl.configure(text="○ LichtFeld ej ansluten (eller kompilerar CUDA på Iter 0)", fg="#8b949e")


if __name__ == "__main__":
    app = OptiGSRiggApp()
    app.mainloop()
