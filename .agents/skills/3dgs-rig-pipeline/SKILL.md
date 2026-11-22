---
name: 3dgs-rig-pipeline
description: Hanterar, felsöker och optimerar vår 3-kamerors DJI Osmo Action 4 3DGS-pipeline (OptiGSRigg) för LichtFeld Studio 0.5.4.1 och Spirula Studio 2026.10.30.
---

# 3DGS Rig Pipeline Guidelines (OptiGSRigg)

1. **Rigg-hårdvara & Kameror:** 3x DJI Osmo Action 4 monterade i 90° porträttläge (`2880x3840` i 4K 4:3 — bredd 2880, höjd 3840 — eller `2160x3840` i 16:9) på en 360mm 2020-profil (`~300.0 mm` total baslinje mellan ytterkamerorna).
   - **Passiv CAD-nominell `cam_from_rig` (`mode_a` relativt `cam2_center` som lutar `+8.0°` uppåt):**
     - `cam1_left` (`CAM0_LEFT.id`): Fysiskt `-0.150 m`, Yaw `-45.0°`, Tilt `-12.0°` -> passiv `cam_from_rig`: `tvec ≈ [+0.106066, +0.022052, -0.103748]`, `euler ≈ [yaw +46.516°, pitch +17.587°, roll -5.926°]`
     - `cam2_center` (`CAM1_MID.id`): Referenssensor `ref_sensor = True` (`tx=0, ty=0, tz=0`, fysiskt Tilt `+8.0°` upp)
     - `cam3_right` (`CAM2_RIGHT.id`): Fysiskt `+0.150 m`, Yaw `+45.0°`, Tilt `-12.0°` -> passiv `cam_from_rig`: `tvec ≈ [-0.106066, +0.022052, -0.103748]`, `euler ≈ [yaw -46.516°, pitch +17.587°, roll +5.926°]`
   - Alla vinklar/positioner mäts exakt upp i **Steg 0 (ChArUco 3-tavelkalibrering)** och sparas i `config/dji_rig_profile.json`.
2. **Kameraparametrar:** `4K 4:3` (`2880×3840` stående), `Wide` FOV (rå fisheye, ALDRIG Dewarp), RockSteady / HorizonSteady **AV**, 50/60 fps, manuell slutare `1/250s–1/500s`, låst vitbalans/ISO. Valfri `D-Log M` detekteras automatiskt via EXIF/FFprobe för LUT i Steg 2.
3. **Steg 0: Exakt ChArUco Rigg-Kalibrering & Förkontroll (`calibration/` + `src/charuco_rig_calibration.py`):**
   - `1_generate_charuco_boards.py`: Skapar 3 unika ChArUco-tavlor (`board_0_left`, `board_1_center`, `board_2_right` i `.svg` + 300 DPI `.png`) + `printable_calibration_kit.html` i 100 % A4 liggande med `100 mm`-kontrollinjal.
   - `2_calibrate_rig.py`: Kalibrerar varje kameras 8-param `OPENCV_FISHEYE` (`fx, fy, cx, cy, k1..k4`) och riggens 6-DoF-geometri via gles Huber Bundle Adjustment + 20 % hold-out korsvalidering + 5-punkts GO/NO-GO. Sparar **enbart** riktiga körningar till `config/dji_rig_profile.json` + `config/calibrated_rig.json` (`--demo-synthetic` skriver till `config/dji_rig_profile_demo.json`, aldrig över skarp profil!).
   - `4_verify_calibration.py`: Pre-flight scenkontroll (upplösningsmatch + SIFT epipolärt pixel-fel under låst `Essential`-matris `< 1.8 px`, fångar om ett kamerafäste flyttats `>= 1°`).
   - `3_export_for_splatting.py`: Komplett fristående rigg-låst export till COLMAP/LichtFeld/Spirula.
   - `test_synthetic_rig.py`: Självtestsvit som verifierar bräddetektion (`24/24`), kalibrering (`RMS < 0.25 px`), `sparse/0` rigg-låsning (`0.000°`, räddning av tappad kamera, 1:1 meter) och epipolär feldetektering (`OK` vs `FAIL`).
4. **Installerade motorer & Korrekta Spirula SfM-flaggor:**
   - **LichtFeld Studio v0.5.4.1:** `D:\LichtFeld-Studio-windows-v0.5.4.1\bin\LichtFeld-Studio.exe`
     - CLI: `--headless -d <dataset> -o <out> --strategy mrnf --gut --use-normal-loss --use-depth-loss --enable-sparsity --max-cap 1500000 --export ply,spz`
     - MCP-server (när GUI körs): `http://127.0.0.1:45677/mcp`
     - Obs på RTX 2080 Ti (`SM 7.5`): `3DGUT` Iteration 0 tar ~4 min 20 s för CUDA PTX JIT-kompilering; därefter ~90 iter/s.
   - **Spirula Studio 2026.10.30:** `D:\spirula-2026.10.30-windows-vulkan-x86_64\spirula.exe`
     - **Viktigt:** Passera **aldrig** `--sequence cam1_left,cam2_center,cam3_right` ihopslaget med kommatecken (tvingar delad 3D-position utan baslinje!). Kör separata `--sequence cam1_left --sequence cam2_center --sequence cam3_right`, låsta per-kamera `--focal` och `--distortion`, `--no-exif-focal --focal-trials 0 --no-refine-extra-params --no-final-extra-params --no-final-principal-point`, samt `--pairs exhaustive` när totala antalet bilder `<= 270`.
     - Efter `spirula.exe sfm auto` körs `enforce_calibrated_rig_on_sparse_model()` automatiskt i `calibrated`/`mode_a`/`mode_b` för att skriva låsta `OPENCV_FISHEYE` till `cameras.bin`, skala till exakt 1:1 meter, rädda tappade bilder och skriva `sparse/0/rigs.txt` + `rig_config.json`.
5. **Ingen Undistortion:** Både LichtFeld Studio 0.5.4.1 (`--gut` / `3DGUT`) och Spirula Studio 2026.10.30 (`--primitive 3dgut`) stöder native `OPENCV_FISHEYE`. Kör ALDRIG `image_undistorter`!
6. **Kataloger:**
   - `D:\OptiGSRigg_app`: Fristående GUI-applikation (`app.py`, `ui/index.html`, `src/process_3dgs_rig.py`, `src/charuco_rig_calibration.py`, `calibration/`), manual i `KALIBRERINGSGUIDE_SV.md`.
   - `D:\OptiGSRigg`: Ursprunglig CLI-pipeline & venv (`D:\OptiGSRigg\.venv\Scripts\python.exe`).
