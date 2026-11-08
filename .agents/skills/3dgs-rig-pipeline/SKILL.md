---
name: 3dgs-rig-pipeline
description: Hanterar, felsöker och optimerar vår 3-kamerors DJI Osmo Action 4 3DGS-pipeline (OptiGSRigg) för LichtFeld Studio 0.5.4.1 och Spirula Studio 2026.10.30.
---

# 3DGS Rig Pipeline Guidelines (OptiGSRigg)

1. **Rigg-hårdvara & Kameror:** 3x DJI Osmo Action 4 monterade i 90° porträttläge (`2880x3840` i 4:3, eller `2160x3840` i 16:9 – pipelinen läser varje kameras faktiska bildupplösning dynamiskt) på en 360mm 2020-profil (`300.0 mm` total baslinje mellan ytterkamerorna).
   - **SD-korts-ID i roten på varje kort:**
     - `CAM0_LEFT.id` $\rightarrow$ `cam1_left` (Vänster kamera, nominellt `X = -0.150m`, `Yaw = -45.0°`, `Pitch = +12.0°`)
     - `CAM1_MID.id` $\rightarrow$ `cam2_center` (Mittenkamera / Referens, `X = 0.000m`, `Yaw = 0.0°`, `Pitch = -8.0°`)
     - `CAM2_RIGHT.id` $\rightarrow$ `cam3_right` (Höger kamera, nominellt `X = +0.150m`, `Yaw = +45.0°`, `Pitch = +12.0°`)
2. **Kameraparametrar:** 50 fps, 1/500s fast slutare, 100 % Fast ISO & Fast vitbalans (C1–C5), 10-bit D-Log M, Wide FOV (184° diagonal fisheye), RockSteady / HorizonSteady **OFF**.
3. **Installerade motorer på `D:\`:**
   - **LichtFeld Studio v0.5.4.1:** `D:\LichtFeld-Studio-windows-v0.5.4.1\bin\LichtFeld-Studio.exe`
     - Träningsflaggor (CLI): `--headless -d <dataset> -o <out> --strategy mrnf --gut --use-normal-loss --use-depth-loss --enable-sparsity --max-cap 1500000 --export ply,spz`
     - **MCP-server (när GUI körs):** `http://127.0.0.1:45677/mcp` (JSON-RPC 2.0: `tools/call` med `training.status`, `training.start`, `scene.load_dataset`, ` {property.set: training_params.gut = True}`).
     - **Viktigt om `3DGUT` på RTX 2080 Ti (`SM 7.5`):** Iteration 0 (Frame 0) tar **~4 minuter och 20 sekunder** p.g.a. CUDA PTX JIT-kompilering av `3DGUT`-kärnorna (under vilket GUI-fönstret tillfälligt fryser). Från Iteration 1+ rullar träningen i **~90 iter/s (11 ms/iter)**.
   - **Spirula Studio 2026.10.30:** `D:\spirula-2026.10.30-windows-vulkan-x86_64\spirula.exe`
     - Inbyggd Vulkan Rigg-SfM: `spirula.exe sfm auto <images> -o <project> --camera-mode folder --camera-model opencv-fisheye --rig cam1_left,cam2_center,cam3_right --sequence cam1_left,cam2_center,cam3_right`
     - Geometri & Träning: `spirula.exe geometry <project> --model moge3-vitl --depth` följt av `spirula.exe train 3dgs --data <project> --primitive 3dgut`
4. **Ingen Undistortion:** Både LichtFeld Studio 0.5.4.1 (`--gut` / `params.gut = True`) och Spirula Studio 2026.10.30 (`--primitive 3dgut`) stöder native `OPENCV_FISHEYE`. Kör ALDRIG `image_undistorter` eftersom det skär bort 184°-täckningen i kanterna.
5. **Freestyle Auto-Kalibrering & 1:1 Meterskala:**
   - I `auto_calibrate`-läge beräknar `calibrate_rig_and_scale_reconstruction()` kamerornas exakta relativa vinklar (`Yaw`, `Pitch`, `Roll`) och skalar om hela COLMAP-rekonstruktionen (`sparse/0/images.bin` och `points3D.bin`) utifrån den fysiska baslinjen (`300.0 mm` mellan `cam1_left` och `cam3_right`) så att 1 enhet = exakt 1,0 meter.
6. **Projektkataloger:**
   - `D:\OptiGSRigg`: Befintlig CLI-pipeline (`src/process_3dgs_rig.py`) och LichtFeld Studio RmlUI-plugin (`lichtfeld_plugin/OptiGSRigg`).
   - `D:\OptiGSRigg_app`: Fristående GUI-applikation och kontrollcenter med full kontroll över hela 10-stegspipelinen, live-feedback och MCP-styrning.
