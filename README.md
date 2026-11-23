# OptiGSRigg Studio (`v2.1.0`)

> **Aktuell version:** [`v2.1.0`](VERSION) — se [`CHANGELOG.md`](CHANGELOG.md) för fullständig ändringshistorik mellan versioner.

Fristående desktop-applikation för **Windows & macOS** som hanterar hela 3D Gaussian Splatting-pipelinen (**3DGUT** med native `OPENCV_FISHEYE`, utan destruktiv undistortion) från **3× DJI Osmo Action 4** i 90° porträttläge till färdig 3D-modell i **Spirula Studio** och **LichtFeld Studio**.

Applikationen drivs av en lokal Python-backend ([`app.py`](app.py)) och ett modernt webbaserat desktop-gränssnitt ([`ui/index.html`](ui/index.html)) som öppnas i ett eget app-fönster.

---

## Huvudfunktioner

- **Steg 0 – ChArUco Rigg-Kalibrering & Förkontroll:** Skapar 3 unika ChArUco-tavlor, kalibrerar varje kameras 8-parameters `OPENCV_FISHEYE`-lins och 6-DoF-geometri samt verifierar inför varje tagning att inget kamerafäste rubbats (`KALIBRERINGSGUIDE_SV.md`).
- **Steg 1 – SD-kort, Inspektion & Import:** Identifierar `CAM0_LEFT.id`, `CAM1_MID.id` och `CAM2_RIGHT.id` automatiskt, läser ut fullständig EXIF/FFprobe-metadata (inkl. 10-bit `D-Log M`, slutartid, ISO, vitbalans), ger snabb 4K-förhandsvisning och kopierar till namngivna projektmappar.
- **Steg 2 – Klapp-synk & Skärpefilter:** Synkroniserar de tre MP4-spåren på millisekunden med FFT-korskorrelation på handklapp, väljer ut skarpaste bildrute-trion per tidsfönster med Laplacian-filter och applicerar valfri 3D-LUT för `D-Log M → Rec.709`.
- **Steg 3 – 3D Rigg-SfM & 1:1 Meterskala:** Kör Vulkan-accelererad SfM i **Spirula Studio** (eller **COLMAP**), låser fisheye-parametrar och riggens relativa geometri samt skalar hela 3D-scenen till exakt `1:1` meter.
- **Steg 4 – 3DGUT-träning & Live MCP:** Startar och övervakar träning i **LichtFeld Studio** (via MCP på `http://127.0.0.1:45677/mcp`) eller **Spirula Studio**.

---

## 1. Installation på en ny dator

### Förutsättningar
- **Python 3.10 eller nyare** *(på Windows kan `setup_ny_dator.bat` installera Python 3.11 automatiskt via `winget`)*
- **FFmpeg & FFprobe i `PATH`** *(behövs för ljudklapp-synk, bilduttag och EXIF; `setup_ny_dator.bat` kan installera via `winget`)*
- **Microsoft Edge eller Google Chrome** *(används för det fristående appfönstret)*
- **3D-motorer (valfritt för Steg 1/kalibrering, krävs för Steg 3–4):**
  - **Spirula Studio** (t.ex. `spirula-2026.10.30-windows-vulkan-x86_64`)
  - **LichtFeld Studio** (t.ex. `LichtFeld-Studio-windows-v0.5.4`)
  - **COLMAP** *(valfritt alternativ för SfM)*

### Windows (Rekommenderat — 1 klick)
1. Klona repot från GitHub:
   ```powershell
   git clone <DIN_GITHUB_REPO_URL> D:\OptiGSRigg_app
   cd D:\OptiGSRigg_app
   ```
2. Dubbelklicka på **[`setup_ny_dator.bat`](setup_ny_dator.bat)** i mappen. Skriptet gör automatiskt följande:
   - Kontrollerar Python ≥ 3.10 (eller erbjuder installation av Python 3.11 via `winget`).
   - Skapar en ren lokal `.venv` för den nya datorn och installerar paketen i [`requirements.txt`](requirements.txt) (`numpy`, `scipy`, `opencv-python-headless` inkl. `cv2.aruco`, `soundfile`, `tqdm`).
   - Kontrollerar `ffmpeg` och `ffprobe` i `PATH` (eller erbjuder `winget install Gyan.FFmpeg`).
   - Kopplar webbläsarfönstrets profil till den nya datorns namn.
   - Letar automatiskt upp **Spirula Studio**, **LichtFeld Studio** och **COLMAP** på `C:\`, `D:\`, i `Program Files` samt i portabla undermappar (t.ex. `C:\portabla-program\` och `D:\Splat programs\`) och skapar lokal `config/settings.json` samt inspelningsmapp (`D:\3DGS_Captures` eller `~/3DGS_Captures`).
   - Skapar genvägen **`OptiGSRigg Studio`** både på **Skrivbordet** och i **appmappen** som startar appen tyst via `.venv\Scripts\pythonw.exe` (utan konsolfönster).
3. Starta appen genom att dubbelklicka på genvägen **`OptiGSRigg Studio`** (eller [`start_app.bat`](start_app.bat) om du vill se terminalloggar; `start_app.bat --silent` startar också tyst).

### macOS
1. Klona repot och skapa lokal virtuell Python-miljö:
   ```bash
   git clone <DIN_GITHUB_REPO_URL> ~/OptiGSRigg_app
   cd ~/OptiGSRigg_app
   python3 -m venv .venv
   .venv/bin/pip install -r requirements.txt
   ```
2. Installera FFmpeg (t.ex. via Homebrew):
   ```bash
   brew install ffmpeg
   ```
3. Gör startskriptet körbart och starta appen:
   ```bash
   chmod +x start_mac.command
   ./start_mac.command
   ```

---

## 2. Jobba på flera datorer / platser (GitHub-flöde)

Projektet är konfigurerat med [`.gitignore`](.gitignore) så att bara källkod, UI, guider och delad kalibrering synkas via GitHub, medan varje dator behåller sin egen miljö och sina egna programsökvägar lokalt.

### Vad som synkas via GitHub vs. hålls lokalt

| Synkas via GitHub (gemensamt) | Förblir lokalt per dator (i `.gitignore`) |
| :--- | :--- |
| Programkod (`app.py`, `src/`, `ui/`, `calibration/`) | `.venv/` och `__pycache__/` (byggs per dator) |
| Skript, version & guider (`setup_ny_dator.bat`, `VERSION`, `README.md`, `CHANGELOG.md`, `KALIBRERINGSGUIDE_SV.md`) | `config/settings.json` (datorspecifika sökvägar till Spirula/LichtFeld/inspelningsmapp) |
| Färg-LUT ([`assets/luts/DJI_Action4_DLogM_to_Rec709.cube`](assets/luts/DJI_Action4_DLogM_to_Rec709.cube)) & ikoner | `config/app_window_profile/` (Edges lokala webbläsarprofil & cache) |
| Delad riggkalibrering (`config/calibrated_rig.json` m.fl.) | `*.lnk`-genvägar, `config/last_*.log`, `config/.running_instance.json` |

### Dagligt arbetsflöde mellan platser (t.ex. arbetsdator ↔ laptop)

1. **Innan du byter dator (spara och pusha dina kodändringar):**
   ```powershell
   git status
   git add .
   git commit -m "Beskrivning av ändring"
   git push
   ```
2. **När du sätter dig vid en annan redan installerad dator:**
   ```powershell
   git pull
   ```
   Om nya Python-paket eller startgenvägar har tillkommit, kör [`setup_ny_dator.bat`](setup_ny_dator.bat) (eller `setup_ny_dator.bat --verify-only` för snabbkontroll).
3. **När du installerar på en helt ny dator första gången:**
   Kör `git clone ...` och dubbelklicka på [`setup_ny_dator.bat`](setup_ny_dator.bat). Öppna sedan **Inställningar** i appen om någon 3D-motor ligger på en anpassad plats.
4. **Lokal inspelningsmapp och egna splats per dator (`captures_root`):**
   Varje dator har normalt sin egen lokala mapp för skanningar och splats (standard `D:\3DGS_Captures` på Windows om `D:\` finns, annars `~/3DGS_Captures` — eller valfri lokal mapp via **Inställningar → Inspelningsmapp**). Sökvägen sparas i `config/settings.json` som ligger i [`.gitignore`](.gitignore), så `git pull` och `git push` synkar bara källkoden utan att röra dina lokala tagningar, splats eller programsökvägar. *(Behöver du någon gång ta med en enskild skanning mellan datorer går det naturligtvis också bra att lägga den på en extern SSD eller kopiera sessionsmappen).*

---

## 3. Hårdvara, kamerainställningar & SD-kort

### Rekommenderade kamerainställningar (3× DJI Osmo Action 4)
Använd identiska inställningar på alla tre kameror vid både kalibrering och skanning:

| Parameter | Inställning | Anmärkning |
| :--- | :--- | :--- |
| **Montering** | 90° porträttläge på 2020-skena | ~`300.0 mm` baslinje mellan vänster och höger kamera |
| **Upplösning** | `4K 4:3` (`2880 × 3840`) *(alt. `2160 × 3840` i 16:9)* | Måste matcha din sparade riggkalibrering |
| **Synfält (FOV)** | **`Wide`** *(Rå fisheye)* | Slå **aldrig** på `Dewarp` |
| **Stabilisering** | **RockSteady / HorizonSteady AV** | Krav för fast `OPENCV_FISHEYE`-optik |
| **FPS & Slutare** | `50` eller `60 fps`, manuell slutare `1/250–1/500 s` | Minimerar rörelseoskärpa mellan fotsteg |
| **Färg & Exponering** | Låst ISO/vitbalans; `Normal` eller `10-bit D-Log M` | `D-Log M` detekteras automatiskt via EXIF/FFprobe |

### Märkning av MicroSD-kort
Lägg en tom ID-fil i roten på varje kameras MicroSD-kort (eller skapa den med ett klick direkt i appen när ett omärkt `DCIM/`-kort sätts i):
- `CAM0_LEFT.id` → `cam1_left` (Vänster)
- `CAM1_MID.id` → `cam2_center` (Mitten, referenskamera)
- `CAM2_RIGHT.id` → `cam3_right` (Höger)

---

## 4. Arbetsflödet i appen (Steg 0 – Steg 4)

1. **Steg 0 – ChArUco Rigg-Kalibrering & Förkontroll:**  
   Generera de tre A4-tavlorna, kalibrera riggen och kör scenkontroll före tagning så att vinklar och baslinje stämmer. Fullständig manual finns i [`KALIBRERINGSGUIDE_SV.md`](KALIBRERINGSGUIDE_SV.md).
2. **Steg 1 – SD-kort, Förhandsvisning & Import:**  
   Inspektera tagningar på SD-kort eller lokal disk, granska EXIF/färgläge/ljud och importera till ett namngivet projekt (`raw/cam1_left.mp4`, `cam2_center.mp4`, `cam3_right.mp4`).
3. **Steg 2 – Klapp-synk & Bildextraktion:**  
   Analyserar handklappen i ljudspåret, extraherar synkade skarpa bild-trior till `processed/images/cam*` och applicerar vid behov `D-Log M → Rec.709`-LUT.
4. **Steg 3 – 3D Rigg-SfM & Skalning till 1:1 meter:**  
   Välj mellan fyra rigg-lägen:
   - `auto_calibrate` *(Freestyle — mäter vinklar direkt ur klippet & skalar med baslinjen)*
   - `calibrated` *(Återanvänder sparad ChArUco-/riggkalibrering)*
   - `mode_a` / `mode_b` *(Nominella CAD-lägen, standard resp. 180° flippad skena)*  
   Kör **Spirula Vulkan SfM** (eller **COLMAP**) och låser rigg-geometrin + exakt meterskala i `processed/sparse/0/`.
5. **Steg 4 – 3DGUT-träning:**  
   Ladda `processed/` i **LichtFeld Studio** (med `--gut` / `3DGUT` aktiverat och realtidsövervakning via MCP) eller öppna/träna i **Spirula Studio**.

### Körning via terminal (CLI)
Hela pipelinen kan även köras från terminalen utan GUI:

```powershell
.\.venv\Scripts\python.exe src\process_3dgs_rig.py `
  --raw_dir "D:\3DGS_Captures\MittProjekt\raw" `
  --project_dir "D:\3DGS_Captures\MittProjekt\processed" `
  --mode auto_calibrate `
  --baseline_mm 300.0 `
  --sfm_backend spirula `
  --engine none
```

*(Byt `--engine none` mot `spirula` eller `lichtfeld` för att starta träning direkt efter SfM).*

---

## 5. Viktiga filer & mappstruktur

| Fil / Mapp | Beskrivning |
| :--- | :--- |
| [`app.py`](app.py) | Huvudserver, hårdvaru-/SD-skanning, FFprobe/EXIF, ljudsynk & MCP-klient |
| [`ui/index.html`](ui/index.html) | Desktop-gränssnittet för Steg 0–4 och inställningar |
| [`setup_ny_dator.bat`](setup_ny_dator.bat) | Helautomatisk installation & verifiering på ny Windows-dator |
| [`src/setup_new_pc_check.py`](src/setup_new_pc_check.py) | Efterkontroll som lagar `settings.json`, kontrollerar paket och skapar genvägar |
| [`create_desktop_shortcut.ps1`](create_desktop_shortcut.ps1) | Skapar `OptiGSRigg Studio.lnk` kopplad direkt till `.venv\Scripts\pythonw.exe` |
| [`start_app.bat`](start_app.bat) / [`start_mac.command`](start_mac.command) | Startskript för Windows (terminal eller `--silent`) resp. macOS |
| [`app_tkinter.py`](app_tkinter.py) | Äldre Tkinter-reservgränssnitt (`python app.py --classic`) |
| [`src/process_3dgs_rig.py`](src/process_3dgs_rig.py) | Kärnpipeline för ljudsynk, bildextraktion, SfM och rigg-kalibrering |
| [`src/charuco_rig_calibration.py`](src/charuco_rig_calibration.py) | ChArUco-kalibrering, `sparse/0`-låsning och epipolär scenverifiering |
| [`src/build_app_icon.py`](src/build_app_icon.py) | Genererar programikoner i `assets/icons/` |
| [`calibration/`](calibration/) | Fristående CLI-skript för ChArUco (`1_..4_*.py`) och självtest (`test_synthetic_rig.py`) |
| [`KALIBRERINGSGUIDE_SV.md`](KALIBRERINGSGUIDE_SV.md) | Detaljerad svensk manual för ChArUco-kalibrering |
| [`PIPELINE_STEG_FOR_STEG.md`](PIPELINE_STEG_FOR_STEG.md) | Teknisk specifikation över pipelinen |
| [`assets/luts/DJI_Action4_DLogM_to_Rec709.cube`](assets/luts/DJI_Action4_DLogM_to_Rec709.cube) | Färg-LUT för DJI Osmo Action 4 D-Log M |
| [`VERSION`](VERSION) | Enkel sanningskälla för aktuell programversion (`2.1.0`), läses av `app.py` & UI |
| [`CHANGELOG.md`](CHANGELOG.md) | Kronologisk ändringslogg per version (*Keep a Changelog*) |
| [`.gitignore`](.gitignore) | Hindrar `.venv`, Edge-cache, loggar, `.lnk` och lokal `settings.json` från att committas |

---

## 6. Felsökning på ny dator

- **Snabb hälsokontroll:** Kör `setup_ny_dator.bat --verify-only` i terminalen för att verifiera Python-paket, FFmpeg, motorer och lokal genväg utan att installera om.
- **Appen öppnas inte från genvägen:** Kör [`start_app.bat`](start_app.bat) i terminal för att se eventuella fel direkt, eller kontrollera `config/last_app_startup.log`.
- **Spirula eller LichtFeld hittades inte automatiskt:** Öppna **Inställningar** i appen och klicka på **Auto-detektera** eller bläddra fram respektive `.exe`-fil.
- **Kör självtest för ChArUco & rigg-låsning:**
  ```powershell
  .\.venv\Scripts\python.exe calibration\test_synthetic_rig.py
  ```
- **Alternativt klassiskt fönster (utan Edge/Chrome):**
  ```powershell
  .\.venv\Scripts\python.exe app.py --classic
  ```

---

## 7. Versionshantering & Release-flöde

Projektet använder **Semantisk versionering** (`MAJOR.MINOR.PATCH`) med [`VERSION`](VERSION) som enda sanningskälla och [`CHANGELOG.md`](CHANGELOG.md) för detaljerade ändringar.

### Släppta versioner (översikt)

| Version | Datum | Huvudpunkter | Detaljer |
| :--- | :--- | :--- | :--- |
| **v2.1.0** | 2026-11-23 | SD-kortets `.id`-detektering i Steg 0B, 10-bit `D-Log M` i ChArUco, fixad fisheye/BA-lösare (`~1.06 px`), auto-FPS för ljudsynk, `VERSION` & `CHANGELOG.md` | [`CHANGELOG.md`](CHANGELOG.md#210---2026-11-23) |
| **v2.0.0** | 2026-11-22 | Fristående desktop-app (Windows & macOS), ChArUco 3-tavelskalibrering, Steg 1 EXIF/preview, D-Log M LUT, Spirula Vulkan & LichtFeld MCP | [`CHANGELOG.md`](CHANGELOG.md#200---2026-11-22) |
| **v1.0.0** | 2026-11-08 | Första OptiGSRigg-pipelinen (`process_3dgs_rig.py`), initialt webb-UI, FFT klappsynk & Laplacian skärpefilter | [`CHANGELOG.md`](CHANGELOG.md#100---2026-11-08) |

### Så skapar du en ny version

1. **Uppdatera [`VERSION`](VERSION)** med nytt versionsnummer enligt `MAJOR.MINOR.PATCH` (t.ex. `2.1.1` för buggfix, `2.2.0` för ny funktion eller `3.0.0` för brytande förändring). [`app.py`](app.py) och toppanelen i [`ui/index.html`](ui/index.html) visar automatiskt nya numret vid nästa start.
2. **Uppdatera [`CHANGELOG.md`](CHANGELOG.md) & denna `README.md`:** Flytta punkter från `[Osläppt]` till en ny daterad rubrik `[X.Y.Z] - ÅÅÅÅ-MM-DD` och uppdatera rubriken samt tabellen ovan.
3. **Spara & tagga versionen i Git:**
   ```powershell
   git add VERSION CHANGELOG.md README.md
   git commit -m "Release v2.1.0"
   git tag -a v2.1.0 -m "OptiGSRigg Studio v2.1.0"
   git push origin main --tags
   ```
4. **Hoppa mellan versioner vid behov:**
   ```powershell
   git tag -l                  # Lista alla sparade versioner
   git checkout v2.0.0         # Öppna en tidigare version för test/jämförelse
   git switch -                # Gå tillbaka till senaste arbetsgrenen
   ```
