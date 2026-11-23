# Ändringslogg (Changelog)

Alla nämnvärda ändringar i **OptiGSRigg Studio** dokumenteras i den här filen.

Formatet följer [Keep a Changelog](https://keepachangelog.com/sv/1.1.0/) och projektet använder [Semantisk versionering](https://semver.org/lang/sv/) (`MAJOR.MINOR.PATCH`):
- **MAJOR** (`X.0.0`): Stora arkitekturförändringar eller brytande ändringar i kalibreringsformat/pipeline.
- **MINOR** (`x.Y.0`): Nya funktioner, arbetsflöden eller verktyg (bakåtkompatibla).
- **PATCH** (`x.y.Z`): Buggfixar, kalibreringsjusteringar och mindre förbättringar.

Aktuell version läses alltid från [`VERSION`](VERSION) vid programstart och visas i desktop-appen och CLI.

---

## [Osläppt]

### Tillagt
- *(Lägg in kommande funktioner här innan nästa release taggas.)*

### Ändrat
- *(Ändringar av befintligt beteende som ännu inte släppts.)*

### Fixat
- *(Buggfixar som ännu inte släppts.)*

---

## [2.1.0] - 2026-11-23

### Tillagt
- **Central versionshantering:** Lade till [`VERSION`](VERSION) (`2.1.0`) som enda sanningskälla samt denna [`CHANGELOG.md`](CHANGELOG.md). `app.py` läser versionen dynamiskt till `APP_VERSION` / `get_overview()` och `ui/index.html` visar aktuell version direkt i toppanelen.
- **Automatisk SD-kortsdetektering för kalibrering:** Avläsning av kamera-ID-filer (`CAM0_LEFT.id`, `CAM1_MID.id`, `CAM2_RIGHT.id`) i Steg 0B (`app.py`, `ui/index.html`) och CLI (`calibration/2_calibrate_rig.py --scan-sd` / `--from-sd`), inklusive tidsstämpelgruppering med fallback vid osynkade klockor, lämplighetspoäng, hantering av ett SD-kort i taget (partiell session), inline-ID-taggning samt direktflödet `Importera & Kalibrera direkt`.
- **10-bit `D-Log M` direkt i ChArUco-kalibrering:** `MultiBoardDetector` använder linjär kontraststräckning och CLAHE-fallback vid hörndetektering, medan `assets/luts/DJI_Action4_DLogM_to_Rec709.cube` enbart används visuellt på `reproj_*.jpg`-förhandsvisningarna.
- **Automatisk FPS-detektering:** Verklig videobildfrekvens läses nu automatiskt för korrekt klappsynkronisering.

### Fixat
- **ChArUco-kalibreringslösaren (`src/charuco_rig_calibration.py`):** Ersatte den tidigare glesa `lsmr`-lösaren med `cv2.fisheye.calibrate()` (inklusive tvåstegs outlier-rensning), tvåstegs exakt Bundle Adjustment (`tr_solver="exact"`, `soft_l1` följt av `huber`) och striktare stillastående-tröskel (`2.0 px/frame`), vilket sänkte reprojektionsfelet på 4K-inspelning från `243.7 px` till ~`1.06 px` (`±0.24°`, `346.2 mm` optisk baslinje).
- **Sidoprocess-säker import & uppstart:** `get_backend()` i `app.py` instansierar nu backend först vid behov så att CLI-skript kan importera hjälpfunktioner utan att starta bakgrundstrådar, och ett kvarglömt skrivfel (`w`) sist i `app.py` togs bort.

---

## [2.0.0] - 2026-11-22

### Tillagt
- **Fristående desktop-app (Windows & macOS):** Eget app-fönster via [`app.py`](app.py) och [`ui/index.html`](ui/index.html), kompletterat med [`setup_ny_dator.bat`](setup_ny_dator.bat), [`src/setup_new_pc_check.py`](src/setup_new_pc_check.py), [`create_desktop_shortcut.ps1`](create_desktop_shortcut.ps1), [`start_mac.command`](start_mac.command) samt app-ikoner i `assets/icons/`.
- **Steg 0 – ChArUco-kalibrering med 3 tavlor:** Fullständig 6-DoF rigg- och linskalibrering i [`src/charuco_rig_calibration.py`](src/charuco_rig_calibration.py), CLI-skript i `calibration/1_..4_*.py`, utskrivbara A4-tavlor (`calibration/boards/printable_calibration_kit.html`), självtest (`calibration/test_synthetic_rig.py`) och svensk manual ([`KALIBRERINGSGUIDE_SV.md`](KALIBRERINGSGUIDE_SV.md)).
- **Steg 1 – Kamera & EXIF-inspektion:** Automatisk skanning av SD-kort, DJI-EXIF/FFprobe-detektering (`D-Log M`), snabb 4K-förhandsvisning och chunkad filkopiering.
- **3DGUT, Spirula & LichtFeld-integration:** 33×33×33 färg-LUT (`assets/luts/DJI_Action4_DLogM_to_Rec709.cube`), Vulkan GPU-bildextraktion i Spirula, rigglåst SfM (`enforce_calibrated_rig_on_sparse_model`) och live MCP-telemetri mot LichtFeld Studio.

---

## [1.0.0] - 2026-11-08

### Tillagt
- **Första fungerande OptiGSRigg-pipelinen:** Kärna i [`src/process_3dgs_rig.py`](src/process_3dgs_rig.py), initialt webb-UI i [`app.py`](app.py), [`start_app.bat`](start_app.bat), [`requirements.txt`](requirements.txt) samt dokumentationen [`PIPELINE_STEG_FOR_STEG.md`](PIPELINE_STEG_FOR_STEG.md) och [`README.md`](README.md).
- **Ljudsynk & skärpefilter:** FFT-baserad klappsynkronisering mellan de tre kamerorna samt skärpefiltrering baserad på Laplacian-varians.
- **Initial riggkonfiguration:** Startprofiler i `config/calibrated_rig.json` och `config/rig_config.json`.
