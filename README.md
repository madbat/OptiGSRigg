# OptiGSRigg – Dokumentation & Användarmanual

Komplett mjukvarupipeline och UI-plugin (**`OptiGSRigg`**) för **3x DJI Osmo Action 4** monterade i 90° porträttläge för 3D Gaussian Splatting (**3DGUT** med native `OPENCV_FISHEYE`) i **LichtFeld Studio v0.5.4.1** och **Spirula Studio 2026.10.30**.

---

## 0. Hela flödet steg för steg (Från SD-kort till färdig 3D-modell)

Här är exakt vad som händer under huven i varje steg från att du har filmat klart tills 3D-modellen är färdigtränad:

1. **Identifiera kamerorna via `.id`-filer på SD-korten (`scan_sd_cards`)**
   * Mjukvaran skannar alla enhetsbokstäver i Windows (`D:\` till `Z:\`) och letar efter de tre ID-filerna i roten på MicroSD-korten:
     * `CAM0_LEFT.id` $\rightarrow$ **Vänster kamera (`cam1_left`)**
     * `CAM1_MID.id` $\rightarrow$ **Mittenkamera (`cam2_center`)**
     * `CAM2_RIGHT.id` $\rightarrow$ **Höger kamera (`cam3_right`)**
   * På varje kort letar den upp alla inspelade `.MP4`-filer i `DCIM/` (och ignorerar DJI:s lågupplösta `.LRF`-filer), sorterar dem i tidsordning och visar alla tagningar i panelens rullgardinsmeny.

2. **Kopiera och döp om filmerna till datorn (`import_from_sd_cards` / `import_all_takes_from_sd_cards`)**
   * Du väljer antingen **en specifik tagning** (knapp `0A`) eller **alla tagningar på korten samtidigt** (knapp `0B`).
   * För varje tagning skapas en egen sessionsmapp (t.ex. `D:\3DGS_Captures\Session_20261108_115800\raw` och `\processed`).
   * Filmerna kopieras från de tre korten in i `raw/` och döps automatiskt om så att resten av pipelinen vet exakt vilken fil som tillhör vilken kamera:
     * `cam1_left.mp4`
     * `cam2_center.mp4`
     * `cam3_right.mp4`

3. **Filkontroll av råvideon (`_refresh_raw_status`)**
   * Innan bearbetningen startar kontrollerar panelen att alla 3 `.mp4`-filer finns i `raw/`-mappen och visar deras storlek i MB.

4. **Ljudsynkronisering på handklappen (`compute_audio_sync_offsets`)**
   * När du klickar på **`1. Synka (Klapp), Auto-Kalibrera & Kör Rigg-SfM`** läser FFmpeg ut de första **15 sekundernas ljudspår** (48 kHz mono) från alla tre videofilerna.
   * Med **FFT-korskorrelation (Cross-Correlation)** på ljudets energi-kurva hittar algoritmen exakt var din handklapp ligger i mittenkameran (`cam2_center`) jämfört med vänster (`cam1_left`) och höger (`cam3_right`).
   * Den räknar ut tidsskillnaden på millisekunden när och avrundar till exakt antal bildrutor vid 50 fps (t.ex. `0 frames` och `-1 frame`).

5. **Skärpefilter & Bildextrahering av synkade bild-trior (`extract_sharpest_synced_triplets`)**
   * Skriptet hoppar över de första **3 sekunderna** (där du står stilla och klappar).
   * Därefter delar det in videon i tidsfönster om **`0.40` sekunder** (20 bildrutor per fönster vid 50 fps).
   * Inom varje 0.4 s-fönster mäter OpenCV skärpan med ett **Laplacian-variansfilter** och letar upp det exakta ögonblick (mellan dina fotsteg) där **alla tre kamerorna samtidigt är som allra skarpast**.
   * Endast den skarpaste bildrutan per fönster sparas ut som högupplöst JPEG (`3840` px på höjden) i tre undermappar med exakt matchande filnamn (`frame_00000.jpg`, `frame_00001.jpg`, osv.):
     * `processed/images/cam1_left/`
     * `processed/images/cam2_center/`
     * `processed/images/cam3_right/`
   * *(Om du kryssat i D-Log M LUT appliceras även färgkorrigering till Rec.709 i detta steg).*

6. **Skapa Fisheye- och Rigg-konfiguration (`generate_colmap_rig_config`)**
   * Mjukvaran läser av den faktiska bildupplösningen i varje kameramapp (t.ex. `2880×3840` för 4:3 eller `2160×3840` för 16:9) och sätter upp linsmodellen **`OPENCV_FISHEYE`** för DJI Osmo Action 4:s vidvinkellins.
   * **Ingen destruktiv undistortion** görs på bilderna – alla pixlar ut i kanterna behålls i originalkvalitet.

7. **3D-rekonstruktion / Structure-from-Motion (`run_spirula_rig_sfm` / `run_colmap_rig_sfm`)**
   * **Spirula Studio (`spirula.exe sfm auto`)** (eller COLMAP) letar efter tusentals gemensamma punkter (features) mellan alla bilder på GPU:n via Vulkan.
   * Den räknar ut exakt var i rummet varje bild togs och skapar det glesa 3D-punktmolnet i `processed/sparse/0/` (`cameras.bin`, `images.bin`, `points3D.bin`).

8. **Freestyle Auto-Kalibrering & 1:1 Meterskalning (`calibrate_rig_and_scale_reconstruction`)**
   * Ur de 360 registrerade kamerapositionerna i `sparse/0/` mäter pipelinen hur `cam1_left` och `cam3_right` faktiskt sitter vridna och placerade i förhållande till `cam2_center` vid varje tidssteg.
   * Med **robusta medianvärden (SE3 / Quaternion-medelvärde)** räknar den ut kamerornas verkliga vinklar (**Yaw, Pitch, Roll** på `0.01°` när) och sparar dem i [`config/calibrated_rig.json`](file:///d:/OptiGSRigg/config/calibrated_rig.json).
   * Samtidigt jämför den det uppmätta avståndet mellan vänster och höger kamera med din fysiska baslinje (**`300 mm`**) och skalar om hela 3D-scenen så att **1 enhet i 3D-modellen = exakt 1.0 meter i verkligheten**.

9. **Aktivera `3DGUT` och ladda in scenen i LichtFeld Studio (`_enable_gut_for_fisheye`)**
   * Eftersom kamerorna använder fisheye-linser (`OPENCV_FISHEYE`) slår pluginen automatiskt på **`GUT = True` (`3DGUT` Unscented Transform)** i LichtFeld Studio så att fisheye-strålarna kan ritas upp direkt utan att räta ut bilderna.

10. **Träning av 3D Gaussian Splatting (`start_training`)**
    * **Iteration 0 (CUDA JIT-kompilering):** På ett RTX 2080 Ti kompilerar grafikkortet `3DGUT`-kärnorna på den allra första bildrutan (tar ~4 minuter, under vilket fönstret ser fryst ut).
    * **Iteration 1 till 36 000 (~90 iter/s):** Grafikkortet placerar ut upp till **1 500 000 Gaussians (3D-splats)** och jämför i varje iteration 3D-modellen mot dina 360 kamerabilder. Värdet **`Loss`** visar avvikelsen mot verkliga foton och sjunker allt eftersom 3D-scenen blir skarp och fotorealistisk.

---

## 1. Hårdvara & Rigg-geometri

### Nominell kameramontering på 2020-skena (Läge A – Standard)

| Kamera | Mappnamn / Filnamn | Placering på skena (X) | Horisontell vinkel (Yaw) | Vertikal vinkel (Pitch/Tilt) | Vertikal täckningszon |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Cam 1 (Vänster)** | `cam1_left` | `-150 mm` (från mitten) | `-45.0°` (Vänster) | `+12.0°` (Uppåt) | Från `-47°` (golv) till `+71°` (tak) |
| **Cam 2 (Mitten)** | `cam2_center` | `0 mm` (Centrum / Ref) | `0.0°` (Rakt fram) | `-8.0°` (Nedåt) | Från `-67°` (golv) till `+51°` (tak) |
| **Cam 3 (Höger)** | `cam3_right` | `+150 mm` (från mitten) | `+45.0°` (Höger) | `+12.0°` (Uppåt) | Från `-47°` (golv) till `+71°` (tak) |

*(Vid **Läge B – 180° Flippad skena** blir Pitch istället `-12.0°` på sidorna och `+8.0°` i mitten för extra täckning av höga väggar och undertak).*

### Kamerainställningar (DJI Osmo Action 4)
* **Fysisk montering:** Roterad **90° på högkant** (porträttläge, `2880 × 3840` pixlar i 4:3).
* **Bildfrekvens & Slutare:** **50 fps**, fast slutartid **1/500 s** (eller snabbare utomhus) för att frysa steg-oskärpa.
* **Exponering & Färg:** **Fast ISO** och **Fast vitbalans (Kelvin)** låsta i Custom-lägen (`C1–C5`), **10-bit D-Log M**.
* **Linsparametrar:** **Wide FOV** (184° diagonal fisheye), **RockSteady / HorizonSteady AVSTÄNGT (OFF)**.
  * *Viktigt:* Elektronisk stabilisering måste vara avstängd så att linsens optiska centrum och fisheye-geometri (`OPENCV_FISHEYE`) är 100 % fasta i varje bildruta.

---

## 2. Inspelning ute i fält & Automatisk SD-Kortsimport (`.id`-filer)

### A. Engångsförberedelse: ID-filer på MicroSD-korten
För att Windows aldrig ska kunna blanda ihop vänster, mitten och höger kamera (oavsett vilken USB-port eller ordning korten kopplas in i) lägger du en tom textfil i **roten** på respektive kameras MicroSD-kort:

* **Vänster kamera (Cam 1):** `CAM0_LEFT.id` *(mappas automatiskt till `cam1_left.mp4`)*
* **Mittenkamera (Cam 2):** `CAM1_MID.id` *(mappas automatiskt till `cam2_center.mp4`)*
* **Höger kamera (Cam 3):** `CAM2_RIGHT.id` *(mappas automatiskt till `cam3_right.mp4`)*

### B. Så filmar du ute i fält
Du behöver **ingen separat provfilmning** för kalibrering – allt sköts direkt på din riktiga tagning:

1. **Starta inspelning (Rec)** på alla tre kameror (det gör inget om du trycker igång dem med några sekunders mellanrum).
2. **Gör en tydlig handklapp** (eller knäpp med en klappa) inom de första **10–15 sekunderna** så att alla tre kamerornas mikrofoner fångar det skarpa ljudet.
3. **Gå lugnt genom scenen.** Pipelinen hoppar automatiskt över de första 3 sekunderna (där du klappar) och plockar därefter ut den skarpaste bildrutan mellan dina fotsteg var `0.4:e` sekund.

### C. Hämta videofilerna till datorn (Automatisk SD-Import & Statuskontroll)
När du kommer till datorn kan du antingen koppla in **alla 3 kameror/kort samtidigt via en USB-hubb**, eller stoppa i **ett MicroSD-kort i taget** i en vanlig kortläsare:

1. **Tydlig Live-status för alla 3 kamerakort i panelen:**
   Högst upp i `OptiGSRigg`-panelen ser du en statusruta för alla 3 kameror (`CAM0_LEFT.id`, `CAM1_MID.id`, `CAM2_RIGHT.id`):
   * **`[REDO] E:\ — 4 MP4 | Val #1: DJI_0012.MP4 (412.5 MB)`** (Grön text när kortet är inkopplat och har videofiler).
   * **`[TOMT KORT] E:\ sitter i, men 0 MP4-filer hittades i DCIM!`** (Röd varning om kortet sitter i men du råkat radera eller glömt filma på den kameran).
   * **`[EJ ANSLUTET] Inget SD-kort med CAMx.id hittades (D:–Z:)`** (Grå/varning när kortet inte är inkopplat).
   * **Omärkt SD-kort med `DCIM/`-mapp:** Om du stoppar i ett nytt/nyformaterat MicroSD-kort som har en `DCIM`-mapp men saknar `.id`-fil dyker en gul varningsruta upp med **3 snabbknappar** (`Sätt som CAM0_LEFT`, `Sätt som CAM1_MID`, `Sätt som CAM2_RIGHT`) så att du kan märka kortet med ett klick direkt i LichtFeld Studio!
2. Klicka på **`0A. Skapa ny tidsstämplad Sessionsmapp`** (skapar t.ex. `D:\3DGS_Captures\Session_20261030_153500\raw` och `processed`).
3. Välj vilket klipp från slutet av kortet du vill hämta i **`Klipp # (1=Senaste)`** (`1` = den allra senaste inspelningen på kortet, `2` = näst senaste, osv.) eller klicka på **`Skanna SD-kort igen`** om du nyss satt i ett kort.
4. Klicka på **`0B. Hämta MP4 från anslutna SD-kort -> Råvideomappen`**:
   * Skriptet ([`import_from_sd_cards`](file:///d:/OptiGSRigg/src/process_3dgs_rig.py#L140-L245)) skannar alla enhetsbokstäver (`D:` till `Z:`) efter `CAM0_LEFT.id`, `CAM1_MID.id` och `CAM2_RIGHT.id`.
   * Det går in i `DCIM/` på respektive kort, hittar rätt `.MP4`-fil (och struntar i DJI:s lågupplösta `.LRF`-filer), kopierar in den till `raw/` och döper om den till exakt:
     * `cam1_left.mp4`
     * `cam2_center.mp4`
     * `cam3_right.mp4`
   * *(Om du bara har en kortläsare för 1 kort åt gången: sätt i kort 1 -> klicka `0B`, byt till kort 2 -> klicka `0B`, byt till kort 3 -> klicka `0B`!)*
5. **Kontroll av Råvideomappen (`raw/`):**
   Precis under `Råvideo (3x MP4)` visar panelen alltid en live-checklista för de 3 filerna i mappen (`[OK]` i grönt med filstorlek i MB, eller `[SAKNAS I MAPPEN]` i rött). Om någon av de 3 filerna saknas varnar knappen **`1. Synka (Klapp)...`** direkt och talar om exakt vilken kamera som saknas.

---

## 3. Freestyle-läge vs. Fasta CAD-vinklar

När du "freestylar" med kamerafästen/kulleder i början (eller om ett 3D-printat fäste diffar på någon grad) ska du **inte** tvinga in felaktiga fasta vinklar i SfM, eftersom det ger dubbla väggar ("ghosting").

Därför har pipelinen fyra lägen (`Rigg-läge`):

1. **`Freestyle – Auto-kalibrera vinklar direkt ur klippet` (`auto_calibrate` – Standard!)**
   * Du kan skruva fast kamerorna på frihand ungefär som i tabellen ovan (bara de sitter stumt under promenaden).
   * Mät avståndet i millimeter mellan vänster (`cam1_left`) och höger (`cam3_right`) kamera med tumstock och ange i **`Baslinje Cam1–Cam3 (mm)`** (standard `300.0` mm).
   * När du kör steg 1 mäter mjukvaran kamerornas **exakta verkliga vinklar (Yaw, Pitch, Roll på 0.01° när)** direkt ur ditt riktiga videoklipp, skalar hela 3D-modellen till **1:1 meter** utifrån din baslinje och sparar profilen i `config/calibrated_rig.json`.
2. **`Kalibrerad – Återanvänd senast sparade kalibrering` (`calibrated`)**
   * Laddar de uppmätta vinklarna från `config/calibrated_rig.json` från din förra körning (bra om kamerorna sitter kvar orörda på skenan).
3. **`Läge A – Låst CAD (-8° Mitten / +12° Sidor)` (`mode_a`)**
   * Låser riggen direkt till CAD-ritningens nominella vinklar (`±45°` Yaw, `-8°/+12°` Pitch, `±150 mm`).
4. **`Läge B – Låst CAD 180° Flippad (+8° Mitten / -12° Sidor)` (`mode_b`)**
   * Samma som Läge A när hela skenan vänds 180° upp-och-ner.

---

## 4. Guide till UI-panelen i LichtFeld Studio (`OptiGSRigg`)

Pluginen ligger installerad i `C:\Users\patri\.lichtfeld\plugins\OptiGSRigg` (med källkod i [`lichtfeld_plugin/OptiGSRigg/`](file:///d:/OptiGSRigg/lichtfeld_plugin/OptiGSRigg/panels/main_panel.py)) och laddas automatiskt som fliken **"OptiGSRigg"** uppe till höger i LichtFeld Studio.

### Inställningsfält
* **`Råvideo (3x MP4)`**: Mappen med dina tre råvideofiler (`cam1_left.mp4`, `cam2_center.mp4`, `cam3_right.mp4`).
* **`Projektmapp (Utdata)`**: Mappen där alla synkade bilder (`images/`), kamerakalibrering (`sparse/0/`) och tränade modeller (`trained_models/`) skapas.
* **`Rigg-läge`**: Välj `Freestyle – Auto-kalibrera` när du testar/freestylar, eller `Läge A`/`Läge B` för fasta CAD-fästen.
* **`SfM-motor`**:
  * `Auto (Spirula Vulkan SfM / COLMAP)` – Använder Spirulas snabba GPU-accelererade Vulkan Rigg-SfM som standard.
  * `Spirula Studio Vulkan Rigg-SfM` – Tvingar körning via `spirula.exe sfm auto`.
  * `Fristående COLMAP` – Kör via `colmap.exe` (med automatisk snabb för-kalibrering på de första 18 bild-triorna om du är i Freestyle-läge).
* **`Baslinje Cam1–Cam3 (mm)`**: Fysiskt avstånd mellan ytterkamerorna (standard `300.0` mm = `150 mm` per sida). Sätter exakt **1:1 meterskala**.
* **`Videons FPS`**: Standard `50.0` fps.
* **`Skärpefönster (sek)`**: Standard `0.40` s (20 rutor vid 50 fps). Inom varje 0.4 s-fönster mäter ett Laplacian-filter skärpan och plockar ut den skarpaste bildrute-trion.
* **`Applicera D-Log M -> Rec.709 (.cube LUT)`**: Om ikryssad konverteras 10-bit D-Log M-färgerna via `.cube`-filen med FFmpeg under bildextraheringen.
* **`Max Splats`**: Målvärde för maximalt antal Gaussians (standard `1 500 000`).

### Knappar i panelen
* **`1. Synka (Klapp), Auto-Kalibrera & Kör Rigg-SfM` *(Huvudknapp)***
  Kör hela förarbetet:
  1. Extraherar de första 15 sekundernas ljud och synkar kamerorna med FFT-korskorrelation på handklappen.
  2. Väljer ut skarpaste bild-triorna med Laplacian-filtret till `images/cam1_left`, `cam2_center`, `cam3_right`.
  3. Kör Rigg-SfM med `OPENCV_FISHEYE` (utan destruktiv undistortion!), auto-kalibrerar vinklarna, skalar modellen till 1:1 meter och laddar in scenen i LichtFeld Studio.
* **`1B. Endast Klapp-Synk & Skärpefilter (Ingen SfM)`**
  Kör enbart ljudsynkningen och sparar ut de synkade, skarpa bild-triorna i `images/` (utan att starta SfM).
* **`2A. Ladda in projektmapp & Starta träning i LichtFeld`**
  Laddar in projektmappen i LichtFeld Studio och startar träningen direkt.
* **`2B. Kör MoGe-3 Djup + 3DGUT i Spirula (Headless)`**
  Kör Spirula Studio i bakgrunden på det synkade projektet: beräknar först AI-djup & normaler (`MoGe-3`) och tränar sedan en `3DGUT`-modell.
* **`2C. Öppna det synkade projektet i Spirula Studio GUI`**
  Startar **Spirula Studios grafiska gränssnitt** med din färdigsynkade och rigg-kalibrerade projektmapp direkt inladdad!

---

## 5. Så kör du med Spirula Studio + Klapp-synkningen

Eftersom Spirula Studio inte har en egen ljudklapp-synkronisering för tre separata MP4-filer, låter du vår pipeline göra synkningen först:

### Alternativ A: Via knappen i LichtFeld-panelen (Enklast)
1. Välj `Råvideo` och `Projektmapp` i panelen **OptiGSRigg**.
2. Klicka på **`1. Synka (Klapp), Auto-Kalibrera & Kör Rigg-SfM`**.
3. När det står `[KLAR]` i loggrutan, klicka på **`2C. Öppna det synkade projektet i Spirula Studio GUI`** (för att jobba vidare i Spirulas fönster) eller **`2B`** (för att låta Spirula träna klart automatiskt i bakgrunden).

### Alternativ B: Direkt från PowerShell-terminalen (Utan att öppna LichtFeld)
Du kan köra hela kedjan (Klapp-synk -> Skärpefilter -> Spirula Vulkan Rigg-SfM -> MoGe-3 -> Spirula 3DGUT-träning) med ett enda kommando i Antigravitys terminal:

```powershell
.\.venv\Scripts\python.exe src\process_3dgs_rig.py `
  --raw_dir "D:\3DGS_Captures\MinSkanning\Raw" `
  --project_dir "D:\3DGS_Captures\MinSkanning\Processed" `
  --mode auto_calibrate `
  --baseline_mm 300.0 `
  --sfm_backend spirula `
  --engine spirula
```

Vill du bara synka, extrahera bilder och köra Rigg-SfM (för att sedan öppna projektmappen manuellt i valfritt program) sätter du `--engine none`:

```powershell
.\.venv\Scripts\python.exe src\process_3dgs_rig.py `
  --raw_dir "D:\3DGS_Captures\MinSkanning\Raw" `
  --project_dir "D:\3DGS_Captures\MinSkanning\Processed" `
  --mode auto_calibrate `
  --sfm_backend spirula `
  --engine none
```

---

## 6. Viktiga sökvägar & Filer i projektet

* **Kärnmotor (Synk, Kalibrering, SfM & Träning):** [`src/process_3dgs_rig.py`](file:///d:/OptiGSRigg/src/process_3dgs_rig.py)
* **LichtFeld Studio v0.5.4.1 Plugin (Källkod):** [`lichtfeld_plugin/OptiGSRigg/panels/main_panel.py`](file:///d:/OptiGSRigg/lichtfeld_plugin/OptiGSRigg/panels/main_panel.py)
* **Installerad LichtFeld-plugin (Aktiv i UI):** `C:\Users\patri\.lichtfeld\plugins\OptiGSRigg`
* **Syntetiskt testskript (Torrkörning av synk & kalibrering):** [`tests/generate_mock_session.py`](file:///d:/OptiGSRigg/tests/generate_mock_session.py)
* **Sparad Freestyle-kalibrering:** `config/calibrated_rig.json` (skapas automatiskt vid `auto_calibrate`)
* **Låst CAD-konfiguration:** [`config/rig_config.json`](file:///d:/OptiGSRigg/config/rig_config.json)
* **3D LUT (.cube):** [`assets/luts/DJI_Action4_DLogM_to_Rec709.cube`](file:///d:/OptiGSRigg/assets/luts/DJI_Action4_DLogM_to_Rec709.cube)
* **LichtFeld Studio binär:** `D:\LichtFeld-Studio-windows-v0.5.4.1\bin\LichtFeld-Studio.exe`
* **Spirula Studio binär:** `D:\spirula-2026.10.30-windows-vulkan-x86_64\spirula.exe`

