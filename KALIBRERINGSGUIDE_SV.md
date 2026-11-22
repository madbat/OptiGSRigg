# OptiGSRigg — Komplett Guide: Exakt ChArUco Rigg-Kalibrering & 100 % Verifiering

Denna guide beskriver steg för steg hur du kalibrerar din **3-kamerarigg med DJI Osmo Action 4** (`cam1_left`, `cam2_center`, `cam3_right`), kontrollerar med **100 % säkerhet** att kalibreringen stämmer innan du åker ut och filmar, och använder den låsta rigg-profilen i **OptiGSRigg Studio**, **Spirula Studio 2026.10.30** och **LichtFeld Studio 0.5.4.1 (`3DGUT`)**.

> [!IMPORTANT]
> Du behöver **inte** mäta exakta millimetermått eller grader på riggens CAD-fästen själv. Så länge kamerorna sitter mekaniskt stumt i sina fästen mäter ChArUco-kalibreringen automatiskt upp varje kameras verkliga **fisheye-lins (`fx, fy, cx, cy, k1..k4`)** och **exakta 6-DoF position/vinkel** på bråkdelen av en millimeter och hundradels grader.

---

## 1. Förberedelser på kamerorna (DJI Osmo Action 4)

Ställ in **identiska inställningar på alla tre kameror** både vid kalibrering och vid all riktig fältfilmning:

| Inställning | Rekommenderat värde | Varför? |
| :--- | :--- | :--- |
| **Orientering & Format** | **`4K 4:3` (`2880 × 3840` stående)** *(eller det format ni alltid scannar i)* | Stående 4:3 fångar golv + tak samtidigt och matchar vår standardprofil. Formatet **måste** vara exakt samma vid kalibrering och scen-scanning! |
| **Synfält (FOV)** | **`Wide`** *(Rå fisheye)* | Stäng **av** `Dewarp`. Hela 155°-fisheye-bilden bevaras som `OPENCV_FISHEYE` för `3DGUT`. |
| **Bildstabilisering (EIS)** | **`AV`** *(RockSteady / HorizonSteady AVSTÄNGD)* | **Absolut krav!** Elektronisk stabilisering flyttar bilden digitalt varje millisekund, vilket förstör både rigg-kalibrering och SfM. |
| **Bildfrekvens (FPS)** | `50 fps` eller `60 fps` | Skarpare rutor med minimal rörelseoskärpa och tätare tidsfönster. |
| **Slutartid** | Manuell **`1/250 s` – `1/500 s`** *(ute `1/500`–`1/1000 s`)* | Förhindrar rörelseoskärpa på ChArUco-hörn vid handhållen/stativ-svepning. |
| **Färgprofil** | `Normal 8-bit/10-bit` eller **`D-Log M`** | Skanningar i D-Log M får automatiskt D-Log M -> Rec.709 LUT i OptiGSRigg Studio via EXIF. |
| **Vitbalans & ISO** | Låst manuell WB (t.ex. `5500K`) + begränsat ISO (`100–800`) | Jämn färg och kontrast mellan Cam 1, Cam 2 och Cam 3 i 3DGUT. |

---

## 2. Steg A: Generera och skriv ut de 3 ChArUco-tavlorna

### I OptiGSRigg Studio (Rekommenderat)
1. Öppna fliken **`STEG 0: ChArUco Rigg-Kalibrering`** högst upp i programmet.
2. Klicka på **`1. Generera 3 ChArUco-tavlor & Öppna Utskrift`**.

### Alternativ via terminal (CLI)
```bash
D:\OptiGSRigg\.venv\Scripts\python.exe calibration\1_generate_charuco_boards.py
```

Detta skapar filerna i `calibration\boards\`:
- `board_0_left.svg` / `.png` (Vänster vägg — unika ArUco ID `0..19`)
- `board_1_center.svg` / `.png` (Mitten — unika ArUco ID `20..39`)
- `board_2_right.svg` / `.png` (Höger vägg — unika ArUco ID `40..59`)
- `printable_calibration_kit.html` (komplett utskrifts-kit för 3 st **A4 liggande**)

### Kritiska utskriftsregler (100 % skala!)
1. Öppna `calibration\boards\printable_calibration_kit.html` i Chrome eller Edge och tryck `Ctrl + P`.
2. Välj **Liggande (Landscape)** på **A4** och ställ in Skala på **`100 %` / `Verklig storlek`** (*inte* "Anpassa till sida").
3. Stäng **av** sidhuvud/sidfot och välj **Marginaler: Inga** (eller Minimal).
4. **Mät kontrollinjalen med en vanlig linjal:** På sidan finns en utskriven `100.0 mm`-linjal.
   - Visar din linjal t.ex. `98.5 mm` i verkligheten? Ingen fara — skriv in `98.5` i fältet **`Uppmätt 100mm-linjal (mm)`** i Steg 0 (eller `--ruler-100mm 98.5` i CLI) så skalar koden automatiskt alla ChArUco-rutor till exakt verklig meterskala!
5. **Montera tavlorna 100 % plant (VIK ALDRIG EN TAVLA!):**
   > [!WARNING]
   > **Vik eller böj ALDRIG `board_1_center` inne i själva 90°-hörnvecket!** Varje papper måste vara helt slätt och styvt (limma/tejpa t.ex. på kartong, kapaskiva eller direkt på slät vägg).
   - **Metod A (Enklast — direkt på de 2 väggarna i ett vanligt 90°-hörn, utan extra skiva):**
     - `board_0_left`: Plant på **vänster vägg**, ca `35–50 cm` ut från hörnvecket.
     - `board_1_center`: **Helt plant på ena väggen bredvid hörnvecket** (ca `5–10 cm` från vecket — *inte över vecket*!) och **ca `15–25 cm` högre upp** för mittenkamerans `+8°` uppåtvinkel.
     - `board_2_right`: Plant längre ut på **höger vägg**, ca `50–65 cm` ut från hörnvecket.
   - **Metod B ("Sned brygga" över 90°-hörnet med en plan skiva):**
     - Tejpa `board_1_center` plant på en styv skiva (kartong/kapaskiva/hyllplan) som du sätter **snett (`45°`) tvärs över 90°-hörnet** (`15–25 cm` högre upp), medan `board_0_left` och `board_2_right` sitter på väggarna.
   - *(Tips: Det måste inte vara exakt 90° — `70°–120°` eller tre stadiga skivor i U-form på ett bord fungerar lika bra! Optimeraren mäter tavlornas verkliga 3D-plats automatiskt.)*

---

## 3. Steg B: Filma kalibreringsklippet (ca 45 sekunder i ett klipp)

1. Montera kamerorna på riggen och dra åt alla fästen ordentligt. **Rör inte fästena efter detta steg!**
2. Starta inspelning på alla tre Action 4-kameror samtidigt (`4K 4:3`, Wide, RockSteady AV).
3. Gör **en tydlig, hög handklapp** nära kamerorna (för exakt bildrutesynk via ljudvåg).
4. Gör **3 korta rundor med totalt ~17 lugna "frys-stopp"** (flytta lugnt mellan stoppen och **håll riggen helt stilla ~1 sekund** i varje stopp):
   - **Runda 1 — Rakt mot hörnet (`1.0 – 1.2 m` bort, Stopp 1–5):**
     1. Brösthöjd rakt mot hörnet (`Vänster` ser Tavla 0, `Mitten` ser Tavla 1, `Höger` ser Tavla 2).
     2. Sänk riggen `~30 cm` (frys 1 s).
     3. Höj riggen `~30 cm` (frys 1 s).
     4. Luta riggen lätt uppåt `~15°` (frys 1 s).
     5. Luta lätt neråt + sidledes som en ratt `~15°` (frys 1 s).
   - **Runda 2 — Vrid `±40°` så kamerorna delar tavlor (`1.0 – 1.2 m` bort, Stopp 6–11 — *låser ihop alla 3 kameror!*):**
     - **Stopp 6–8:** Vrid hela riggen **`~40°` åt HÖGER** (3 stopp i lite olika höjd, 1 s var) så **Cam 1 (Vänster)** ser `Tavla 1 (Mitten)` samtidigt som **Cam 2 (Mitten)** ser `Tavla 2 (Höger)`.
     - **Stopp 9–11:** Vrid hela riggen **`~40°` åt VÄNSTER** (3 stopp i lite olika höjd, 1 s var) så **Cam 3 (Höger)** ser `Tavla 1 (Mitten)` samtidigt som **Cam 2 (Mitten)** ser `Tavla 0 (Vänster)`.
   - **Runda 3 — Kliv NÄRA för bildkanterna (`0.5 – 0.7 m` bort, Stopp 12–17 — *mäter 155° fisheye-böjningen!*):**
     - Ta ett halvt kliv närmare (`50–70 cm` från tavlorna) och gör 6 frys-stopp vinklade lätt upp, ner, vänster och höger så att ChArUco-rutorna hamnar **ända ut i kanterna och hörnen** i alla 3 linser.
   - **Ljus:** Undvik starka blänk/reflexer från lampor rakt på tavlorna (matt papper fungerar bäst).
5. Kopiera in kalibreringsklippen via SD-importen som en session (t.ex. `Calib_Rig_v1`) eller placera `cam1_left.mp4`, `cam2_center.mp4`, `cam3_right.mp4` i valfri råmapp.

---

## 4. Steg C: Kör kalibrering & kontrollera 5-punktslistan (GO / NO-GO)

### I OptiGSRigg Studio
1. Välj din kalibreringssession (eller ange råmappen i **`STEG 0`**).
2. Kontrollera värdet i **`Uppmätt 100mm-linjal (mm)`** (standard `100.0`).
3. Klicka på **`2. Kalibrera Rigg från ChArUco-videor`**.

### I terminal (CLI)
```bash
D:\OptiGSRigg\.venv\Scripts\python.exe calibration\2_calibrate_rig.py --raw-dir D:\3DGS_Captures\Calib_01\raw --ruler-100mm 100.0
```
Kalibreringsmotorn skapar nu:
- `config\dji_rig_profile.json` *(fullständig profil med låsta `OPENCV_FISHEYE`-linser + 6-DoF rigg + kvalitetsmått)*
- `config\calibrated_rig.json` *(kompatibel profil för GUI-tabellen och Steg 2–10)*
- `config\calib_preview\calibration_verification_report.html` *(visuell rapport + kontrollbilder)*

### Hur du vet till 100 % att kalibreringen är perfekt (5-punkts GO/NO-GO):

| # | Kontrollpunkt | Grönt ljus (100 % OK) | Vad göra om GUL / RÖD? |
| :-: | :--- | :--- | :--- |
| **1** | **Global RMS-återprojektionsfel** | **`< 0.50 px`** *(helst `0.10 – 0.35 px`)* | För hög rörelseoskärpa, fel FPS/synk eller speglingar på tavlor. Filma om med kortare slutartid (`1/500 s`). |
| **2** | **Blind Hold-Out RMS (20 % oanvända rutor)** | **`< 0.70 px`** och nära Global RMS | Overfitting — kamerorna såg inte tavlorna i tillräckligt många vinklar/hörn. Ta fler lägen (minst 15–20 st). |
| **3** | **Cross-Camera Rigg-stabilitet** | Spridning **`< 0.50°`** & **`< 3.0 mm`** | Ljudklapp-synken slog fel på 1–2 rutor eller en kamera sitter lite löst i fästet. Dra åt skruvar & klappa tydligt. |
| **4** | **Linstäckning (Kanter & Hörn)** | **`>= 50 %`** av bildytan per kamera | Håll tavlorna närmare/sväng längre ut åt sidorna så linsernas ytterkanter också får ChArUco-träffar. |
| **5** | **Fysisk Baslinje & Vinklar** | L–R baslinje nära verkligt mått (t.ex. `~300 mm`), L/R yaw `≈ ∓45°` | Om avståndet är helt fel (t.ex. 150 mm eller 600 mm) har utskriften skalats fel — mät `100 mm`-linjalen igen! |

**Visuell kontroll i programmet:**
- Växla mellan **`Återprojektion (Grön/Röd)`** och **`Linstäckning (Hörn)`** i Steg 0 och håll inne musknappen på bilderna (**1:1 100 %-lupp**) för att se med egna ögon att varje rött/grönt siktkors sitter mitt i de svarta/vita Schackrutornas hörn ända ut i bildkanterna!

---

## 5. Steg D: Pre-Flight Kontroll (Innan ni åker ut och scannar!)

Innan du ska iväg på en viktig skanning (eller om riggen åkt i väskan) kan du kontrollera på **15 sekunder** att inga fästen har rört sig en enda grad — **helt utan ChArUco-tavlor**:

1. Gör ett snabbt 5–10 sekunders testklipp på valfri texturerad yta (t.ex. en bokhylla, fasad eller gård) med ett handklapp.
2. Klicka på **`3. Kontrollera Rigg på Aktiv Scen (Epipolär 3D-Check)`** i **`STEG 0`** (eller kör CLI):
```bash
D:\OptiGSRigg\.venv\Scripts\python.exe calibration\4_verify_calibration.py --raw-dir D:\3DGS_Captures\Testklipp\raw
```
3. Programmet matchar hundratals naturliga SIFT-punkter mellan `Cam 2 <-> Cam 1` och `Cam 2 <-> Cam 3`, ritar ut den låsta riggens böjda fisheye-epipolära linjer och ger dig ett omedelbart svar:
   - **`GODKÄND (< 1.8 px)`**: Upplösning matchar och alla 3 kameror sitter exakt kvar på sina kalibrerade platser!
   - **`FAIL (>= 3.5 px)` eller `FEL BILDFÖRHÅLLANDE`**: Något fäste har flyttat sig (redan `1°–2°` fångas direkt!) eller en kamera står i fel läge (t.ex. 16:9 i stället för 4:3). Kör om Steg B+C innan du scannar skarpt!

---

## 6. Steg E: Använd kalibreringen för 3DGS-scanning (Spirula & LichtFeld)

### Alternativ 1: Direkt i OptiGSRigg Studio (`STEG 2–10`)
När `config\dji_rig_profile.json` finns sparad väljs automatiskt läget:
**`Uppmätt ChArUco Rigg-Profil (Rekommenderad - Låst Lins & Rigg)`**.

1. **Steg 2:** Ljudsynkar och extraherar skarpa bild-trior (med automatisk D-Log M LUT om EXIF visar D-Log M).
2. **Steg 3:** Kör Spirula SfM med **låsta** `OPENCV_FISHEYE`-linsparametrar per kamera + separata kamerasekvenser (fixar kraschade/sammanslagna positioner och trasig brännvidd), följt av vår exakta **1:1 metriska Rigg-Låsning**:
   - Låser `cameras.bin` exakt till ChArUco-kalibreringens `fx, fy, cx, cy, k1..k4`.
   - Skalar världen till exakt **1:1 meter** via den uppmätta baslinjen.
   - Räddar automatiskt eventuella kamerabilder som Spirula missade att registrera (t.ex. i trånga passager) så att alla 3 kameror i varje trio behåller exakt `0.000°` rigg-låsning.
   - Skriver `sparse\0\rigs.txt`, `rig_config.json` och `rig_config_legacy.json`.
3. **Steg 4:** Öppna/kör i **LichtFeld Studio (`--gut` / 3DGUT)** eller **Spirula Studio (`--primitive 3dgut`)**. Hela 155°-fisheye-bilden tränas direkt utan `image_undistorter`!

### Alternativ 2: Via fristående exportskript (CLI)
```bash
D:\OptiGSRigg\.venv\Scripts\python.exe calibration\3_export_for_splatting.py ^
  --rig-profile config\dji_rig_profile.json ^
  --raw-dir D:\3DGS_Captures\MinScen\raw ^
  --output-dir D:\3DGS_Captures\MinScen\processed ^
  --fps 2.5
```
Dra sedan bara mappen `D:\3DGS_Captures\MinScen\processed` direkt in i **LichtFeld Studio (med 3DGUT aktiverat)** eller ladda in den i **Spirula Studio**.

---

## 7. Köra kodens självtest (Utan riktiga videor)

Vill du testa hela matematiken, bräddetekteringen, bundle-adjustern, 1:1 COLMAP-binärlåsningen och epipolära feldetekteringen när som helst:
```bash
D:\OptiGSRigg\.venv\Scripts\python.exe calibration\test_synthetic_rig.py
```
Testet använder temporära mappar och ändrar **aldrig** din riktiga riggprofil (`config\dji_rig_profile.json`).
