# OptiGSRigg – Hela Pipelinen Steg för Steg (Specifikation för Eget GUI & Program)

Detta dokument beskriver **exakt varje steg** i `OptiGSRigg`-pipelinen från att du har filmat klart med de tre DJI Osmo Action 4-kamerorna tills en färdig 3D Gaussian Splatting-modell (`3DGUT`) är tränad. Dokumentet fungerar även som arkitektur- och kravspecifikation för att bygga ett **eget fristående GUI-program** med full kontroll och realtidsfeedback.

---

## Översikt av Arbetsflödet

```
[3x MicroSD-kort]
   ├── CAM0_LEFT.id  (Vänster)
   ├── CAM1_MID.id   (Mitten)
   └── CAM2_RIGHT.id (Höger)
          │
          ▼  (Steg 1–3: Identifiera, kopiera & validera)
[Session_YYYYMMDD_HHMMSS/raw/]
   ├── cam1_left.mp4
   ├── cam2_center.mp4
   └── cam3_right.mp4
          │
          ▼  (Steg 4: FFT Ljudklapp-synk på 15 s ljudspår)
[Exakta bildrute-offsets (t.ex. 0 frames, -1 frame vid 50 fps)]
          │
          ▼  (Steg 5: Laplacian Skärpefilter var 0.40 s + ev. D-Log M LUT)
[Session_YYYYMMDD_HHMMSS/processed/images/]
   ├── cam1_left/frame_00000.jpg ...
   ├── cam2_center/frame_00000.jpg ...
   └── cam3_right/frame_00000.jpg ...
          │
          ▼  (Steg 6–7: OPENCV_FISHEYE + Spirula Vulkan Rigg-SfM)
[Session_YYYYMMDD_HHMMSS/processed/sparse/0/]
   ├── cameras.bin
   ├── images.bin
   └── points3D.bin
          │
          ▼  (Steg 8: Freestyle Auto-Kalibrering & 1:1 Meterskalning via 300 mm baslinje)
[config/calibrated_rig.json + Skalat 3D-punktmoln i sparse/0/]
          │
          ▼  (Steg 9–10: 3DGUT Fisheye-träning i LichtFeld Studio / Spirula Studio)
[Färdig 3DGS-modell (.ply / .splat)]
```

---

## Detaljerad Steg-för-Steg-Specifikation

### Steg 1: Identifiera kamerorna via `.id`-filer på SD-korten
* **Källkod:** [`scan_sd_cards()`](file:///d:/OptiGSRigg/src/process_3dgs_rig.py#L96-L163) i [`src/process_3dgs_rig.py`](file:///d:/OptiGSRigg/src/process_3dgs_rig.py)
* **Indata:** Inkopplade USB-enheter / MicroSD-kort i Windows (`D:\` till `Z:\`).
* **Vad som händer:**
  1. Mjukvaran går igenom alla enhetsbokstäver (`D:\`–`Z:\`) och letar efter tre tomma ID-filer i roten på korten:
     * `CAM0_LEFT.id` $\rightarrow$ **Vänster kamera (`cam1_left`)**
     * `CAM1_MID.id` $\rightarrow$ **Mittenkamera (`cam2_center`)**
     * `CAM2_RIGHT.id` $\rightarrow$ **Höger kamera (`cam3_right`)**
  2. På varje kort genomsöks `DCIM/` rekursivt efter `.MP4`-filer (medan DJI:s lågupplösta `.LRF`-filer filtreras bort).
  3. Alla inspelade klipp sorteras i tidsordning (`#1` äldst till `#N` nyast) med tidsstämpel och filstorlek i MB.
* **Feedback att visa i ett eget GUI:**
  * Kortstatus per kamera (Grön = Anslutet med X klipp, Gul = Anslutet men saknar `.id`-fil, Röd = Kort saknas eller är tomt).
  * Tabell över alla tagningar (`Take #1`, `Take #2`, ...) där man direkt ser om alla 3 kameror har matchande klipp och längd.

---

### Steg 2: Kopiera filmerna till datorn och döp om dem rätt
* **Källkod:** [`import_from_sd_cards()`](file:///d:/OptiGSRigg/src/process_3dgs_rig.py#L166-L246) och [`import_all_takes_from_sd_cards()`](file:///d:/OptiGSRigg/src/process_3dgs_rig.py#L249-L331)
* **Indata:** Valda `.MP4`-filer på de tre SD-korten.
* **Utdata:**
  * `D:\3DGS_Captures\Session_YYYYMMDD_HHMMSS\raw\cam1_left.mp4`
  * `D:\3DGS_Captures\Session_YYYYMMDD_HHMMSS\raw\cam2_center.mp4`
  * `D:\3DGS_Captures\Session_YYYYMMDD_HHMMSS\raw\cam3_right.mp4`
* **Vad som händer:**
  1. En ny sessionsmapp skapas med undermapparna `raw\` och `processed\`.
  2. Videofilerna kopieras från SD-korten in i `raw\` och döps automatiskt om efter sin `.id`-fil så att resten av pipelinen vet exakt vilken fil som är vänster, mitten och höger.
  3. Vid batch-import kopieras alla tagningar på korten till varsin egen sessionsmapp.
* **Feedback att visa i ett eget GUI:**
  * Kopieringshastighet (MB/s), procentbar per kamera (`0–100 %`) och uppskattad tid kvar.

---

### Steg 3: Kontrollera råvideofilerna (Format, Upplösning & FPS)
* **Källkod:** [`_refresh_raw_status()`](file:///d:/OptiGSRigg/lichtfeld_plugin/OptiGSRigg/panels/main_panel.py#L276-L315)
* **Indata:** `raw\cam1_left.mp4`, `raw\cam2_center.mp4`, `raw\cam3_right.mp4`
* **Vad som händer:**
  1. Kontrollerar att alla tre videofilerna finns och är läsbara.
  2. Läser ut upplösning (t.ex. `2880×3840` 4:3 eller `2160×3840` 16:9), bildfrekvens (`50 fps`) och videolängd i sekunder.
* **Feedback att visa i ett eget GUI:**
  * Tydlig varning om en av kamerorna råkat stå i `16:9` istället för `4:3` eller annan FPS, samt miniatyrbild (thumbnail) från varje kamera.

---

### Steg 4: Ljudsynkronisering på handklappen (Millisekund- & Bildrutesynk)
* **Källkod:** [`compute_audio_sync_offsets()`](file:///d:/OptiGSRigg/src/process_3dgs_rig.py#L462-L545)
* **Indata:** De första **15 sekunderna** av ljudspåret i de 3 MP4-filerna.
* **Utdata:** Exakt förskjutning i millisekunder och bildrutor (`frame_offset`) för varje kamera relativt `cam2_center`.
* **Vad som händer:**
  1. FFmpeg extraherar 15 sekunders okomprimerat PCM-ljud (`48 000 Hz`, mono) från alla tre kameror.
  2. Ljudsignalen omvandlas till en energi-envelope (absolutbelopp/ljudtoppar).
  3. **FFT-korskorrelation (Fast Fourier Transform Cross-Correlation)** jämför `cam1_left` och `cam3_right` mot `cam2_center` (`0.0 ms`).
  4. Den högsta korrelationstoppen (din handklapp) ger tidsskillnaden på millisekunden när (t.ex. `-5.9 ms` och `-18.3 ms`), vilket avrundas till närmaste hel bildruta vid 50 fps (`0 frames`, `-1 frame`).
* **Feedback att visa i ett eget GUI:**
  * Visuell ljudvågform (waveform) för alla 3 kameror där handklappen är inzoomad och markerad med ett grönt streck, så att man med blotta ögat ser att klapparna ligger 100 % synkat!

---

### Steg 5: Skärpefilter (Laplacian) & Extrahering av synkade bild-trior
* **Källkod:** [`extract_sharpest_synced_triplets()`](file:///d:/OptiGSRigg/src/process_3dgs_rig.py#L611-L744)
* **Indata:** De 3 MP4-filerna + `frame_offsets` från Steg 4.
* **Utdata:** Synkade, skarpa JPEG-bildtrior i:
  * `processed\images\cam1_left\frame_00000.jpg ...`
  * `processed\images\cam2_center\frame_00000.jpg ...`
  * `processed\images\cam3_right\frame_00000.jpg ...`
* **Vad som händer:**
  1. De första **3.0 sekunderna** hoppas över (där operatören står stilla och klappar händerna).
  2. Resten av videon delas in i tidsfönster om **`0.40` sekunder** (`20` bildrutor per fönster vid 50 fps).
  3. För varje synkad bildruta i fönstret mäter OpenCV skärpan med ett **Laplacian-variansfilter** (`cv2.Laplacian(gray, cv2.CV_64F).var()`).
  4. Det ögonblick i fönstret där **alla 3 kameror samtidigt har högst skärpa** (mellan två fotsteg) väljs ut. De övriga 19 bildrutorna i fönstret kastas.
  5. Om D-Log M LUT är aktiverat konverteras färgerna samtidigt via `.cube`-filen till Rec.709.
* **Feedback att visa i ett eget GUI:**
  * Live-räknare (`Extraherar bild-trio 45 / 120...`), aktuell skärpepoäng (Laplacian-kurva) och förhandsgranskning av de senast utvalda 3 bilderna bredvid varandra.

---

### Steg 6: Skapa Fisheye- och Rigg-konfiguration
* **Källkod:** [`generate_colmap_rig_config()`](file:///d:/OptiGSRigg/src/process_3dgs_rig.py#L747-L820)
* **Utdata:** `processed\rig_config.json`
* **Vad som händer:**
  1. Läser av den faktiska bildupplösningen i respektive kameramapp (`2880×3840` eller `2160×3840`).
  2. Skapar kameraparametrar för linsmodellen **`OPENCV_FISHEYE`** (`fx, fy, cx, cy, k1, k2, k3, k4`) anpassade för DJI Osmo Action 4 i porträttläge.
  3. **Ingen "undistortion" görs på bilderna** – alla pixlar ända ut i vidvinkelhörnen bevaras oförstörda.

---

### Steg 7: 3D-rekonstruktion / Rigg-SfM (Structure-from-Motion)
* **Källkod:** [`run_spirula_rig_sfm()`](file:///d:/OptiGSRigg/src/process_3dgs_rig.py#L823-L884)
* **Kommando under huven:**
  ```powershell
  D:\spirula-2026.10.30-windows-vulkan-x86_64\spirula.exe sfm auto --device auto --data-dir "<processed>" --image-dir "<processed>\images"
  ```
* **Utdata:** `processed\sparse\0\cameras.bin`, `images.bin`, `points3D.bin`
* **Vad som händer:**
  1. Spirula Studio körs på GPU:n via Vulkan och extraherar nyckelpunkter (features) i alla bilder.
  2. Matchar punkter mellan kamerorna och längs promenaden.
  3. Beräknar varje kameras 3D-position och skapar det glesa 3D-punktmolnet (t.ex. `360 / 360` registrerade bilder och `176 672` 3D-punkter).
* **Feedback att visa i ett eget GUI:**
  * Steg-indikator för SfM (`Feature Extraction` $\rightarrow$ `Feature Matching` $\rightarrow$ `Bundle Adjustment`), antal registrerade bilder (`360/360 = 100 %`) och antal 3D-punkter.

---

### Steg 8: Freestyle Auto-Kalibrering & 1:1 Meterskalning
* **Källkod:** [`calibrate_rig_and_scale_reconstruction()`](file:///d:/OptiGSRigg/src/process_3dgs_rig.py#L887-L1035)
* **Utdata:**
  * Sparad kalibreringsprofil i [`config/calibrated_rig.json`](file:///d:/OptiGSRigg/config/calibrated_rig.json)
  * Omskalad 1:1 meter-modell i `processed\sparse\0\images.bin` och `points3D.bin`
* **Vad som händer:**
  1. Läser de beräknade kamerapositionerna i `sparse\0\` för varje bild-trio (`frame_00000` till `frame_00119`).
  2. Beräknar den relativa rotationen och förflyttningen från mittenkameran (`cam2_center`) till vänster (`cam1_left`) och höger (`cam3_right`) kamera vid varje tidssteg.
  3. Tar fram ett robust medianvärde (Quaternion / SE3-medelvärde) för kamerornas exakta fysiska vinklar (**Yaw, Pitch, Roll** på `0.01°` när).
  4. Jämför det uppmätta avståndet mellan `cam1_left` och `cam3_right` i SfM-modellen med det verkliga måttet på skenan (**`300.0 mm`**) och skalar om alla kamerapositioner och 3D-punkter i `sparse\0\` så att **1 enhet = exakt 1,0 meter**.
* **Feedback att visa i ett eget GUI:**
  * Visuell 3D- eller 2D-skiss av skenan med de uppmätta vinklarna (`Yaw`, `Pitch`, `Roll`) och baslinjen i millimeter (`X = -155.5 mm` / `X = +144.4 mm`).

---

### Steg 9 & 10: 3DGUT-träning (LichtFeld Studio eller Spirula Studio)
* **Källkod:** [`_enable_gut_for_fisheye()`](file:///d:/OptiGSRigg/lichtfeld_plugin/OptiGSRigg/panels/main_panel.py#L456-L477) / [`run_lichtfeld_training()`](file:///d:/OptiGSRigg/src/process_3dgs_rig.py#L1038-L1079)
* **Vad som händer:**
  1. Projektmappen `processed\` laddas in i träningsmotorn med **`GUT = True` (`3DGUT`)** aktiverat (krävs för att rita fisheye-kameror utan undistortion).
  2. **Iteration 0 (CUDA PTX JIT-kompilering):** På RTX 2080 Ti (`SM 7.5`) kompilerar CUDA `3DGUT`-kärnorna på första bildrutan (tar ~4 minuter).
  3. **Iteration 1 $\rightarrow$ 36 000 (~90 iter/s):** Punktmolnet växer till `1 500 000` Gaussians medan felvärdet **`Loss`** (skillnaden mellan 3D-modellen och dina 360 foton) minimeras ner mot `~0.02`.

---

## Idé för Fristående OptiGSRigg GUI (Utanför LichtFeld-panelen)

Genom att bygga ett **eget fristående Python-program (t.ex. med PySide6 / Qt eller CustomTkinter)** får vi flera stora fördelar jämfört med att ligga inlåsta som en RmlUI-flik inne i LichtFeld Studio:

1. **Fryser aldrig under CUDA-kompilering:**
   * När LichtFeld Studio kompilerar CUDA-kärnor i 4 minuter på Iteration 0 fryser LichtFelds eget fönster, men vårt fristående kontrollprogram förblir **100 % responsivt** och kan visa en tydlig statusrullning: *"Kompilerar 3DGUT CUDA-kärnor på RTX 2080 Ti (tar ca 4 min på första rutan)..."*.
2. **Realtidsövervakning av LichtFeld via MCP (`http://127.0.0.1:45677/mcp`):**
   * Vårt fristående GUI kan prata direkt med LichtFeld Studio över dess lokala MCP-port i bakgrunden och hämta `trainer_state`, `iteration`, `loss` och `num_gaussians` varje sekund – med en egen graf över hur `Loss` sjunker!
3. **Visuell Ljudklapp-graf & Bildgranskning:**
   * Vi kan rita ut ljudvågorna för alla 3 kameror vid klappen och visa miniatyrbilder av `cam1_left`, `cam2_center` och `cam3_right` sida vid sida under hela bearbetningen.

