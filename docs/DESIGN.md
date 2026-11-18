# TrailStab – Design-Entwurf

Stand: 17.11.2026 · Prototyp (Python) + Plugin v0.2 (C++/OpenFX), erste Tests in Resolve Studio 21.1.2

## 1. Ziel

Ein OpenFX-Plugin für DaVinci Resolve Studio (macOS, M1), das iPhone-Clips im
Action Mode nachstabilisiert. Es soll ohne die typischen Fehler von Resolves
Stabilisierer auskommen: kein Kippen nach vorne, kein seitliches Umfallen bei
Schwenks.

### Feste Anforderung: keine Analyse-Dateien im fertigen Plugin

Die `.tstab`-Dateien sind nur ein Werkzeug für die Entwicklung, damit die Analyse
nicht bei jedem Build neu laufen muss. Das fertige Plugin funktioniert universell:
Effekt auf den Clip ziehen, analysieren, fertig. Keine Datei wird geschrieben oder
ausgewählt. Das Analyseergebnis (≈ 30 Byte pro Frame) liegt in einem versteckten
Parameter des Effekts und damit im Resolve-Projekt. Es wandert mit Projekt, Kopie,
Archiv und Render-Rechner mit.

## 2. Warum Resolve bei diesen Clips kippt (Analyse der drei Beispielclips)

| Befund | Messung |
|---|---|
| Objektiv | Ultraweitwinkel (Metadaten: „iPhone 14 Pro back camera 2.22mm f/2.2“) |
| Effektive Brennweite des Action-Mode-Bildes | f = 1,0185 × Bildbreite ≈ 21,6 mm KB-äquivalent, in allen 3 Clips identisch selbst-kalibriert |
| Restbewegung nach iPhone-Stabilisierung | fast nur 0,5–2 Hz (Schrittrhythmus): im Band 0,5–1 Hz Yaw ≈ 1,5–2,5° rms, Pitch ≈ 0,9–1,4°, Roll ≈ 0,3–1,1°; dazu 1–2 Hz je ≈ 0,4–1,8°. Über 2 Hz ≤ 0,12° (macht das iPhone schon gut) |
| Kamera-Neigung | im Mittel ≈ 15–17° nach unten |
| Scheinbare 2D-Bilddrehung im Schwenk IMG_1853, 10,3–12,8 s | bis 25° (Similarity-Modell), echtes Rollen nur ±3° |
| Translation, als Rotation fehlinterpretiert | +2,5 … +3,6 °/s Schein-Pitch beim Laufen über Boden |

**Ursache 1 – seitliches Kippen:** Wenn die Kamera nach unten geneigt ist und
schwenkt (Gieren um die Senkrechte), dreht sich der Bildinhalt in 2D tatsächlich.
Ein 2D-Stabilisierer (Similarity/Perspective) hält das für Rollen und „korrigiert“
es. Dadurch kippt der Horizont, bei schwachen Tracks auch um viele zehn Grad. Die
70° in IMG_1853 sind **nicht** im Original, sie entstehen in Resolve.

**Ursache 2 – Kippen nach vorne:** Beim Laufen wandert der nahe Boden im Bild nach
unten. Ein Perspektiv-/Homographie-Modell passt sich an die Bodenebene an und
erzeugt Trapezverzerrung und Nickbewegungen, die es real nicht gibt.

## 3. Algorithmus (im Prototyp umgesetzt und gemessen)

1. **Tracking:** Shi-Tomasi-Ecken im Raster (4×8 Zellen), pyramidaler
   Lucas-Kanade mit Vorwärts-Rückwärts-Prüfung, Analyse in 540 px Breite.
2. **Reine 3D-Rotation pro Frame:** Lochkameramodell mit bekannter Brennweite,
   RANSAC mit 2-Punkt-Lösungen, danach IRLS (Cauchy-Gewichte). Keine
   Homographie, also keine physikalisch unmöglichen Verzerrungen.
3. **Bias-Korrektur:** Essential-Matrix über 10-Frame-Basislinien trennt
   Translation von Rotation. Die Differenz zur verketteten Rotation wird als
   langsamer Bias geglättet und abgezogen: Pitch voll, Yaw/Roll gedämpft.
4. **Schwerkraft-Schätzung (EKF):** Zustand ist die Schwerkraftrichtung im
   Kamerasystem, propagiert mit den gemessenen Rotationen. Messung: „Das
   iPhone-Bild ist waagerecht“ (Action Mode nivelliert). Ein schwacher Prior auf
   die Neigung verhindert Drift. Bei schlechtem Tracking (z. B. Nebel) steigt das
   Prozessrauschen, geglättet, damit keine Sprünge entstehen. Vorwärts- plus
   Rückwärtsdurchlauf. Ergebnis: Kamerapfad (Yaw integriert, Pitch/Roll relativ
   zur Schwerkraft); der geschätzte Eingangs-Roll bleibt im Bereich ±3–5°.
5. **Virtuelle Kamera als quadratisches Programm (OSQP):**
   `min Σ‖S−E‖² + λ Σ‖Δ²S‖²` (+ Gewicht auf Roll→0 bei Horizont-Nivellierung),
   unter der harten Nebenbedingung, dass die vier Ecken des gezoomten
   Ausgabebildes im Eingabebild bleiben (linearisiert, 8 Zeilen pro Frame, mit
   exakter Nachprüfung). λ = (Glättung·fps)⁴. Ergebnis: die ruhigste Bahn, die das
   Crop-Budget nie verlässt. **Keine schwarzen Ränder, kein dynamischer Zoom.**
6. **Warp:** pro Frame Homographie `K_in · Rᵀ_E · R_S · K_out⁻¹`, also eine reine
   Rotation. Bikubisch, in Float, damit HDR/HLG und Log unverändert bleiben.

### Messergebnisse (unabhängig nachgemessen: Ausgabe neu getrackt)

Hochfrequentes Rest-Wackeln (Winkelgeschwindigkeit > ~0,6 Hz), Reduktion gegenüber Original:

| Clip | Zoom 1,15 | Zoom 1,25 | Zoom 1,4 |
|---|---|---|---|
| IMG_1852 | 30 % | 51 % | 67 % |
| IMG_1853 | 50 % | 64 % | – |
| IMG_1854 | 40 % | 57 % | – |

Die Messgröße unterschätzt die sichtbare Verbesserung etwas: Ein Teil des
gemessenen Rests ist das Auf-und-ab des Läufers (Translation, nahe Bodenparallaxe),
das keine Rotationsstabilisierung entfernen kann. Das Crop-Budget ist der
entscheidende Hebel. Der Schrittrhythmus braucht etwa ±4° Spielraum, also Zoom ≈ 1,25.

## 4. Plugin-Architektur

### Parameter (sichtbar in Resolve)

| Parameter | Default | Wirkung |
|---|---|---|
| Stabilisierung aktiv | an | Bypass |
| Analyse-Datei | – | `.tstab` (v0; später ersetzt durch „Analysieren“-Button) |
| Glättung (s) | 0,6 | Zeitkonstante; größer = ruhiger, braucht mehr Zoom |
| Zoom (Crop-Budget) | 1,25 | fester Zoom; der Pfad bleibt garantiert innerhalb |
| **Horizont nivellieren** | an | Checkbox: Roll der virtuellen Kamera → waagerecht |
| Zeitversatz (s) | 0 | Notbehelf für Synchronisation, bis das Zeit-Mapping geklärt ist |
| Status | – | Frames, Solver-Status, maximale Korrektur |

Glättung, Zoom und Horizont werden **live im Plugin** neu gelöst (C++-Solver,
≈ 0,6 s für 1100 Frames auf einer 2-Kern-VM, auf dem M1 schneller). Die teure
Analyse läuft nur einmal.

### Phasen

| Phase | Inhalt | Stand |
|---|---|---|
| 0 | **Probe-Plugin**: klärt Frame-Zugriff, Zeit-Mapping, Metal, Bildgröße in Resolve | fertig, auf dem Mac zu testen |
| 1 | **TrailStab v0/v0.2**: Analyse per Python-Tool → `.tstab`; Plugin löst den Pfad live und warpt auf der CPU | läuft in Resolve; v0.2 behebt Pan/Pillarbox-Lage und Frame-Zuordnung |
| 2 | Analyse **im Plugin** (C++), Button „Analysieren“ | Entscheidung nach Probe-Ergebnis (siehe unten) |
| 3 | **Metal**-Renderpfad (Warp als Compute-Kernel) | danach |
| 4 | Feinschliff: Rolling-Shutter-Rest, Auto-Zoom-Vorschlag, mehrere Clips, Presets für 24/30/60 fps | später |

### Analyse im Plugin – zwei Varianten

Boris FX dokumentiert, dass Resolve Plugins keinen Zugriff auf Frames zu weit
auseinanderliegenden Zeiten gibt. Gleichzeitig nennt das Resolve-Handbuch
„multi-frame random access“ für Studio. Das klärt das Probe-Plugin.

* **A – Frames von Resolve holen** (wenn die Probe es erlaubt): Der Button holt
  alle Frames des Clips über `clipGetImage`. Vorteil: Trims, Speed-Änderungen und
  Input-Sizing stimmen automatisch, weil analysiert wird, was Resolve liefert.
  Fallback A': Analyse während einmaligem Abspielen (benachbarte Frames sind
  immer erlaubt).
* **B – Datei direkt dekodieren:** Dateipfad wählen, Dekodierung per
  AVFoundation/VideoToolbox (Hardware-HEVC auf dem M1, sehr schnell). Die
  Zuordnung Zeit → Quell-Frame erfolgt über Bildvergleich des aktuellen
  Resolve-Frames mit den dekodierten Frames, das ist robust gegen Trims.
  Nebeneffekt: Zugriff auf die Apple-Metadaten (Brennweite, Objektiv).

Tracking in C++: eigener KLT und Rotations-Solver (klein, ohne Abhängigkeiten,
gegen den Prototyp testbar). Für die Essential-Matrix: entweder OpenCV statisch
linken oder einen 5-Punkt-Solver portieren. Entscheidung in Phase 2.

### Zeit, Auflösung, Farbe

* **Zeit (ab v0.3.2):** Resolve zeigt zur Zeit t das Quellbild mit dem nächstgelegenen
  Zeitstempel, bei Gleichstand das spätere (gemessen an IMG_1853/1854). iPhone-Dateien laufen
  mit 59,97 statt 60 fps; sobald die Drift ½ Frame überschreitet (IMG_1854 ab Frame ≈ 1194),
  wiederholt Resolve ein Bild. v0.2/v0.3.1 rechneten mit dem festen Raster und lagen ab dort
  einen Frame daneben (Korrektur auf das falsche Bild → doppelte Unruhe). `frameIndexAt`
  folgt jetzt Resolves Regel; Selbsttest mit synthetischer Drift bis 0,7 Frame.

## 5. Offene Punkte und Risiken

1. Frame-Zugriff in Resolve für die Analyse im Plugin (Probe) – Voraussetzung für
   „keine Analyse-Dateien“.
2. Zeit-Mapping bei 24/30 fps-Timelines, Trim und Speed (Log von v0.2 bzw. Probe).
   60 → 60 fps ist verifiziert.
3. Horizontgenauigkeit: Die Schwerkraftschätzung stützt sich auf den
   iPhone-Horizont. Fehler bis ≈ 3° sind möglich, vor allem in Nebel/Boden-Szenen.
   Option: Bild-basierte Horizont-/Vertikalenprüfung.
4. Die Brennweite ist für iPhone 14 Pro Action Mode (Ultraweit) kalibriert. Für
   andere Modi ist eine Auto-Kalibrierung im Analyseschritt nötig (im Prototyp
   vorhanden, `calib_f2`).
5. Translation (Auf und Ab beim Laufen) bleibt sichtbar; das ist mit reiner
   Rotation prinzipiell so.

## 6. Erste Tests in Resolve (17.11.2026, IMG_1853, 60 fps-Timeline)

Gesteuert über den DaVinci-Resolve-MCP-Server, Renders H.264, Auswertung durch
erneutes Tracking der gerenderten Bilder.

* **Rechnung exakt:** Resolves Ausgabe von TrailStab stimmt pixelgenau (< 0,1 px) mit
  dem C++-Kern überein, zum richtigen Frame.
* **Resolve Perspective (Crop Ratio 0,50, Smooth 0,891, Strength 1,0):** zoomt ≈ 1,98×,
  bei Frame 747→748 springt ein einzelnes Bild um ≈ 265 px und 3,5° zur Seite, dazu
  alle ≈ 10 Frames kleinere Ausreißer (708, 718, 728, 758). Mit Crop 0,5 / Smooth 0,25 /
  Strength 0,8 kippte es diesmal nicht.

Hochfrequente Unruhe über den ganzen Clip (Ausgabe neu getrackt, > 0,6 Hz):

| | Original | TrailStab (Zoom 1,25) | Resolve (Zoom ≈ 1,98) |
|---|---|---|---|
| Drehung (°/s) | 8,6 | 3,5 (−59 %)¹ | 29,7 (+244 %) |
| Bildbewegung oben, Hintergrund (px/Frame)² | 2,7 | 1,2 (−56 %) | 6,5 (+136 %) |
| Bildbewegung unten, naher Boden (px/Frame)² | 4,6 | 2,8 (−39 %) | 7,5 (+63 %) |

¹ mit korrigierter Frame-Zuordnung (v0.2); v0 in Resolve gerendert: −52 %.
² auf den Maßstab des Originals umgerechnet (geteilt durch den Zoom).

Beim nahen Boden bleibt prinzipbedingt mehr Rest: Das ist Translation (Auf und Ab beim
Laufen), die eine reine Rotation nicht entfernen kann. Eine erste 2D-Messung über das
ganze Bild hatte TrailStab fälschlich als wirkungslos ausgewiesen; die Messung wurde
durch getrennte Regionen und echte 3D-Rotationsschätzung ersetzt.

Render-Zeit (1093 Frames, 3240×1920, M1): Original 80 s, Resolve-Stabilisierung 77 s,
TrailStab v0 (CPU) 63 s. Der Export wird vom Dekodieren/Kodieren bestimmt, nicht vom
Plugin.

## 7. Dateien

```
CMakeLists.txt             Build (holt OpenFX + OSQP automatisch)
probe/ProbePlugin.cpp      Diagnose-Plugin
ofx/TrailStabPlugin.cpp    Stabilisierer v0.2 (Bildlage-Erkennung, Frame-Raster, Log)
core/                      portabler Kern: Pfad-Solver (OSQP), Warp, .tstab-Format
tools/trailstab_analyze.py Analyse (Referenzimplementierung) + Vorschau
tools/selftest.cpp         C++-Kern gegen Python-Referenz
prototype/                 Forschungs-Skripte der Nacht (Kalibrierung, Auswertung)
```

## Vertikalanteil (v0.3, 17.11.2026)

Messung am ganzen Clip IMG_1853: In den Frames 1–600 erklärt die 3D-Drehung nur 32 % der senkrechten
Bildbewegung (Laufschritt, Nahbereich), danach 72 %. Ein reines Drehmodell kann das nicht entfernen.

- Analyse: pro Frame der Median der senkrechten Bewegung kräftiger Eckpunkte (Pfad, Boden, Bäume), die die
  fertige Orientierungsspur nicht erklärt, aufsummiert als Winkel `V` (rad). Sechste, optionale Spalte der `.tstab`.
- Solver: nur der schnelle Anteil von `V` (Hochpass 1 s, Gauß mit ungerader Spiegelung an den Rändern) wird zur
  Nickachse der Eingangskamera addiert („scheinbares Nicken"). Langsame Anteile (Vorwärtsfluss) bleiben draußen,
  sonst wüchse der Winkel auf hunderte Grad. Der QP glättet das wie jede andere Drehung, der Crop-Korridor gilt weiter.
- Version: Der Inspector zeigt unter dem Plugin „Version" mit Build-Zeit (kTrailStabVersion in TrailStabPlugin.cpp bei jedem Dev-Build erhöhen).
- Plugin: Regler „Vertikale Oszillation" 0..1 (Standard 1). 0 = altes Verhalten. Alte `.tstab` ohne Spalte laufen weiter.
- Grenze: Das ist eine Näherung (Verschiebung wird als Drehung ausgeglichen), keine echte Parallaxen-Korrektur.
- Vorhersage am Clip (Modell am alten Render geprüft, Korrelation 0,94): senkrechte Rest-Unruhe Frames 1–600 von
  3,99 auf ca. 1,05 px/Frame. Die Vorhersage nutzt dieselben Merkmale wie die Messung, deshalb zählt erst der echte Render.

### Befund 17.11. nachmittags: Vertikalanteil verschlechtert das Bild, alte Messung war zirkulär
- Jens sah v0.3 (Vertikal 1.0) deutlich unruhiger als v0.2. Die Zonen-Messung (`tools/eval/zone_motion.py`,
  festes Punktraster, Median je Bilddrittel) bestätigt das: senkrechte Unruhe oben (Ferne) 0,78 → 4,49 px/Frame,
  Sprünge < 0,05 s 0,34 → 2,63. Bei 0,2 oben 0,78 → 1,24 (auch schlechter), unten 3,00 → 2,85 (kaum besser).
- Die vorher gemeldete Verbesserung war ein Messfehler: Ganzbild-Median kräftiger Eckpunkte = dieselbe Größe,
  aus der V berechnet wird; die Korrektur löscht genau diese Statistik aus. Diese Metrik nicht mehr verwenden.
- Ursache: V ist überwiegend Vorwärtsfluss des Bodens (Nahbereich wandert beim Laufen nach unten, Stärke
  schwankt mit Tempo und Gelände), keine Kamerawippe. Ein Dreh-Warp gleicht ihn aus, indem er die Ferne bewegt.
  Verschiebung erzeugt tiefenabhängige Parallaxe – ein globaler Warp kann nur eine Tiefenebene ruhigstellen,
  und das Auge hält sich an die Ferne.
- Hauptmaß ab jetzt: senkrechte und waagerechte Unruhe in der oberen Zone (Ferne) plus Sprung-Maß.

### Namen und Symbol (14:00)
- Anzeigename „Trail Stability" (Plugin und Gruppe), Probe „Trail Stability Probe". Interne Kennungen
  (`de.trailstab.Stabilizer`, `de.trailstab.Probe`) und Bundle-Namen (`TrailStab.ofx.bundle`) bleiben, damit
  bestehende Effekte in Projekten weiter gefunden werden und keine doppelten Bundles entstehen.
- Symbol: `Stability.png` im Projektordner wird beim Bauen als `Contents/Resources/<Kennung>.png` ins Bundle kopiert.

## Probe-Ergebnisse (Resolve Studio 21.1.2, Mac, 17.11.2026)
Quelle: TrailStabProbe.log, getestet auf 60 fps (IMG_1854), Timeline A 24 fps (IMG_1853, 60-fps-Clip), Clip mit 80 % Speed, „getrimmter" Clip.
- Host: `temporalClipAccess=1`, `metal=1`, opencl/cuda 0, `tiles=0`, `multiRes=0`, `sequentialRender=0`, ofxAPI 1.4.
  Resolve ruft render in beliebiger Reihenfolge auf (Zeitsprünge im Log) und kennt keine Kacheln: Analyse nicht über den Render-Pfad, sondern über Button.
- Frames von anderen Zeitpunkten holen: Button 8/8 OK (t = 0, 10, 100, Ende, außerhalb des Bereichs), im Render-Aufruf ebenfalls
  (offset ±1 aus Cache 0 ms, +30 / +200 Frames 80–480 ms). 60 aufeinanderfolgende Frames: 30–32 ms/Frame, unabhängig von der Rate.
  Hochrechnung: 1413 Frames ≈ 45 s reines Holen.
- Das geholte Bild ist die ganze Timeline-Fläche (3240×1920, Float): der Clip liegt darin nach Resolves Fit/Pan. Bildausschnitt-Erkennung bleibt nötig.
- Zeitachse: `srcFps`/`effectFps` = Timeline-Rate (24 fps bei 60-fps-Quelle), Bereich 0..436 (437 Frames). Mit 80 % Speed: 0..546 (437/0,8).
  Retiming ist also in Zeit und Bild schon eingerechnet; `srcUnmapped` ist 0..0, die Originaldauer/-rate ist nicht abfragbar.
- Offen: Trim am Clipanfang. Der „getrimmte" Test lieferte denselben Bereich wie der ungetrimmte (0..436); ob t=0 der Quellanfang oder der Schnittanfang ist, ist nicht geklärt.
  Test: Abspielkopf auf das erste Bild des getrimmten Clips, Button drücken, t ablesen.
- Auch aufgerufen: Thumbnail-Renders mit renderScale 0,128 (Fenster 416×234). Alle Größen im Plugin aus dem Bild ableiten, nicht aus Konstanten.


## Sprünge (v0.3.2, 17.11.2026)
Werkzeuge: `tools/eval/panel_measure.py` (Original-Panel, nur texturierte Punkte),
`predict_eval.py` (Ausgabe = Original − Warp-Bewegung; die Warp-Kette stimmt mit Resolve auf
< 0,05 px überein, `warp_check.py`). Hauptzone „mitte“; „oben“ enthält Himmel/Wolken.
Ergebnis der Ursachensuche:
1. **Frame-Zuordnung** (siehe Zeit): IMG_1854 Frames 1210–1412, Sprung x/y mitte:
   Original 0,29/0,19, v0.3.1 0,45/0,26, v0.3.2 0,09/0,15.
2. **Nick- und Rollrauschen:** die Bild-zu-Bild-Werte von Pitch/Roll der Analyse sind zu
   großen Teilen Schätzfehler (Rotation schluckt die Parallaxe des nahen Bodens; Kohärenz
   mit der Bildbewegung bricht bei 2–4 Hz, der Schrittfrequenz, ein). Der Solver glättet
   gemessenen Pitch/Roll jetzt mit σ = 1 Frame (`denoiseFrames`), Yaw bleibt exakt.
   IMG_1853 mitte Sprung y: Original 0,15, v0.3.1 0,27, v0.3.2 0,14.
3. Virtueller Pfad selbst ist glatt (keine Knicke), Zeitversatz bei 1852/1853 null.
Verworfen: Essential-Matrix pro Framepaar (deutlich schlechter), Analyse in 1080 px
(kleiner Gewinn, 4× Zeit; für die Analyse im Plugin wieder prüfen).
Offen: IMG_1854 hat große horizontale Sprünge im Original (Sprung x 0,6–1,0), die die
Analyse nur teilweise erfasst; wahrscheinlich Rolling Shutter/Bewegungsunschärfe in den
technischen Abschnitten.
Vertikale Oszillation: Standard jetzt 0 (experimentell).
Bestätigt im Resolve-Render (17.11.2026, B_185x_60fps_v032.mp4): Plugin wendet exakt die
berechnete Warp zum richtigen Frame an (IMG_1854 Frames 1150–1400: 0,16 px, Versatz 0).
Gemessen im TrailStab-Panel (texturierte Punkte, auf Originalmaßstab), mitte, Sprung x/y:
1852 Original 0,23/0,11 · v0.3.1 0,06/0,16 · v0.3.2 0,06/0,10;
1853 0,26/0,15 · 0,18/0,26 · 0,18/0,14;
1854 (ohne Doppelbild 1185–1205) 0,27/0,17 · 0,20/0,19 · 0,11/0,15;
1854 1210–1412 0,29/0,19 · 0,45/0,25 · 0,10/0,14.
Resolve Perspective (0,5/0,313/0,115), nicht zoom-normalisiert: 1,29/0,71 · 0,71/0,30 · 2,99/1,25.

## Analyse im Plugin (v0.4.x, 17.11.2026)
`core/analysis.cpp`: C++-Port ohne OpenCV (Shi-Tomasi je Zelle, pyramidales LK wie OpenCV –
Punkte/Tracking identisch auf 0,001 px –, 2-Punkt-RANSAC + IRLS, Ketten über 10 Frames mit
robuster Pose (R + Translationsrichtung, LM auf Epipolarfehler, startet bei integrierter Rotation;
ersetzt die Essential-Matrix, gleiche Bias-Werte wie Python), Schwerkraft-EKF, Fusion).
Streaming: nur Vorgängerbild im Speicher. Speicherung im versteckten String-Parameter
(`TSA1`-Header + base64, 1e-5 rad, ≈ 9 Byte/Frame), Index = Effektzeit − t0 (folgt Resolves
Frame-Zuordnung inkl. Doppelbild automatisch). Bereich per Set Start/End (+1 s Rand).
Resolve M1: 1414 Frames in 88 s (62 ms/Frame inkl. Holen), Kerne ~50 % ausgelastet.
Ergebnis (Render v0.4.3, mitte Sprung x/y): 1852 0,06/0,11 · 1853 0,17/0,15 · 1854 0,11/0,15 –
gleichwertig zu v0.3.2 mit Python-Analyse.
Offen: Laden und Rechnen überlappen; nur Performance-Kerne; Layout-Hinweis
(OfxParamPropLayoutHint, Natron-Erweiterung) wird von Resolve ignoriert.

### Effizienz und Laufzeit v0.5.0 (17.11. abends)
- Messung v0.4.5 (3240er Vergleichs-Timeline, 1414 Frames): Full 90,7 s (Laden 29,6 ms, Graubild ~9,8, Tracking-Teile
  ~24,6 ms/Frame), Half 65,0 s, Quarter 52,5 s. Half = gleiche Qualität wie Full (Ausschnitt 540 px = Analysebreite).
- Resolve setzt Playback Resolution NICHT über renderScale um (scale=1, aber kleinere Fläche, z. B. 1620×960).
  Volle Auflösung lässt sich vom Plugin nicht erzwingen. Regel: Bildausschnitt < Analysebreite -> Analyse verweigert
  („Playback resolution too low for analysis (N px). Set Playback Resolution to a higher setting"). Nie hochrechnen.
- `effectDur` und Clip-Range zeigen immer den ganzen Quellclip, auch bei gekürztem Timeline-Clip; Effektzeit =
  Quellframe. „Set Cut Clip" ist über OFX nicht machbar -> Set Playhead as Start/End nutzen.
- v0.5.0: gemeinsamer Thread-Pool (`core/threadpool.*`) mit Performance-Kern-Zahl (hw.perflevel0.logicalcpu, M1: 4),
  QoS user-initiated; Analyse als Pipeline (Hauptthread lädt + Graubild parallel, Arbeitsthread trackt, Warteschlange 3);
  Laden nur des Bildausschnitts (Fallback auf ganze Frames); Developer-Regler „Analysis Width" (360–720, Std. 540);
  Render-Zeitmessung im Log alle 300 Frames. Pixel überspringen im Graubild bewusst NICHT (Qualität soll von der
  Auflösung unabhängig bleiben).
- Wiedergabe: 1080×1920-Timeline Full 44 fps, 3240er Timeline Full 20,5 fps (CPU-Warp + GPU<->CPU-Kopie). Lösung: Metal (v0.6).
- Ergebnis v0.5.0 (1414 Frames, B_IMG_1854): Full 49,1 s (vorher 90,7), Half 44,8 s (vorher 65,0). Ausschnitt-Laden
  bringt nichts (Resolve rendert trotzdem alles). Full: Grenze = Laden 29,8 ms; Half: Grenze = Tracking-Thread 31,5 ms.
- v0.5.1: Bild-zu-Bild-Punkte und alle Chain-Punkte in EINEM Tracking-Durchlauf (vorher 2–3 kleine nacheinander),
  Pyramide (pyrDown, Scharr) zeilenweise parallel. Ergebnis identisch (Selbsttest unverändert 0,204 px).

### Metal-Renderpfad v0.6.0
- Recherche: OFX 1.5 Metal-Buffer-Render (`kOfxImageEffectPropMetalRenderSupported`, pro Render `MetalEnabled` +
  `MetalCommandQueue`); Bilder kommen als `id<MTLBuffer>` mit Bounds + rowBytes (wie Gyroflow-OFX es nutzt).
  Spec: Arbeit auf Resolves Queue legen, am Ende NICHT warten. Instanz darf die Descriptor-Angabe überschreiben
  (Developer-Schalter „GPU Rendering (Metal)“; ob Resolve dem folgt, zeigt das Log: `render#… METAL/CPU`).
- Umsetzung: `ofx/metal_kernels.h` (MSL-Quelltext, zur Laufzeit kompiliert, pro MTLDevice gecacht),
  `ofx/metal_render.mm` (ARC). Bildsuche auf der GPU (Spalten/Zeilen-Flags wie findContentRect, kleines
  Shared-Buffer-Ergebnis, einziges Warten), Platzierung + Homographie auf der CPU wie bisher, Warp (bikubisch,
  Catmull-Rom) auf der GPU ohne Warten. Homographie relativ zur Bildmitte -> float reicht (< 2e-4 Abweichung).
- Prüfung ohne Mac: `tools/run_metal_emu_test.sh` kompiliert den MSL-Text als C++ (tools/msl_shim.h) und vergleicht
  mit CPU-Warp und Bildsuche, inkl. 3240×1920. Ergebnis: Bildsuche identisch, Warp max. 1,7e-4.
- Offen: Zeilenreihenfolge der Metal-Buffer in Resolve (angenommen wie CPU: Zeile 0 = unten) -> am Mac prüfen
  (CPU/GPU-Vergleich); falls Bildsuche-Warten bremst: Platzierung vom Vorframe übernehmen.
- Ergebnis v0.6.0 am Mac: Metal aktiv (Log METAL), Laden 0,0 ms, Warp-Encode 0,0 ms, aber Bildsuche 16,7 ms = Warten
  auf Resolves GPU-Arbeit am Frame. v0.6.1: Bildsuche läuft im selben Command Buffer wie der Warp, Ergebnis per
  Completion-Handler in einen Cache (pro Bounds + Skalierung) -> gilt ab dem nächsten Frame. Gewartet wird nur noch
  einmal pro neuer Bildgröße/Skalierung. Folge: Nach einer Inspector-Änderung ist genau ein Frame noch alt platziert.
- Prüfung v0.6 am Mac: Render B_1854_60fps_v060_metal.mp4 (Metal) gegen v043 (CPU): Bewegung im TrailStab-Panel
  identisch (Korrelation 1,000, Sprung/Wackeln gleich), PSNR 38,7 dB (H.264-Rauschen allein ~41 dB im unveränderten
  Original-Panel) -> Zeilenreihenfolge stimmt. v0.6.1: Plugin-Zeit pro Frame 0,2 ms (vorher CPU ~49 ms, v0.6.0 17 ms).
  Wiedergabe 3240er-Timeline Full 45–49 fps (vorher 20,5); Grenze jetzt GPU 100 % + Resolve selbst.
- Resolve ignoriert die Instanz-Überschreibung von MetalRenderSupported (Log: „requested: off“, trotzdem METAL).
  v0.6.2: Schalter entfernt, Metal ist der Renderpfad. Der CPU-Pfad bleibt im Code für Hosts ohne Metal
  (Analyse läuft ohnehin auf der CPU).

### Schritt Effizienz und Laufzeit – Ergebnis
- Analyse Full 90,7 s -> 45,8 s (1414 Frames, 3240er Timeline), Half 65,0 -> 38,6 s.
- Wiedergabe Full: 1080er Timeline 44 -> 60+ fps, 3240er Vergleichs-Timeline 20,5 -> 45–49 fps (GPU 100 %).
- Plugin-Zeit beim Abspielen 0,1–0,2 ms/Frame.

## v0.9.0 – Aufräumen für 1.0

- Entfernt: `.tstab`-Fallback im Plugin (Parameter `trackFile`, `reload`, `timeOffset`), `vertical`
  (Solver rechnet im Plugin immer mit 0) und der alte versteckte Parameter `smoothness` (v0.1–v0.4.2).
  `core/sidecar.*` bleibt für die Werkzeuge (analyze_cli, dump_*), das Plugin nutzt nur noch
  `encodeTrack`/`decodeTrack`.
- Identität: Bundle-ID `de.trailstab.Stabilizer` bleibt endgültig. Version kommt aus
  `project(TrailStab VERSION x.y.z)` in CMakeLists.txt (Info.plist und Versionslabel). Die OFX-Plugin-Version
  (getPluginIDs, 0.1) wird erst beim Release auf 1.0 gesetzt.
- Analyseformat: Tag `TSA1`; ein inkompatibles Format muss `TSA2` heißen, ältere Plugins melden dann
  "stored by a different plugin version".
- Fehlermeldungen: Solver-Fehler (Status ≠ 1/2/7) -> unstabilisiert durchreichen + Hinweis im Status;
  Clip geändert -> "not stabilised"; Analysefehler behalten die alte Analyse und sagen das.
- Developer-Gruppe (eingeklappt): Analysis Width, Precise Solver, Delete Analysis.
- Log: `~/Library/Logs/TrailStab.log` nur Kernereignisse (Analyse Start/Ende/Abbruch/Fehler, Metal-Fehler),
  über 1 MB -> `TrailStab.old.log`. Detail-Log (Render-Zeilen, Timing, Analyse-Split) nur, wenn
  `~/Library/Logs/TrailStab.debug` existiert (wird beim Laden des Plugins geprüft).

## v0.9.1/v0.9.2 – Speed-Änderung erkennen
- Resolve meldet eine Speed-Änderung nicht über changedClip, fragt aber getClipPreferences neu ab.
  Dort vergleicht das Plugin Bereich und fps mit dem letzten Stand und schreibt Status/Interval neu
  (Resolve erlaubt das Setzen dort). Render reicht bei unpassender Analyse unstabilisiert durch und
  loggt jeden Wechsel. Getestet 18.11.2026: 100 % -> 80 % -> 100 % zeigt den Hinweis und nimmt ihn wieder weg.
- Analyse lässt sich nicht abbrechen: Resolve friert während instanceChanged ein und bedient den
  Progress-Dialog nicht. Bleibt als Einschränkung (README).

## v1.0.0 (18.11.2026)
- Testrunde in Resolve Studio 21.1.2: 60-fps- und 30-fps-Timeline, zwei verschiedene Clips in einer
  Timeline, Speed 80 % ↔ 100 %, Speichern/Öffnen. OFX-Plugin-Version 1.0 (vorher 0.1): alte
  Testprojekte müssen den Effekt ggf. neu anlegen.
