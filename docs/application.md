# LilyGo T-Watch Ultra – UI & Funktionalitäts-Spezifikation

Dieses Dokument beschreibt das User Interface und die grundlegende Funktionalität
der Firmware/Anwendung für die LilyGo T-Watch Ultra. Es dient als lebendes
Referenzdokument und wird um weitere Screens ergänzt, sobald diese definiert sind.

---

## 1. Designprinzipien

- **Schwarzer Hintergrund überall.** Da das Display ein OLED-Panel ist, verbrauchen
  schwarze Pixel keinen (bzw. kaum) Strom. Alle Screens nutzen `#000000` als
  Basis-Hintergrundfarbe. Weiße/farbige Elemente werden so klein und sparsam wie
  möglich gehalten.
- **Energieeffizienz hat Priorität** vor visueller Verspieltheit:
  - Keine großflächigen hellen Flächen, keine unnötigen Animationen.
  - Refresh-Rate der Sekundenanzeige minimal (nur die betroffenen Digits neu zeichnen,
    kein Full-Screen-Redraw).
  - Peripherie (GPS, LoRa, SD-Karte, etc.) wird nur bei Bedarf aktiv geschaltet;
    Zustand wird über Statusicons kommuniziert, nicht über Text.
  - **Display-Timeout: komplettes Abschalten** nach Inaktivität (Wert TBD, z. B.
    10–15 s, in Einstellungen konfigurierbar). Kein Always-on-Modus – das Display
    geht vollständig aus, um maximal Strom zu sparen. Aufwecken erfolgt über
    **einen der folgenden drei Trigger**: Power-Taste, Boot-Taste oder
    Doppel-Tap auf das Display.
- **Klarheit vor Dichte.** Wenige, große, gut lesbare Elemente statt vieler kleiner.
- **Konsistente Statusleiste** über allen Screens hinweg (siehe Abschnitt 3), damit
  der Nutzer den Systemzustand jederzeit unabhängig vom aktuellen Screen sieht.
- **Abgerundete Ecken beachten.** Das 410×502-Panel hat physisch abgerundete Ecken
  (2.06" CO5300 AMOLED) – UI-Elemente, die zu nah an einer Ecke platziert werden,
  werden dort abgeschnitten bzw. vom Gehäuse verdeckt. Ein Safe-Area-Overlay liegt
  bereits im Repo (`assets/ui/safe_area_transparent.png`, 410×502, transparent =
  sicherer Bereich), wird aber aktuell in keinem Screen ausgewertet. Aus dem Overlay
  gemessen: an den geraden Kanten (oben/unten/links/rechts) ca. **16 px** Randabstand,
  in den vier Ecken ein deutlich größerer abgerundeter Ausschnitt (Radius ca.
  **90–100 px**). Gilt für **alle** Screens, insbesondere für die Statusleiste
  (Abschnitt 3; seit der Positionierungs-Korrektur bei y=54 mit ≥40px
  Randabstand, davor bei y=4 ohne Rücksicht auf diesen Bereich platziert) und
  für jedes andere eckennahe Element – nicht nur für die zentrierten
  Hauptelemente wie die Uhrzeit.
- **Vor dem Flashen im Simulator verifizieren.** `sim/` (LVGL-PC-Simulator,
  siehe `sim/CMakeLists.txt`) spiegelt die Screens auf dem Host, ohne
  Hardware zu benötigen. Bei jeder sichtbaren UI-Änderung: zuerst `cmake
  --build` + den Simulator starten, das Ergebnis prüfen (Screenshot reicht,
  wenn keine echte Interaktion nötig ist) und die Freigabe abwarten, bevor
  auf das Gerät geflasht wird. Der Simulator ist kein automatischer Spiegel
  von `main/lvgl_app.c` – neue Screens/Elemente müssen von Hand nachgezogen
  werden (siehe Kommentar am Kopf jeder `sim/*.c`-Datei).

---

## 2. Screen-Übersicht

| # | Screen             | Zweck                                                | Status |
|---|--------------------|--------------------------------------------------------|--------|
| 1 | Hauptscreen (Zeit) | Lokalzeit groß, UTC klein, Zeitzone, Statusleiste       | spezifiziert (s. Abschnitt 4) |
| 2 | GPS/Karte          | Position, Fix-Qualität, ggf. einfache Kartendarstellung | Entwurf (s. Abschnitt 6) |
| 3 | LoRa-Nachrichten   | Empfangene/gesendete Nachrichten, Verbindungsstatus     | Entwurf (s. Abschnitt 7) |
| 4 | Alarme/Timer       | Anlegen, Anzeigen, Verwalten von Alarmen und Timern     | Entwurf (s. Abschnitt 8) |
| 5 | Einstellungen      | Systemkonfiguration (Zeitzone, Display, Peripherie, ...)| Entwurf (s. Abschnitt 9) |

Screens 1–4 bilden den horizontalen Navigationsring; Einstellungen (5) liegt
davon separat auf einer vertikalen Achse unterhalb des Hauptscreens (siehe
Abschnitt 4.3). Navigation zwischen allen Screens erfolgt ausschließlich per
**Swipe-Geste** – keine physischen Tasten/Krone für die Screen-Navigation.

---

## 3. Statusleiste (global, auf allen Screens sichtbar)

Am oberen Bildschirmrand wird eine Reihe kleiner Icons angezeigt, die den Zustand
verschiedener Peripheriegeräte/Subsysteme kommunizieren. Jedes Icon hat einen
festen Platz, damit sich der Nutzer die Position einprägen kann (kein Verschieben
bei Ein-/Ausblenden – stattdessen Graustufe für "inaktiv").

### 3.1 Farbcode-Konvention (allgemein)

| Farbe  | Bedeutung                                   |
|--------|----------------------------------------------|
| Grau   | Peripherie ausgeschaltet / nicht vorhanden   |
| Orange | Übergangszustand / wird initialisiert / Warnung |
| Grün   | Peripherie aktiv und im gewünschten Zielzustand |
| Rot    | Peripherie aktiv, aber in einem besonderen/kritischen Zustand (z. B. gemountet, Fehler) |

> Hinweis: Rot ist hier bewusst nicht ausschließlich als "Fehler" definiert,
> sondern kontextabhängig (siehe SD-Karte unten). Das sollte pro Icon einheitlich
> dokumentiert und ggf. vereinheitlicht werden, um Verwirrung zu vermeiden.

### 3.2 Icon-Definitionen

| Icon        | Zustand       | Farbe  | Bedeutung                                  |
|-------------|---------------|--------|---------------------------------------------|
| SD-Karte    | not inserted  | Grau   | Keine SD-Karte gesteckt                     |
| SD-Karte    | inserted      | Grün   | SD-Karte gesteckt, im Ruhezustand (nicht gemountet) – der Normalzustand |
| SD-Karte    | mounted       | Rot    | SD-Karte kurzzeitig gemountet, aktiver Schreib-/Lesezugriff läuft gerade |
| GPS         | off           | Grau   | GPS-Modul ausgeschaltet                     |
| GPS         | acquiring     | Orange | GPS eingeschaltet, sucht nach Fix           |
| GPS         | 3D lock       | Grün   | GPS hat gültigen 3D-Fix                     |
| GPX-Tracking| inaktiv       | Grau   | Track-Aufzeichnung nicht aktiv              |
| GPX-Tracking| aktiv         | Grün   | Track-Aufzeichnung läuft, wird auf SD-Karte geloggt |
| LoRa        | off           | Grau   | LoRa-Modul ausgeschaltet                    |
| LoRa        | powered/rx    | Grün   | LoRa aktiv und empfangsbereit               |
| Bluetooth   | off           | Grau   | Bluetooth ausgeschaltet                     |
| Bluetooth   | on, kein Gerät| Orange | Bluetooth aktiv, aber nicht verbunden       |
| Bluetooth   | verbunden     | Grün   | Bluetooth aktiv und mit Gerät verbunden     |
| WLAN        | off           | Grau   | WLAN ausgeschaltet                          |
| WLAN        | verbindet     | Orange | WLAN sucht/verbindet mit Netzwerk           |
| WLAN        | verbunden     | Grün   | WLAN aktiv und verbunden                    |
| Akkustand   | –             | –      | **Nur grafisches Füllstand-Symbol**, keine Prozentzahl in der Statusleiste (per explizitem Feedback entfernt – die genaue Zahl bekommt einen anderen Platz auf dem Hauptscreen); Farbe rot bei kritischem Stand (≤15 %) |
| Ladezustand | nicht ladend  | – (Icon ausgeblendet oder grau) | Kein Ladekabel angeschlossen |
| Ladezustand | ladend        | Grün (z. B. Blitz-Symbol) | Ladekabel angeschlossen, Akku lädt |

> Bluetooth/WLAN folgen dem gleichen Grau/Orange/Grün-Schema wie GPS/LoRa
> (aus/verbindet/aktiv). Akkustand und Ladezustand sind Sonderfälle, da sie
> primär eine Füllstandsanzeige bzw. einen Boolean (lädt/lädt nicht) statt eines
> dreistufigen Zustands darstellen – Details (Icon-Form, Prozentanzeige als
> Zahl oder rein grafisch) sind noch offen.

> **SD-Karte wird on-demand gemountet**, nicht für die gesamte Wachzeit: sie
> wird nur für die kurze Dauer eines tatsächlichen Schreib-/Lesevorgangs
> gemountet (Tages-/Aktivitäts-Log, GPX-Track-Punkt, Mesh-Textlog, Crash-Dump,
> Screenshot, Debug-Konsolen-Befehle) und direkt danach wieder ausgehängt –
> damit ist sie den Rest der Zeit vor Beschädigung durch einen unsauberen
> Stromausfall (z. B. Akku leer, Battery-Pull) geschützt. Der Rot-Zustand ist
> deshalb nur für Sekundenbruchteile sichtbar; der Normalzustand einer
> gesteckten Karte ist Grün.

### 3.3 Layout

- Icons werden **von links nach rechts in fester Reihenfolge** angeordnet, in
  zwei Zeilen gruppiert nach Funktion (festgelegt, nicht mehr TBD):
  - **Zeile 1 – alle Funkmodule** (RF): Bluetooth, WLAN, GNSS, LoRa/Meshtastic
    – ein Blick auf die obere Zeile beantwortet "sendet/empfängt gerade
    etwas".
  - **Zeile 2 – alles andere**: SD-Karte, GPX-Tracking (übernimmt jetzt das
    frühere GNSS-Pin-Symbol), Ladezustand, Akkustand.
  - GNSS wird mit einem kleinen Satelliten-Icon dargestellt, LoRa mit einem
    3-Knoten-Mesh-Netzwerk-Icon statt dem reinen "LoRa"-Text – funktional
    unverändert, nur die Darstellung. LVGLs eingebauter Symbol-Font
    (Montserrat/FontAwesome-Subset) enthält weder ein Satelliten- noch ein
    Mesh-Symbol, daher wurde dafür eine kleine Bild-Asset-Pipeline eingeführt
    (`main/screens/status_icons.h/.c`, 28×28 A8-Alpha-Bitmaps, per
    `lv_obj_set_style_image_recolor()` genauso eingefärbt wie die übrigen
    Text-Icons). Beide Icons stammen aus bereitgestellter Quellgrafik
    (`assets/ui/satellite.png`, `assets/ui/network.png` – transparente
    schwarze Strichzeichnungen, nicht das offizielle Meshtastic-Logo,
    Markenrechtsfrage bewusst umgangen); ein anderes Logo kann später
    eingesetzt werden, indem dieselbe Konvertierung auf eine neue PNG-Datei
    angewendet wird.
- Größe: klein genug, um nicht abzulenken, aber eindeutig erkennbar (Vorschlag:
  16×16 px bei aktueller Displayauflösung, anpassbar).
- Icons ohne aktiven/relevanten Zustand können optional komplett ausgeblendet
  statt grau dargestellt werden – **TBD, aktuell: immer sichtbar, nur eingefärbt**,
  damit die Position stabil bleibt und der Nutzer sich orientieren kann.

---

## 4. Hauptscreen (Zeitanzeige)

Der Hauptscreen ist der Standard-/Ruhescreen der Uhr und zeigt primär die Uhrzeit.

### 4.1 Layout (von oben nach unten)

1. **Statusleiste** (siehe Abschnitt 3) – oberster Bildschirmrand.
2. **Zeitzonen-Abkürzung** – klein, zentriert oder linksbündig über der lokalen Zeit
   (z. B. `CEST`, `CET` – **Kürzel, kein UTC-Offset**).
3. **Lokale Zeit** – sehr groß, zentrales Element des Screens, Format `HH:MM:SS`
   (inkl. Sekunden).
4. **UTC-Zeit** – mittlere Schriftgröße, darunter, Format `HH:MM` (ohne Sekunden).

### 4.2 Typografie & Farben

**Schriftart: durchgängig Monospace** (Cascadia Code) für
alle Ziffern-Anzeigen (Zeit, UTC, Countdown, Koordinaten) – verhindert
"Zittern"/Breitenänderungen beim Ziffernwechsel und wirkt technisch/klar,
passend zum Gesamtkonzept.

| Element              | Größe        | Farbe   | Anmerkung |
|----------------------|--------------|---------|-----------|
| Zeitzonen-Abkürzung  | klein        | Grau/Weiß gedimmt | niedrige Priorität, nur Kontext |
| Lokale Zeit          | sehr groß    | Weiß    | Hauptelement, Monospace-Schrift |
| UTC-Zeit             | mittel       | Weiß/Grau gedimmt | sekundäre Information, Monospace-Schrift |

### 4.3 Verhalten

- Nur die sich ändernden Ziffern (v. a. Sekunden) werden neu gezeichnet, um
  Rechenzeit und Stromverbrauch zu minimieren (partielles Redraw statt Full-Screen).
- Bei Zeitzonenwechsel (z. B. durch GPS-Ortsbestimmung oder manuelle Einstellung)
  aktualisiert sich die Abkürzung automatisch.
- Die UTC-Zeit dient als stabile Referenz, unabhängig von der lokal eingestellten
  Zeitzone – nützlich z. B. für Logging, GPS-Zeitstempel, Funkprotokolle.
- Navigation erfolgt über **Swipe-Gesten**, der Hauptscreen (Zeit) ist dabei
  der zentrale Ausgangspunkt. Zwei unabhängige Achsen (festgelegt):

  ```
                    [ Einstellungen ]
                          ↑ swipe-up
                          ↓ swipe-down
  [ Hauptscreen ] ⟶ GPS/Karte ⟶ LoRa/Mesh-Nachrichten ⟶ Alarme/Timer ⟶ zurück
        ⟵                                                              ⟵
  ```

  - **Horizontal – geschlossener Ring aus vier Screens** (Hauptscreen,
    GPS/Karte, LoRa/Mesh-Nachrichten, Alarme/Timer): Swipe nach links
    schaltet einen Screen im Ring weiter, Swipe nach rechts einen zurück;
    durchgängiges Wischen in eine Richtung führt nach allen vier Screens
    wieder zum Hauptscreen. Reihenfolge nach Nutzungshäufigkeit sortiert
    (am häufigsten benötigter Screen am nächsten zum Hauptscreen).
  - **Vertikal, nur auf dem Hauptscreen – Einstellungen liegt außerhalb des
    Rings**: Swipe nach unten auf dem Hauptscreen öffnet Einstellungen,
    Swipe nach oben auf der Einstellungen-Kategorieliste geht zurück zum
    Hauptscreen. Einstellungen war ursprünglich Teil des horizontalen Rings,
    wurde aber bewusst herausgelöst (kein Screen, den man beiläufig
    durchwischt) – zusätzlich weiterhin per Tap-and-Hold auf dem Hauptscreen
    erreichbar (direkt zur Display-Unterseite, siehe Abschnitt 9.3).
  - Auf dem Mesh-Screen ist die vertikale Swipe-up-Geste bereits für die
    Node-Übersicht belegt (siehe Abschnitt 7.2) – vertikale Gesten sind sonst
    auf keinem Ring-Screen außer dem Hauptscreen belegt.
  - **Wichtig:** Diese Ring-/Achsen-Navigation gilt nur für den Wechsel
    zwischen den vier Hauptscreens und Einstellungen selbst. Innerhalb von
    Unterseiten (z. B. den Einstellungs-Kategorien, siehe Abschnitt 9.3, oder
    der Node-Übersicht) gelten **eigene, lokale Swipe-Regeln** (Swipe/Button
    zurück zur jeweils übergeordneten Seite) – diese lösen keine Ring-
    Navigation aus und werden unabhängig davon behandelt.
  - Die BHI-Sensor- und NFC-Screens sind **nicht** über Swipe erreichbar (nur
    über Debug-Konsolen-Befehle) und haben dementsprechend aktuell auch
    keine Swipe-zurück-Geste – der Rückweg läuft dort ausschließlich über das
    Inaktivitäts-Timeout.

---

## 6. GPS/Karte-Screen (Entwurf)

### 6.1 Zweck
Anzeige der aktuellen Position sowie des GPS-Status im Detail (Ergänzung zum
reinen Statusicon aus Abschnitt 3).

### 6.2 Vorschlag Layout
1. Statusleiste (global, wie immer).
2. Fix-Status ausgeschrieben (z. B. "Kein Fix" / "Suche..." / "3D-Fix", passend
   zur Icon-Farbe grau/orange/grün).
3. Koordinaten (Lat/Lon), **gleichzeitig in beiden Formaten, untereinander**
   angezeigt: zuerst WGS84-Dezimalgrad (z. B. `48.1234°`), darunter
   Grad/Minuten/Sekunden (DMS, z. B. `48°07'24"N`) – sowie Höhe über NN in
   **Metern**.
4. Anzahl empfangener Satelliten, HDOP/Genauigkeit in Metern.
5. **Keine Kartendarstellung** – reine Text-/Zahlen-Ansicht, da eine
   Kartenrenderung deutlich mehr Strom und Speicher (Kartenmaterial) benötigen
   würde. Passt damit zum Grundsatz "minimaler Stromverbrauch".
6. **Track-Logging als GPX-Datei auf der SD-Karte** – Aufzeichnung der Position
   **alle 30 Sekunden**, start-/stoppbar über den GPS-Screen (z. B. Tap auf
   ein Start/Stop-Element).

### 6.3 Offene Fragen
- Keine architekturrelevanten offenen Punkte mehr für diesen Screen.

---

## 7. LoRa-Nachrichten-Screen (Entwurf)

### 7.1 Zweck
Übersicht über ein **Meshtastic-kompatibles** LoRa-Mesh-Netzwerk: gesendete/
empfangene Nachrichten, erreichbare Knoten und Verbindungsdetails (Ergänzung
zum Statusicon aus Abschnitt 3).

### 7.2 Vorschlag Layout
1. Statusleiste (global).
2. Liste der letzten Nachrichten (Zeitstempel klein, Absender-Knoten, kurzer
   Text-Preview) – neueste oben, scrollbar per **vertikalem Wischen innerhalb
   der Liste** (Scroll-Geste, keine Screen-Navigation).
3. Verbindungsdetails am oberen Rand oder per Tap aufrufbar: Kanal/Frequenz,
   Signalstärke (RSSI/SNR) zum jeweiligen Absender, Anzahl aktuell erreichbarer
   Mesh-Knoten (Node-Liste).
4. **Separater Node-Übersicht-Screen**, erreichbar über eine **schnelle
   Fling-Geste nach oben** (deutlich schneller/weiter als normales Scrollen):
   Liste bekannter Mesh-Knoten mit Name, letzter Kontakt, Signalqualität,
   ggf. Position (falls der Knoten GPS-Daten sendet). Die Nachrichtenliste
   selbst wird weiterhin per **normalem, langsamerem vertikalem
   Wischen/Ziehen** innerhalb der Liste gescrollt (lokale Scroll-Geste, siehe
   Punkt 2) – die Unterscheidung erfolgt über eine
   **Geschwindigkeits-/Distanzschwelle** (ein "Fling" löst den Screen-Wechsel
   aus, ein gemächliches Ziehen scrollt nur die Liste).
5. **Senden-UI mit vier festen Presets**: "Bin ok", "Verzögerung", "Notfall",
   "Standort senden" – Auswahl per Tap/Swipe aus einer Liste, kein
   Freitext-Eintippen auf der Uhr. **Presets sind über den
   Einstellungen-Screen editierbar** (siehe Abschnitt 9.2, Kategorie "Presets
   verwalten"), diese vier sind die Standardbelegung.
6. **Nachrichten werden dauerhaft auf der SD-Karte gespeichert**, als
   **einfaches Textlog** (nicht JSON/CSV) – ein Eintrag pro Zeile im Format
   `Zeitstempel | Absender | Nachricht` (Pipe-getrennt, z. B.
   `2026-08-29 14:32:01 | Node-A | Bin ok`). **Ist keine SD-Karte gemountet**
   (siehe Statusicon, Abschnitt 3.2), werden Nachrichten nur im **RAM**
   gehalten – mit entsprechendem Datenverlust bei einem Neustart.
7. **Vibration bei neuer Nachricht**, unabhängig davon, welcher Screen gerade
   aktiv ist (globales Verhalten, ähnlich einer Push-Benachrichtigung).
8. **Ein-/Aus-Schalter für das LoRa-Funkmodul** direkt auf diesem Screen
   (gleiche Position/Bauart wie der GNSS-Schalter auf dem GPS-Screen, siehe
   Abschnitt 6) – **Standard: aus**. Ausgeschaltet ist der SX1262 vollständig
   abgeschaltet (Rail aus, nicht nur Standby), es wird nichts empfangen und
   das Statusicon (Abschnitt 3.2) zeigt Grau. Die Wahl wird persistiert.

---

## 8. Alarme/Timer-Screen (Entwurf)

### 8.1 Zweck
Anlegen, Anzeigen und Verwalten von Weckalarmen und Countdown-Timern.

### 8.2 Vorschlag Layout
1. Statusleiste (global).
2. Liste vorhandener Alarme/Timer (Uhrzeit bzw. verbleibende Zeit groß,
   Name/Label klein, Ein/Aus-Toggle pro Eintrag).
3. **Presets zum Anlegen** neuer Alarme/Timer statt Freitext-/Zifferneingabe:
   - Timer: feste Presets **1/5/10/15/30/60/90/120 min**, ggf. "Letzten
     wiederholen".
   - Alarme: feste Presets für **volle und halbe Stunden** (z. B. 06:00,
     06:30, 07:00, 07:30, ...) statt Zifferneingabe, Auswahl aus einer
     scrollbaren Liste.
   - Jeder Alarm kann als **täglich wiederholend** oder mit **individueller
     Wochentag-Auswahl** (z. B. nur Mo–Fr) markiert werden (Ein/Aus-Toggle
     bzw. Wochentag-Picker pro Alarm-Eintrag, siehe Layout-Punkt 2).
4. Aktiver Countdown (falls Timer läuft) prominent, ähnlich groß wie die
   Zeitanzeige auf dem Hauptscreen.

### 8.3 Auslösen eines Alarms
- **Feedback-Art (Vibration/Ton/beides) ist einstellbar** – konfigurierbar
  vermutlich im Einstellungen-Screen, Kategorie "Ton & Vibration" (siehe
  Abschnitt 9.2).
- **Gestoppt wird der Alarm über eine physische Taste** (Power- oder
  Boot-Taste, dieselben wie die Wake-Trigger, siehe Abschnitt 1) – **jeder
  Tastendruck stoppt den Alarm vollständig**, beide Tasten wirken gleich.
- **Snooze-Funktion vorhanden**: fester **10-Minuten-Snooze**. Da Tastendruck
  ausschließlich zum vollständigen Stoppen dient, wird Snooze **separat über
  das Display ausgewählt** (Tap auf ein eigenes Snooze-Element im
  Alarm-Vollbild) – die einzige Stelle, an der Touch beim Alarm-Handling
  verwendet wird.

### 8.4 Offene Fragen
- Genaue Preset-Werte für Timer und Alarme (siehe oben) – welche Werte decken
  den tatsächlichen Bedarf am besten ab?

---

## 9. Einstellungen-Screen (Entwurf)

### 9.1 Zweck
Zentrale Konfiguration von System- und Anzeigeverhalten.

### 9.2 Struktur: verschachtelte Menüs (Kategorien)

Der Einstellungen-Screen ist eine Kategorie-Übersicht; jede Kategorie öffnet
per Tap eine eigene Unterseite mit den zugehörigen Optionen.

1. **Zeit & Zeitzone**
   - Zeitzone (manuell oder automatisch via GPS)
   - Zeitzonen-Abkürzung/Format
2. **Display**
   - Timeout-Dauer
   - Helligkeit
3. **Peripherie**
   - GPS ein/aus (manuelles Powermanagement, unabhängig vom automatischen Zustand)
   - LoRa ein/aus
   - Bluetooth ein/aus
   - WLAN ein/aus
4. **Ton & Vibration**
   - Alarme
   - Benachrichtigungen (z. B. neue LoRa-Nachricht)
   - **Vibrationsmuster** (Unterseite "Vibration pattern"): eine global
     geltende Auswahl aus 6 kuratierten DRV2605-Effekten (Strong Click,
     Sharp Click, Double Click, Triple Click, Soft Bump, Buzz) - der
     DRV2605-Chip bietet insgesamt 123 Effekte in seiner Waveform-Library,
     die Auswahl beschränkt sich bewusst auf spürbar unterschiedliche
     Muster. Antippen eines Eintrags wählt ihn aus (persistiert) und löst
     ihn sofort als Vorschau aus. Gilt für Alarm-/Timer-Klingeln und
     LoRa-Benachrichtigung gleichermaßen (ein Setting, kein separates pro
     Kontext).
5. **Ultra-Sparmodus**
   - Schwellwert (Prozent)
   - Verhalten beim Verlassen (automatische Reaktivierung der Peripherie ja/nein)
6. **Presets verwalten** (falls editierbar, s. Abschnitt 7/8)
   - LoRa-Kurznachrichten-Presets
   - Timer-/Alarm-Presets
7. **Info**
   - Firmware-Version
   - Akkustand in Prozent
   - Speicherplatz auf SD-Karte

### 9.3 Schnellzugriff
**Display-Timeout und Helligkeit** sind per **Tap-and-Hold auf dem
Hauptscreen** direkt erreichbar, ohne durch die Kategorien navigieren zu
müssen.

Änderungen werden **sofort übernommen** – es gibt **keinen separaten
Speichern-Schritt**. Jede Option wirkt unmittelbar nach der Auswahl/Änderung.

Navigation innerhalb der Unterseiten erfolgt per **Swipe-zurück-Geste**
(z. B. Swipe von links nach rechts, analog zu gängigen Touch-UIs). Diese Geste
ist eine **lokale Regel der Einstellungs-Unterseiten** und unabhängig von der
horizontalen Ring-Navigation zwischen den vier Hauptscreens sowie der
vertikalen Hauptscreen↔Einstellungen-Achse (siehe Abschnitt 4.3) – beide
greifen nur, solange man sich auf der Einstellungen-Kategorieliste selbst
befindet, nicht innerhalb einer Kategorie-Unterseite. Die Kategorieliste
selbst wird per Swipe-nach-oben wieder verlassen (zurück zum Hauptscreen).

---

## 10. Ultra-Sparmodus (Low-Battery)

### 10.1 Zweck
Maximale Stromersparnis bei niedrigem Akkustand, um die Restlaufzeit zu strecken
und wenigstens die Kernfunktion (Uhrzeit) so lange wie möglich verfügbar zu halten.

### 10.2 Aktivierung
- **Automatisch** unterhalb von 10 % Akkustand (siehe 10.3 für Details).
- **Manuell** durch den Nutzer jederzeit aktivierbar (z. B. über den
  Einstellungen-Screen, s. Abschnitt 9.2, Kategorie "Ultra-Sparmodus") –
  unabhängig vom aktuellen Akkustand.

### 10.3 Technisches Verhalten (ESP32-S3)
- Der ESP32-S3 wird in **Deep-Sleep** versetzt und wacht **einmal pro Minute
  rein intern** auf (z. B. für RTC-Housekeeping, Akkustand-Messung o. Ä.) –
  **ohne** dabei das Display anzusteuern oder neu zu zeichnen. Dieser
  automatische Wake ist für den Nutzer unsichtbar.
- **Nur ein expliziter Wake-Trigger** (Power-Taste, Boot-Taste oder
  Doppel-Tap, siehe Abschnitt 1) aktiviert das Display tatsächlich und zeigt
  dann die aktuelle Uhrzeit an. Zwischen zwei expliziten Triggern bleibt das
  zuletzt gezeichnete Bild unverändert auf dem Display stehen (siehe 10.3
  unten zur Genauigkeit der angezeigten Zeit).
- **Alle Peripherie wird aktiv abgeschaltet** (nicht nur in Ruhezustand versetzt):
  GPS, LoRa, Bluetooth, WLAN, SD-Karte – unabhängig vom vorherigen Nutzer-Zustand.
- **Display-Darstellung im Ultra-Sparmodus:**
  - Nur die Uhrzeit, **ohne Sekunden** (Format `HH:MM`).
  - Schriftfarbe **Rot** statt Weiß (rote OLED-Subpixel verbrauchen tendenziell
    weniger Strom als weiße/blaue, zusätzlich signalisiert Rot visuell den
    Sparzustand).
  - **Reduzierte Displayhelligkeit.**
  - Statusleiste entfällt vollständig (alle zugehörige Peripherie ist ohnehin
    abgeschaltet).
- Da der automatische Minuten-Wake das Display nicht berührt, ist kein
  kontinuierliches Redraw nötig – **die zuletzt angezeigte Zeit bleibt auf
  dem Display stehen**, bis der nächste explizite Wake-Trigger (Taste/Touch)
  sie aktualisiert. Das bedeutet, die angezeigte Zeit ist potenziell veraltet,
  bis sie durch einen bewussten Trigger neu gezeichnet wird – die minütlichen
  internen Wakes dienen nicht der Displayaktualisierung.

### 10.4 Verhalten beim Verlassen
- Der **Einstellungen-Screen bleibt im Ultra-Sparmodus erreichbar**: Einer der
  drei Wake-Trigger (Power-Taste, Boot-Taste, Doppel-Tap, siehe Abschnitt 1)
  weckt den ESP32-S3 aus dem Deep-Sleep und ermöglicht die normale
  Swipe-nach-unten-Geste vom Hauptscreen zum Einstellungen-Screen, um den
  Modus dort manuell zu verlassen.
- **Automatisches Verlassen beim Anschließen des USB-Ladekabels**: Sobald der
  ESP32-S3 einen USB-Anschluss erkennt, wird der Ultra-Sparmodus automatisch
  beendet – unabhängig vom aktuellen Akkustand.
- Beim Verlassen des Ultra-Sparmodus wird der **vorherige Peripherie-Zustand
  automatisch wiederhergestellt** (z. B. war GPS vorher aktiv, wird es beim
  Verlassen wieder eingeschaltet; war es aus, bleibt es aus) – keine manuelle
  Reaktivierung durch den Nutzer nötig.
- **Beim Trennen des USB-Kabels**: Ist der Akkustand zu diesem Zeitpunkt
  weiterhin unter 10 %, **wird der Ultra-Sparmodus sofort wieder aktiviert**
  (die Uhr kehrt also direkt in den Sparzustand zurück, statt erst auf ein
  erneutes Unterschreiten der Schwelle zu warten). Liegt der Akkustand beim
  Trennen bei 10 % oder darüber, bleibt die Uhr im Normalbetrieb.

---

## 11. Offene Punkte / TODO

**Geklärt:**
- ~~Weitere Screens~~ → GPS/Karte, LoRa-Nachrichten, Alarme/Timer, Einstellungen
- ~~Navigationskonzept~~ → ausschließlich Swipe-Gesten, geschlossener Ring
- ~~Display-Timeout-Verhalten~~ → komplettes Abschalten
- ~~Wake-Trigger~~ → Power-Taste, Boot-Taste, Doppel-Tap auf Display
- ~~Low-Battery-Verhalten~~ → Ultra-Sparmodus (s. Abschnitt 10)
- ~~Statusicons~~ → zusätzlich Bluetooth, WLAN, Akkustand, Ladezustand (s. 3.2)
- ~~Abgerundete Ecken~~ → Safe-Area-Overlay vorhanden (`assets/ui/safe_area_transparent.png`),
  ca. 16px Randabstand an geraden Kanten, ca. 90–100px Eckradius (s. Abschnitt 1);
  Statusleiste entsprechend auf y=54 mit ≥40px Randabstand korrigiert
- ~~Statusicons als Icons statt Text~~ → LVGL-Symbolfont (Montserrat 14, s.
  `lv_symbol_def.h`) für SD/GPS/Bluetooth/WLAN/Ladezustand/Akku; GPX-Tracking
  und LoRa bleiben Text (kein passendes Built-in-Symbol vorhanden, kein
  Emoji-Font im Projekt eingebunden)
- ~~LoRa-Kommunikationsart~~ → Mesh-Netzwerk, Meshtastic-kompatibel
- ~~GPS-Screen: Karte vs. Status~~ → nur Koordinaten/Status, keine Kartendarstellung
- ~~Eingabemethode Nachrichten/Alarme~~ → Presets statt Freitext/Zifferneingabe
- ~~Einstellungen-Struktur~~ → verschachtelte Menüs nach Kategorien (s. 9.2)
- ~~Zeitzonen-Format~~ → Kürzel (z. B. CEST), kein UTC-Offset
- ~~Ultra-Sparmodus-Schwellwert~~ → 10 %, zusätzlich manuell aktivierbar
- ~~Zurück-Navigation in Einstellungen~~ → Swipe-zurück-Geste
- ~~Ultra-Sparmodus technisches Verhalten~~ → ESP32-S3 Deep-Sleep, minütliches
  Wake, rote Zeitanzeige ohne Sekunden, reduzierte Helligkeit, alle Peripherie
  aktiv abgeschaltet (s. Abschnitt 10.3)
- ~~Gesten-Konflikt Haupt-Ring vs. Einstellungen~~ → Einstellungen liegt
  außerhalb des horizontalen Rings (eigene vertikale Achse ab dem
  Hauptscreen), Unterseiten haben zusätzlich eigene lokale Swipe-Regeln
- ~~Einstellungen im Ultra-Sparmodus erreichbar~~ → ja, per Wake-Trigger
- ~~Display zwischen Wake-Zyklen~~ → zuletzt angezeigte Zeit bleibt stehen
- ~~Trigger-Unterscheidung Ultra-Sparmodus~~ → Minuten-Wake ist rein intern
  (kein Display-Update), nur Taste/Touch aktiviert das Display
- ~~Einstellungen: sofort vs. speichern~~ → sofortige Übernahme, kein
  Speichern-Schritt
- ~~Akkustand-Darstellung~~ → Batteriesymbol + Prozentzahl kombiniert
- ~~Alarme wiederholbar~~ → ja, täglich oder mit Wochentag-Auswahl
- ~~Alarm-Feedback~~ → einstellbar (Vibration/Ton/beides)
- ~~Alarm stoppen~~ → über physische Taste (Power/Boot), beide gleich
- ~~Snooze-Funktion~~ → ja, 10 Minuten, ausgelöst per Touch
- ~~GPS-Tracking~~ → ja, GPX-Logging auf SD-Karte, alle 30 Sekunden
- ~~Koordinatenformat/Einheiten~~ → WGS84 über DMS untereinander, metrisch
- ~~GPX-Tracking-Statusicon~~ → ja, eigenes Icon (s. Abschnitt 3.2)
- ~~LoRa-Nachrichtenspeicherung~~ → dauerhaft auf SD-Karte, Textlog
  `Zeitstempel | Absender | Nachricht`; ohne SD-Karte nur im RAM
- ~~LoRa-Presets~~ → "Bin ok", "Verzögerung", "Notfall", "Standort senden",
  editierbar über den Einstellungen-Screen
- ~~LoRa-Benachrichtigung~~ → Vibration bei neuer Nachricht, global
- ~~Node-Übersicht~~ → eigener Screen, eigene Swipe-Richtung (z. B. nach oben)
- ~~Timer-Presets~~ → 1/5/10/15/30/60/90/120 min
- ~~Alarm-Preset-Uhrzeiten~~ → volle/halbe Stunden
- ~~Einstellungen-Schnellzugriff~~ → Tap-and-Hold auf dem Hauptscreen für
  Display-Timeout & Helligkeit
- ~~Ultra-Sparmodus-Ausstieg~~ → automatisch bei USB-Anschluss, vorheriger
  Peripherie-Zustand wird wiederhergestellt
- ~~Schriftart~~ → durchgängig Monospace (z. B. Roboto Mono, JetBrains Mono)
- ~~Node-Übersicht-Geste vs. Listen-Scroll~~ → Fling-Geste (schnell/weit) löst
  Screen-Wechsel aus, normales Ziehen scrollt nur die Liste
- ~~USB-Trennen im Ultra-Sparmodus~~ → sofortige Reaktivierung, falls Akku
  weiterhin unter 10 % liegt

**Noch offen:**
Keine offenen Architektur- oder Detailfragen mehr. Alle in diesem Dokument
aufgeworfenen Punkte sind geklärt. Weitere Klärungsbedarfe entstehen erst,
sobald mit der konkreten Implementierung begonnen wird (z. B. exakte
Pin-Belegung, Timing-Feinabstimmung, konkrete UI-Assets).

---

## 12. Bekannte Hardware-Einschränkung: interner DMA-RAM

Der ESP32-S3 hat 8 MB PSRAM (extern, per QSPI), das für die meisten
Anwendungszwecke reichlich Platz bietet - aber die Funkmodule (BLE-
Controller, WiFi-Treiber) und die Display-DMA-Übertragung können **nur
auf internen SRAM zugreifen, nicht auf PSRAM** (Hardware-Limitierung des
DMA-Controllers, kein Konfigurationsschalter). Dieser interne,
DMA-fähige Speicher ist auf diesem Board chronisch knapp: Display-
Draw-Buffer, LVGL und die Treiber-Allokationen beanspruchen den
Großteil davon bereits im Normalbetrieb - live gemessen (Debug-Kommando
`heap`) liegen nur ca. 33-45 KB frei, je nach Zeitpunkt.

**Konkrete Auswirkungen:**
- Der BLE-Debug-Bridge (`ble_debug.c`) ist deshalb dauerhaft deaktiviert
  (`ble_debug_init()` in `uwatch_main.c` auskommentiert) - der
  BT-Controller reserviert DMA-Speicher, der mit dem Display-Buffer
  kollidiert.
- Ein erster Versuch, einen reinen BLE-Scan-Screen (Bluetooth-Geräte
  auflisten, ohne Verbindungsaufbau) hinzuzufügen, endete live auf echter
  Hardware in einem reproduzierbaren Absturz: `BLE_INIT: Malloc failed`
  beim Initialisieren des BT-Controllers, gefolgt von einem
  Watchdog-Panic (Interrupt WDT Timeout, Core 0). Die Uhr hat sich danach
  selbstständig über den Crash-Dump-Mechanismus neu gestartet - kein
  bleibender Schaden.
- Ein Versuch, durch `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL` (IDF-Default
  16 KB → 1 KB gesenkt) mehr kleine Allokationen (v. a. Task-Stacks) ins
  PSRAM zu verlagern und so mehr internen Speicher freizugeben, hat den
  freien DMA-Speicher tatsächlich mehr als verdoppelt (34 KB → 79 KB,
  live gemessen). Er hat aber gleichzeitig einen realen WLAN-Fehler
  ausgelöst: bei gleichzeitig aktivem LoRa **und** WLAN schlug WLANs
  eigene statische RX-Buffer-Allokation fehl (`wifi: malloc buffer
  fail` → `ESP_ERR_NO_MEM`), und die fehlgeschlagene
  Deinitialisierung hat ca. 62 KB internen Speicher dauerhaft für den
  Rest der Boot-Session leakt (kein Absturz, aber WLAN blieb bis zum
  nächsten Neustart komplett funktionsunfähig). Die Änderung wurde
  deshalb wieder zurückgenommen (`git revert`); `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL`
  bleibt auf dem IDF-Standardwert (16 KB).

**Sicher umgesetzte Gegenmaßnahmen** (jeweils live verifiziert, kein
Regressionsrisiko):
- `CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT` von 2 auf 1 gesenkt: LVGL braucht pro
  Render-Worker einen eigenen 32 KB-Thread-Stack (durch FreeType/ThorVG
  erzwungenes Minimum, siehe `esp_lvgl_adapter`-CMakeLists). Ein zweiter
  Worker bringt hier keinen Nutzen (kein PPA aktiviert, einfache UI) -
  die Halbierung sparte 32 KB internen Speicher komplett ohne
  funktionalen Verlust (live gemessen: freier DMA-Speicher sprang um
  denselben Betrag nach oben, sauberer Boot).
- `CONFIG_BT_CTRL_BLE_MAX_ACT` von 6 auf 1 gesenkt: der reine
  Scan-Screen (Observer-Rolle, keine Verbindungen) braucht nur eine
  gleichzeitige BLE-"Activity", nicht sechs.
- Zusammen mit einem Preflight-Check (`BLE_SCAN_MIN_FREE_DMA_BYTES` in
  `ble_scan.c`, `heap_caps_get_free_size(MALLOC_CAP_DMA)` vor jedem
  `nimble_port_init()`) konnte der BLE-Scan-Screen danach sicher
  eingeführt werden: bei zu wenig freiem DMA-Speicher verweigert er sich
  kontrolliert (UI-Meldung "Not enough memory right now") statt
  abzustürzen. Ist Teil der aktuellen Firmware.
- **Task-Konsolidierung** (`main/services/housekeeping.c`): `daily_log.c`,
  `gpx_log.c` und `syslog_capture.c` liefen ursprünglich als drei
  eigene, dauerhaft laufende FreeRTOS-Tasks mit identischer Form (kurz
  aufwachen, wenig I/O, wieder schlafen) - zu drei separaten Task-Stacks
  für wenig CPU-Bedarf. Zusammengelegt in einen gemeinsamen
  `housekeeping`-Task (4096 Byte Stack) spart das zwei volle Task-Stacks
  (~6 KB) direkt beim Task-Count statt bei der Größe eines einzelnen
  Stacks - live verifiziert (kein Absturz, alle drei Subsysteme feuern
  weiter nach Plan: Tageslog ~60s, GPX ~30s, Syslog-Flush ~10s).

**Als unsicher erwiesen und zurückgenommen:** einzelne Task-Stack-Größen
anhand einer Idle-Messung (`stacks`-Debug-Kommando) nach unten zu
schätzen. `gps_ctrl_task`s Stack wurde probeweise von 8192 auf 3072 Byte
gesenkt (Idle-Messung zeigte nur ~960 Byte Nutzung) - der reale, tiefe
Aufruf-Pfad beim tatsächlichen GNSS-Einschalten (ubxlib
`uDeviceOpen()`/`uGnssPwrOn()`/`mga_ini_seed()`) blieb dabei
unberücksichtigt und führte beim ersten echten Einschalten zu einem
Stack-Overflow-Absturz (`***ERROR*** A stack overflow in task gps_ctrl
has been detected.`) - und weil der GNSS-Zustand in NVS persistiert und
beim Boot automatisch wiederhergestellt wird, zu einer Boot-Crash-Loop.
Alle sieben in derselben Runde probeweise gesenkten Task-Stacks (u. a.
`gps_ctrl`, `mesh_log`, `pm_wake`, `alarm_ring`, `rtccal`) wurden
vollständig auf ihre ursprünglichen Werte zurückgesetzt. **Lehre:** eine
Idle-Momentaufnahme ist für Tasks mit tiefen, selten ausgeführten
Zweigen keine verlässliche Grundlage für eine Stack-Größe - Speicher
lieber über Task-Anzahl (Konsolidierung) statt über Einzelstack-Größe
zurückgewinnen, und wenn doch an einer Einzelgröße gedreht wird, den
echten Worst-Case-Pfad vorher auslösen und großzügig Marge (1.5-2x)
lassen.

**Stand:** BLE-Scan-Screen ist Teil der aktuellen Firmware (mit
Preflight-Guard). `housekeeping.c` konsolidiert drei ehemals
eigenständige Tasks. `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL` bleibt auf
IDF-Standard. Weitere Einzelstack-Reduktionen sind nicht geplant, bevor
nicht der jeweilige Worst-Case-Pfad live unter realer Last gemessen
wurde.
