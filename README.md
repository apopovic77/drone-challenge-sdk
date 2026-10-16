# drone-challenge

Python-Paket für Teams der **Drone Challenge** (Trajectory Lab): eigene Positionsschätzung abgeben, Flugereignisse markieren, Kurs abrufen. Lokale Sicherung, Senden im Hintergrund, Wiederholung und Uhrabgleich laufen intern.

**Dokumentation:** https://drone-challenge.arkturian.com/dokumentation

## Installation

```bash
python3 -m pip install ./drone-challenge-0.8.2.zip
```

Das ZIP vorher unter **Dokumentation → Downloads** herunterladen und den Befehl im Download-Ordner ausführen. Alternativ nach dem Entpacken im Ordner mit `pyproject.toml`: `python3 -m pip install .`.

**Version prüfen und aktualisieren.** `drone-challenge --version` zeigt die installierte Version. Auf die aktuelle Version kommt ihr direkt vom Server, im selben Python-Umfeld (venv, conda) wie bei der ersten Installation; die Anmeldung bleibt erhalten:

```bash
python3 -m pip install --upgrade https://drone-challenge.arkturian.com/api/v1/public/downloads/drone-challenge-0.8.2.zip
drone-challenge --version
```

Unter Windows `py -m pip …` statt `python3 -m pip …`.

Python ab 3.10, keine weiteren Abhängigkeiten. Für den ROS2-Weg wird `rclpy` aus eurer ROS2-Installation verwendet.

## Schnellstart

```bash
drone-challenge login team03          # einmal pro Rechner, Passwort wird abgefragt
drone-challenge courses               # freigegebene Kurse, der aktive Challenge-Kurs ist markiert
drone-challenge session new           # Session für den aktiven Challenge-Kurs (--course "Name" für einen anderen)
drone-challenge course                # Wegpunkte relativ zur Startpose
```

```python
from drone_challenge import Drone

with Drone() as drone:                    # angemeldet: neue Session für den aktiven Challenge-Kurs
    drone.start()                         # vor dem Abheben; bereits durch with gemeldet
    course = drone.course()               # course.waypoints: relativ zur Startpose
    drone.takeoff()
    while fliegt:
        drone.pose(x, y, z, yaw, zeitstempel)   # Hallenkoordinaten, M0, Unix-Sekunden
```

Den aktiven Challenge-Kurs legt der Veranstalter fest. `Drone(course="Kursname")` wählt einen anderen freigegebenen Kurs, `Drone(new_session=False)` bleibt bei einer Kopplung per QR-Code (`drone-challenge pair`). Ist kein Challenge-Kurs aktiv, warnt `Drone()` und zeichnet ohne Kopplung nur lokal auf.

ROS2 ohne eigenen Code für die Posen:

```bash
drone-challenge ros2 /navigation/pose
```

Offline aufzeichnen und später nachreichen: derselbe Python-Code ohne Kopplung, danach `drone-challenge upload flight-…`.

## Inhalt

| Paket | Zweck |
|---|---|
| `drone_challenge` | Teilnehmer-Schnittstelle: `Drone`, `Course`, `ros2.Challenge`, Kommandozeile `drone-challenge` |
| `diha_participant` | Unterbau: Aufzeichnung, Upload mit Wiederholung, Uhrabgleich, Forwarder `drone-live` (ROS2, VRPN, CSV) |
| `examples/` | `beispiel_python.py`, `beispiel_relativ.py`, `beispiel_ros2_node.py`, `beispiel_simulation.py`, `beispiel_auswertung.py`, `beispiel_hindernisse.py`, `beispiel_ziel.py` |

Geheimnisse (Passwort, Team-Schlüssel, Session-Token) werden nie als Kommandozeilenargument übergeben und mit Rechten 0600 abgelegt.

## Tests

Im öffentlichen Quellcode-Repository liegen eigenständig ausführbare Tests. Das Download-ZIP enthält Paket und Beispiele, keine Tests.

```bash
pip install pytest && pytest
```

Dieses Repository wird aus der Plattform veröffentlicht. Fragen und Fehler bitte an den Veranstalter der Drone Challenge.

### Start und Wertung

Live-Flüge werden ab `system_start` gewertet, ohne GNSS-Emulation oder Zehn-Sekunden-Frist. `Drone` meldet den Start beim Eintritt in den `with`-Block; `drone.start()` ist idempotent. Ohne Kontextmanager vor dem Abheben ausdrücklich `drone.start()` aufrufen. Bei ROS2 meldet `drone-challenge event start` den Start. Das Flugende meldet `drone-challenge event stop`; gleichwertig sind `event flight_end` und `event end` (ab 0.8.2). `switch()` bleibt für alte Aufrufer als protokolliertes Ereignis erhalten, beeinflusst die Live-Wertung aber nicht.

**Regel Positionsquellen (ab 0.8.1 meldbar).** Die bereitgestellte Kursgeometrie (OBJ/JSON) dürft ihr als Vorab-Karte nutzen, Startposition und -orientierung dürfen bekannt sein. Verboten ist jede externe Positionsquelle während des Laufs (OptiTrack/Hallendaten, GNSS, sonstige Fremdmessung). Wurde doch eine genutzt, meldet es: `drone.external_reference_used()` bzw. `drone-challenge event external_reference`.

`drone.course().start_rule`, `.assist_distance_m` und `.assist_max_s` sowie `drone-challenge course --json` liefern die eingefrorene Sessionregel. Fehlende Starthilfe bedeutet 0 m, eine fehlende Zeitgrenze 10 s. Liefert ein älterer Server keine `start_rule`, ist sie im SDK `None` (JSON `null`): Die Regel dieses Servers ist dann unbekannt. Der historische CSV-Import mit `switch.txt` bleibt unverändert.

### Freiwillige Starthilfe ab 0.7.4

Der Veranstalter kann bei einem Kurs mit vorgegebener Route Starthilfe freigeben, etwa **2 m / 10 s**. Sie endet an der zuerst erreichten Grenze: Fortschritt entlang der öffentlichen Sollroute oder Zeit seit `system_start`. Die Wertung beginnt trotzdem am Start. Bei 0 m bleibt die Hilfe aus. Advanced-Kurse ohne vorgegebene Route bieten in dieser Stufe keine Starthilfe; positive Werte werden bei Freigabe und Sessionstart abgelehnt.

Es erfolgt kein automatischer Abruf. `drone.assist()` fragt einmal ab; offline kommt `None`. Nur eine Antwort mit `pose` enthält eine aktuelle Messung. Die Pose bezieht sich auf M0 im Hallenframe: Position in Metern, Yaw in Radiant, `stamp_ns` als Unix-Nanosekunden-String. Bei `pose=None` darf eine zuvor gelieferte Position nicht weiter als aktuelle Hilfe verwendet werden.

Für regelmäßige Abrufe kann das Team ausdrücklich `drone.watch_assist(callback, interval_s=0.1)` verwenden. Der Callback läuft auf einem eigenen Thread und erhält auch Antworten ohne Pose; die Subscription mit `close()` oder einem Kontextmanager beenden. Neben einem bestehenden Forwarder `Drone(new_session=False)` mit derselben Kopplung verwenden, damit keine zweite Session entsteht. Eigene Positionsschätzungen werden weiterhin selbst berechnet und übertragen.

ROS2 bietet opt-in `AssistancePublisher(node, drone)` aus `drone_challenge.ros2`: `geometry_msgs/PoseStamped` auf `/drone_challenge/assist`, originaler Zeitstempel und Hallenframe, volatile QoS mit Tiefe 1 und Lebensdauer 0,5 s. Bei Ende `close()` aufrufen. Empfänger müssen Zeitstempelalter und ausbleibende Nachrichten beachten; das Topic sendet keinen separaten Ende-Status.

### Fahrzeuggröße ab 0.7.5

Optional beim Anlegen einer neuen Session (Breite, Tiefe, Höhe in Metern,
**einschließlich Propeller**):

```python
with Drone(course="stiegenhaus-ideal", size_m=(0.35, 0.35, 0.12)) as drone:
    profile = drone.course().vehicle_profile
    # Wie gewohnt eigene Posen senden.
```

Jede Dimension muss endlich, größer als 0 und höchstens 5 m sein. Ohne Angabe
bleibt das Kursprofil maßgeblich. Eine für das Team hinterlegte Veranstaltergröße
hat Vorrang; eine abweichende Teamangabe wird gespeichert und als Warnung angezeigt.
Der Sicherheitsabstand kommt immer aus dem Kurs. Session, Kursantwort und Bericht
weisen die eingefrorene Größe und ihre Herkunft aus. Die Größe kann während einer
Session nicht geändert werden. Eine neue Veranstaltervorgabe gilt erst beim nächsten
Sessionstart.

```bash
drone-challenge session new --course stiegenhaus-ideal --size-m 0.35 0.35 0.12
drone-live --new-session --course stiegenhaus-ideal --size-m 0.35 0.35 0.12 --ros2-pose-topic /team/pose
```

Dafür ist der Team-Login erforderlich. Mit bestehender Kopplung,
`new_session=False`, `DRONE_LIVE_*`-Zugang oder rein offline kann `size_m` kein
Profil ändern; der SDK-Konstruktor meldet dann einen Fehler. Bei `drone-live`
erfordert `--size-m` ausdrücklich `--new-session`. Für Hallentests geht das auch
mit `--hall --fake-team --api …` und dem separat gesetzten `DRONE_HALL_KEY`.

Die Sollroute wird mit der Sessiongröße erneut geprüft. Bei zu wenig Abstand kommt
eine Warnung mit Minimum, erforderlichem Abstand und Unterschreitung; der Start
bleibt erlaubt. Advanced hat keine öffentliche Sollroute für diesen Nachweis.
Kurse ohne Fahrzeugprofil bleiben unverändert: Größen werden gespeichert und
angezeigt, aber nicht ausgewertet. Das Modell bleibt eine Hüllkugel um M0, am Boden
ein Hüllkreis unter M0; ein Versatz zwischen Marker und Fahrzeugmitte wird nicht
berücksichtigt.

### Relativ zum Start navigieren ab 0.7.6

Navigiert euer System relativ zum Startpunkt, übernimmt das SDK die Umrechnung. Derselbe Code
läuft in der Halle und beim Eigentest ohne Kamerasystem:

```python
with Drone(trial_id="team03-1530") as drone:
    halle = drone.wait_start()               # Drohne steht still; True = Start in der Halle gemessen
    ziele = drone.course().waypoints         # relativ zum Start
    drone.takeoff()
    while fliegt:
        drone.pose_relative(x, y, z, yaw, zeitstempel)
```

`pose_relative()` nimmt x rechts, y vorne, z oben (Meter) und yaw (Radiant, 0 = Blickrichtung
beim Start). Hat das Kamerasystem den Start gemessen, gehen die Posen in Hallenkoordinaten hinaus,
sonst in Kurskoordinaten; dann erstellt der Server einen Übungsbericht. Offline wird die relative
Pose unverändert aufgezeichnet. Bis die Entscheidung gefallen ist (höchstens 3,5 s), werden Posen
mit ihrem Zeitstempel zurückgehalten und danach gesendet. Hat sich die Drohne beim Start bewegt,
wirft `wait_start()` einen `StartFrameError`: neu starten. `pose()` bleibt unverändert für Teams,
die selbst in Hallenkoordinaten rechnen.

## Daten nach dem Flug laden (ab 0.7.7)

Sobald eine Session abgeschlossen ist (nach „Beenden“ bzw. „Auswerten“ des Veranstalters
oder nach der eigenen Testauswertung im Team-Portal), lädt das Team seine Daten:

```bash
drone-challenge data --list            # eigene Sessions mit ID und Zustand
drone-challenge data                   # aktuelle Session (letztes `session new`)
drone-challenge data <SESSION-ID> --out flug1
```

Der Ordner enthält `team.csv` (eigene Posen), `reference.csv` (OptiTrack-Referenz in
Hallenkoordinaten), `events.csv`, `course.json` (Kurs mit gemessenem Startanker) und
`finish.json` (Ergebnis). Während eine Session läuft, gibt es keine Referenzdaten.
Dieselben Daten stehen im Team-Portal unter „Meine Sessions“ als JSON zum Download.


## Advanced-Training: Hindernisse und bewegliche Ziele (ab 0.8.0)

Ohne GNSS selbst navigieren: Bekannt sind nur Start, Ziel und die Hindernisse, den Weg
plant ihr. Training läuft im Simulator (`--simulation`), ohne Halle und ohne OptiTrack; Zufallskurse gehen auch in der Halle (siehe unten). Am
Ende meldet der Server, ob und wann ihr das Ziel erreicht habt, ob ihr ein Hindernis
berührt habt und wie nah ihr einem beweglichen Ziel gekommen seid. Das ist Training,
keine Wertung.

| Stufe | Session | Was ihr bekommt |
|---|---|---|
| 1 · Hinderniskurs | `session new --simulation --course <Advanced-Kurs>` | ein fester Kurs (`drone-challenge courses` zeigt ihn als „Advanced“) |
| 2 · Zufallskurs | `session new --simulation --random [--seed 42] [--obstacles 8]` | jede Session ein neuer Kurs aus Halle und eurem Fahrzeug, garantiert lösbar: fester Start der Vorlage, Ziel und Hindernisse zufällig; gleicher Seed, gleicher Kurs |
| 3 · Bewegliche Ziele | `session new --simulation --random --targets 1` | zusätzlich 1 bis 3 Ziele, die nach dem Start auf einer bekannten Bahn fahren |

```bash
drone-challenge session new --simulation --random --targets 1
drone-challenge course            # Start, Ziel, Hindernisse, bewegliche Ziele
drone-challenge course --json     # dasselbe maschinenlesbar
# ... euren Simulator laufen lassen ...
drone-challenge finish            # Auswertung
drone-challenge data              # Daten als CSV
```

```python
from drone_challenge import Drone

drone = Drone(simulation=True, random=True, targets=1)  # seed=42 wiederholt den Kurs
kurs = drone.course()
start, ziel = kurs.start_course, kurs.goal_course       # Kurskoordinaten, Meter
kurs.obstacles                  # Boxen: Mitte x/y, width (X), depth (Y), height, base_z
kurs.collides(x, y, z)          # berührt euer Fahrzeug dort eine Box oder Wand?
kurs.clearance(x, y, z)         # freier Abstand in Metern (negativ = Kontakt)
kurs.target_position(0, t)      # bewegliches Ziel 0, t Sekunden nach dem Start-Ereignis
```

**Koordinaten.** Hindernisse, Start, Ziel und Zielbahnen stehen in Kurskoordinaten
(rechtshändig, Z nach oben). Simuliert in denselben Koordinaten: Dann verankert sich der
Kurs ohne Drehung genau auf eurem Start. Dafür trägt das Start-Ereignis den Zeitstempel
der ersten Position, und das Fahrzeug steht danach mindestens 0,3 s still, **auch ohne
zu drehen**.

**Kontakt.** `collides()` und `clearance()` rechnen wie der Server: eine Kugel um M0 mit
halber Diagonale des Fahrzeugs (am Boden ein Kreis, Höhe zählt nicht) plus
Sicherheitsabstand, gegen die Boxen und die Hallenwände.

**Bewegliche Ziele.** Jedes Ziel wartet `start_delay_s` und fährt dann seine Bahn `path`
mit `speed_mps` ab: `loop` schließt die Bahn, `pingpong` fährt hin und zurück. Erreicht
ist es, sobald ihr innerhalb `radius_m` seid. `target_position()` ist genau die Funktion,
mit der der Server auswertet.

**Ergebnis** (`finish`, auch im Team-Portal unter „Meine Sessions“ mit 3D-Ansicht) unter
`practice.course_evidence`:

| Feld | Inhalt |
|---|---|
| `goal_reached` | `reached`, `reached_at_s` (Sekunden nach dem Start), `minimum_m` (kleinster Abstand zum Ziel) |
| `wall_contacts` | `count`, `duration_s`, `minimum_m`, je Hindernis `obstacles` |
| `targets` | je Ziel: `reached`, `reached_at_s`, `minimum_m`, `mean_m`, `inside_s` (Zeit im Radius) |

Grundlage ist eure eigene Schätzung über den Selbstanker, keine Referenzmessung.

**Zufallskurs in der Halle (Boden).** `--random` geht auch ohne `--simulation`: Der Server erzeugt den Kurs genau beim Anlegen der Session, vorher kennt ihn niemand. Für Bodenfahrzeuge als Vorlage den Boden-Parcours nehmen (Halle 5 × 5 m; dessen feste Kisten und Route werden nicht übernommen):

```bash
drone-challenge session new --random --course boden-parcours-5x5   # ohne --simulation
drone-challenge course --json    # Start, Ziel, Hindernisse dieses Laufs
```

1. Session erst anlegen, wenn ihr am Startplatz steht und der Veranstalter bereit ist. Jede Session ist ein neuer Kurs; Kurse nicht auf Vorrat holen (höchstens 30 Sessions pro Stunde).
2. Der Start ist immer derselbe: der Startpunkt der Vorlage, beim Boden-Parcours der Slalom-Start links unten (x −1,9, y −1,9), Blick in +X (`kurs.start_yaw_deg` = 0°). Nur Ziel und Hindernisse wechseln. Fahrzeug dort aufstellen; Kurs- und Hallenkoordinaten fallen dann praktisch zusammen.
3. Start-Ereignis senden, mindestens 0,3 s stillstehen. Das Kamerasystem misst dabei die Startpose; der ganze Kurs wird genau dort verankert (Position und Blickrichtung).
4. Nach der Verankerung liefert ein neues `drone.course()` die Hallenform (`kurs.anchored`, `kurs.hall_goal()`); `kurs.anchor` enthält Startpose und Drehung für eigene Umrechnungen.
5. Fahren, dabei eure Posen live senden. Abschluss und Auswertung macht der Veranstalter: OptiTrack-Vergleich wie immer, dazu Ziel erreicht und Hindernis-Kontakte (virtuelle Hindernisse).

**Beispiele:** `examples/beispiel_hindernisse.py` plant mit A* um die Hindernisse und
fliegt den Weg ab; `examples/beispiel_ziel.py` verfolgt ein bewegliches Ziel. Beide laufen
ohne Änderung, sobald `drone-challenge login` erledigt ist.
