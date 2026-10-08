# drone-challenge

Python-Paket für Teams der **Drone Challenge** (Trajectory Lab): eigene Positionsschätzung abgeben, Flugereignisse markieren, Kurs abrufen. Lokale Sicherung, Senden im Hintergrund, Wiederholung und Uhrabgleich laufen intern.

**Dokumentation:** https://drone-challenge.arkturian.com/dokumentation

## Installation

```bash
python3 -m pip install ./drone-challenge-0.7.4.zip
```

Das ZIP vorher unter **Dokumentation → Downloads** herunterladen und den Befehl im Download-Ordner ausführen. Alternativ nach dem Entpacken im Ordner mit `pyproject.toml`: `python3 -m pip install .`.

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
| `examples/` | `beispiel_python.py`, `beispiel_ros2_node.py` |

Geheimnisse (Passwort, Team-Schlüssel, Session-Token) werden nie als Kommandozeilenargument übergeben und mit Rechten 0600 abgelegt.

## Tests

Im öffentlichen Quellcode-Repository liegen eigenständig ausführbare Tests. Das Download-ZIP enthält Paket und Beispiele, keine Tests.

```bash
pip install pytest && pytest
```

Dieses Repository wird aus der Plattform veröffentlicht. Fragen und Fehler bitte an den Veranstalter der Drone Challenge.

### Start und Wertung

Live-Flüge werden ab `system_start` gewertet, ohne GNSS-Emulation oder Zehn-Sekunden-Frist. `Drone` meldet den Start beim Eintritt in den `with`-Block; `drone.start()` ist idempotent. Ohne Kontextmanager vor dem Abheben ausdrücklich `drone.start()` aufrufen. Bei ROS2 meldet `drone-challenge event start` den Start. `switch()` bleibt für alte Aufrufer als protokolliertes Ereignis erhalten, beeinflusst die Live-Wertung aber nicht.

`drone.course().start_rule`, `.assist_distance_m` und `.assist_max_s` sowie `drone-challenge course --json` liefern die eingefrorene Sessionregel. Fehlende Starthilfe bedeutet 0 m, eine fehlende Zeitgrenze 10 s. Liefert ein älterer Server keine `start_rule`, ist sie im SDK `None` (JSON `null`): Die Regel dieses Servers ist dann unbekannt. Der historische CSV-Import mit `switch.txt` bleibt unverändert.

### Freiwillige Starthilfe ab 0.7.4

Der Veranstalter kann bei einem Kurs mit vorgegebener Route Starthilfe freigeben, etwa **2 m / 10 s**. Sie endet an der zuerst erreichten Grenze: Fortschritt entlang der öffentlichen Sollroute oder Zeit seit `system_start`. Die Wertung beginnt trotzdem am Start. Bei 0 m bleibt die Hilfe aus. Advanced-Kurse ohne vorgegebene Route bieten in dieser Stufe keine Starthilfe; positive Werte werden bei Freigabe und Sessionstart abgelehnt.

Es erfolgt kein automatischer Abruf. `drone.assist()` fragt einmal ab; offline kommt `None`. Nur eine Antwort mit `pose` enthält eine aktuelle Messung. Die Pose bezieht sich auf M0 im Hallenframe: Position in Metern, Yaw in Radiant, `stamp_ns` als Unix-Nanosekunden-String. Bei `pose=None` darf eine zuvor gelieferte Position nicht weiter als aktuelle Hilfe verwendet werden.

Für regelmäßige Abrufe kann das Team ausdrücklich `drone.watch_assist(callback, interval_s=0.1)` verwenden. Der Callback läuft auf einem eigenen Thread und erhält auch Antworten ohne Pose; die Subscription mit `close()` oder einem Kontextmanager beenden. Neben einem bestehenden Forwarder `Drone(new_session=False)` mit derselben Kopplung verwenden, damit keine zweite Session entsteht. Eigene Positionsschätzungen werden weiterhin selbst berechnet und übertragen.

ROS2 bietet opt-in `AssistancePublisher(node, drone)` aus `drone_challenge.ros2`: `geometry_msgs/PoseStamped` auf `/drone_challenge/assist`, originaler Zeitstempel und Hallenframe, volatile QoS mit Tiefe 1 und Lebensdauer 0,5 s. Bei Ende `close()` aufrufen. Empfänger müssen Zeitstempelalter und ausbleibende Nachrichten beachten; das Topic sendet keinen separaten Ende-Status.
