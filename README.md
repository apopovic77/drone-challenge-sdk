# drone-challenge

Python-Paket für Teams der **Drone Challenge** (Trajectory Lab): eigene Positionsschätzung abgeben, Flugereignisse markieren, Kurs abrufen. Lokale Sicherung, Senden im Hintergrund, Wiederholung und Uhrabgleich laufen intern.

**Dokumentation:** https://drone-challenge.arkturian.com/dokumentation

## Installation

```bash
pip install "git+https://github.com/apopovic77/drone-challenge-sdk@v0.7.0"
```

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
    course = drone.course()               # course.waypoints: relativ zur Startpose
    drone.takeoff()
    while fliegt:
        drone.pose(x, y, z, yaw, zeitstempel)   # Hallenkoordinaten, M0, Unix-Sekunden
        if gnss_abgeschaltet:
            drone.switch()
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

```bash
pip install pytest && pytest
```

Dieses Repository wird aus der Plattform veröffentlicht. Fragen und Fehler bitte an den Veranstalter der Drone Challenge.
