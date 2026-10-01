# drone-challenge

Python-Paket für Teams der **Drone Challenge** (Trajectory Lab): eigene Positionsschätzung abgeben, Flugereignisse markieren, Kurs abrufen. Lokale Sicherung, Senden im Hintergrund, Wiederholung und Uhrabgleich laufen intern.

**Dokumentation:** https://drone-challenge.arkturian.com/dokumentation

## Installation

```bash
pip install "git+https://github.com/apopovic77/drone-challenge-sdk@v0.6.0"
```

Python ab 3.10, keine weiteren Abhängigkeiten. Für den ROS2-Weg wird `rclpy` aus eurer ROS2-Installation verwendet.

## Schnellstart

```bash
drone-challenge login team03          # einmal pro Rechner, Passwort wird abgefragt
drone-challenge courses               # freigegebene Kurse
drone-challenge session new --course "Kursname"
drone-challenge course                # Wegpunkte relativ zur Startpose
```

```python
from drone_challenge import Drone

with Drone(course="Kursname") as drone:   # legt eine Session für den Kurs an
    course = drone.course()               # course.waypoints: relativ zur Startpose
    drone.takeoff()
    while fliegt:
        drone.pose(x, y, z, yaw, zeitstempel)   # Hallenkoordinaten, M0, Unix-Sekunden
        if gnss_abgeschaltet:
            drone.switch()
```

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
