"""Drone Challenge: Positionsschätzung live übertragen, ohne ROS2.

Einmal pro Rechner:  drone-challenge login team03
Dann legt das Programm vor jedem Flug selbst eine Session an: für den aktiven Challenge-Kurs,
den der Veranstalter festlegt, oder für KURS, wenn ihr dort einen Namen eintragt.
Ohne Anmeldung läuft es offline und zeichnet nur lokal auf;
nachreichen mit:  drone-challenge upload flight-<Ordner>
"""

import math
import time

from drone_challenge import Drone


def eure_positionsschaetzung(t: float) -> tuple[float, float, float, float]:
    """Platzhalter für eure Navigation: Position in Metern (Hallensystem), yaw in Radiant."""
    return math.sin(t / 3), math.cos(t / 3) - 1, 1.0, 0.0


KURS = None  # None: aktiver Challenge-Kurs; sonst z. B. "Zufallskurs" (drone-challenge courses)

with Drone(course=KURS) as drone:
    drone.takeoff()
    start = time.time()
    umgeschaltet = False
    while time.time() - start < 20:  # euer Flug
        t_messung = time.time()  # Zeitpunkt, für den die Schätzung gilt (Systemuhr)
        x, y, z, yaw = eure_positionsschaetzung(t_messung)
        drone.pose(x, y, z, yaw, t_messung)
        if not umgeschaltet and time.time() - start > 5:
            drone.switch()  # externe Positionierung (GNSS-Nachbildung) aus
            umgeschaltet = True
        time.sleep(1 / 30)  # 30 Positionen pro Sekunde
print(f"fertig: {drone.pending} offen, {drone.dropped} verworfen, Dateien in {drone.directory}")
