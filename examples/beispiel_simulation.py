"""Drone Challenge: Simulator-Lauf ohne Halle und ohne OptiTrack.

Einmal pro Rechner:  drone-challenge login team10
Das Programm legt eine Simulations-Session an (nie Hallendaten, der Kurs wird an der
simulierten Startpose verankert), überträgt die simulierten Positionen und wertet aus.
Danach:  drone-challenge data   (CSV) und  python beispiel_auswertung.py daten-<Versuch>

Wichtig für den Startanker: Das Start-Ereignis trägt genau den Zeitstempel der ersten
Position, und die Drohne steht danach mindestens 0,3 s still.
"""

import math
import time

from drone_challenge import Drone, pairing, team

KURS = "Stiegenhaus Labor 5×5×5 · idealisierte Route"  # drone-challenge courses
RATE_HZ = 30


def euer_simulator(route, start_yaw, t):
    """Platzhalter für euren Simulator: fliegt die Sollroute mit 0,5 m/s ab.

    route: Sollroute in Hallenkoordinaten (Meter), start_yaw: Startrichtung (Radiant).
    Rückgabe: x, y, z, yaw in Hallenkoordinaten. Wer im Simulator dieselben Koordinaten
    nutzt, bekommt einen Startanker ohne Drehung: Sollroute und Bahn liegen deckungsgleich.
    """
    weg = 0.5 * max(0.0, t - 1.0)  # erst 1 s stillstehen (Startmessung)
    if weg == 0.0:
        return (*route[0], start_yaw)
    for a, b in zip(route, route[1:]):
        laenge = math.dist(a, b)
        if weg <= laenge:
            f = weg / laenge if laenge else 0.0
            p = [a[k] + f * (b[k] - a[k]) for k in range(3)]
            return p[0], p[1], p[2], math.atan2(b[1] - a[1], b[0] - a[0])
        weg -= laenge
    return (*route[-1], start_yaw)


drone = Drone(course=KURS, simulation=True)
kurs = drone.client.course()  # Kurs dieser Session: Route, Hindernisse, Halle (Hallenkoordinaten)
route = [tuple(p[:3]) for p in kurs["route"]]
start_yaw = math.radians(kurs["course"]["start_yaw_deg"])
t0 = time.time()
drone.start(t0)  # Start = Zeitstempel der ersten Position
drone.pose(*euer_simulator(route, start_yaw, 0.0), t0)
while True:
    time.sleep(1 / RATE_HZ)
    t = time.time()
    x, y, z, yaw = euer_simulator(route, start_yaw, t - t0)
    drone.pose(x, y, z, yaw, t)
    if (x, y, z) == route[-1]:
        break
drone.stop()
session = pairing.load()["session"]
ergebnis = team.finish(session)["finish"]["practice"]
vergleich = ergebnis.get("route_comparison") or {}
print(
    f"Weg {ergebnis['path_length_m']:.2f} m, seitliche Abweichung zur Sollroute mittel "
    f"{vergleich.get('cross_track_mean_m', float('nan')):.3f} m"
)
print("Daten:  drone-challenge data", session)
