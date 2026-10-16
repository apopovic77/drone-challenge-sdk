"""Drone Challenge Advanced: selbst einen Weg durch einen Hinderniskurs finden (Simulator).

Einmal pro Rechner:  drone-challenge login team10
Jeder Lauf erzeugt einen neuen Zufallskurs (gleicher SEED = gleicher Kurs). Bekannt sind
nur Start, Ziel und die Hindernisse; den Weg plant dieses Beispiel mit einer einfachen
A*-Suche auf einem 10-cm-Raster und fliegt ihn im Simulator ab. Danach meldet der Server:
Ziel erreicht nach wie vielen Sekunden, Kontakte mit Hindernissen, kleinster Abstand.

Alles in Kurskoordinaten (Meter, Z nach oben): Wer im Simulator dieselben Koordinaten
nutzt, bekommt einen Startanker ohne Drehung. Ersetzt plane_weg() durch euren Planer
und fliege() durch euren Simulator.
"""

import heapq
import math
import time

from drone_challenge import Drone, pairing, team

SEED = None  # z. B. 42, um denselben Kurs zu wiederholen
KURS = None  # Vorlage (Halle, Fahrzeug); None = euer aktiver Challenge-Kurs
RASTER = 0.1  # m
TEMPO = 0.5  # m/s
RATE_HZ = 30


def plane_weg(kurs, hoehe):
    """A* auf einer Ebene in ``hoehe``: Rasterpunkte, an denen das Fahrzeug frei ist."""
    h = kurs.hall
    sx, sy, _ = kurs.start_course
    zx, zy, _ = kurs.goal_course
    nx = int((h["x_max"] - h["x_min"]) / RASTER)
    ny = int((h["y_max"] - h["y_min"]) / RASTER)

    def punkt(i, j):
        return h["x_min"] + (i + 0.5) * RASTER, h["y_min"] + (j + 0.5) * RASTER

    def zelle(x, y):
        return int((x - h["x_min"]) / RASTER), int((y - h["y_min"]) / RASTER)

    frei = {}

    def ist_frei(c):
        if c not in frei:
            frei[c] = 0 <= c[0] < nx and 0 <= c[1] < ny and not kurs.collides(
                *punkt(*c), hoehe, margin=0.05
            )
        return frei[c]

    start, ziel = zelle(sx, sy), zelle(zx, zy)
    kosten, vorher, offen = {start: 0.0}, {}, [(0.0, start)]
    while offen:
        _, c = heapq.heappop(offen)
        if c == ziel:
            weg = [c]
            while c in vorher:
                c = vorher[c]
                weg.append(c)
            return [(*punkt(*c), hoehe) for c in reversed(weg)]
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                n = (c[0] + dx, c[1] + dy)
                if (dx or dy) and ist_frei(n):
                    k = kosten[c] + math.hypot(dx, dy)
                    if k < kosten.get(n, math.inf):
                        kosten[n], vorher[n] = k, c
                        heapq.heappush(offen, (k + math.dist(n, ziel), n))
    raise RuntimeError("kein Weg gefunden")


def glaette(kurs, weg):
    """Abkürzungen nehmen, wo die gerade Linie frei bleibt."""
    def frei(a, b):
        n = max(1, int(math.dist(a, b) / 0.05))
        return all(
            not kurs.collides(*(a[k] + (b[k] - a[k]) * i / n for k in range(3)), margin=0.03)
            for i in range(n + 1)
        )

    aus, i = [weg[0]], 0
    while i < len(weg) - 1:
        j = len(weg) - 1
        while j > i + 1 and not frei(weg[i], weg[j]):
            j -= 1
        aus.append(weg[j])
        i = j
    return aus


def fliege(drone, weg, yaw):
    """Platzhalter für euren Simulator: fährt die Punkte mit TEMPO ab, 1 s Stillstand am Start.

    Stillstand heißt auch: nicht drehen. Erst nach der Startmessung dreht das Fahrzeug.
    """
    t0 = time.time()
    start_yaw = yaw
    drone.start(t0)  # Start-Ereignis = Zeitstempel der ersten Position
    drone.pose(*weg[0], yaw, t0)
    teile = list(zip(weg, weg[1:]))
    gesamt = sum(math.dist(a, b) for a, b in teile)
    while True:
        time.sleep(1 / RATE_HZ)
        t = time.time()
        if t - t0 < 1.0:
            drone.pose(*weg[0], start_yaw, t)
            continue
        s = TEMPO * (t - t0 - 1.0)
        p = weg[-1]
        for a, b in teile:
            laenge = math.dist(a, b)
            if s <= laenge:
                f = s / laenge if laenge else 0.0
                p = tuple(a[k] + f * (b[k] - a[k]) for k in range(3))
                yaw = math.atan2(b[1] - a[1], b[0] - a[0]) if laenge else yaw
                break
            s -= laenge
        drone.pose(*p, yaw, t)
        if TEMPO * (t - t0 - 1.0) > gesamt + 0.5:
            break
    drone.stop()


drone = Drone(course=KURS, simulation=True, random=True, seed=SEED)
kurs = drone.course()
print(f"Kurs: {kurs.name}, {len(kurs.obstacles)} Hindernisse")
print(f"Start {kurs.start_course}, Ziel {kurs.goal_course}")
start, ziel = kurs.start_course, kurs.goal_course
hoehe = start[2] if kurs.ground else ziel[2]
weg = [start] if kurs.ground else [start, (start[0], start[1], hoehe)]
weg += glaette(kurs, plane_weg(kurs, hoehe))[1:-1] + [(ziel[0], ziel[1], hoehe)]
print(f"Geplanter Weg: {len(weg)} Punkte, {sum(math.dist(a, b) for a, b in zip(weg, weg[1:])):.1f} m")
fliege(drone, weg, math.radians(kurs.start_yaw_deg))

ergebnis = team.finish(pairing.load()["session"])["finish"]["practice"]
nachweis = ergebnis.get("course_evidence") or {}
ziel = nachweis.get("goal_reached") or {}
kontakte = nachweis.get("wall_contacts") or {}
print(
    f"Ziel erreicht: {ziel.get('reached')} nach {ziel.get('reached_at_s') or 0:.1f} s, "
    f"Kontakte: {kontakte.get('count')}, kleinster Abstand {kontakte.get('minimum_m', 0):.2f} m"
)
print("Daten: drone-challenge data   ·   3D-Ansicht: Team-Portal, Sessions")
