"""Drone Challenge Advanced: ein bewegliches Ziel verfolgen (Simulator).

Einmal pro Rechner:  drone-challenge login team10
Der Zufallskurs bekommt ein bewegliches Ziel. Seine Bahn ist bekannt:
kurs.target_position(0, t) liefert die Position t Sekunden nach dem Start-Ereignis
(Kurskoordinaten, genau die Funktion, mit der der Server auswertet). Dieses Beispiel
plant alle 0,5 s mit A* einen Weg dorthin, wo das Ziel gleich sein wird, um die
Hindernisse herum, und meldet am Ende: wann erreicht, kleinster und mittlerer Abstand,
Zeit im Zielradius.
"""

import heapq
import math
import time

from drone_challenge import Drone, pairing, team

SEED = None
DAUER_S = 45
TEMPO = 0.8  # m/s, maximal
RASTER = 0.1  # m
NEU_PLANEN_S = 0.5
RATE_HZ = 30


def plane(kurs, von, nach, hoehe, frei_cache):
    """A* auf der Ebene ``hoehe`` von ``von`` nach ``nach`` (x, y); Liste von (x, y)."""
    h = kurs.hall
    nx = int((h["x_max"] - h["x_min"]) / RASTER)
    ny = int((h["y_max"] - h["y_min"]) / RASTER)

    def punkt(c):
        return h["x_min"] + (c[0] + 0.5) * RASTER, h["y_min"] + (c[1] + 0.5) * RASTER

    def zelle(x, y):
        return (min(nx - 1, max(0, int((x - h["x_min"]) / RASTER))),
                min(ny - 1, max(0, int((y - h["y_min"]) / RASTER))))

    def frei(c):
        if c not in frei_cache:
            frei_cache[c] = not kurs.collides(*punkt(c), hoehe, margin=0.05)
        return frei_cache[c]

    start, ziel = zelle(*von), zelle(*nach)
    if not frei(ziel):  # Ziel gerade hinter/in einem Hindernis: nächste freie Zelle
        nah = sorted(((ziel[0] + dx, ziel[1] + dy) for dx in range(-6, 7) for dy in range(-6, 7)
                      if 0 <= ziel[0] + dx < nx and 0 <= ziel[1] + dy < ny),
                     key=lambda c: math.dist(c, ziel))
        ziel = next((c for c in nah if frei(c)), start)
    kosten, vorher, offen = {start: 0.0}, {}, [(0.0, start)]
    while offen:
        _, c = heapq.heappop(offen)
        if c == ziel:
            weg = [c]
            while c in vorher:
                c = vorher[c]
                weg.append(c)
            return [punkt(c) for c in reversed(weg)]
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                n = (c[0] + dx, c[1] + dy)
                if (dx or dy) and 0 <= n[0] < nx and 0 <= n[1] < ny and frei(n):
                    k = kosten[c] + math.hypot(dx, dy)
                    if k < kosten.get(n, math.inf):
                        kosten[n], vorher[n] = k, c
                        heapq.heappush(offen, (k + math.dist(n, ziel), n))
    return [von]


drone = Drone(simulation=True, random=True, seed=SEED, targets=1)
kurs = drone.course()
ziel = kurs.targets[0]
print(f"Kurs: {kurs.name}; Ziel „{ziel['name']}“ mit {ziel['speed_mps']} m/s ({ziel['mode']})")

x, y, z = kurs.start_course
hoehe = z if kurs.ground else ziel["path"][0][2]
yaw = math.radians(kurs.start_yaw_deg)
frei_cache = {}
weg, geplant = [], -math.inf
t0 = time.time()
drone.start(t0)  # Start-Ereignis = Zeitstempel der ersten Position
drone.pose(x, y, z, yaw, t0)
letzte = t0
while time.time() - t0 < DAUER_S:
    time.sleep(1 / RATE_HZ)
    t = time.time()
    dt, letzte = t - letzte, t
    if t - t0 < 1.0:  # 1 s Stillstand ohne Drehen: Startmessung
        drone.pose(x, y, z, yaw, t)
        continue
    if not kurs.ground and z < hoehe:  # erst steigen
        z = min(hoehe, z + TEMPO * dt)
    else:
        if t - geplant >= NEU_PLANEN_S:
            zx, zy, _ = kurs.target_position(0, t - t0 + 0.5)  # dorthin, wo es gleich ist
            weg, geplant = plane(kurs, (x, y), (zx, zy), z, frei_cache)[1:], t
        rest = TEMPO * dt
        while weg and rest > 0:
            wx, wy = weg[0]
            d = math.hypot(wx - x, wy - y)
            if d <= rest:
                x, y, rest = wx, wy, rest - d
                weg.pop(0)
            else:
                yaw = math.atan2(wy - y, wx - x)
                x, y, rest = x + rest * math.cos(yaw), y + rest * math.sin(yaw), 0
    drone.pose(x, y, z, yaw, t)
drone.stop()

ergebnis = team.finish(pairing.load()["session"])["finish"]["practice"]
for z in (ergebnis.get("course_evidence") or {}).get("targets", []):
    erreicht = f"nach {z['reached_at_s']:.1f} s" if z.get("reached") else "nicht"
    print(
        f"{z['name']}: erreicht {erreicht}, kleinster Abstand {z['minimum_m']:.2f} m, "
        f"mittlerer {z['mean_m']:.2f} m, {z['inside_s']:.1f} s im Radius {z['radius_m']} m"
    )
