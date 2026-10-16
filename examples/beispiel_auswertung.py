"""Drone Challenge: einen Lauf darstellen (Draufsicht und Höhe, Soll gegen Ist).

    drone-challenge data                     # legt daten-<Versuch>/ an
    python beispiel_auswertung.py daten-<Versuch>

Braucht matplotlib (pip install matplotlib). Zeichnet die Sollroute (am Startanker
platziert), die eigene Bahn und, falls vorhanden, die OptiTrack-Referenz.
"""

import csv
import json
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt

ordner = Path(sys.argv[1])


def lies(name):
    with (ordner / name).open(encoding="utf-8") as handle:
        return [[float(v) for v in row] for row in list(csv.reader(handle))[1:]]


eigen, referenz = lies("team.csv"), lies("reference.csv")
kurs = json.loads((ordner / "course.json").read_text(encoding="utf-8"))
ergebnis = json.loads((ordner / "finish.json").read_text(encoding="utf-8")) or {}

# Sollroute in den Rahmen der Messung legen: p' = Start + Rz(rotation) * (p - Kursstart)
route = kurs.get("route") or []
anker = kurs.get("anchor") or {}
if route and anker.get("measured_start_m"):
    sx, sy, sz = anker["measured_start_m"]
    cx, cy, cz = anker["course_start_m"]
    r = math.radians(anker["rotation_deg"])
    route = [
        (sx + math.cos(r) * (x - cx) - math.sin(r) * (y - cy),
         sy + math.sin(r) * (x - cx) + math.cos(r) * (y - cy), sz + z - cz)
        for x, y, z in (p[:3] for p in route)
    ]

fig, (oben, hoehe) = plt.subplots(1, 2, figsize=(13, 6))
if route:
    oben.plot([p[0] for p in route], [p[1] for p in route], "-", color="gold", lw=3, label="Sollroute")
if referenz:
    oben.plot([p[1] for p in referenz], [p[2] for p in referenz], color="darkorange", label="OptiTrack")
if eigen:
    oben.plot([p[1] for p in eigen], [p[2] for p in eigen], color="green", label="eigene Bahn")
oben.set_aspect("equal")
oben.set_xlabel("x [m]")
oben.set_ylabel("y [m]")
oben.set_title("Draufsicht")
oben.legend()
for daten, farbe, name in ((referenz, "darkorange", "OptiTrack"), (eigen, "green", "eigene Bahn")):
    if daten:
        t0 = daten[0][0]
        hoehe.plot([p[0] - t0 for p in daten], [p[3] for p in daten], color=farbe, label=name)
hoehe.set_xlabel("Zeit [s]")
hoehe.set_ylabel("z [m]")
hoehe.set_title("Höhe")
hoehe.legend()

uebung = ergebnis.get("practice") or {}
vergleich = uebung.get("route_comparison") or {}
text = []
if uebung:
    text.append(f"Weg {uebung['path_length_m']:.2f} m · Dauer {uebung['duration_s']:.1f} s")
if vergleich.get("status") == "computed":
    text.append(
        f"Abweichung zur Sollroute: mittel {vergleich['cross_track_mean_m']:.3f} m, "
        f"max {vergleich['cross_track_max_m']:.3f} m"
    )
for flug in ergebnis.get("flights") or []:
    m = flug.get("metrics") or {}
    if m.get("rmse_m") is not None:
        text.append(f"Flug {flug.get('flight', 1)}: RMSE {m['rmse_m']:.3f} m · P95 {m['p95_m']:.3f} m")
fig.suptitle(" | ".join(text) or ordner.name)
fig.tight_layout()
plt.savefig(ordner / "auswertung.png", dpi=120)
print(f"gespeichert: {ordner / 'auswertung.png'}")
plt.show()
