"""Drone Challenge: ein Programm für Halle und Eigentest, Navigation relativ zum Start.

Einmal pro Rechner:  drone-challenge login team03
Die Drohne steht beim Programmstart still auf der Startmarke. Hat das Kamerasystem der
Halle den Start gemessen, sendet das SDK in Hallenkoordinaten (gewertet), sonst in
Kurskoordinaten (Übung). Der Code bleibt derselbe.
"""

import time

from drone_challenge import Drone


def eure_schaetzung():
    """Platzhalter: Position relativ zum Start (x rechts, y vorne, z oben, Meter),
    yaw (Radiant, 0 = Blickrichtung beim Start) und der Zeitpunkt der Messung."""
    return 0.0, 0.0, 0.0, 0.0, time.time()


with Drone(trial_id=time.strftime("team03-%Y%m%d-%H%M%S")) as drone:
    halle = drone.wait_start()  # still stehen, bis das zurückkommt
    print("Hallenflug, gewertet" if halle else "Eigentest ohne Kamerasystem")
    kurs = drone.course()  # kurs.waypoints: relativ zum Start
    drone.takeoff()
    ende = time.time() + 20
    while time.time() < ende:  # euer Flug
        drone.pose_relative(*eure_schaetzung())
        time.sleep(1 / 30)
