# -*- coding: utf-8 -*-
"""
Created on Mon Jun  1 20:41:06 2015
@author: olga
"""

import sys
import json
import numpy as np
import mechanics_module as mech
import warnings
from itertools import combinations
from entities import Circle, Rectangle

warnings.filterwarnings('ignore', 'The iteration is not making good progress')

input_string = sys.stdin.read()
scene = json.loads(input_string)

scene["frames"] = []

frames_col = int(scene["duration"] / scene["interval"])
t = np.arange(0, scene["duration"], scene["interval"])
gravity = int(scene["gravity"])
entities = []

if gravity:
    g = 9.8
else:
    g = 0

for entity in scene["entities"]:
        if entity["type"] == "circle":
            circle = Circle(
                entity["m"],
                entity["x"], entity["vx"],
                entity["y"], entity["vy"],
                entity["type"],
                entity["r"],
            )
            entities.append(circle)

        elif entity["type"] == "rect":
            rect = Rectangle(
                entity["m"],
                entity["x"], entity["vx"],
                entity["y"], entity["vy"],
                entity["type"],
                entity["width"],
                entity["height"]
            )
            entities.append(rect)


def collision():
    """Recalculates coordinates and velocities of 2 entities after collision"""
    """Search the moment of collision"""
    for pair in combinations(entities, 2):
        t_collision = int(mech.search_collision(pair))

        if t_collision:
            vx1, vy1, vx2, vy2 = mech.collision(
                pair[0].x[t_collision], pair[0].vx[t_collision],
                pair[0].y[t_collision], pair[0].vy[t_collision],
                pair[1].x[t_collision], pair[1].vx[t_collision],
                pair[1].y[t_collision], pair[1].vy[t_collision],
                pair[0].r, pair[1].r,
                pair[0].m, pair[1].m,
                scene["interval"], scene["c_recovery"],
                scene["c_friction"], scene["k"])

            """calculate initial velocities after central collision"""
            pair[0].initial_vector = [pair[0].x[t_collision], vx1, pair[0].y[t_collision], vy1]
            pair[1].initial_vector = [pair[1].x[t_collision], vx2, pair[1].y[t_collision], vy2]

            """
            slice coordinates and velocities before collision for erase old
            coordinates and velocities starting from the moment
            when collision occurs
            """
            pair[0].slice(t_collision)
            pair[1].slice(t_collision)
            t_new = np.arange(t[t_collision], scene["duration"], scene["interval"])

            """calculate new coordinates and velocities after collision"""
            pair[0].trajectory(t_new, gravity)
            pair[1].trajectory(t_new, gravity)


def main():
    for entity in entities:
        entity.trajectory(t, gravity)
    collision()


if __name__ == "__main__":
    main()


for frame_index in xrange(1, frames_col):
    frame = {}
    for j, entity in enumerate(entities):
        frame[j] = {}
        frame[j]["x"] = entity.x[frame_index]
        frame[j]["y"] = entity.y[frame_index]
        frame[j]["vx"] = entity.vx[frame_index]
        frame[j]["vy"] = entity.vy[frame_index]
        frame[j]["v"] = np.sqrt(entity.vx[frame_index]**2 + entity.vy[frame_index]**2)
        frame[j]["p"] = frame[j]["v"]*entity.m
        frame[j]["Ek"] = entity.m*frame[j]["v"]**2/2  # Kinetic Energy
        frame[j]["Ep"] = entity.m*g*entity.y[frame_index]  # Potential Energy
        frame[j]["E"] = frame[j]["Ek"] + frame[j]["Ep"]  # Full Energy
        frame[j]["t"] = scene["interval"]*frame_index
    scene["frames"].append(frame)

sys.stdout.write(json.dumps(scene))
