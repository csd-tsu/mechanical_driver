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
warnings.filterwarnings('ignore', 'The iteration is not making good progress')

input_string = sys.stdin.read()
scene = json.loads(input_string)
scene["frames"] = []
frames_col = int(scene["duration"] / scene["interval"])

# initial conditions
t = np.arange(0, scene["duration"], scene["interval"])
initial_vector = np.zeros((len(scene["entities"]), 4), float)
for j in range(len(scene["entities"])):
    initial_vector[j] = [scene["entities"][j]["x"], scene["entities"][j]["vx"],
                         scene["entities"][j]["y"], scene["entities"][j]["vy"]]
    
# lists for coordinates and velocities store
x = []
y = []
vx = []
vy = []

for j in range(len(scene["entities"])):
    x.append([])
    vx.append([])
    y.append([])
    vy.append([])


def collision_circles():
    """Recalculate coordinates and velocities of 2 circles after collision."""
    t_collision = 0
    """Search the moment of collision."""
    for j in range(len(scene["entities"])-1):
        t_collision = int(mech.search_collision(
            x[j], x[j+1], y[j], y[j+1],
            scene["entities"][j]["r"] + scene["entities"][j+1]["r"])
        )
     
        if t_collision:
            """Calculate initial velocities after central collision."""
            vx1, vy1, vx2, vy2, phi1, phi2 = mech.collision(
                vx[j][t_collision], vy[j][t_collision],
                vx[j+1][t_collision], vy[j+1][t_collision],
                x[j][t_collision], y[j][t_collision],
                x[j+1][t_collision], y[j+1][t_collision],
                scene["entities"][j]["m"], scene["entities"][j+1]["m"],
                scene["entities"][j]["r"], scene["entities"][j+1]["r"],
                scene["interval"],
                scene["elasticity"]
            )
            
            """
            Slice coordinates and velocities before collision for erase old 
            coordinates and velocities starting from the moment 
            when collision occurs.
            """
            x[j], vx[j], y[j], vy[j], x[j+1], vx[j+1], y[j+1], vy[j+1] = \
                x[j][:t_collision+1], vx[j][:t_collision+1],\
                y[j][:t_collision+1], vy[j][:t_collision+1],\
                x[j+1][:t_collision+1], vx[j+1][:t_collision+1],\
                y[j+1][:t_collision+1], vy[j+1][:t_collision+1]
 
            """Calculate new coordinates and velocities after collision."""
            initial_vector[j] = [x[j][t_collision], vx1, 
                                 y[j][t_collision], vy1]
            initial_vector[j+1] = [x[j+1][t_collision], vx2, 
                                   y[j+1][t_collision], vy2]
            t_new = np.arange(t[t_collision], 
                              scene["duration"], 
                              scene["interval"]) 
            
            x[j], vx[j], y[j], vy[j] = mech.trajectory(
                initial_vector[j],
                x[j], vx[j],
                y[j], vy[j], 
                t_new
            )
            x[j+1],vx[j+1],y[j+1], vy[j+1] = mech.trajectory(
                initial_vector[j+1],
                x[j+1],vx[j+1],
                y[j+1],vy[j+1],
                t_new
            )
            
            
def main():
        # calculate coordinates    
        x[j], vx[j], y[j], vy[j] = mech.trajectory(
            initial_vector[j], 
            x[j], vx[j],
            y[j],vy[j], 
            t
        )

        # search for collision between circles
        if len(scene["entities"])>1:
            collision_circles()

if __name__ == "__main__":
    main()
    
for frame_index in xrange(1, frames_col):
    frame = {}
    for j in range(len(scene["entities"])):
        frame[j] = {}
        frame[j]["x"] = x[j][frame_index]
        frame[j]["y"] = y[j][frame_index]
        frame[j]["vx"] = vx[j][frame_index]
        frame[j]["vy"] = vy[j][frame_index]
    scene["frames"].append(frame)

sys.stdout.write(json.dumps(scene))
