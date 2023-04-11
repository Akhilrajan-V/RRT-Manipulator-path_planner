"""
Author(s):
Akhilrajan(Akhil) Vethirajan (v.akhilrajan@gmail.com)
Masters in Robotics,
University of Maryland, College Park
"""

import numpy as np
import random

# Arm config-space boundaries
# Order --> [Shoulder pan, Shoulder lift, Elbow, Wrist3, Wrist2, Wrist1]
bounds = np.array([[-2*np.pi/2, 2*np.pi/2], [-np.pi/2, 0], [-np.pi/4, 0], [-np.pi, np.pi], [-np.pi, np.pi],
                   [-2*np.pi, 2*np.pi]])

# New Empty config
q_set = np.zeros(6)


def create_config():
    for i in range(len(bounds)):
        q_set[i] = round(random.uniform(bounds[i][0], bounds[i][1]), 3)
    return q_set
