"""
Author(s):
Akhilrajan(Akhil) Vethirajan (v.akhilrajan@gmail.com)
Masters in Robotics,
University of Maryland, College Park
"""

import numpy as np

# dh_parameters:
d1 = 0.181
d2 = 0
d3 = 0
d4 = 0.174        # wrist1_length = d4 - elbow_offset - shoulder_offset
d5 = 0.120
d6 = 0.117

# a = Twist
a1, a4, a5, a6 = 0, 0, 0, 0
a2 = -0.613
a3 = -0.571

# Alpha
a_1 = np.pi/2
a_2, a_3 = 0, 0
a_4 = np.pi/2
a_5 = -a_4
a_6 = 0

d = [d1, d2, d3, d4, d5, d6]
a = [a1, a2, a3, a4, a5, a6]
a_l = [a_1, a_2, a_3, a_4, a_5, a_6]


# Returns end effector Pose
def forward_kinematics(q_config):
    T_set = []
    for q in range(len(q_config)):
        T = [[np.cos(q_config[q]), -np.sin(q_config[q])*np.cos(a_l[q]), np.sin(q_config[q])*np.sin(a_l[q]), a[q]*np.cos(q_config[q])],
             [np.sin(q_config[q]), np.cos(q_config[q])*np.cos(a_l[q]), -np.cos(q_config[q])*np.sin(a_l[q]), a[q]*np.sin(q_config[q])],
             [0, np.sin(a_l[q]), np.cos(a_l[q]), d[q]],
             [0, 0, 0, 1]]
        T_set.append(np.asarray(T).round(decimals=4))
    P = end_effector_pose(T_set)
    return P[0:3, :][:, -1]


def end_effector_pose(T):
    P = T[0]@T[1]@T[2]@T[3]@T[4]@T[5]
    return np.asarray(P)


# q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# r = forward_kinematics(q_start)
# print(r)
# print("[X, Y, Z]")
# print(r[0:3, :][:, -1])
