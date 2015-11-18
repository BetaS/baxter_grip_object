__author__ = 'betas'

import numpy as np
from mathlib import Quaternion

def reflex_joint_angles(angles):
    angles[0] = -angles[0]
    angles[2] = -angles[2]
    angles[4] = -angles[4]

    return angles

def translate_to_shoulder_frame(pose):
    d_x = np.array(pose[0:3], dtype=np.float32)
    d_r = Quaternion.from_xyzw(pose[3:7])

    Rm = d_r.to_rotation_matrix()
    Tl = np.dot(Rm, np.array([0, 0, -0.15855]))
    d_x += Tl

    d_x -= [0.025, 0.219, 0.108]
    q = Quaternion(-0.001, -0.000, 0.380, 0.925)
    Rm = q.to_rotation_matrix()
    Tl = np.dot(Rm, np.array([0.056, 0, 0.011]))
    d_x -= Tl
    d_r = d_r * q.inv()

    return d_x.tolist()+d_r.decompose()