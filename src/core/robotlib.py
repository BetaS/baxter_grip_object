__author__ = 'betas'

from baxter_kdl import kdl_kinematics

import math

import numpy as np
import mathlib

joint_num = 7
JOINT_BASE  = 0
JOINT_MOUNT = 1
JOINT_S0    = 2
JOINT_S1    = 3
JOINT_E0    = 4
JOINT_E1    = 5
JOINT_W0    = 6
JOINT_W1    = 7
JOINT_W2    = 8
JOINT_HAND  = 9

joint_translate = np.array([
        [0.069,     0,          0.27    ], # s0->s1
        [0.102,     0,          0       ], # s1->e0
        [0.069,     0,          0.26242 ], # e0->e1
        [0.104,     0,          0       ], # e1->w0
        [0.01,      0,          0.271   ], # w0->w1
        [0.11597,   0,          0       ], # w1->w2
        [0,         0,          0.11355 ], # w2->hand
        [0,         0,          0.045   ]  # hand->gripper
    ], dtype=np.float32)

joint_axis = np.array([
        [0,      0,         0           ], # mount->s0
        [-1.571, 0,         0           ], # s0->s1
        [1.571,  1.571,     0           ], # s1->e0
        [-1.571, 0,         -1.571      ], # e0->e1
        [1.571,  1.571,     0           ], # e1->w0
        [-1.571, 0,         -1.571      ], # w0->w1
        [1.571,  1.571,     0           ], # w1->w2
        [0,      0,         0           ], # w2->hand
        [0,      0,         0           ]  # hand->gripper
    ], dtype=np.float32)

def reflex_joint_angles(angles):
    angles[0] = -angles[0]
    angles[2] = -angles[2]
    angles[4] = -angles[4]

    return angles

def translate_to_shoulder_frame(pos):
    M = mathlib.translate_matrix2(joint_translate[0])
    M = np.dot(M, mathlib.translate_matrix2(joint_translate[1]))
    Rm = np.identity(4, dtype=np.float32)
    Rm[0:3, 0:3] = mathlib.eular_to_rotation_matrix2(joint_axis[1])
    M = np.dot(M, Rm)
    M = np.dot(M, mathlib.translate_matrix2(joint_translate[2]))

    pos = np.dot(np.linalg.inv(M), [pos[0], pos[1], pos[2], 1])

    return [pos[0], pos[1], pos[2]]

def GST(theta):
    j = theta

    joint_angles = np.array([
        [0,     0,      j[0]], # mount->s0
        [0,     0,      j[1]], # s0->s1
        [0,     0,      j[2]], # s1->e0
        [0,     0,      j[3]], # e0->e1
        [0,     0,      j[4]], # e1->w0
        [0,     0,      j[5]], # w0->w1
        [0,     0,      j[6]], # w1->w2
        [0,     0,      0   ], # w2->hand
        [0,     0,      0   ]  # hand->gripper
    ], dtype=np.float32)

    # Set Axis
    joint_angles += joint_axis

    start = np.array([0, 0, 0])
    end = start
    pre = np.eye(3)
    ret = []
    for i in range(7):
        rm = np.dot(pre, mathlib.eular_to_rotation_matrix(joint_angles[i][0], joint_angles[i][1], joint_angles[i][2]))
        end = start+np.dot(rm, joint_translate[i])

        TF = np.identity(4)
        TF[0:3,0:3] = rm
        TF[0:3,3] = start
        ret.append(TF)

        start = end

        pre = rm


    TF = np.identity(4)
    TF[0:3,0:3] = pre
    TF[0:3,3] = start
    ret.append(TF)

    return ret

def inv_kin(arm, init_joint, pos, ori):
    np.set_printoptions(precision=4,suppress=True)


    joint_limits = [
        [-2.3,    0.7],
        [-2.0,    0.9],
        [-2.9,    2.9],
        [0,       2.5],
        [-2.9,    2.9],
        [-1.4,    1.9],
        [-2.9,    2.9]
    ]

    init_joint = [0, 0, 0, 0, 0, 0, 0]

    for i in range(7):
        init_joint[i] = (joint_limits[i][0]+joint_limits[i][1])/2

    kin = kdl_kinematics.create_kdl_kin(arm+"_arm_mount", arm+"_wrist", "baxter_urdf.xml")

    N = 10000

    th1 = np.array(init_joint, dtype=np.float32)

    g = GST(th1)
    target = np.identity(4)
    target[0:3,3] = pos
    target[0:3,0:3] = mathlib.eular_to_rotation_matrix(ori[0], ori[1], ori[2])

    th = th1

    for i in range(N):
        dist = mathlib.tr2diff(target, g[7])
        #dist[3] = 0
        dist[5] = 0

        J = kin.jacobian(th)
        J_d = np.linalg.pinv(J)

        end = True
        for j in range(3):
            if math.fabs(dist[j]) > 0.001:
                end = False
        #for j in range(2):
        if math.fabs(dist[3]) > 0.2 and math.fabs(dist[3]) < math.pi-0.2:
            end = False
        if math.fabs(dist[4]) > 0.2 and math.fabs(dist[4]) < math.pi-0.2:
            end = False

        if end:
            break

        #print i, dist
        #dist[0:3] = mathlib.unit_vector(dist[0:3])
        #di = np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)
        di = dist*0.01
        #di[0:3] = dist[0:3]*0.01
        #di[3:6] = dist[3:6]*0.01

        theta_dot = np.dot(J_d, di)

        th = th+(theta_dot)
        th = th.tolist()[0]

        for j in range(7):
            angle = th[j]
            th[j] = math.atan2(math.sin(angle), math.cos(angle))

        for j in range(7):
            angle = th[j]

            if angle < joint_limits[j][0]:
                th[j] = joint_limits[j][0]
            elif angle > joint_limits[j][1]:
                th[j] = joint_limits[j][1]

        g = GST(th)

    return th, dist