import rospy, math
import numpy as np

import src.core.mathlib as mathlib
import src.core.rvizlib as rvizlib

from geometry_msgs.msg import PolygonStamped, Pose, Point, Quaternion
from visualization_msgs.msg import MarkerArray, InteractiveMarkerControl

import time

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

_joint_translate = np.array([
        [0.025,     0.219,      0.108   ], # b->mount
        [0.056,     0,          0.011   ], # mount->s0
        [0.069,     0,          0.27    ], # s0->s1
        [0.102,     0,          0       ], # s1->e0
        [0.069,     0,          0.26242 ], # e0->e1
        [0.104,     0,          0       ], # e1->w0
        [0.01,      0,          0.271   ], # w0->w1
        [0.11597,   0,          0       ], # w1->w2
        [0,         0,          0.11355 ], # w2->hand
        [0,         0,          0.045   ]  # hand->gripper
    ], dtype=np.float32)

_joint_axis = np.array([
        [0,      0,         0           ],
        [-0.002, 0.001,     0.780       ], # b->mount
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

def get_joint_pose(joint, j, all_info=False):
    joint_angles = np.array([
        [0,     0,      0   ], # offset
        [0,     0,      0   ], # b->mount
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
    joint_angles += _joint_axis

    start = np.array([0, 0, 0])
    end = start
    pre = np.eye(3)
    ret = []
    for i in range(joint+1):
        rm = np.dot(pre, mathlib.eular_to_rotation_matrix(joint_angles[i][0], joint_angles[i][1], joint_angles[i][2]))
        end = start+np.dot(rm, _joint_translate[i])

        q = mathlib.Quaternion.from_matrix(rm)
        if all_info:
            ret.append({"pos":start, "ori":q})

        start = end

        pre = rm

    if joint+1 <= len(_joint_axis):
        pre = np.dot(pre, mathlib.eular_to_rotation_matrix(joint_angles[joint+1][0], joint_angles[joint+1][1], joint_angles[joint+1][2]))

    q = mathlib.Quaternion.from_matrix(pre)
    ret.append({"pos":end, "ori":q})
    return ret

def main():
    rospy.init_node("arm_test")

    pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=100)
    markers = MarkerArray()

    angle = [-1.0426821095740018, 0.5173414269013347, 1.441594731869995, 1.8060577350724691, -2.512244319531336, 0.3583257396836843, -0.24057972956145984]
    angles = get_joint_pose(JOINT_HAND, angle, all_info=True)

    while True:
        for i in range(1, len(angles)):
            rvizlib.create_arrow(markers, "tf_left", i, angles[i-1]["pos"], angles[i]["pos"])

        for i in range(0, len(angles)):
            rvizlib.create_axis(markers, "tf_left", i, angles[i]["pos"], angles[i]["ori"])

        pos = [1, 0, 0]

        rvizlib.create_arrow(markers, "tf_object", 1, [0, 0, 0], pos)

        print pos

        rvizlib.create_arrow(markers, "tf_object", 2, angles[2]["pos"], angles[2]["pos"]+pos)

        pub.publish(markers)

if __name__ == "__main__":
    main()