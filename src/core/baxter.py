from src.importer import rospy, mathlib, baxter_interface, CHECK_VERSION, Defines

from sensor_msgs.msg import (
    Image,
)

import roslib, rosmsg
import numpy as np
from functools import partial
import argparse

class Baxter:
    class Arm:
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

        def __init__(self, name):
            self._name = name
            self._limb = baxter_interface.Limb(name)

            if name == "right":
                idx = -1
            else:
                idx = 1

            self.joint_translate = np.array([
                [0.025,     idx*0.219,  0.108   ], # b->mount
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

            self.joint_axis = np.array([
                [0,      0,         0           ],
                [-0.002, 0.001,     (idx)*0.780 ], # b->mount
                [0,      0,         0           ], # mount->s0
                [-1.571, 0,         0           ], # s0->s1
                [ 1.571, -1.571,    0           ], # s1->e0
                [-1.571, 0,         -1.571      ], # e0->e1
                [0,      -1.571,    1.571       ], # e1->w0
                [-1.571, 0,         -1.571      ], # w0->w1
                [0,      -1.571,    1.571       ], # w1->w2
                [0,      0,         0           ], # w2->hand
                [0,      0,         0           ]  # hand->gripper
            ], dtype=np.float32)

        def get_joint_angle(self):
            angles = self._limb.joint_angles()
            return np.array([angles[self._name+"_s0"], angles[self._name+"_s1"], angles[self._name+"_e0"], angles[self._name+"_e1"], angles[self._name+"_w0"], angles[self._name+"_w1"], angles[self._name+"_w2"]], dtype=np.float32)

        def set_joint_angle(self, target=[], time=5000):
            time /= 1000.0

            pos = {}
            if type(target) == list:
                m = ["s0", "s1", "e0", "e1", "w0", "w1", "w2"]
                for i in range(len(target)):
                    if target[i] != None:
                        pos[self._name+"_"+m[i]] = target[i]
            else:
                pos = target

            self._limb.move_to_joint_positions(pos, time)

        def get_joint_pose(self, joint, angles=None, all_info=False):
            if not angles:
                j = self.get_joint_angle()
            else:
                j = angles;

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
            joint_angles += self.joint_axis

            start = np.array([0, 0, 0])
            end = start
            pre = np.eye(3)
            ret = []
            for i in range(joint+1):
                rm = np.dot(pre, mathlib.eular_to_rotation_matrix(joint_angles[i][0], joint_angles[i][1], joint_angles[i][2]))
                end = start+np.dot(rm, self.joint_translate[i])

                q = mathlib.Quaternion.from_matrix(rm)
                if all_info:
                    ret.append({"pos":start, "ori":q})

                start = end

                pre = rm

            if joint+1 <= len(self.joint_axis):
                pre = np.dot(pre, mathlib.eular_to_rotation_matrix(joint_angles[joint+1][0], joint_angles[joint+1][1], joint_angles[joint+1][2]))

            q = mathlib.Quaternion.from_matrix(pre)
            ret.append({"pos":end, "ori":q})
            return ret

        def get_end_effector_pos(self):
            pose = self.get_joint_pose(self.JOINT_HAND)
            pos = pose[0]["pos"]
            rot = pose[0]["ori"]
            return [pos[0], pos[1], pos[2]], [rot._x, rot._y, rot._z, rot._w]

        def get_camera_pos(self):
            pos, rot = self.get_end_effector_pos()
            q = mathlib.Quaternion(rot[0], rot[1], rot[2], rot[3])
            pos = pos+q.distance(0.03825, 0.012, 0.015355)
            rot = q*mathlib.Quaternion(0.000, 0.000, -0.707, 0.707)
            rot = rot.decompose()
            return pos, rot

    class Camera:
        distort_matrix = np.array(
            [0.0203330914024, -0.0531389002992, 0.000622878864307, -0.00225405996481, 0.0137897514515],
            np.float32)

        focal_length = 405.792519532
        frame_width = 666.049300318
        frame_height = 397.038941784

        intrinsic_matrix = np.array([
            [focal_length,  0.0,            frame_width],
            [0.0,           focal_length,   frame_height],
            [0.0,           0.0,            1.0          ]], np.float32)

        @classmethod
        def extrinsic_matrix(cls, pos, rot):
            R = mathlib.Quaternion(rot[0], rot[1], rot[2], rot[3]).to_rotation_matrix()
            R = np.transpose(R)
            T = np.array([-pos[0], -pos[1], -pos[2]], dtype=np.float32)
            T = np.dot(R, T)
            return np.array(
                   [[R[0][0], R[0][1], R[0][2], T[0]],
                    [R[1][0], R[1][1], R[1][2], T[1]],
                    [R[2][0], R[2][1], R[2][2], T[2]]], dtype=np.float32)

        @classmethod
        def camera_matrix(cls, pos, rot):
            In = cls.intrinsic_matrix
            Ex = cls.extrinsic_matrix(pos, rot)

            Cm = np.dot(In, Ex)
            return Cm

        @classmethod
        def inv_camera_matrix(cls, pos, rot):
            In = np.linalg.inv(cls.intrinsic_matrix)
            Ex = np.linalg.pinv(cls.extrinsic_matrix(pos, rot))

            Cmi = np.dot(Ex, In)
            return Cmi

        @classmethod
        def find_boundary(cls, pos, rot):
            Cm = np.linalg.inv(cls.intrinsic_matrix)
            qrot = mathlib.Quaternion.from_xyzw(rot)

            width = 1280
            height = 800

            image_boundary = [np.array([0, 0, 1]), np.array([width-1, 0, 1]), np.array([width-1, height-1, 1]), np.array([0, height-1, 1])]

            boundary = []
            for p in image_boundary:
                p = np.dot(Cm, p)
                p = qrot.distance(p[0], p[1], p[2])
                p += pos
                boundary.append(p)

            center = np.dot(Cm, np.array([cls.frame_width, cls.frame_height, 1], dtype=np.float32))
            center = qrot.distance(center[0], center[1], center[2])
            center += pos

            return center, boundary

        @classmethod
        def find_point(cls, pos, rot, x, y):
            Cm = np.linalg.inv(cls.intrinsic_matrix)
            qrot = mathlib.Quaternion.from_xyzw(rot)

            p = np.dot(Cm, np.array([x+1280/4, y+800/4, 1], dtype=np.float32))
            p = qrot.distance(p[0], p[1], p[2])
            p += pos

            return p

    def __init__(self, name="pymodules"):
        arg_fmt = argparse.RawDescriptionHelpFormatter
        parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                         description=__doc__)
        parser.parse_args(rospy.myargv()[1:])

        self._name = "baxter_"+name
        self._head_image = None

    def start(self, first=False):

        print("Init ROS Node... ["+self._name+"]")
        rospy.init_node(self._name)

        self._head = baxter_interface.Head()
        #self._hand_cam[LEFT] = baxter_interface.CameraController("left_hand_camera")
        #self._hand_cam[RIGHT] = baxter_interface.CameraController("right_hand_camera")
        #self._head_cam = rospy.Subscriber("/cameras/head_camera/image", Image)

        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        self._rate = rospy.Rate(30)

        print("Enabling robot... ")
        self._rs.enable()

        print("Initializing camera...")
        camera_mapper = ["left_hand_camera", "right_hand_camera"]
        self._camera = {}

        for idx in Defines.ARMS:
            camera_name = idx+"_hand_camera"
            camera = None
            self._camera[idx] = {}

            if first:
                camera = baxter_interface.CameraController(camera_name)
                camera.resolution = (640, 400)
                camera.fps = 30
                camera.exposure = baxter_interface.CameraController.CONTROL_AUTO
                camera.gain = baxter_interface.CameraController.CONTROL_AUTO
                camera.white_balance_red = baxter_interface.CameraController.CONTROL_AUTO
                camera.white_balance_green = baxter_interface.CameraController.CONTROL_AUTO
                camera.white_balance_blue = baxter_interface.CameraController.CONTROL_AUTO
                camera.half_resolution = False

                print("Open "+camera_name+"...")
                #camera.open()

            republish = partial(self.republish_camera, idx)

            sub = rospy.Subscriber("/cameras/"+camera_name+"/image", Image, republish, None, 1)
            self._camera[idx] = {"name": camera_name, "camera": camera, "sub": sub, "image": None}


        print("Initializing robot...")
        self._display = rospy.Publisher('/robot/xdisplay', Image, queue_size=10)
        self._arms = {}
        self._arms[Defines.LEFT] = Baxter.Arm("left")
        self._arms[Defines.RIGHT] = Baxter.Arm("right")

        self.init()

        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        """
        Exits example cleanly by moving head to neutral position and
        maintaining start state
        """
        print("\nExiting...")
        for idx in self._camera:
            self._camera[idx]["sub"].unregister()

        self.init()

        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()

    def init(self):
        self.head_angle(0)
        print("Initialized!!!")

    def head_angle(self, angle=0.0):
        self._head.set_pan(angle)

    def get_hand_camera_image(self, type=Defines.LEFT, wait=True):
        while wait and self._camera[type]["image"] == None:
            self._rate.sleep()

        return self._camera[type]["image"]

    def republish_camera(self, idx, msg):
        self._camera[idx]["image"] = msg

    def display(self, img):
        self._display.publish(img)
