from src.importer import rospy, mathlib, baxter_interface, CHECK_VERSION, ClassProperty

from sensor_msgs.msg import (
    Image,
)

import roslib, rosmsg

import numpy as np

import argparse

LEFT = 0
RIGHT = 1

class Baxter:
    class Camera:
        distort_matrix = np.array(
            [0.0203330914024, -0.0531389002992, 0.000622878864307, -0.00225405996481, 0.0137897514515],
            np.float32)

        intrinsic_matrix = np.array([
            [405.792519532, 0.0,            666.049300318],
            [0.0,           405.792519532,  397.038941784],
            [0.0,           0.0,            1.0          ]], np.float32)

        @ClassProperty
        @classmethod
        def extrinsic_matrix(cls):
            pos = [0.79983, 1.0118, 0.28273]
            rot = [-0.65328, 0.2706, -0.2706, 0.65328]

            R = mathlib.quat_to_rotation_matrix(rot[0], rot[1], rot[2], rot[3])
            R = np.transpose(R)
            C = np.array([pos[0], pos[1], pos[2]], dtype=np.float32)
            T = -np.dot(R, C)
            return np.array(
                   [[R[0][0], R[0][1], R[0][2], T[0]],
                    [R[1][0], R[1][1], R[1][2], T[1]],
                    [R[2][0], R[2][1], R[2][2], T[2]]], dtype=np.float32)

        @classmethod
        def camera_matrix(cls):
            In = cls.intrinsic_matrix
            Ex = cls.extrinsic_matrix

            Cm = np.dot(In, Ex)
            return Cm

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
        print("Enabling robot... ")
        self._rs.enable()

        print("Initializing camera...")

        #self._left_camera = baxter_interface.CameraController("left_hand_camera")
        #self._left_camera.close()

        camera_name = "left_hand_camera"
        if first:
            self._camera = baxter_interface.CameraController(camera_name)
            self._camera.resolution = (640, 400)
            self._camera.fps = 30
            self._camera.exposure = 100
            self._camera.gain = 0

            print("Open camera...")
            self._camera.open()

        self._rate = rospy.Rate(30)

        print("Initializing robot...")
        self._camera_sub = rospy.Subscriber("/cameras/"+camera_name+"/image", Image, self.republish, None, 1)
        self._display = None#rospy.Publisher('/robot/xdisplay', Image, queue_size=10)

        self.init()

        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        """
        Exits example cleanly by moving head to neutral position and
        maintaining start state
        """
        print("\nExiting...")
        self._camera_sub.unregister()

        self.init()

        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()

    def init(self):
        self.head_angle(0)

    def head_angle(self, angle=0.0):
        self._head.set_pan(angle)

    def get_camera_image(self, type=0, wait=True):
        while wait and self._camera_image == None:
            self._rate.sleep()

        return self._camera_image

    def republish(self, msg):
        self._camera_image = msg

    def display(self, img):
        self._display.publish(img)

    def get_joint_angle(self):
        # [s0, s1, ....]
        pass

    def get_end_effector_pos(self):
        pass