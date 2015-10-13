from src.importer import rospy, baxter_interface, CHECK_VERSION

from sensor_msgs.msg import (
    Image,
)

import roslib, rosmsg

import argparse

LEFT = 0
RIGHT = 1

class Baxter:
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
            self._head_camera = baxter_interface.CameraController(camera_name)
            self._head_camera.resolution = (640, 400)
            self._head_camera.fps = 30
            self._head_camera.exposure = 100
            self._head_camera.gain = 0
            #self._head_camera.white_balance_red = 1500
            #self._head_camera.white_balance_green = 1500
            #self._head_camera.white_balance_blue = 1500

            print("Open camera...")
            self._head_camera.open()

        self._head_image_rate = rospy.Rate(30)

        print("Initializing robot...")
        self._head_camera_sub = rospy.Subscriber("/cameras/"+camera_name+"/image", Image, self.republish, None, 1)
        self._display = None#rospy.Publisher('/robot/xdisplay', Image, queue_size=10)

        self.init()

        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        """
        Exits example cleanly by moving head to neutral position and
        maintaining start state
        """
        print("\nExiting...")
        self._head_camera_sub.unregister()

        self.init()

        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()

    def init(self):
        self.head_angle(0)

    def head_angle(self, angle=0.0):
        self._head.set_pan(angle)

    def get_head_image(self, wait=True):
        while wait and self._head_image == None:
            self._head_image_rate.sleep()

        return self._head_image

    def republish(self, msg):
        self._head_image = msg

    def display(self, img):
        print "displayed"
        self._display.publish(img)

