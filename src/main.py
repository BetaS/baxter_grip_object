#!/usr/bin/env python

import time
from src.importer import *
from geometry_msgs.msg import PolygonStamped
from visualization_msgs.msg import MarkerArray

def nothing(x):
    pass


class Main:
    cv_window_name = "image"

    def __init__(self, init=False):
        self.pub = {}
        self.is_run = True

        baxter.start(init)

        self.setup_cv()
        self.setup_rviz()

    def setup_cv(self):
        self.cv_window = {}
        self.cv_window["camera"] = {}
        self.cv_window["camera"][Defines.LEFT] = "left_hand"
        self.cv_window["camera"][Defines.RIGHT] = "right_hand"

        self.cv_bridge = cv_bridge.CvBridge()

        cv2.namedWindow(self.cv_window["camera"][Defines.LEFT], 1)
        cv2.namedWindow(self.cv_window["camera"][Defines.RIGHT], 1)
        #cv2.createTrackbar("dist", self.cv_window_name, 100, 1000, nothing)

    def setup_rviz(self):
        self.pub["camera"] = {}
        self.pub["camera"][Defines.LEFT] = rospy.Publisher("/polygon/camera_left", PolygonStamped, queue_size=10)
        self.pub["camera"][Defines.RIGHT] = rospy.Publisher("/polygon/camera_right", PolygonStamped, queue_size=10)

        self.pub["marker"] = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)

        self.markers = MarkerArray()

    def update_camera_surface(self):
        for arm in Defines.ARMS:
            pos, rot = baxter._arms[arm].get_camera_pos()
            center, bound = Baxter.Camera.find_boundary(pos, rot)

            # Create Camera Surface Polys
            p = rvizlib.create_polygon_list(bound)
            self.pub["camera"][arm].publish(p)

            # Create Normal Vector Marker
            marker = rvizlib.create_arrow(arm, 1, pos, center, [0, 1, 0])
            self.markers.markers.append(marker)

    def detect_marker(self, mat):
        # Find Camera Matrix
        hsv = cv2.cvtColor(mat, cv2.COLOR_BGR2HSV)
        yuv = cv2.cvtColor(mat, cv2.COLOR_BGR2YUV)
        ret, sub = cv2.threshold(yuv[:,:,0], 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        edge = cv2.Canny(sub, 255, 255)

        return markerlib.MarkerDetection(hsv, edge)

    def print_marker(self, mat, markers):
        for m in markers:
            cv2.fillConvexPoly(mat, np.array(m["bound"], np.int32), m["color"])

    def publish_rviz(self):
        self.pub["marker"].publish(self.markers)

    def run(self):
        object = {}

        while self.is_run:
            m = {}
            for arm in Defines.ARMS:
                img = baxter.get_hand_camera_image(arm)
                mat = self.cv_bridge.imgmsg_to_cv2(img)
                marker = self.detect_marker(mat)
                if len(marker) > 0:
                    m[arm] = list(marker[0]["pos"])

                    # Align for clipped size
                    m[arm][0] += 1280/4
                    m[arm][1] += 800/4

                    pos, rot = baxter._arms[arm].get_camera_pos()
                    point = baxter.Camera.find_point(pos, rot, m[arm][0], m[arm][1])
                    object[arm] = {"start": pos, "end": point}

                    self.print_marker(mat, marker)

                cv2.imshow(self.cv_window["camera"][arm], mat)

            if "left" in object and "right" in object:
                pt = mathlib.line_intersect_point(object["left"]["start"], object["left"]["end"], object["right"]["start"], object["right"]["end"])
                shape = rvizlib.create_shape("object", 1, pt)
                self.markers.markers.append(shape)

            self.publish_rviz()

            cv2.waitKey(1)


if __name__ == '__main__':
    main = Main()
    main.run()