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
            #print baxter._arms[Defines.LEFT].get_joint_angle()
            #continue

            self.update_camera_surface()

            m = {}
            for arm in Defines.ARMS:
                img = baxter.get_hand_camera_image(arm)
                mat = self.cv_bridge.imgmsg_to_cv2(img)
                marker = self.detect_marker(mat)
                if len(marker) > 0:
                    m[arm] = list(marker[0]["pos"])

                    pos, rot = baxter._arms[arm].get_camera_pos()
                    point = baxter.Camera.find_point(pos, rot, m[arm][0], m[arm][1])
                    object[arm] = {"start": pos, "end": point}

                    self.print_marker(mat, marker)

                    shape = rvizlib.create_arrow(arm, 2, pos, point, [1, 0, 0])
                    self.markers.markers.append(shape)


                cv2.imshow(self.cv_window["camera"][arm], mat)

            if "left" in object and "right" in object:
                pt = mathlib.line_intersect_point(object["left"]["start"], object["left"]["end"], object["right"]["start"], object["right"]["end"])
                shape = rvizlib.create_shape("object", 1, pt)
                self.markers.markers.append(shape)

            self.publish_rviz()

            cv2.waitKey(1)

    def color_blob(self):
        """
        cv2.namedWindow("R", 1)
        cv2.createTrackbar("hueMin", "R", 0, 255, nothing)
        cv2.createTrackbar("hueMax", "R", 255, 255, nothing)
        cv2.createTrackbar("satMin", "R", 31, 255, nothing)
        cv2.createTrackbar("satMax", "R", 212, 255, nothing)
        cv2.createTrackbar("valMin", "R", 0, 255, nothing)
        cv2.createTrackbar("valMax", "R", 255, 255, nothing)
        hm = cv2.getTrackbarPos("hueMin", "R")
        hM = cv2.getTrackbarPos("hueMax", "R")
        sm = cv2.getTrackbarPos("satMin", "R")
        sM = cv2.getTrackbarPos("satMax", "R")
        vm = cv2.getTrackbarPos("valMin", "R")
        vM = cv2.getTrackbarPos("valMax", "R")
        """

        params = cv2.SimpleBlobDetector_Params()
        params.minDistBetweenBlobs = 50.0
        params.filterByInertia = False
        params.filterByConvexity = True
        params.minConvexity = 0.7
        params.filterByColor = False
        params.filterByCircularity = False
        params.filterByArea = True
        params.minArea = 200.0
        params.maxArea = 5000.0
        detector = cv2.SimpleBlobDetector(params)

        while self.is_run:
            objects = []
            cnt = 1

            self.update_camera_surface()

            for arm in Defines.ARMS:
                img = baxter.get_hand_camera_image(arm)
                mat = self.cv_bridge.imgmsg_to_cv2(img)

                pos, rot = baxter._arms[arm].get_camera_pos()

                hsv = cv2.cvtColor(mat, cv2.COLOR_BGR2HSV)
                hsv = cv2.GaussianBlur(hsv, (5, 5), 3)
                blob = cv2.inRange(hsv, (0, 37, 20), (255, 165, 118))

                blob_info = detector.detect(blob)
                mat = cv2.drawKeypoints(blob, blob_info, None, (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

                for keypoint in blob_info:
                    point = baxter.Camera.find_point(pos, rot, keypoint.pt[0], keypoint.pt[1])
                    if arm == Defines.LEFT:
                        objects.append([pos, point])
                    elif arm == Defines.RIGHT:
                        min = 0
                        tp = None
                        for o in objects:
                            # Calculate Distance
                            dist = mathlib.line_intersect_distance(o[0], o[1], pos, point)
                            if tp == None or min >= dist:
                                min = dist
                                tp = o

                        if tp != None:
                            pt = mathlib.line_intersect_point(tp[0], tp[1], pos, point)

                            shape = rvizlib.create_shape("object", cnt+1, pt)
                            self.markers.markers.append(shape)
                            cnt += 1

                cv2.imshow(self.cv_window["camera"][arm], mat)

            cv2.waitKey(1)
            self.publish_rviz()


if __name__ == '__main__':
    main = Main(False)
    main.color_blob()