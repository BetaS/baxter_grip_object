#!/usr/bin/env python

import time
from src.importer import *
from interactive_markers.interactive_marker_server import *
from geometry_msgs.msg import PolygonStamped
from visualization_msgs.msg import MarkerArray

STATUS_INIT             = 0
STATUS_DETECT_MARKER    = 1
STATUS_DETECT_OBJECT    = 2
STATUS_MOVE_TO_OBJECT   = 3
STATUS_ALIGN_TO_OBJECT  = 4
STATUS_GRASP_OBJECT     = 5

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

        self.object_pos = [0, 0, 0]
        self.forus_arm = "left"
        self.status = STATUS_INIT

    def setup_cv(self):
        self.cv_window = {}
        self.cv_window["camera"] = {}
        self.cv_window["camera"][Defines.LEFT] = "left_hand"
        self.cv_window["camera"][Defines.RIGHT] = "right_hand"

        self.cv_bridge = cv_bridge.CvBridge()

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

        self.detector = detector

        cv2.namedWindow(self.cv_window["camera"][Defines.LEFT], 1)
        #cv2.namedWindow(self.cv_window["camera"][Defines.RIGHT], 1)

    def setup_rviz(self):
        self.pub["camera"] = {}
        self.pub["camera"][Defines.LEFT] = rospy.Publisher("/polygon/camera_left", PolygonStamped, queue_size=1)
        self.pub["camera"][Defines.RIGHT] = rospy.Publisher("/polygon/camera_right", PolygonStamped, queue_size=1)

        self.pub["marker"] = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=100)
        self.markers = MarkerArray()

        self.marker_server = InteractiveMarkerServer("simple_marker")

    def draw_camera_surface(self, arm):
        pos, rot = baxter.get_camera_pos(arm)
        center, bound = Baxter.Camera.find_boundary(pos, rot)

        # Create Camera Surface Polys
        p = rvizlib.create_polygon_list(bound)
        self.pub["camera"][arm].publish(p)

        # Create Normal Vector Marker
        rvizlib.create_arrow(self.markers, arm, 1, pos, center, [0, 1, 0])

    def draw_arm_position(self, arm):
        angles = baxter.get_all_joint_pose(arm)
        """
        for i in range(1, len(angles)):
            rvizlib.create_arrow(self.markers, "tf_"+arm, i, angles[i-1]["pos"], angles[i]["pos"])
        """
        for i in range(0, len(angles)):
            rvizlib.create_axis(self.markers, "tf_"+arm, i, angles[i]["pos"], angles[i]["ori"])

    def publish_rviz(self):
        self.pub["marker"].publish(self.markers)
        self.marker_server.applyChanges()

    def detect_marker(self, mat):
        # Find Camera Matrix
        hsv = cv2.cvtColor(mat, cv2.COLOR_BGR2HSV)
        yuv = cv2.cvtColor(mat, cv2.COLOR_BGR2YUV)
        ret, sub = cv2.threshold(yuv[:,:,0], 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        edge = cv2.Canny(sub, 255, 255)

        return markerlib.MarkerDetection(hsv, edge)

    def detect_blob(self, mat):
        # Convert color to HSV
        hsv = cv2.cvtColor(mat, cv2.COLOR_BGR2HSV)

        # Smoothing image (for reduce noises)
        hsv = cv2.GaussianBlur(hsv, (5, 5), 3)

        # Eliminate background color and thresholding
        blob = cv2.inRange(hsv, (0, 37, 20), (255, 165, 118))

        # Detect object informations from thresholding image
        return self.detector.detect(blob)

    def print_marker(self, mat, markers, color=None):
        for m in markers:
            c = m["color"]
            if color != None:
                c = color

            cv2.fillConvexPoly(mat, np.array(m["bound"], np.int32), c)

    def process_init(self):
        baxter.init()
        self.status = STATUS_DETECT_MARKER

    def process_detect_marker(self):
        objects = []
        cnt = 0

        for arm in Defines.ARMS:
            #self.draw_camera_surface(arm)
            #self.draw_arm_position(arm)

            img = baxter.get_hand_camera_image(arm)
            mat = self.cv_bridge.imgmsg_to_cv2(img)

            pos, rot = baxter.get_camera_pos(arm)

            markers = self.detect_marker(mat)

            idx = 0
            for marker in markers:
                # Find color of object
                px = marker["pos"][0]
                py = marker["pos"][1]
                color = marker["color"]
                color = [color[2]/255, color[1]/255, color[0]/255]

                # Find real world position from pixel point
                #point = baxter.Camera.find_point(pos, rot, keypoint[0], keypoint[1])
                point = baxter.Camera.find_point(pos, rot, px, py)

                # Draw Vectors
                rvizlib.create_arrow(self.markers, arm, idx, pos, point, color)
                idx += 1

                # Add object point from primary image
                if arm == Defines.LEFT:
                    objects.append([pos, point, color])

                # Find same object from opposite image
                elif arm == Defines.RIGHT:
                    intersect_object = {}
                    for k in range(len(objects)):
                        o = objects[k]

                        # Calculate Distance
                        p, dist = mathlib.line_intersect_skewed(o[0], o[1], pos, point)

                        # Find nearest object in opposite image.
                        if dist < 0.05:
                            intersect_object[k] = [o, p]

                    min = None

                    for j in intersect_object:
                        # Calculate Color Distance
                        color_dist = graphiclib.color_distance(color, intersect_object[j][0][2])

                        if min == None or min["color"] > color_dist:
                            min = {"color": color_dist, "idx": j, "tp": intersect_object[j][1]}

                    # print the object when it's available
                    if min != None:
                        tp = min["tp"]
                        rvizlib.create_interactive_marker(self.marker_server, cnt, tp, 0.05, color, self.click_marker)

                        # delete object from set
                        del objects[min["idx"]]

                        cnt += 1

    def process_detect_object(self):
        objects = []
        cnt = 0

        for arm in Defines.ARMS:
            #self.draw_camera_surface(arm)
            #self.draw_arm_position(arm)

            img = baxter.get_hand_camera_image(arm)
            mat = self.cv_bridge.imgmsg_to_cv2(img)

            # remove markers
            markers = self.detect_marker(mat)
            self.print_marker(mat, markers, [255, 255, 255])

            # find the blobs
            keypoints = self.detect_blob(mat)

            pos, rot = baxter.get_camera_pos(arm)

            #mat = cv2.drawKeypoints(blob, keypoints, None, (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            # Find real world positions for each detected objects
            idx = 0
            for keypoint in keypoints:
                # Find color of object
                px = keypoint.pt[0]
                py = keypoint.pt[1]
                color = cv2.mean(mat[py-keypoint.size/10:py+keypoint.size/10, px-keypoint.size/10:px+keypoint.size/10])
                color = [color[2]/255, color[1]/255, color[0]/255]

                # Find real world position from pixel point
                #point = baxter.Camera.find_point(pos, rot, keypoint[0], keypoint[1])
                point = baxter.Camera.find_point(pos, rot, px, py)

                # Draw Vectors
                rvizlib.create_arrow(self.markers, arm, idx, pos, point, color)
                idx += 1

                # Add object point from primary image
                if arm == Defines.LEFT:
                    objects.append([pos, point, color])

                # Find same object from opposite image
                elif arm == Defines.RIGHT:
                    intersect_object = {}
                    for k in range(len(objects)):
                        o = objects[k]

                        # Calculate Distance
                        p, dist = mathlib.line_intersect_skewed(o[0], o[1], pos, point)

                        # Find nearest object in opposite image.
                        if dist < 0.05:
                            intersect_object[k] = [o, p]

                    min = None

                    for j in intersect_object:
                        # Calculate Color Distance
                        color_dist = graphiclib.color_distance(color, intersect_object[j][0][2])

                        if min == None or min["color"] > color_dist:
                            min = {"color": color_dist, "idx": j, "tp": intersect_object[j][1]}

                    # print the object when it's available
                    if min != None:
                        tp = min["tp"]

                        rvizlib.create_interactive_marker(self.marker_server, cnt, tp, 0.05, color, self.click_object)

                        # delete object from set
                        del objects[min["idx"]]

                        cnt += 1

            #cv2.imshow(self.cv_window["camera"][arm], blob)

    def process_move_to_object(self):
        pos = self.object_pos
        pos[2] = pos[2] + 0.1

        if baxter.move_arm(self.focus_arm, pos):
            self.status = STATUS_ALIGN_TO_OBJECT
        else:
            baxter.init()
            self.status = STATUS_DETECT_OBJECT

    def process_align_to_object(self):
        # Find object
        img = baxter.get_hand_camera_image(self.focus_arm)
        mat = self.cv_bridge.imgmsg_to_cv2(img)

        self.detect_blob(mat)
        #

        pass

    def run(self):
        while self.is_run:
            self.marker_server.clear()

            if self.status == STATUS_INIT:
                self.process_init()
            elif self.status == STATUS_DETECT_MARKER:
                self.process_detect_marker()
            elif self.status == STATUS_DETECT_OBJECT:
                self.process_detect_object()
            elif self.status == STATUS_MOVE_TO_OBJECT:
                self.process_move_to_object()
            elif self.status == STATUS_ALIGN_TO_OBJECT:
                self.process_align_to_object()

            #cv2.waitKey(1)
            self.publish_rviz()

    def click_marker(self, feedback):
        p = feedback.pose.position
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            print "[CLICK] " + feedback.marker_name + " at (" + str(p.x) + ", " + str(p.y) + ", " + str(p.z)+")"

            self.object_pos = [p.x, p.y, p.z]
            if p.y >= 0:
                self.focus_arm = "left"
            else:
                self.focus_arm = "right"

            self.status = STATUS_DETECT_OBJECT

    def click_object(self, feedback):
        p = feedback.pose.position
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            print "[CLICK] " + feedback.marker_name + " at (" + str(p.x) + ", " + str(p.y) + ", " + str(p.z)+")"

            self.object_pos = [p.x, p.y, p.z]
            if p.y >= 0:
                self.focus_arm = "left"
            else:
                self.focus_arm = "right"

            self.status = STATUS_MOVE_TO_OBJECT

if __name__ == '__main__':
    main = Main(False)
    main.run()