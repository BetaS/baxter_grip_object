#!/usr/bin/env python

import time
from src.importer import *
from interactive_markers.interactive_marker_server import *
from geometry_msgs.msg import PolygonStamped
from visualization_msgs.msg import MarkerArray, InteractiveMarkerControl

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

        #cv2.namedWindow(self.cv_window["camera"][Defines.LEFT], 1)
        #cv2.namedWindow(self.cv_window["camera"][Defines.RIGHT], 1)
        #cv2.createTrackbar("dist", self.cv_window_name, 100, 1000, nothing)

    def setup_rviz(self):
        self.pub["camera"] = {}
        self.pub["camera"][Defines.LEFT] = rospy.Publisher("/polygon/camera_left", PolygonStamped, queue_size=10)
        self.pub["camera"][Defines.RIGHT] = rospy.Publisher("/polygon/camera_right", PolygonStamped, queue_size=10)

        self.pub["marker"] = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
        self.markers = MarkerArray()

        self.marker_server = InteractiveMarkerServer("simple_marker")

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
        self.marker_server.applyChanges()

    def run(self):
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
            cnt = 0

            self.update_camera_surface()
            self.marker_server.clear()

            for arm in Defines.ARMS:
                img = baxter.get_hand_camera_image(arm)
                mat = self.cv_bridge.imgmsg_to_cv2(img)

                pos, rot = baxter._arms[arm].get_camera_pos()

                # Convert color to HSV
                hsv = cv2.cvtColor(mat, cv2.COLOR_BGR2HSV)

                # Smoothing image (for reduce noises)
                hsv = cv2.GaussianBlur(hsv, (5, 5), 3)

                # Eliminate background color and thresholding
                blob = cv2.inRange(hsv, (0, 37, 20), (255, 165, 118))

                # Detect object informations from thresholding image
                blob_info = detector.detect(blob)
                #mat = cv2.drawKeypoints(blob, blob_info, None, (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

                # Find real world positions for each detected objects
                for keypoint in blob_info:
                    # Find real world position from pixel point
                    point = baxter.Camera.find_point(pos, rot, keypoint.pt[0], keypoint.pt[1])

                    # Add object point from primary image
                    if arm == Defines.LEFT:
                        objects.append([pos, point])

                    # Find same object from opposite image
                    elif arm == Defines.RIGHT:
                        min = 0
                        tp = None
                        for o in objects:
                            # Calculate Distance
                            p, dist = mathlib.line_intersect_skewed(o[0], o[1], pos, point)

                            # Find nearest object in opposite image.
                            if tp == None or min >= dist:
                                min = dist
                                tp = p

                        # print the object when it's available
                        if tp != None:

                            # Make interactive marker for mouse selection
                            int_marker = InteractiveMarker()
                            int_marker.header.frame_id = "base"
                            int_marker.name = "object"+str(cnt)
                            int_marker.pose.position.x = tp[0]
                            int_marker.pose.position.y = tp[1]
                            int_marker.pose.position.z = tp[2]

                            # Make marker shape
                            shape = rvizlib.create_shape("object", cnt, tp)

                            # Add click control
                            box_control = InteractiveMarkerControl()
                            box_control.always_visible = True
                            box_control.interaction_mode = InteractiveMarkerControl.BUTTON
                            box_control.markers.append( shape )

                            # add the control to the interactive marker
                            int_marker.controls.append( box_control )
                            self.marker_server.insert(int_marker, processFeedback)

                            cnt += 1

                #cv2.imshow(self.cv_window["camera"][arm], mat)

            #cv2.waitKey(1)
            self.publish_rviz()


def processFeedback(feedback):
    p = feedback.pose.position
    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        print "[CLICK] " + feedback.marker_name + " at (" + str(p.x) + ", " + str(p.y) + ", " + str(p.z)+")"

if __name__ == '__main__':
    main = Main(False)
    main.run()