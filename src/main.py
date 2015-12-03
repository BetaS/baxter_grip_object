#!/usr/bin/env python

import time, math
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
STATUS_MOVE_TO_RELEASE  = 6
STATUS_RELEASE_OBJECT   = 7

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
        self.object_color = [0, 0, 0]
        self.release_pos = [0, 0, 0]
        self.target_ori = None
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

        self.detector = cv2.SimpleBlobDetector(params)

        params = cv2.SimpleBlobDetector_Params()
        params.minDistBetweenBlobs = 50.0
        params.filterByInertia = False
        params.filterByConvexity = True
        params.minConvexity = 0.7
        params.filterByColor = False
        params.filterByCircularity = False
        params.filterByArea = True
        params.minArea = 200.0

        self.detector2 = cv2.SimpleBlobDetector(params)

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
        baxter.home()
        self.status = STATUS_DETECT_MARKER

    def process_detect_marker(self):
        objects = []
        cnt = 0

        for arm in Defines.ARMS:
            #self.draw_camera_surface(arm)
            #self.draw_arm_position(arm)

            img = baxter.get_hand_camera_image(arm)
            mat = self.cv_bridge.imgmsg_to_cv2(img)

            pos, rot = baxter.get_camera_pose(arm)

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
                        rvizlib.create_interactive_marker(self.marker_server, cnt, tp, 0.1, color, self.click_marker)

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

            pos, rot = baxter.get_camera_pose(arm)

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
                        if dist < 0.02:
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
        pos[2] = pos[2] + 0.05

        if baxter.move_arm(self.focus_arm, pos):
            self.status = STATUS_ALIGN_TO_OBJECT
            self.target_ori = None
        else:
            baxter.home()
            self.status = STATUS_DETECT_OBJECT

    def process_align_to_object(self):
        print "[STATUS] ALIGN"

        if self.focus_arm == "right":
            center_x = 360
            center_y = 130
        else:
            center_x = 380
            center_y = 130

        pos, rot = baxter.get_camera_pose(self.focus_arm)
        e_pos, e_ori = baxter.get_end_effector_pose(self.focus_arm)

        # Find object
        img = baxter.get_hand_camera_image(self.focus_arm)
        mat = self.cv_bridge.imgmsg_to_cv2(img)

        # remove markers
        markers = self.detect_marker(mat)
        self.print_marker(mat, markers, [0, 0, 0])

        # Convert color to HSV
        hsv = cv2.cvtColor(mat, cv2.COLOR_BGR2HSV)

        # Smoothing image (for reduce noises)
        hsv = cv2.GaussianBlur(hsv, (5, 5), 3)

        # Eliminate background color and thresholding
        blob = cv2.inRange(hsv, (0, 37, 20), (255, 165, 118))
        self.print_marker(blob, markers, [0])

        # Eliminate gripper
        if self.focus_arm == Defines.LEFT:
            cv2.fillConvexPoly(blob, np.array([(300, 0), (290, 130), (240, 80), (180, 0)], dtype=np.int32), 0)
            cv2.fillConvexPoly(blob, np.array([(500, 0), (495, 130), (600, 90), (600, 40), (640, 5), (640, 0)], dtype=np.int32), 0)
        else:
            cv2.fillConvexPoly(blob, np.array([(220, 0), (220, 130), (140, 80), (80, 0)], dtype=np.int32), 0)
            cv2.fillConvexPoly(blob, np.array([(490, 0), (500, 130), (620, 85), (630, 20), (640, 10), (640, 0)], dtype=np.int32), 0)

        # Detect object informations from thresholding image
        (cnts, _) = cv2.findContours(blob.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cnts = sorted(cnts, key = lambda x: cv2.arcLength(x, True), reverse = True)
        if len(cnts) > 5:
            cnts = cnts[0:5]

        cnt_info = []
        for cnt in cnts:
            moment = cv2.moments(cnt)
            if moment["m00"] == 0:
                continue

            obj_x = moment["m10"]/moment["m00"]
            obj_y = moment["m01"]/moment["m00"]
            color = cv2.mean(mat[obj_y-5:obj_y+5, obj_x-5:obj_x+5])
            size = cv2.contourArea(cnt)

            cnt_info.append({
                "contour": cnt,
                "pos": [obj_x, obj_y],
                "dist": mathlib.dist([center_x, center_y], [obj_x, obj_y]),
                "color": [color[2]/255, color[1]/255, color[0]/255],
                "size": size
            })

        target = None
        c_min = -1
        for cnt in cnt_info:
            if cnt["size"] > 10000:
                if target == None or c_min > cnt["dist"]:
                    target = cnt
                    c_min = cnt["dist"]

        #cv2.imshow("test", mat)
        #cv2.waitKey(1)

        if target != None:
            obj_x = target["pos"][0]
            obj_y = target["pos"][1]

            #cv2.circle(mat, (int(obj_x), int(obj_y)), 10, (0, 0, 0))
            #cv2.imshow(self.cv_window["camera"][self.focus_arm], mat)

            range = baxter.get_range(self.focus_arm)
            if range < 0.11:
                print "OK!"
                self.status = STATUS_GRASP_OBJECT
                return

            if range > 0:
                center = baxter.Camera.find_point(pos, rot, center_x, center_y)
                object = baxter.Camera.find_point(pos, rot, obj_x, obj_y)

                dist = object-center
                length = mathlib.dist(center, pos)
                dist = (dist*range/length)[0:2]

                print range, [obj_x, obj_y], dist
                if np.linalg.norm(dist) > 0.01:
                    e_pos[0] += dist[0]
                    e_pos[1] += dist[1]
                    if self.target_ori == None:
                        baxter.move_arm(self.focus_arm, e_pos)
                    else:
                        baxter.move_arm(self.focus_arm, e_pos, self.target_ori)
                else:
                    if self.target_ori == None:
                        rect = cv2.minAreaRect(target["contour"])
                        print rect

                        if rect[1][0] > 0 and rect[1][0] > 0:
                            ratio = min(rect[1][1]/rect[1][0], rect[1][0]/rect[1][1])
                            if ratio > 0.75:
                                # don't care orientation
                                self.target_ori = [math.pi, 0, 0]
                            else:
                                # Rotate to desire angle
                                if rect[1][0] < rect[1][1]:
                                    angle = math.radians(-90+rect[2])
                                else:
                                    angle = math.radians(90+rect[2])

                                self.target_ori = [math.pi, 0, angle]
                                baxter.move_arm(self.focus_arm, e_pos, self.target_ori)
                    else:
                        if range > 0.2:
                            e_pos[2] -= 0.05
                        elif range > 0.11:
                            e_pos[2] -= 0.02
                        baxter.move_arm(self.focus_arm, e_pos, self.target_ori)

    def process_grasp_object(self):
        # Grip
        baxter._arms[self.focus_arm].grasp()

        # Move to up
        baxter.home()

        self.status = STATUS_MOVE_TO_RELEASE

    def process_move_to_release(self):
        baxter.target(self.focus_arm)
        self.status = STATUS_RELEASE_OBJECT

    def process_release_object(self):
        baxter._arms[self.focus_arm].ungrasp()

        time.sleep(1)

        # reset
        self.focus_arm = None
        self.object_pos = [0, 0, 0]
        self.object_color = [0, 0, 0]
        self.target_ori = None

        baxter.home()

        self.status = STATUS_DETECT_OBJECT

    def click_marker(self, feedback, color):
        if self.status != STATUS_DETECT_MARKER:
            return

        p = feedback.pose.position
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            print "[CLICK] " + feedback.marker_name + " at (" + str(p.x) + ", " + str(p.y) + ", " + str(p.z)+")"
            self.release_pos = [p.x, p.y, p.z]
            target = self.release_pos
            target[2] += 0.05

            self.status = -1
            if baxter.set_target(target):
                self.status = STATUS_DETECT_OBJECT

    def click_object(self, feedback, color):
        if self.status != STATUS_DETECT_OBJECT:
            return

        p = feedback.pose.position
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            print "[CLICK] " + feedback.marker_name + " at (" + str(p.x) + ", " + str(p.y) + ", " + str(p.z)+")"

            self.object_pos = [p.x, p.y, p.z]
            self.object_color = color
            if p.y >= 0:
                self.focus_arm = "left"
            else:
                self.focus_arm = "right"

            self.status = STATUS_MOVE_TO_OBJECT

    def run(self):
        while self.is_run:
            self.markers.markers = list()
            self.marker_server.clear()

            if self.status == STATUS_INIT:
                self.process_init()
            elif self.status == STATUS_DETECT_MARKER:
                self.process_detect_marker()
            elif self.status == STATUS_DETECT_OBJECT:
                self.process_detect_object()
            elif self.status == STATUS_MOVE_TO_OBJECT:
                self.publish_rviz()
                self.process_move_to_object()
            elif self.status == STATUS_ALIGN_TO_OBJECT:
                self.process_align_to_object()
            elif self.status == STATUS_GRASP_OBJECT:
                self.process_grasp_object()
            elif self.status == STATUS_MOVE_TO_RELEASE:
                self.process_move_to_release()
            elif self.status == STATUS_RELEASE_OBJECT:
                self.process_release_object()

            #cv2.waitKey(1)
            self.publish_rviz()

if __name__ == '__main__':
    main = Main(False)
    #baxter.calibration()
    main.focus_arm = "right"
    main.status = 10
    main.run()
