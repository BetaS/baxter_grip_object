#!/usr/bin/env python

import time
from src.importer import *
from interactive_markers.interactive_marker_server import *
from geometry_msgs.msg import PolygonStamped, Pose, Point, Quaternion
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
        pos, rot = baxter._arms[arm].get_camera_pos()
        center, bound = Baxter.Camera.find_boundary(pos, rot)

        # Create Camera Surface Polys
        p = rvizlib.create_polygon_list(bound)
        self.pub["camera"][arm].publish(p)

        # Create Normal Vector Marker
        rvizlib.create_arrow(self.markers, arm, 1, pos, center, [0, 1, 0])


    def draw_arm_position(self, arm):
        angles = baxter._arms[arm].get_joint_pose(Baxter.Arm.JOINT_HAND, all_info=True)
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

    def print_marker(self, mat, markers):
        for m in markers:
            cv2.fillConvexPoly(mat, np.array(m["bound"], np.int32), m["color"])

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

            self.marker_server.clear()

            for arm in Defines.ARMS:
                self.draw_camera_surface(arm)
                self.draw_arm_position(arm)

                img = baxter.get_hand_camera_image(arm)
                mat = self.cv_bridge.imgmsg_to_cv2(img)

                #cv2.imwrite("sample.png", mat)

                pos, rot = baxter._arms[arm].get_camera_pos()

                # Convert color to HSV
                hsv = cv2.cvtColor(mat, cv2.COLOR_BGR2HSV)

                # Smoothing image (for reduce noises)
                hsv = cv2.GaussianBlur(hsv, (5, 5), 3)

                # Eliminate background color and thresholding
                blob = cv2.inRange(hsv, (0, 37, 20), (255, 165, 118))
                                # Detect object informations from thresholding image
                keypoints = detector.detect(blob)
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
                    point = baxter.Camera.find_point(pos, rot, keypoint.pt[0], keypoint.pt[1])

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

                            # Make interactive marker for mouse selection
                            int_marker = InteractiveMarker()
                            int_marker.header.frame_id = "base"
                            int_marker.name = "object"+str(cnt)
                            int_marker.pose.position.x = tp[0]
                            int_marker.pose.position.y = tp[1]
                            int_marker.pose.position.z = tp[2]

                            #color = [1, 0, 0]

                            # Add click control
                            box_control = InteractiveMarkerControl()
                            box_control.always_visible = True
                            box_control.interaction_mode = InteractiveMarkerControl.BUTTON
                            rvizlib.create_shape(box_control, "object", cnt, tp, size=0.05, color=color)

                            # add the control to the interactive marker
                            int_marker.controls.append( box_control )
                            self.marker_server.insert(int_marker, processFeedback)

                            # delete object from set
                            del objects[min["idx"]]

                            cnt += 1

                cv2.imshow(self.cv_window["camera"][arm], blob)

            cv2.waitKey(1)
            self.publish_rviz()

    def forward_position(self):
        while True:
            self.update_arm_position()
            self.publish_rviz()

import math

def processFeedback(feedback):
    p = feedback.pose.position
    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        print "[CLICK] " + feedback.marker_name + " at (" + str(p.x) + ", " + str(p.y) + ", " + str(p.z)+")"

        angles = baxter._arms[Defines.LEFT].get_joint_angle()
        pos = [ 0.20703122, 0.77710983, 0.31864885]
        rot = mathlib.Quaternion.from_euler([2.880644176473962, 1.5683547289343642, -2.0990489867913644])
        angles = [0, 0, 0, 0, 0, 0, 0]
        result = baxter._arms[Defines.LEFT]._kin.inverse_kinematics(pos, None, angles)

        #result = baxter._arms[Defines.LEFT]._kin.inverse_kinematics([p.x, p.y, p.z+0.1], None, angles)
        print "[INVKIN] "+str(p.x)+", "+str(p.y)+", "+str(p.z)
        if result != None:
            for i in range(0, 7):
                result[i] = math.radians(result[i])
            #result += angles
            baxter._arms[Defines.LEFT].set_joint_angle(result.tolist())

        print result

if __name__ == '__main__':
    main = Main(False)

    #home_position = [-0.0543, -2.8186, -1.5383, 1.9077, 0.4807, 1.0787, 0.3275]
    home_position = [-1.0426821095740018, 0.5173414269013347, 1.441594731869995, 1.8060577350724691, -2.512244319531336, 0.3583257396836843, -0.24057972956145984]
    #[0.13460681, -1.37329626, -0.97791272, 1.03083503, -0.1599175, 1.74375272, 0.11006312]
    #home_position = [0, 0, 0, 0, 0, 0, 0]
    #home_position = [-0.8598, 0.5846, 0.1318, -1.4543, 0.6806, 0.3411, -1.1776]
    #home_position = [0.0103, 0.0178, -0.0126, -0.0084, 0.0271, -0.0122, -0.0140]
    baxter._arms[Defines.LEFT].set_joint_angle(home_position)
    #baxter._arms[Defines.RIGHT].set_joint_angle(robotlib.reflex_joint_angles(home_position))

    print  baxter._arms[Defines.LEFT].get_joint_angle()
    """
    pos, rot = baxter._arms[Defines.LEFT].get_end_effector_pos()
    pose = baxter._arms[Defines.LEFT].get_joint_pose(Baxter.Arm.JOINT_HAND, home_position, False)
    pose = pose[0]["pos"].tolist()+pose[0]["ori"].decompose()
    print pose
    pose = robotlib.translate_to_shoulder_frame(pose)
    print pose
    """


    angles = baxter._arms[Defines.LEFT].get_joint_angle()
    pos = Point(x=0.20703122, y=0.77710983, z=0.31864885)
    #rot = Quaternion([2.880644176473962, 1.5683547289343642, -2.0990489867913644])
    angles = [0, 0, 0, 0, 0, 0, 0]
    #result = baxter._arms[Defines.LEFT]._kin.inverse_kinematics(pos, None, angles)
    #result = baxter._arms[Defines.LEFT]._kin.inverse(Pose(position=pos))

    #print "[INVKIN] "+str(pos[0])+", "+str(pos[1])+", "+str(pos[2])
    #print "[INVKIN] "+str(result)
    #if result != None:
    #    for i in range(0, 7):
    #        result[i] = math.radians(result[i])
        #result += angles
    #    baxter._arms[Defines.LEFT].set_joint_angle(result.tolist())

    #main.run()

    """
    while True:
        main.draw_arm_position("left")
        main.publish_rviz()
    """