#!/usr/bin/env python

import time
from src.importer import *
from geometry_msgs.msg import PolygonStamped, Point32
from visualization_msgs.msg import MarkerArray

def nothing(x):
    pass

cv_window_name = "image"

if __name__ == '__main__':
    cv2.namedWindow(cv_window_name, 1)
    cv2.createTrackbar("dist", cv_window_name, 100, 1000, nothing)
    bridge = cv_bridge.CvBridge()

    baxter.start(False)

    width = 1280
    height = 800

    pub1 = rospy.Publisher("/polygon/camera_left", PolygonStamped, queue_size=10)
    pub2 = rospy.Publisher("/polygon/camera_left/rotation", PolygonStamped, queue_size=10)
    pub3 = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)

    #print baxter._left_arm.get_joint_angle()
    #print baxter._left_arm.get_spatial_jacobian()
    #pass

    print "Image Getting"
    while True:
        dist = cv2.getTrackbarPos("dist", cv_window_name)
    #    img = baxter.get_camera_image()
    #    mat = bridge.imgmsg_to_cv2(img, "bgr8")
        mat = cv2.imread("sample.png")

        # Otsu Threshold Method
        sub = cv2.cvtColor(mat, cv2.COLOR_BGR2YUV)
        hsv = cv2.cvtColor(mat, cv2.COLOR_BGR2HSV)

        # Find Camera Matrix
        pos = [0.79983, 1.0118, 0.28273]
        Cm = Baxter.Camera.camera_matrix()
        Cmi = np.linalg.pinv(Cm)

        # Find Real World Image Surface
        image_boundary = [np.array([0, 0, 1]), np.array([width-1, 0, 1]), np.array([width-1, height-1, 1]), np.array([0, height-1, 1])]
        world_boundary = []
        for p in image_boundary:
            p =  np.dot(Cmi, p)
            world_boundary.append(p)

        # Find Center Position of Image Plane
        cp = mathlib.center_point(world_boundary[0], world_boundary[1], world_boundary[2], world_boundary[3])

        # Rotation for Arbitary Axis
        # 1. Translate to Origin
        Tr = mathlib.translate_matrix(-cp[0], -cp[1], -cp[2])
        Tri = mathlib.translate_matrix(cp[0], cp[1], cp[2])

        # 2. Get Axis for Rotation
        mid1 = (world_boundary[0]+world_boundary[3])/2
        mid2 = (world_boundary[1]+world_boundary[2])/2
        axis = (mid2-mid1)[:3]

        # 3. Get Distance Angle
        normal = mathlib.unit_vector(np.cross((world_boundary[2]-world_boundary[0])[:3], (world_boundary[3]-world_boundary[1])[:3]))
        angle = np.dot(normal, np.array([0, 0, 1], dtype=np.float32))

        # 4. Get Transformation Matrix
        Rr = mathlib.rotation_matrix(axis, -angle+mathlib.math.radians(90))
        Tr = np.dot(np.dot(Tr, Rr), Tri)

        # 5. Display in RViz

        ### Origin Image Plane
        p = PolygonStamped()
        p.header.frame_id = "base"
        p.header.stamp = rospy.Time.now()

        for b in world_boundary:
            point = Point32()
            point.x = b[0]+pos[0]
            point.y = b[1]+pos[1]
            point.z = b[2]+pos[2]
            p.polygon.points.append( point )

        pub1.publish(p)

        ### Normal Image Plane
        p = PolygonStamped()
        p.header.frame_id = "base"
        p.header.stamp = rospy.Time.now()

        for b in world_boundary:
            pt = np.array([b[0], b[1], b[2], 1], dtype=np.float32)
            b = np.dot(Tr, pt)[:3]

            point = Point32()
            point.x = b[0]+pos[0]
            point.y = b[1]+pos[1]
            point.z = b[2]+pos[2]

            p.polygon.points.append( point )

        pub2.publish(p)

        ### Normal Vectors each Plane
        markers = MarkerArray()

        start = [cp[0]+pos[0], cp[1]+pos[1], cp[2]+pos[2]]
        end = [cp[0]+pos[0]+normal[0], cp[1]+pos[1]+normal[1], cp[2]+pos[2]+normal[2]]
        marker = rvizlib.create_arrow(0, start, end, [0, 1, 0])
        markers.markers.append(marker)

        end = [cp[0]+pos[0], cp[1]+pos[1], cp[2]+pos[2]+1]
        marker = rvizlib.create_arrow(1, start, end, [1, 0, 0])
        markers.markers.append(marker)

        pub3.publish(markers)

        """
        # Projection 2D -> 3D matrix
        A1 = np.array([
            [1, 0, -width/2],
            [0, 1, -height/2],
            [0, 0,    1],
            [0, 0,    0]], dtype=np.float32)

        # Composed rotation matrix with (RX,RY,RZ)
        alpha = (-141.303)*mathlib.math.pi/180
        beta = (-27.266)*mathlib.math.pi/180
        gamma = (-121.165)*mathlib.math.pi/180
        R = mathlib.eular_to_rotation_matrix2(alpha, beta, gamma)

        # Translation matrix on the Z axis change dist will change the height
        T = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, dist],
            [0, 0, 0, 1]], dtype=np.float32)

        # Camera Intrisecs matrix 3D -> 2D
        A2 = Baxter.Camera.intrinsic_matrix
        A2 = np.concatenate((A2, np.array([[0], [0], [0]])), axis=1)

        # Final and overall transformation matrix
        transfo = np.dot(A2, np.dot(T, np.dot(R,A1)))

        # Apply matrix transformation
        destination = cv2.warpPerspective(mat, transfo, (width, height), flags=cv2.INTER_CUBIC | cv2.WARP_INVERSE_MAP)

        cv2.imshow(cv_window_name, destination)
        cv2.waitKey(1)
        """
        """
        K = Baxter.Camera.intrinsic_matrix
        R = mathlib.eular_to_rotation_matrix(-2.466, -0.476, -2.116)
        #T = np.array([[0.365], [0.413], [0.791]], dtype=np.float32)
        #T = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,100], [0,0,0,1]], dtype=np.float32)
        #RT = np.concatenate((R, T), axis=1)

        #A = np.dot(K, RT)
        A = np.dot(K, R)
        Ainv = np.linalg.pinv(A)

        #Projection 2D -> 3D matrix
        #A = np.array([[1, 0, -width/2],
        #              [0, 1, -height/2],
        #              [0, 0, 0],
        #              [0, 0, 1]], dtype=np.float32)
        #print K
        #transfo = np.dot(K, np.dot(T, np.dot(R,A)))

        #A = np.dot(K, R)
        #Ainv = np.linalg.inv(A)

        # corners of the image, for here hard coded

        pixel_corners=[ np.array( c, dtype=np.float32 ) for c in [ (0,0,1), (0,height-1,1), (width-1,height-1,1), (width-1,0,1)] ]
        scene_corners=[]
        for c in pixel_corners:
            c = np.dot( Ainv, c )
            #c = c/(c[2])
            c = c*100

            scene_corners.append( (c[0], c[1]) )

        pixel_corners = [(pixel_corners[0][0], pixel_corners[0][1]),
                         (pixel_corners[1][0], pixel_corners[1][1]),
                         (pixel_corners[2][0], pixel_corners[2][1]),
                         (pixel_corners[3][0], pixel_corners[3][1])]

        scene_corners_sc = np.array(scene_corners, dtype=np.int)
        scene_corners_sc[:,0] += width/2
        scene_corners_sc[:,1] += height/2
        print scene_corners_sc

        P = cv2.getPerspectiveTransform(np.array(scene_corners), np.array(pixel_corners))
        #print P
        #print Ainv

        cv2.fillConvexPoly(mat, scene_corners_sc, (0, 0, 255))
        scale = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]], dtype=np.float32)
        P = np.dot(P, scale)

        mat = cv2.warpPerspective(mat, P, (width, height))


        cv2.imshow(cv_window_name, mat)
        cv2.waitKey(0)
        """

        """
        sub = sub[:,:,0]

        ret, sub = cv2.threshold(sub, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        edge = cv2.Canny(sub, 255, 255)

        markers = markerlib.MarkerDetection(hsv, edge)

        base_marker = None
        dest_marker = None
        # Draw marker
        for m in markers:
            cv2.fillConvexPoly(mat, np.array(m["bound"], np.int32), m["color"])
            if not base_marker and m["color"] == markerlib.MARKER_BLACK:
                base_marker = m
            if not dest_marker and m["color"] == markerlib.MARKER_RED:
                dest_marker = m

        print len(markers)

        if base_marker:
            pts = np.array(base_marker["bound"], dtype = "float32")

            # apply the four point tranform to obtain a "birds eye view" of
            # the image
            M = mathlib.four_point_transform(pts)
            warped = cv2.warpPerspective(mat, M, (960, 600), flags = cv2.INTER_CUBIC)

            cv2.imshow(cv_window_name, warped)

        #cv2.imshow(cv_window_name, ret)
        cv2.waitKey(1)
        """