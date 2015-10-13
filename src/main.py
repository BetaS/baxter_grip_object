#!/usr/bin/env python

import time
from src.importer import *

def nothing(x):
    pass

cv_window_name = "image"

if __name__ == '__main__':
    cv2.namedWindow(cv_window_name, 1)
    bridge = cv_bridge.CvBridge()

    baxter.start(False)

    time.sleep(5)
    """
    print "Image Getting"
    while True:
        thr = 80#cv2.getTrackbarPos('thr',cv_window_name)

        img = baxter.get_head_image()
        mat = bridge.imgmsg_to_cv2(img, "bgr8")

        var = cv2.inRange(mat, (thr, thr, thr), (255, 255, 255))

        cv2.imshow(cv_window_name, var)
        cv2.waitKey(1)

    pass
    """
    print "Image Getting"
    while True:
        img = baxter.get_head_image()
        mat = bridge.imgmsg_to_cv2(img, "bgr8")

        # Otsu Threshold Method
        sub = cv2.cvtColor(mat, cv2.COLOR_BGR2YUV)
        hsv = cv2.cvtColor(mat, cv2.COLOR_BGR2HSV)
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


baxter.get_jacobian()