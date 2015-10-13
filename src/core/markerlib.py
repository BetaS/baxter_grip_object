import cv2
import mathlib

MARKER_BLACK    = (0, 0, 0)
MARKER_RED      = (0, 0, 255)
MARKER_GREEN    = (0, 255, 0)
MARKER_BLUE     = (255, 0, 0)
MARKER_YELLOW   = (0, 255, 255)
MARKER_WHITE    = (255, 255, 255)

def MarkerDetection(hsv, edge):
    (cnts, _) = cv2.findContours(edge.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(cnts, key = lambda x: cv2.arcLength(x, True), reverse = True)
    squares = []

    # Detect square
    for c in cnts:
        # approximate the contour
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.1 * peri, True)

        # if our approximated contour has four points, then
        # we can assume that we have found our screen
        if len(approx) == 4 and peri < 400 and peri > 50:
            sq = mathlib.Square(approx, peri)
            if sq.center != None:
                squares.append(sq)

    markers = []
    # Refine square and Detecting pattern
    for base in squares:
        squares.remove(base)
        for square in squares:

            area = mathlib.math.fabs(base.area - square.area)
            dist = mathlib.dist(base.center, square.center)
            if dist < 5:
                # Remove same square
                squares.remove(square)

            if dist < 5 and area > 10:
                roi = hsv[(base.center[1]-1):(base.center[1]+1), (base.center[0]-1):(base.center[0]+1)]
                color = cv2.mean(roi)

                h = color[0]
                v = color[2]

                if v < 50:
                    code = MARKER_BLACK
                elif v > 200:
                    code = MARKER_WHITE
                else:
                    if h > 165:
                        code = MARKER_RED
                    elif h < 100:
                        code = MARKER_GREEN
                    else:
                        code = MARKER_BLUE


                markers.append({"bound": base.bound(), "pos": base.center, "color": code})

                break


    return markers