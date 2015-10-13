import math as math
import numpy as np
import cv2

def sum_of_square(data):
    return sum(map(lambda x: x**2, data))

def var2(sum_of_square, sum, len):
    return (sum_of_square - sum**2 / len) / len

def var(data):
    return var2(sum_of_square(data), sum(data), len(data))*10

def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1]) #Typo was here

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y

def dist(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.sqrt((dx**2+dy**2))

def order_points(pts):
    # initialzie a list of coordinates that will be ordered
    # such that the first entry in the list is the top-left,
    # the second entry is the top-right, the third is the
    # bottom-right, and the fourth is the bottom-left
    rect = np.zeros((4, 2), dtype = "float32")

    # the top-left point will have the smallest sum, whereas
    # the bottom-right point will have the largest sum
    s = pts.sum(axis = 1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]

    # now, compute the difference between the points, the
    # top-right point will have the smallest difference,
    # whereas the bottom-left will have the largest difference
    diff = np.diff(pts, axis = 1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]

    # return the ordered coordinates
    return rect

def four_point_transform(pts):
    # obtain a consistent order of the points and unpack them
    # individually
    rect = order_points(pts)
    (tl, tr, br, bl) = rect

    # compute the width of the new image, which will be the
    # maximum distance between bottom-right and bottom-left
    # x-coordiates or the top-right and top-left x-coordinates
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))

    # compute the height of the new image, which will be the
    # maximum distance between the top-right and bottom-right
    # y-coordinates or the top-left and bottom-left y-coordinates
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))

    # now that we have the dimensions of the new image, construct
    # the set of destination points to obtain a "birds eye view",
    # (i.e. top-down view) of the image, again specifying points
    # in the top-left, top-right, bottom-right, and bottom-left
    # order
    """
    dst = np.array([
        [tl[0], tl[1]],
        [tl[0] + maxWidth - 1, tl[1]],
        [tl[0] + maxWidth - 1, tl[1] + maxHeight - 1],
        [tl[0], tl[1] + maxHeight - 1]], dtype = "float32")
    """
    dst = np.array([
        [tl[0], tl[1]],
        [tl[0] + 50, tl[1]],
        [tl[0] + 50, tl[1] +50],
        [tl[0], tl[1] + 50]], dtype = "float32")

    """
    dst = np.array([
        [0, 0],
        [959, 0],
        [959, 599],
        [0, 599]], dtype = "float32")
    """
    # compute the perspective transform matrix and then apply it
    M = cv2.getPerspectiveTransform(rect, dst)
    #M, _ = cv2.findHomography(rect, dst, cv2.RANSAC, 10.0)

    # return the warped image
    return M

class Square:
    def __init__(self, rect, area):
        self.a = rect[0][0]
        self.b = rect[1][0]
        self.c = rect[2][0]
        self.d = rect[3][0]

        self.area = area

        self.center = None
        try:
            self.center = line_intersection([self.a, self.c], [self.b, self.d])
        except:
            pass

    def draw(self, ret, border = True, cross = True, center = True, desc = True):
        if border:
            cv2.line(ret, tuple(self.a), tuple(self.b), (0, 255, 0), 1)
            cv2.line(ret, tuple(self.b), tuple(self.c), (0, 255, 0), 1)
            cv2.line(ret, tuple(self.c), tuple(self.d), (0, 255, 0), 1)
            cv2.line(ret, tuple(self.d), tuple(self.a), (0, 255, 0), 1)

        if cross:
            cv2.line(ret, tuple(self.d), tuple(self.b), (0, 0, 255), 1)
            cv2.line(ret, tuple(self.a), tuple(self.c), (0, 0, 255), 1)

        if center and self.center:
            cv2.circle(ret, tuple(self.center), 3, (0, 0, 255), -1)

    def bound(self):
        return tuple(self.a), tuple(self.b), tuple(self.c), tuple(self.d)