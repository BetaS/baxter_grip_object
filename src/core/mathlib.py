import math as math
import numpy as np
import cv2

def sum_of_square(data):
    return sum(map(lambda x: x**2, data))

def var2(sum_of_square, sum, len):
    return (sum_of_square - sum**2 / len) / len

def var(data):
    return var2(sum_of_square(data), sum(data), len(data))*10

def unit_vector(vec):
    mag  = math.sqrt(vec[0]**2+vec[1]**2+vec[2]**2)
    return [vec[0]/mag, vec[1]/mag, vec[2]/mag]

def quat_to_rotation_matrix(X, Y, Z, W):
    xx = X * X;
    xy = X * Y;
    xz = X * Z;
    xw = X * W;

    yy = Y * Y;
    yz = Y * Z;
    yw = Y * W;

    zz = Z * Z;
    zw = Z * W;

    m00 = 1 - 2 * ( yy + zz );
    m01 =     2 * ( xy - zw );
    m02 =     2 * ( xz + yw );

    m10 =     2 * ( xy + zw );
    m11 = 1 - 2 * ( xx + zz );
    m12 =     2 * ( yz - xw );

    m20 =     2 * ( xz - yw );
    m21 =     2 * ( yz + xw );
    m22 = 1 - 2 * ( xx + yy );

    return np.array([[m00, m01, m02], [m10, m11, m12], [m20, m21, m22]], dtype=np.float32)

def eular_to_rotation_matrix(R, P, Y):
    m00 = math.cos(R) * math.cos(P)
    m01 = math.cos(R) * math.sin(P) * math.sin(Y) - math.sin(R) * math.cos(Y)
    m02 = math.cos(R) * math.sin(P) * math.cos(Y) + math.sin(R) * math.sin(Y)

    m10 = math.sin(R) * math.cos(P)
    m11 = math.sin(R) * math.sin(P) * math.sin(Y) + math.cos(R) * math.cos(Y)
    m12 = math.sin(R) * math.sin(P) * math.cos(Y) - math.cos(R) * math.sin(Y)

    m20 = -math.sin(P)
    m21 = math.cos(P)*math.sin(Y)
    m22 = math.cos(P)*math.cos(Y)

    return np.array([[m00, m01, m02], [m10, m11, m12], [m20, m21, m22]], dtype=np.float32)

def eular_to_rotation_matrix2(R, P, Y):
    RX = np.array([
        [1,          0,           0, 0],
        [0, math.cos(R), -math.sin(R), 0],
        [0, math.sin(R),  math.cos(R), 0],
        [0,          0,           0, 1]], dtype=np.float32)

    RY = np.array([
        [math.cos(P), 0, -math.sin(P), 0],
        [        0, 1,          0, 0],
        [math.sin(P), 0,  math.cos(P), 0],
        [        0, 0,          0, 1]], dtype=np.float32)

    RZ = np.array([
        [math.cos(Y), -math.sin(Y), 0, 0],
        [math.sin(Y),  math.cos(Y), 0, 0],
        [0,            0,           1, 0],
        [0,            0,           0, 1]], dtype=np.float32)

    return np.dot(np.dot(RX,RY),RZ)

def translate_matrix(x, y, z):
    return np.array([
        [1,         0,      0,      -x],
        [0,         1,      0,      -y],
        [0,         0,      1,      -z],
        [0,         0,      0,      1]], dtype=np.float32)

def eular_to_vector(R, P, Y):
    x = math.cos(Y)*math.cos(P)
    y = math.sin(Y)*math.cos(P)
    z = math.sin(P)

    return [x,y,z]

def quat_to_rad(X, Y, Z, W):
    rx = math.atan2((2*(X*Y+Z*W)), 1-2*(Y**2+Z**2))
    ry = math.asin(2*(X*Z-W*Y))
    rz = math.atan2((2*(X*W+Y*Z)), 1-2*(Z**2+W**2))

    return [rx, ry, rz]
"""
def get_extrinsic_matrix(x, y, z, rx, ry, rz):
    R = eular_to_rotation_matrix(rx, ry, rz)
    R = np.transpose(R)
    C = np.array([x, y, z], dtype=np.float32)
    T = -np.dot(R, C)
    return np.array(
           [[R[0][0], R[0][1], R[0][2], T[0]],
            [R[1][0], R[1][1], R[1][2], T[1]],
            [R[2][0], R[2][1], R[2][2], T[2]]], dtype=np.float32)

def get_extrinsic_matrix2(x, y, z, rx, ry, rz, rw):
    R = quat_to_rotation_matrix(rx, ry, rz, rw)
    R = np.transpose(R)
    C = np.array([x, y, z], dtype=np.float32)
    T = -np.dot(R, C)
    return np.array(
           [[R[0][0], R[0][1], R[0][2], T[0]],
            [R[1][0], R[1][1], R[1][2], T[1]],
            [R[2][0], R[2][1], R[2][2], T[2]]], dtype=np.float32)
"""
def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    theta = np.asarray(theta)
    axis = axis/math.sqrt(np.dot(axis, axis))
    a = math.cos(theta/2)
    b, c, d = -axis*math.sin(theta/2)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac), 0],
                     [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab), 0],
                     [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc, 0],
                     [0, 0, 0, 1]])

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

def center_point(a, b, c, d):
    cx = (a[0]+b[0]+c[0]+d[0])/4
    cy = (a[1]+b[1]+c[1]+d[1])/4
    cz = (a[2]+b[2]+c[2]+d[2])/4

    return [cx, cy, cz]

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