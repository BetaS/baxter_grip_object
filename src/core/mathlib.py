import math as math
import numpy as np
import cv2

class Quaternion:
    def __init__(self, x, y, z, w):
        self._x = x
        self._y = y
        self._z = z
        self._w = w

    @classmethod
    def from_xyzw(cls, l):
        return Quaternion(l[0], l[1], l[2], l[3])

    @classmethod
    def from_wxyz(cls, l):
        return Quaternion(l[1], l[2], l[3], l[0])

    def __str__(self):
        return "w = "+str(self._w)+", x = "+str(self._x)+", y = "+str(self._y)+", z = "+str(self._z)

    def __mul__(self, q):
        w1, x1, y1, z1 = (self._w, self._x, self._y, self._z)
        w2, x2, y2, z2 = (q._w, q._x, q._y, q._z)
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

        return Quaternion(x, y, z, w)

    def to_euler(self):
        rx = math.atan2((2*(self._x*self._w+self._y*self._z)), 1-2*(self._x**2+self._y**2))
        ry = math.asin(2*(self._w*self._y-self._x*self._z))
        rz = math.atan2((2*(self._x*self._y+self._z*self._w)), 1-2*(self._y**2+self._z**2))

        return [rx, ry, rz]

    def to_euler_degree(self):
        r = self.to_euler()

        return [math.degrees(r[0]), math.degrees(r[1]), math.degrees(r[2])]

    def to_rotation_matrix(self, size=3):
        ret = np.identity(size, dtype=np.float32)

        xx = self._x * self._x
        xy = self._x * self._y
        xz = self._x * self._z
        xw = self._x * self._w

        yy = self._y * self._y
        yz = self._y * self._z
        yw = self._y * self._w

        zz = self._z * self._z
        zw = self._z * self._w

        m00 = 1 - 2 * ( yy + zz )
        m01 =     2 * ( xy - zw )
        m02 =     2 * ( xz + yw )

        m10 =     2 * ( xy + zw )
        m11 = 1 - 2 * ( xx + zz )
        m12 =     2 * ( yz - xw )

        m20 =     2 * ( xz - yw )
        m21 =     2 * ( yz + xw )
        m22 = 1 - 2 * ( xx + yy )

        ret[:3, :3] = np.array([[m00, m01, m02], [m10, m11, m12], [m20, m21, m22]], dtype=np.float32)

        return ret

    def distance(self, dist_x, dist_y, dist_z):
        rot = self.to_rotation_matrix()
        #rot = np.transpose(rot)
        return np.dot(rot, np.array([dist_x, dist_y, dist_z], dtype=np.float32))

    def decompose(self):
        return [self._x, self._y, self._z, self._w]

def sum_of_square(data):
    return sum(map(lambda x: x**2, data))

def var2(sum_of_square, sum, len):
    return (sum_of_square - sum**2 / len) / len

def var(data):
    return var2(sum_of_square(data), sum(data), len(data))*10

def unit_vector(vec):
    mag  = math.sqrt(vec[0]**2+vec[1]**2+vec[2]**2)
    return [vec[0]/mag, vec[1]/mag, vec[2]/mag]

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

def move_distance(x, y, z, rx, ry, rz, dist):
    pass

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


def line_intersect_skewed(l1_start, l1_end, l2_start, l2_end):
    s1 = l1_start
    s2 = l2_start
    v1 = unit_vector(l1_end-l1_start)
    v2 = unit_vector(l2_end-l2_start)

    r = np.array([0,0,0], dtype=np.float32)
    t = np.array([0,0,0], dtype=np.float32)

    m = np.identity(3, dtype=np.float32)

    # Precomputed values
    v1dotv2 = np.dot(v1, v2)
    v1p2 = np.dot(v1, v1)
    v2p2 = np.dot(v2, v2)

    # Solving matrix
    m[0][0] = -v2p2
    m[1][0] = -v1dotv2
    m[0][1] = v1dotv2
    m[1][1] = v1p2

    # Projected vector
    r[0] = np.dot((s2 - s1), v1)
    r[1] = np.dot((s2 - s1), v2)

    # precomputed value
    d = 1.0 / (v1dotv2 * v1dotv2 - v1p2 * v2p2)

    # Compute time values
    t = d * np.dot(m, r)

    # Compute intersected points on each lines
    p1 = s1 + np.dot(t[0], v1)
    p2 = s2 + np.dot(t[1], v2)

    return (p1+p2)/2, dist2(p1, p2)

"""
def line_intersect_point(l1_start, l1_end, l2_start, l2_end):
    p1, p2, p3, p4 = l1_start, l1_end, l2_start, l2_end

    u = p1 - p2
    v = p3 - p4
    w = p2 - p4

    a = np.dot(u,u)
    b = np.dot(u,v)
    c = np.dot(v,v)
    d = np.dot(u,w)
    e = np.dot(v,w)
    D = a*c - b*b
    sD = D
    tD = D

    SMALL_NUM = 0.00000001

    # compute the line parameters of the two closest points
    if D < SMALL_NUM:   # the lines are almost parallel
        sN = 0.0        # force using point P0 on segment S1
        sD = 1.0        # to prevent possible division by 0.0 later
        tN = e
        tD = c
    else:               # get the closest points on the infinite lines
        sN = (b*e - c*d)
        tN = (a*e - b*d)

        if sN < 0.0:    # sc < 0 => the s=0 edge is visible
            sN = 0.0
            tN = e
            tD = c
        elif sN > sD:   # sc > 1 => the s=1 edge is visible
            sN = sD
            tN = e + b
            tD = c

    if tN < 0.0:        # tc < 0 => the t=0 edge is visible
        tN = 0.0
        # recompute sc for this edge
        if -d < 0.0:
            sN = 0.0
        elif -d > a:
            sN = sD
        else:
            sN = -d
            sD = a

    elif tN > tD:       # tc > 1 => the t=1 edge is visible
        tN = tD
        # recompute sc for this edge
        if (-d + b) < 0.0:
            sN = 0
        elif (-d + b) > a:
            sN = sD
        else:
            sN = (-d + b)
            sD = a

    # finally do the division to get sc and tc
    if abs(sN) < SMALL_NUM:
        sc = 0.0
    else:
        sc = sN / sD

    if abs(tN) < SMALL_NUM:
        tc = 0.0
    else:
        tc = tN / tD

    # get the difference of the two closest points
    dP = w + (sc * u) - (tc * v)  # = S1(sc) - S2(tc)

    return ((p2+sc*u)+(p4+tc*v))/2

    #return [1, 1, 1]
"""

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

def dist2(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt((dx**2+dy**2+dz**2))

def center_point(l):
    return center_point2(l[0], l[1], l[2], l[3])

def center_point2(a, b, c, d):
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