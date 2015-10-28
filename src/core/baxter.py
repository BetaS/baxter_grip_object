from src.importer import rospy, mathlib, baxter_interface, CHECK_VERSION, Defines

from sensor_msgs.msg import (
    Image,
)

import roslib, rosmsg

import numpy as np

import argparse

class Baxter:
    class Arm:
        joint_num = 7
        link_l = np.array([0.2790163480873477, 0.102, 0.2713397434951246, 0.10359, 0.2908719477708361, 0.11597], dtype=np.float32)
        omegas = np.transpose(np.array(
            [[0,0,1],
             [-1,0,0],
             [0,1,0],
             [-1,0,0],
             [0,1,0],
             [-1,0,0],
             [0,1,0]], dtype=np.float32))

        def __init__(self, name):
            self._limb = baxter_interface.Limb(name)

        def get_joint_angle(self):
            angles = self._limb.joint_angles()
            return np.array([angles["left_s0"], angles["left_s1"], angles["left_e0"], angles["left_e1"], angles["left_w0"], angles["left_w1"], angles["left_w2"]], dtype=np.float32)
            #return np.array([0, 0, 0, 0, 0, 0, 0], dtype=np.float32)

        def get_arm_position(self):
            P = np.zeros((6, 3))
            P[0] = [0,0,self.link_l[0]]
            P[1] = [0,self.link_l[1],self.link_l[0]]
            P[2] = [0,self.link_l[2]+self.link_l[1],self.link_l[0]]
            P[3] = [0,self.link_l[3]+self.link_l[2]+self.link_l[1],self.link_l[0]]
            P[4] = [0,self.link_l[4]+self.link_l[3]+self.link_l[2]+self.link_l[1],self.link_l[0]]
            P[5] = [0,self.link_l[5]+self.link_l[4]+self.link_l[3]+self.link_l[2]+self.link_l[1],self.link_l[0]]
            return np.transpose(P)

        def rotation(self, w, theta):
            # Rotations : Rodrigues formula
            e_w_theta = []
            for k in range(self.joint_num):
                e_w_theta.append(np.identity(3, dtype=np.float32))
                e_w_theta[k] += self.skew(w[:,k])*mathlib.math.sin(theta[k])
                e_w_theta[k] += (self.skew(w[:,k])**2)*(1-mathlib.math.cos(theta[k]))

            return e_w_theta

        def exponential_mapping(self, theta):
            # Position vector p(theta) : Exponential mapping
            P = self.get_arm_position()
            w = self.omegas
            e_w_theta = self.rotation(w, theta)

            q = np.zeros((3,7), dtype=np.float32)
            v1 = -np.cross(w[:,0],P[:,0])
            q[:,0] = np.dot((np.identity(3, dtype=np.float32) - e_w_theta[0]), np.cross(w[:,0],v1))+np.dot(np.dot(np.dot(w[:,0],np.transpose(w[:,0])),v1),theta[0])

            for j in range(1, self.joint_num):
                v = -np.cross(w[:,j], P[:,j-1])
                q[:,j] = np.dot((np.identity(3, dtype=np.float32)-e_w_theta[j]),np.cross(w[:,j],v))+np.dot(np.dot(np.dot(w[:,j],np.transpose(w[:,j])),v),theta[j])

            e_xi_theta = []
            for a in range(self.joint_num):
                e_xi_theta.append(np.identity((4), dtype=np.float32))
                e_xi_theta[a][:3,:3] = e_w_theta[a]
                e_xi_theta[a][3,:3] = q[:,a]

            return e_xi_theta

        def adjoint_transform(self, theta):
            # joint_num is not necessarily 7
            # It is not affected by link_l ?
            # Rotation Matrix
            e_xi_theta = self.exponential_mapping(theta);

            # products of e_xi_theta : (4x4)-matrix
            R =np.identity(4, dtype=np.float32)
            for i in range(self.joint_num-1):
                R = e_xi_theta[i] * R


            #Adjoint Transformation "Ad" : (6x6)-matrix

            Ad = np.identity(6, dtype=np.float32)
            Ad[0:3,0:3] = R[:3,:3]
            Ad[3:6,0:3] = np.zeros((3,3), dtype=np.float32)
            skew_R = self.skew(R[:3, 3])
            Ad[0:3,3:6] = np.dot(skew_R, R[:3, :3])
            Ad[3:6,3:6] = R[:3,:3]
            return Ad

        def skew(self, vec):
            # gives the skew symmetric matrix
            return  np.array([[0, -vec[2], vec[1]],
                              [vec[2], 0, -vec[0]],
                              [-vec[1], vec[0], 0]], dtype=np.float32)

        def get_twist(self, P, w):
            xi = np.zeros((7, 6))

            xi[0] = np.concatenate((-1*np.cross(w[:,0],P[:,0]), w[:,0]), axis=0)
            for i in range(1, self.joint_num):
                xi[i] = np.concatenate((-1*np.cross(w[:,i], P[:,i-1]), w[:,i]), axis=0)

            return np.transpose(xi)

        def get_xi_prime(self, xi):
            xi_prime = np.zeros((6, 7))

            for i in range(self.joint_num):
                for j in range(i):
                    if i == 1:
                        xi_prime[:,i-1] = xi[:,i-1]
                    else:
                        xi_prime[:,i-1] = np.dot(self.adjoint_transform(self.get_joint_angle()), xi[:,i-1])

            return xi_prime

        def get_spatial_jacobian(self):
            # Position of each joint : (3x6)-matrix
            # For Forward Kinematics, we need to form Jacobian for Baxter's arm.
            # J is Spatial Jacobian, which is a (6x7)-matrix.
            # Spatial Jacobian is for the world frame( universal coordinate).
            # 6 is for (x,y,z,roll,pitch,yaw) (-> if we use Quaternion, then is it 7??)
            # joint_num(Number of Joints in Baxter's arm) is 7.
            # link_l 6x1 vector of link lengths
            # theta 7x1 vector of angles

            P = self.get_arm_position()

            # Omegas : (3x7)-matrix
            w = self.omegas

            # Twist : (6x7)-matrix
            xi = self.get_twist(P, w)

            # For making Jacobian, we need xi_prime : (6x7)-matrix
            xi_prime = self.get_xi_prime(xi)

            # J : (6x7)-matrix
            J = np.zeros((6, 7))
            for k in range(self.joint_num):
                 J[:,k] = xi_prime[:,k]

            return J

        def get_end_effector_pos(self):
            pose = self._limb.endpoint_pose()
            pos = pose["position"]
            rot = pose["orientation"]
            return [pos.x, pos.y, pos.z], [rot.x, rot.y, rot.z, rot.w]

        def get_end_effector_pos2(self):
            pose = np.dot(self.get_spatial_jacobian(), self.get_joint_angle())
            return pose[:3], pose[3:6]

        def get_camera_pos(self):
            pos, rot = self.get_end_effector_pos()
            q = mathlib.Quaternion(rot[0], rot[1], rot[2], rot[3])
            pos = pos+q.distance(0.03825, 0.012, 0.015355)
            rot = q*mathlib.Quaternion(0.000, 0.000, -0.707, 0.707)
            rot = rot.decompose()
            return pos, rot

    class Camera:
        distort_matrix = np.array(
            [0.0203330914024, -0.0531389002992, 0.000622878864307, -0.00225405996481, 0.0137897514515],
            np.float32)

        focal_length = 405.792519532
        frame_width = 666.049300318
        frame_height = 397.038941784

        intrinsic_matrix = np.array([
            [focal_length,  0.0,            frame_width],
            [0.0,           focal_length,   frame_height],
            [0.0,           0.0,            1.0          ]], np.float32)

        @classmethod
        def extrinsic_matrix(cls, pos, rot):
            R = mathlib.Quaternion(rot[0], rot[1], rot[2], rot[3]).to_rotation_matrix()
            R = np.transpose(R)
            T = np.array([-pos[0], -pos[1], -pos[2]], dtype=np.float32)
            T = np.dot(R, T)
            return np.array(
                   [[R[0][0], R[0][1], R[0][2], T[0]],
                    [R[1][0], R[1][1], R[1][2], T[1]],
                    [R[2][0], R[2][1], R[2][2], T[2]]], dtype=np.float32)

        @classmethod
        def camera_matrix(cls, pos, rot):
            In = cls.intrinsic_matrix
            Ex = cls.extrinsic_matrix(pos, rot)

            Cm = np.dot(In, Ex)
            return Cm

        @classmethod
        def inv_camera_matrix(cls, pos, rot):
            In = np.linalg.inv(cls.intrinsic_matrix)
            Ex = np.linalg.pinv(cls.extrinsic_matrix(pos, rot))

            Cmi = np.dot(Ex, In)
            return Cmi

        @classmethod
        def find_boundary(cls, pos, rot):
            Cm = np.linalg.inv(cls.intrinsic_matrix)
            qrot = mathlib.Quaternion.from_xyzw(rot)
            width = 1280
            height = 800

            image_boundary = [np.array([0, 0, 1]), np.array([width-1, 0, 1]), np.array([width-1, height-1, 1]), np.array([0, height-1, 1])]

            boundary = []
            for p in image_boundary:
                p =  np.dot(Cm, p)
                p = qrot.distance(p[0], p[1], p[2])
                p += pos
                boundary.append(p)


            center = np.dot(Cm, np.array([cls.frame_width, cls.frame_height, 1], dtype=np.float32))
            center = qrot.distance(center[0], center[1], center[2])
            center += pos

            return center, boundary


    def __init__(self, name="pymodules"):
        arg_fmt = argparse.RawDescriptionHelpFormatter
        parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                         description=__doc__)
        parser.parse_args(rospy.myargv()[1:])

        self._name = "baxter_"+name
        self._head_image = None

    def start(self, first=False):

        print("Init ROS Node... ["+self._name+"]")
        rospy.init_node(self._name)

        self._head = baxter_interface.Head()
        #self._hand_cam[LEFT] = baxter_interface.CameraController("left_hand_camera")
        #self._hand_cam[RIGHT] = baxter_interface.CameraController("right_hand_camera")
        #self._head_cam = rospy.Subscriber("/cameras/head_camera/image", Image)

        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        self._rate = rospy.Rate(30)

        print("Enabling robot... ")
        self._rs.enable()

        print("Initializing camera...")
        camera_mapper = ["left_hand_camera", "right_hand_camera"]
        self._camera = {}

        for idx in Defines.ARMS:
            camera_name = idx+"_hand_camera"
            camera = None
            if first:
                camera = baxter_interface.CameraController(camera_name)
                camera.resolution = (640, 400)
                camera.fps = 30
                camera.exposure = 100
                camera.gain = 0

                print("Open camera...")
                camera.open()

            sub = rospy.Subscriber("/cameras/"+camera_name+"/image", Image, lambda x: self.republish_camera(idx, x), None, 1)
            self._camera[idx] = {"name": camera_name, "camera": camera, "sub": sub, "image": None}

        print("Initializing robot...")
        self._display = rospy.Publisher('/robot/xdisplay', Image, queue_size=10)
        self._arms = {}
        self._arms[Defines.LEFT] = Baxter.Arm("left")
        self._arms[Defines.RIGHT] = Baxter.Arm("right")

        self.init()

        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        """
        Exits example cleanly by moving head to neutral position and
        maintaining start state
        """
        print("\nExiting...")
        for idx in self._camera:
            self._camera[idx]["sub"].unregister()

        self.init()

        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()

    def init(self):
        self.head_angle(0)

    def head_angle(self, angle=0.0):
        self._head.set_pan(angle)

    def get_camera_image(self, type=0, wait=True):
        while wait and self._camera_image == None:
            self._rate.sleep()

        return self._camera_image

    def republish_camera(self, idx, msg):
        self._camera[idx]["image"] = msg

    def display(self, img):
        self._display.publish(img)
