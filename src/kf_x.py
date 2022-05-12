#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv2.aruco as aruco
import math
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Empty as EmptyMsg
from sensor_msgs.msg import Imu as ImuMsg
import tf
import math
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

rospy.init_node('kf_x', anonymous=True)

rate = rospy.Rate(15)

# small_marker = 0.0

# my_namespace=rospy.get_namespace()

###########################
# past_t = time.time()
###########################

# pub_pose_kf = rospy.Publisher('/tello_pose_kf', Twist, queue_size=10)

# x, y, z, and yaw
# x, y, z in meter; yaw in degrees
tello_pose_kf = np.zeros((13,), dtype=np.float32)

##################################################################
p_x = np.zeros((9,), dtype=np.float32)
p_y = np.zeros((9,), dtype=np.float32)
p_z = np.zeros((9,), dtype=np.float32)
p_yaw = np.zeros((4,), dtype=np.float32)

k_x = np.zeros((3,), dtype=np.float32)
k_y = np.zeros((3,), dtype=np.float32)
k_z = np.zeros((3,), dtype=np.float32)
k_yaw = np.zeros((2,), dtype=np.float32)
##################################################################

# temp_imu = np.zeros((6,), dtype=np.float32)

pub_pose_kf = rospy.Publisher('tello_pose_kf', numpy_msg(Floats), queue_size=10)

##################################################################
pub_p_x = rospy.Publisher('p_x', numpy_msg(Floats), queue_size=10)
pub_p_y = rospy.Publisher('p_y', numpy_msg(Floats), queue_size=10)
pub_p_z = rospy.Publisher('p_z', numpy_msg(Floats), queue_size=10)
pub_p_yaw = rospy.Publisher('p_yaw', numpy_msg(Floats), queue_size=10)

pub_k_x = rospy.Publisher('k_x', numpy_msg(Floats), queue_size=10)
pub_k_y = rospy.Publisher('k_y', numpy_msg(Floats), queue_size=10)
pub_k_z = rospy.Publisher('k_z', numpy_msg(Floats), queue_size=10)
pub_k_yaw = rospy.Publisher('k_yaw', numpy_msg(Floats), queue_size=10)
##################################################################

##################################################################
K_position = np.array([0.0, 0.0, 0.0], dtype = np.float32).reshape(3, 1)
K_yaw = np.array([0.0, 0.0], dtype = np.float32).reshape(2, 1)
##################################################################

dt = 1.0/15

#####################################
         # for x, y, and z #
#####################################
# A = np.array([[1.0, dt, dt], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], dtype = np.float32)
A = np.array([[1.0, dt, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], dtype = np.float32)
C_position = np.array([1.0, 0.0, 0.0], dtype = np.float32).reshape(1, 3)
C_velocity = np.array([0.0, 1.0, 1.0], dtype = np.float32).reshape(1, 3)

# Q_x = 0.1 * np.identity(3)
Q_x_pos = 0.01
Q_x_vel = 0.01
Q_x_vel_drift = 0.0001
Q_x = np.array([[Q_x_pos, 0.0, 0.0], [0.0, Q_x_vel, 0.0], [0.0, 0.0, Q_x_vel_drift]], dtype = np.float32)

R_position_x = 1.5
R_velocity_x = 1

# Q_y = 0.1 * np.identity(3)
Q_y_pos = 0.01
Q_y_vel = 0.01
Q_y_vel_drift = 0.0001
Q_y = np.array([[Q_y_pos, 0.0, 0.0], [0.0, Q_y_vel, 0.0], [0.0, 0.0, Q_y_vel_drift]], dtype = np.float32)

R_position_y = 1
R_velocity_y = 1

# Q_z = 0.1 * np.identity(3)
Q_z_pos = 0.001
Q_z_vel = 0.001
Q_z_vel_drift = 0.0000001
Q_z = np.array([[Q_z_pos, 0.0, 0.0], [0.0, Q_z_vel, 0.0], [0.0, 0.0, Q_z_vel_drift]], dtype = np.float32)

R_position_z = 1.5
R_velocity_z = 1

P = 0.2 * np.identity(3)
X = np.zeros((3, 1), dtype = np.float32)

#####################################
             # for yaw #
#####################################
A_yaw = np.array([[1.0, dt], [0.0, 1.0]], dtype = np.float32)
C_yaw = np.array([1.0, 0.0], dtype = np.float32).reshape(1, 2)
Q_yaw = 0.1 * np.identity(2)
R_yaw_marker = 0.2
R_yaw_imu = 0.5

P_yaw = 0.2 * np.identity(2)
X_yaw = np.zeros((2, 1), dtype = np.float32)

class Kalman_filter:

    def __init__(self, Q, R_position, R_velocity, P, X, K):
        self.Q = Q
        self.R_position = R_position
        self.R_velocity = R_velocity
        self.P = P
        self.X = X
        self.K = K

    def update_v(self, measure_v):
        temp = self.P.dot(C_velocity.T)
        self.K = temp / (C_velocity.dot(self.P.dot(C_velocity.T)) + self.R_velocity)
        self.X = self.X + self.K * (measure_v - C_velocity.dot(self.X))
        I = np.identity(3)
        temp = I - self.K.dot(C_velocity)
        self.P = temp.dot(self.P)

    def update_p(self, measure_p):
        # now_t = time.time()
        # if ((now_t - past_t) < 20 or (now_t - past_t) > 25):
        temp = self.P.dot(C_position.T)
        self.K = temp / (C_position.dot(self.P.dot(C_position.T)) + self.R_position)
        self.X = self.X + self.K * (measure_p - C_position.dot(self.X))
        I = np.identity(3)
        temp = I - self.K.dot(C_position)
        self.P = temp.dot(self.P)

    def predict(self):
        self.X = A.dot(self.X)
        self.P = A.dot(self.P.dot(A.T)) + self.Q

    def correction(self):
        self.P[0, 1] = 0
        self.P[0, 2] = 0
        self.P[1, 0] = 0
        self.P[1, 2] = 0
        self.P[2, 0] = 0
        self.P[2, 1] = 0

class Kalman_filter_yaw:

    def __init__(self, Q_yaw, R_yaw_marker, R_yaw_imu, P_yaw, X_yaw, K_yaw):
        self.Q_yaw = Q_yaw
        self.R_yaw_marker = R_yaw_marker
        self.R_yaw_imu = R_yaw_imu
        self.P_yaw = P_yaw
        self.X_yaw = X_yaw
        self.K_yaw = K_yaw

    def update_yaw_imu(self, measure_yaw_imu):
        temp = self.P_yaw.dot(C_yaw.T)
        self.K_yaw = temp / (C_yaw.dot(self.P_yaw.dot(C_yaw.T)) + self.R_yaw_imu)
        self.X_yaw = self.X_yaw + self.K_yaw * (measure_yaw_imu - C_yaw.dot(self.X_yaw))
        I = np.identity(2)
        temp = I - self.K_yaw.dot(C_yaw)
        self.P_yaw = temp.dot(self.P_yaw)

    def update_yaw_marker(self, measure_yaw_marker):
        temp = self.P_yaw.dot(C_yaw.T)
        self.K_yaw = temp / (C_yaw.dot(self.P_yaw.dot(C_yaw.T)) + self.R_yaw_marker)
        self.X_yaw = self.X_yaw + self.K_yaw * (measure_yaw_marker - C_yaw.dot(self.X_yaw))
        I = np.identity(2)
        temp = I - self.K_yaw.dot(C_yaw)
        self.P_yaw = temp.dot(self.P_yaw)

    def predict_yaw(self):
        self.X_yaw = A_yaw.dot(self.X_yaw)
        self.P_yaw = A_yaw.dot(self.P_yaw.dot(A_yaw.T)) + self.Q_yaw

    def correction_yaw(self):
        self.P_yaw[0, 1] = 0
        self.P_yaw[1, 0] = 0

drone_x = Kalman_filter(Q_x, R_position_x, R_velocity_x, P, X, K_position)
drone_y = Kalman_filter(Q_y, R_position_y, R_velocity_y, P, X, K_position)
drone_z = Kalman_filter(Q_z, R_position_z, R_velocity_z, P, X, K_position)
drone_yaw = Kalman_filter_yaw(Q_yaw, R_yaw_marker, R_yaw_imu, P_yaw, X_yaw, K_yaw)

def get_imu_message(imu_msg):
    global drone_x, drone_y, drone_z, drone_yaw
    # global temp_imu
    temp_imu = imu_msg.data

    # drone_x.correction()
    # drone_y.correction()
    # drone_z.correction()
    # drone_yaw.correction_yaw()
    drone_x.update_v(temp_imu[0])
    drone_y.update_v(temp_imu[1])
    drone_z.update_v(temp_imu[2])
    drone_yaw.update_yaw_imu(temp_imu[3])

def get_marker_message(marker_msg):
    global drone_x, drone_y, drone_z, drone_yaw
    temp = marker_msg.data

    # drone_x.correction()
    # drone_y.correction()
    # drone_z.correction()
    # drone_yaw.correction_yaw()
    drone_x.update_p(temp[0])
    drone_y.update_p(temp[1])
    drone_z.update_p(temp[2])
    drone_yaw.update_yaw_marker(temp[4])

# def get_marker_lp_message(marker_lp_msg):
#     global drone_x, drone_y, drone_z, drone_yaw
#     temp = marker_lp_msg.data

#     if my_namespace=="/drone2/":
#         if small_marker == 1.0:
#             drone_x.correction()
#             drone_y.correction()
#             drone_z.correction()
#             drone_yaw.correction_yaw()
#             drone_x.update_p(temp[0])
#             drone_y.update_p(temp[1])
#             drone_z.update_p(temp[2])
#             drone_yaw.update_yaw_marker(temp[4])

rospy.Subscriber("/drone2/repub_imu", numpy_msg(Floats), callback=get_imu_message)
rospy.Subscriber("/drone2/tello_pose_marker", numpy_msg(Floats), callback=get_marker_message)
# rospy.Subscriber("tello_pose_marker_lp", numpy_msg(Floats), callback=get_marker_lp_message)

while not rospy.is_shutdown():
    # drone_x.correction()
    # drone_y.correction()
    # drone_z.correction()
    # drone_yaw.correction_yaw()
    drone_x.predict()
    drone_y.predict()
    drone_z.predict()
    drone_yaw.predict_yaw()
    tello_pose_kf[0] = drone_x.X[0, 0]
    tello_pose_kf[1] = drone_y.X[0, 0]
    tello_pose_kf[2] = drone_z.X[0, 0]
    tello_pose_kf[3] = drone_yaw.X_yaw[0, 0]
    tello_pose_kf[4] = drone_x.X[1, 0] # x velocity
    tello_pose_kf[5] = drone_x.X[2, 0] # x velocity drift
    tello_pose_kf[6] = drone_y.X[1, 0] # y velocity
    tello_pose_kf[7] = drone_y.X[2, 0] # y velocity drift
    tello_pose_kf[8] = drone_z.X[1, 0] # z velocity
    tello_pose_kf[9] = drone_z.X[2, 0] # z velocity drift
    # tello_pose_kf[10] = temp_imu[0]
    # tello_pose_kf[11] = temp_imu[1]
    # tello_pose_kf[12] = temp_imu[2]

    p_x[0] = drone_x.P[0, 0]
    p_x[1] = drone_x.P[0, 1]
    p_x[2] = drone_x.P[0, 2]
    p_x[3] = drone_x.P[1, 0]
    p_x[4] = drone_x.P[1, 1]
    p_x[5] = drone_x.P[1, 2]
    p_x[6] = drone_x.P[2, 0]
    p_x[7] = drone_x.P[2, 1]
    p_x[8] = drone_x.P[2, 2]

    p_y[0] = drone_y.P[0, 0]
    p_y[1] = drone_y.P[0, 1]
    p_y[2] = drone_y.P[0, 2]
    p_y[3] = drone_y.P[1, 0]
    p_y[4] = drone_y.P[1, 1]
    p_y[5] = drone_y.P[1, 2]
    p_y[6] = drone_y.P[2, 0]
    p_y[7] = drone_y.P[2, 1]
    p_y[8] = drone_y.P[2, 2]

    p_z[0] = drone_z.P[0, 0]
    p_z[1] = drone_z.P[0, 1]
    p_z[2] = drone_z.P[0, 2]
    p_z[3] = drone_z.P[1, 0]
    p_z[4] = drone_z.P[1, 1]
    p_z[5] = drone_z.P[1, 2]
    p_z[6] = drone_z.P[2, 0]
    p_z[7] = drone_z.P[2, 1]
    p_z[8] = drone_z.P[2, 2]

    p_yaw[0] = drone_yaw.P_yaw[0, 0]
    p_yaw[1] = drone_yaw.P_yaw[0, 1]
    p_yaw[2] = drone_yaw.P_yaw[1, 0]
    p_yaw[3] = drone_yaw.P_yaw[1, 1]

    k_x[0] = drone_x.K[0, 0]
    k_x[1] = drone_x.K[1, 0]
    k_x[2] = drone_x.K[2, 0]

    k_y[0] = drone_y.K[0, 0]
    k_y[1] = drone_y.K[1, 0]
    k_y[2] = drone_y.K[2, 0]

    k_z[0] = drone_z.K[0, 0]
    k_z[1] = drone_z.K[1, 0]
    k_z[2] = drone_z.K[2, 0]

    k_yaw[0] = drone_yaw.K_yaw[0, 0]
    k_yaw[1] = drone_yaw.K_yaw[1, 0]

    pub_p_x.publish(p_x)
    pub_p_y.publish(p_y)
    pub_p_z.publish(p_z)
    pub_p_yaw.publish(p_yaw)

    pub_k_x.publish(k_x)
    pub_k_y.publish(k_y)
    pub_k_z.publish(k_z)
    pub_k_yaw.publish(k_yaw)

    pub_pose_kf.publish(tello_pose_kf)
    rate.sleep()