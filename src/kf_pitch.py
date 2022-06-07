#!/usr/bin/env python
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

rospy.init_node('kf_pitch', anonymous=True)

rate = rospy.Rate(15)

tello_pose_kf_pitch = np.zeros((5,), dtype=np.float32)

pub_pose_kf_pitch = rospy.Publisher('tello_pose_kf_pitch', numpy_msg(Floats), queue_size=10)

dt = 1.0/15

A_pitch = np.array([[1.0, dt], [0.0, 1.0]], dtype = np.float32)
C_pitch = np.array([1.0, 0.0], dtype = np.float32).reshape(1, 2)
Q_pitch = 0.1 * np.identity(2)
R_pitch_imu = 0.5
P_pitch = 0.2 * np.identity(2)
X_pitch = np.zeros((2, 1), dtype = np.float32)

class Kalman_filter_pitch:

    def __init__(self, Q_pitch, R_pitch_imu, P_pitch, X_pitch):
        self.Q_pitch = Q_pitch
        self.R_pitch_imu = R_pitch_imu
        self.P_pitch = P_pitch
        self.X_pitch = X_pitch

    def update_pitch_imu(self, measure_pitch_imu):
        temp = self.P_pitch.dot(C_pitch.T)
        K = temp / (C_pitch.dot(self.P_pitch.dot(C_pitch.T)) + self.R_pitch_imu)
        self.X_pitch = self.X_pitch + K * (measure_pitch_imu - C_pitch.dot(self.X_pitch))
        I = np.identity(2)
        temp = I - K.dot(C_pitch)
        self.P_pitch = temp.dot(self.P_pitch)

    def predict_pitch(self):
        self.X_pitch = A_pitch.dot(self.X_pitch)
        self.P_pitch = A_pitch.dot(self.P_pitch.dot(A_pitch.T)) + self.Q_pitch

drone_pitch = Kalman_filter_pitch(Q_pitch, R_pitch_imu, P_pitch, X_pitch)

def get_imu_message(imu_msg):
    global drone_pitch
    temp_imu = imu_msg.data
    drone_pitch.update_pitch_imu(temp_imu[4])

rospy.Subscriber("/drone2/repub_imu", numpy_msg(Floats), callback=get_imu_message)

while not rospy.is_shutdown():
    drone_pitch.predict_pitch()
    tello_pose_kf_pitch[4] = drone_pitch.X_pitch[0, 0]
    pub_pose_kf_pitch.publish(tello_pose_kf_pitch)
    rate.sleep()