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

rospy.init_node('min', anonymous=True)

rate = rospy.Rate(15)

pub_min = rospy.Publisher("min", Float32, queue_size=10)

get_stage_two_pdc_on_flag = 0.0

finish_velocity_estimator_flag = 0.0

min_angle = 0.0

flag = 0.0

def get_imu_message(imu_msg):
    global min_angle
    temp_imu = imu_msg.data
    if (flag == 0.0):
        if (get_stage_two_pdc_on_flag == 1.0):
            if (temp_imu[4] < min_angle):
                min_angle = temp_imu[4]

def get_stage_two_pdc_on(data):
    global get_stage_two_pdc_on_flag
    get_stage_two_pdc_on_flag = data.data

def get_finish_velocity_estimator(data):
    global finish_velocity_estimator_flag
    global flag
    finish_velocity_estimator_flag = data.data
    if (finish_velocity_estimator_flag == 1.0):
        flag = 1.0

rospy.Subscriber("/drone2/repub_imu", numpy_msg(Floats), callback=get_imu_message)
rospy.Subscriber("/drone1/stage_two_pdc_on", Float32, callback=get_stage_two_pdc_on)
rospy.Subscriber("/drone2/finish_velocity_estimator", Float32, callback=get_finish_velocity_estimator)

while not rospy.is_shutdown():
    if (finish_velocity_estimator_flag == 1.0):
        pub_min.publish(min_angle)
    rate.sleep()