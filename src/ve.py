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

rospy.init_node('velocity_estimator_test', anonymous=True)

rate = rospy.Rate(15)

# roll pitch yaw
# attitude_data = np.zeros((3,), dtype=np.float32)

# velocity_estimator_flag = 0.0
# velocity_estimator_flag_finish = 0.0
# finish_velocity_estimator_flag = 0.0

# total_pitch_angle = 0.0
# count = 0.0
# averaged_pitch_angle = 0.0
estimated_velocity = 0.0

flag = 0

finish_velocity_estimator_flag = 0.0

get_stage_two_pdc_on_flag = 0.0

time_drone1_move = time.time()
time_deg_pre = time.time()
time_deg_now = time.time()
time_deg = time.time()

time_start = time.time()

imu_pre = 0.0
imu_now = 0.0

count = 0

test = np.zeros((6,), dtype=np.float)

pub_estimated_velocity = rospy.Publisher("estimated_velocity_test", Float32, queue_size=10)
pub_test_1 = rospy.Publisher('test_1', Float32, queue_size=10)
pub_test_2 = rospy.Publisher('test_2', Float32, queue_size=10)
pub_test_3 = rospy.Publisher('test_3', Float32, queue_size=10)
pub_test_4 = rospy.Publisher('test_4', Float32, queue_size=10)
pub_test_5 = rospy.Publisher('test_5', Float32, queue_size=10)
pub_test_6 = rospy.Publisher('test_6', Float32, queue_size=10)

def get_imu_message(imu_msg):
    global flag
    global time_deg_pre, time_deg_now
    global imu_pre, imu_now
    global count
    global time_deg
    global test
    global time_start
    temp_imu = imu_msg.data
    if (count == 0):
        time_start = time.time()
        time_deg_pre = time.time()
        imu_pre = temp_imu[4]
        count = count + 1
    else:
        time_deg_now = time.time()
        imu_now = temp_imu[4]
        if (flag == 0):
            if(get_stage_two_pdc_on_flag == 1):
                if (abs(imu_now) >= 6.0):
                    time_deg = (6.0 - abs(imu_pre)) * ((time_deg_now - time_deg_pre) / (abs(imu_now) - abs(imu_pre))) + time_deg_pre
                    flag = 1
                    # test = [time_deg_now time_deg_pre imu_now imu_pre time_drone1_move time_deg]
                    test[0] = time_deg_now - time_start
                    test[1] = time_deg_pre - time_start
                    test[2] = imu_now
                    test[3] = imu_pre
                    test[4] = time_drone1_move - time_start
                    test[5] = time_deg - time_start
                    pub_test_1.publish(time_deg_now - time_start)
                    pub_test_2.publish(time_deg_pre - time_start)
                    pub_test_3.publish(imu_now)
                    pub_test_4.publish(imu_pre)
                    pub_test_5.publish(time_drone1_move - time_start)
                    pub_test_6.publish(time_deg - time_start)
                    # pub_test.publish(test)
                    # print(time_deg_now)
                    # print(time_deg_pre)
                    # print(imu_now)
                    # print(imu_pre)
                    # print(time_drone1_move)
                    # print(time_deg)
                    print(test[0])
                    print(test[1])
                    print(test[2])
                    print(test[3])
                    print(test[4])
                    print(test[5])
        time_deg_pre = time_deg_now
        imu_pre = imu_now
    # global total_pitch_angle, count
    # temp_imu = imu_msg.data
    # attitude_data[0] = temp_imu[3] # roll
    # attitude_data[1] = temp_imu[4] # pitch
    # attitude_data[2] = temp_imu[5] # yaw

    # if (velocity_estimator_flag == 1.0):
    #     total_pitch_angle = total_pitch_angle + temp_imu[4]
    #     count = count + 1.0

def get_stage_two_pdc_on(data):
    global get_stage_two_pdc_on_flag
    global time_drone1_move
    get_stage_two_pdc_on_flag = data.data
    if (get_stage_two_pdc_on_flag == 1):
        time_drone1_move = time.time()
    # global velocity_estimator_flag, velocity_estimator_flag_finish
    # velocity_estimator_flag = data.data
    # if (velocity_estimator_flag == 1.0):
    #     velocity_estimator_flag_finish = 1.0
    #     print("CALCULATING AVERAGING ANGLE")
    # if ((velocity_estimator_flag == 0.0) and (velocity_estimator_flag_finish == 1.0)):
    #     velocity_estimator_flag_finish = 0.0
    #     print("FINISH CALCULATING AVERAGING ANGLE")

def get_finish_velocity_estimator(data):
    global estimated_velocity
    global finish_velocity_estimator_flag
    finish_velocity_estimator_flag = data.data
    if (finish_velocity_estimator_flag == 1.0):
        estimated_velocity = time_deg - time_drone1_move
#     global averaged_pitch_angle, estimated_velocity
#     finish_velocity_estimator_flag = data.data
#     if (finish_velocity_estimator_flag == 1.0):
#         averaged_pitch_angle = (total_pitch_angle / count)
#         estimated_velocity = averaged_pitch_angle 

rospy.Subscriber("/drone2/repub_imu", numpy_msg(Floats), callback=get_imu_message)
rospy.Subscriber("/drone1/stage_two_pdc_on", Float32, callback=get_stage_two_pdc_on)
rospy.Subscriber("/drone2/finish_velocity_estimator", Float32, callback=get_finish_velocity_estimator)

while not rospy.is_shutdown():
    if (finish_velocity_estimator_flag == 1.0):
        pub_estimated_velocity.publish(estimated_velocity)
    rate.sleep()