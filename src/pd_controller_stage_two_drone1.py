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
import functools
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

rospy.init_node('pd_controller_stage_two_drone1', anonymous=True)

rate = rospy.Rate(15)

current_pose = np.zeros((10,), dtype=np.float32)
desired_pose = np.zeros((4,), dtype=np.float32)
cmd_vel_test = np.zeros((6,), dtype=np.float32)


pid_internal = np.zeros((8,), dtype=np.float32)

my_namespace=rospy.get_namespace()

small_marker = 0.0

kp_x = 1.0
kp_y = 1.4
kp_z = 1.2
kp_yaw = 1.8
kp = np.array([kp_x, kp_y, kp_z, kp_yaw], dtype=np.float32)

kd_x = 0.1
kd_y = 0.5
kd_z = 0.01
kd_yaw = 0.5
kd = np.array([kd_x, kd_y, kd_z, kd_yaw], dtype=np.float32)

pid_gain = np.array([kp_x, kp_y, kp_z, kp_yaw, kd_x, kd_y, kd_z, kd_yaw], dtype=np.float32)

previous_error_x = 0.0
previous_error_y = 0.0
previous_error_z = 0.0
previous_error_yaw = 0.0

count = 0.0

vel_max_linear = 0.8
vel_max_angular = 0.8

vel_min_linear = 0.25
vel_min_angular = 0.5

zero_vel_zone_linear_hs_x = 0.07
zero_vel_zone_linear_hs_y = 0.07
zero_vel_zone_linear_hs_z = 0.07
zero_vel_zone_angular_hs = 1.5
constant_velocity_error = 0.13
constant_speed_forward = 0.3
constant_speed_forward_flag = 0.0

zero_velocity = np.array([zero_vel_zone_linear_hs_x, zero_vel_zone_linear_hs_y, zero_vel_zone_linear_hs_z, zero_vel_zone_angular_hs], dtype=np.float32)
velocity_limit = np.array([vel_max_linear, vel_max_angular, vel_min_linear, vel_min_angular], dtype=np.float32)

pub_vel = rospy.Publisher("/drone1/tello/cmd_vel_test", Twist, queue_size=10)
pub_pid_internal = rospy.Publisher('pid_internal', numpy_msg(Floats), queue_size=10)
pub_cmd_vel_test = rospy.Publisher("cmd_vel_test_array", numpy_msg(Floats), queue_size=10)

pub_pid_gain = rospy.Publisher("pid_gain", numpy_msg(Floats), queue_size=10)
pub_zero_velocity = rospy.Publisher("zero_velocity", numpy_msg(Floats), queue_size=10)
pub_velocity_limit = rospy.Publisher("velocity_limit", numpy_msg(Floats), queue_size=10)

pub_zero_velocity.publish(zero_velocity)
pub_velocity_limit.publish(velocity_limit)

vel_msg = Twist()
vel_msg.linear.x = 0.0
vel_msg.linear.y = 0.0
vel_msg.linear.z = 0.0
vel_msg.angular.x = 0.0
vel_msg.angular.y = 0.0
vel_msg.angular.z = 0.0

stage_two_pdc_on = 0.0

def get_kf_position(data):
    global current_pose
    current_pose = data.data

def get_desired_pose(data):
    global desired_pose
    desired_pose = data.data

def get_stage_two_pdc_on(data):
    global stage_two_pdc_on
    stage_two_pdc_on = data.data
    if (stage_two_pdc_on == 1.0):
        print("START MOVING")

def pd_controller(c_pose, d_pose, kp_gain, kd_gain):
    global count
    global previous_error_x, previous_error_y, previous_error_z, previous_error_yaw
    global vel_msg
    global pid_internal
    global cmd_vel_test
    global constant_speed_forward_flag

    x_p = c_pose[0]
    y_p = c_pose[1]
    z_p = c_pose[2]
    yaw_a = c_pose[3]
    
    dx_p = d_pose[0]
    dy_p = d_pose[1]
    dz_p = d_pose[2]
    dyaw_a = d_pose[3]

    KP_X = kp_gain[0]
    KP_Y = kp_gain[1]
    KP_Z = kp_gain[2]
    KP_YAW = kp_gain[3]

    KD_X = kd_gain[0]
    KD_Y = kd_gain[1]
    KD_Z = kd_gain[2]
    KD_YAW = kd_gain[3]

    if count == 0.0:
        yaw_a = math.radians(yaw_a)
        A = np.array([[math.cos(-yaw_a), -math.sin(-yaw_a)],
                    [math.sin(-yaw_a), math.cos(-yaw_a)]])
        A_transpose = A.T
        now_position = np.array([[x_p], [y_p]])
        T_position = - A_transpose.dot(now_position)
        Trans_matrix = np.array([[A_transpose[0][0], A_transpose[0][1], T_position[0][0]],
                                [A_transpose[1][0], A_transpose[1][1], T_position[1][0]], 
                                [0                , 0                , 1              ]])
        goal_position = np.array([[dx_p], [dy_p], [1]])
        goal_wrt_drone = Trans_matrix.dot(goal_position)
        x_goal_wrt_drone = goal_wrt_drone[0][0]
        y_goal_wrt_drone = goal_wrt_drone[1][0]

        previous_error_x = x_goal_wrt_drone - 0.0
        previous_error_y = y_goal_wrt_drone - 0.0

        now_z_position = z_p
        previous_error_z = dz_p - z_p

        now_yaw_angle = yaw_a
        previous_error_yaw = dyaw_a - now_yaw_angle

        count = count + 1

    else:
        vel_msg = Twist()

        yaw_a = math.radians(yaw_a)
        A = np.array([[math.cos(-yaw_a), -math.sin(-yaw_a)],
                    [math.sin(-yaw_a), math.cos(-yaw_a)]])
        A_transpose = A.T
        now_position = np.array([[x_p], [y_p]])
        T_position = - A_transpose.dot(now_position)
        Trans_matrix = np.array([[A_transpose[0][0], A_transpose[0][1], T_position[0][0]],
                                [A_transpose[1][0], A_transpose[1][1], T_position[1][0]], 
                                [0                , 0                , 1              ]])
        goal_position = np.array([[dx_p], [dy_p], [1]])
        goal_wrt_drone = Trans_matrix.dot(goal_position)
        x_goal_wrt_drone = goal_wrt_drone[0][0]
        y_goal_wrt_drone = goal_wrt_drone[1][0]       

        now_error_x = x_goal_wrt_drone - 0.0
        now_error_y = y_goal_wrt_drone - 0.0

        vel_msg.linear.x = now_error_x * KP_X + ((now_error_x - previous_error_x) / (1.0/15)) * KD_X
        vel_msg.linear.y = now_error_y * KP_Y + ((now_error_y - previous_error_y) / (1.0/15)) * KD_Y

        ##
        pid_internal[0] = now_error_x
        pid_internal[1] = now_error_y
        pid_internal[4] = previous_error_x
        pid_internal[5] = previous_error_y
        ##

        previous_error_x = now_error_x
        previous_error_y = now_error_y

        now_z_position = z_p
        now_error_z = dz_p - z_p

        vel_msg.linear.z = now_error_z * KP_Z + ((now_error_z - previous_error_z) / (1.0/15)) * KD_Z

        ##
        pid_internal[2] = now_error_z
        pid_internal[6] = previous_error_z
        ##

        previous_error_z = now_error_z

        now_yaw_angle = yaw_a
        now_error_yaw = dyaw_a - now_yaw_angle

        vel_msg.angular.z = (now_error_yaw * KP_YAW + ((now_error_yaw - previous_error_yaw) / (1.0/15)) * KD_YAW)

        ##
        pid_internal[3] = now_error_yaw
        pid_internal[7] = previous_error_yaw
        ##

        previous_error_yaw = now_error_yaw

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        cmd_vel_test[0] = vel_msg.linear.x
        cmd_vel_test[1] = vel_msg.linear.y
        cmd_vel_test[2] = vel_msg.linear.z
        cmd_vel_test[3] = vel_msg.angular.x
        cmd_vel_test[4] = vel_msg.angular.y
        cmd_vel_test[5] = vel_msg.angular.z

        sign = functools.partial(math.copysign, 1)
        if abs(vel_msg.linear.x) > vel_max_linear:
            vel_msg.linear.x = sign(vel_msg.linear.x) * vel_max_linear
        if abs(vel_msg.linear.y) > vel_max_linear:
            vel_msg.linear.y = sign(vel_msg.linear.y) * vel_max_linear
        if abs(vel_msg.linear.z) > vel_max_linear:
            vel_msg.linear.z = sign(vel_msg.linear.z) * vel_max_linear
        if abs(vel_msg.angular.z) > vel_max_angular:
            vel_msg.angular.z = sign(vel_msg.angular.z) * vel_max_angular

        if abs(vel_msg.linear.x) < vel_min_linear:
            vel_msg.linear.x = sign(vel_msg.linear.x) * vel_min_linear
        if abs(vel_msg.linear.y) < vel_min_linear:
            vel_msg.linear.y = sign(vel_msg.linear.y) * vel_min_linear
        if abs(vel_msg.linear.z) < vel_min_linear:
            vel_msg.linear.z = sign(vel_msg.linear.z) * vel_min_linear
        if abs(vel_msg.angular.z) < vel_min_angular:
            vel_msg.angular.z = sign(vel_msg.angular.z) * vel_min_angular

        if abs(now_error_x) < zero_vel_zone_linear_hs_x:
            vel_msg.linear.x = 0
        if abs(now_error_y) < zero_vel_zone_linear_hs_y:
            vel_msg.linear.y = 0
        if abs(now_error_z) < zero_vel_zone_linear_hs_z:
            vel_msg.linear.z = 0
        if abs(now_error_yaw) < zero_vel_zone_angular_hs*(math.pi / 180.0):
            vel_msg.angular.z = 0

        # if (constant_speed_forward_flag == 0.0) and (stage_two_pdc_on == 1.0):
        #     if now_error_y > constant_velocity_error:
        #         vel_msg.linear.x = 0.0
        #         vel_msg.linear.y = constant_speed_forward
        #         vel_msg.linear.z = 0.0
        #         vel_msg.angular.x = 0.0
        #         vel_msg.angular.y = 0.0
        #         vel_msg.angular.z = 0.0
        #     else:
        #         constant_speed_forward_flag = 1.0

        if (constant_speed_forward_flag == 0.0):
            if now_error_y > constant_velocity_error:
                vel_msg.linear.x = 0.0
                vel_msg.linear.y = constant_speed_forward
                vel_msg.linear.z = 0.0
                vel_msg.angular.x = 0.0
                vel_msg.angular.y = 0.0
                vel_msg.angular.z = 0.0
            else:
                constant_speed_forward_flag = 1.0

rospy.Subscriber('/drone1/tello_pose_kf', numpy_msg(Floats), callback=get_kf_position)
rospy.Subscriber('/drone1/desired_pose', numpy_msg(Floats), callback=get_desired_pose)
rospy.Subscriber('/drone1/stage_two_pdc_on', Float32, callback=get_stage_two_pdc_on)

while not rospy.is_shutdown():
    pd_controller(current_pose, desired_pose, kp, kd)
    if (stage_two_pdc_on == 1.0):
        pub_vel.publish(vel_msg)
        pub_pid_internal.publish(pid_internal)
        pub_cmd_vel_test.publish(cmd_vel_test)
        pub_pid_gain.publish(pid_gain)
        # print("11111111111111111111")
    rate.sleep()