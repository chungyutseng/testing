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
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from tello_driver.msg import TelloStatus
import functools

rospy.init_node('new_center', anonymous=True)

rate = rospy.Rate(15)

pub_drone1_mt = rospy.Publisher("/drone1/tello/manual_takeoff", EmptyMsg, queue_size=10)
pub_drone2_mt = rospy.Publisher("/drone2/tello/manual_takeoff", EmptyMsg, queue_size=10)
pub_pc_drone1 = rospy.Publisher("/drone1/pc_on", Float32, queue_size=10)
pub_pdc_drone1 = rospy.Publisher("/drone1/pdc_on", Float32, queue_size=10)
pub_pc_drone2 = rospy.Publisher("/drone2/pc_on", Float32, queue_size=10)
pub_pdc_drone2 = rospy.Publisher("/drone2/pdc_on", Float32, queue_size=10)

# two drones are hovering at their zore-velocity zones (original)
# stage one
pub_stage_one_drone_one = rospy.Publisher("/stage_one_drone_one", Float32, queue_size=10)
pub_stage_one_drone_two = rospy.Publisher("/stage_one_drone_two", Float32, queue_size=10)

############################################################################
# stage 1 to stage 2 transition (both drones zero velocity)
pub_zero_velocity_drone1 = rospy.Publisher("/zero_velocity_drone1", Float32, queue_size=10)
pub_zero_velocity_drone2 = rospy.Publisher("/zero_velocity_drone2", Float32, queue_size=10)
############################################################################

############################################################################
# stage 2
pub_stage_two_drone_one = rospy.Publisher("/stage_two_drone_one", Float32, queue_size=10)
pub_stage_two_pdc_drone1 = rospy.Publisher("/drone1/stage_two_pdc_on", Float32, queue_size=10)
pub_velocity_estimator_drone2 = rospy.Publisher("/drone2/velocity_estimator", Float32, queue_size=10)
pub_finish_velocity_estimator_drone2 = rospy.Publisher("/drone2/finish_velocity_estimator", Float32, queue_size=10)
pub_start_calculating_velocity_drone2 = rospy.Publisher("/drone2/start_calculating_velocity", Float32, queue_size=10)
############################################################################

############################################################################
# stage 3
pub_stage_three_drone_two = rospy.Publisher("/stage_three_drone_two", Float32, queue_size=10)
pub_stage_three_pdc_drone2 = rospy.Publisher("/drone2/stage_three_pdc_on", Float32, queue_size=10)
############################################################################

time.sleep(5)

# manual take off (stay on the ground)
pub_drone1_mt.publish()
pub_drone2_mt.publish()

time.sleep(3.5)
# time.sleep(50)

pub_pc_drone1.publish(1.0)
pub_pdc_drone1.publish(0.0)
pub_pc_drone2.publish(1.0)
pub_pdc_drone2.publish(0.0)
time.sleep(2)
pub_stage_one_drone_one.publish(1.0)
pub_pc_drone1.publish(0.0)
pub_pdc_drone1.publish(1.0)
pub_stage_one_drone_two.publish(1.0)
pub_pc_drone2.publish(0.0)
pub_pdc_drone2.publish(1.0)

############################################################################
# leader first takes off with p controllers and
# stays within its zero-veloity zone with pd controller

# pub_pc_drone1.publish(1.0)
# pub_pdc_drone1.publish(0.0)
# time.sleep(3.5)
# pub_stage_one_drone_one.publish(1.0)
# pub_pc_drone1.publish(0.0)
# pub_pdc_drone1.publish(1.0)

# time.sleep(4)

# follower then takes off with p controllers and
# stays within its zero-veloity zone with pd controller

# pub_pc_drone2.publish(1.0)
# pub_pdc_drone2.publish(0.0)
# time.sleep(3.5)
# pub_stage_one_drone_two.publish(1.0)
# pub_pc_drone2.publish(0.0)
# pub_pdc_drone2.publish(1.0)
############################################################################

############################################################################
# stage 1 to stage 2 transition (both drones zero velocity)
time.sleep(50)

pub_stage_one_drone_one.publish(0.0)
pub_stage_one_drone_two.publish(0.0)
pub_pc_drone1.publish(0.0)
pub_pdc_drone1.publish(0.0)
pub_pc_drone2.publish(0.0)
pub_pdc_drone2.publish(0.0)
pub_zero_velocity_drone1.publish(1.0)
pub_zero_velocity_drone2.publish(1.0)
############################################################################

############################################################################
# stage 2
time.sleep(5)

pub_zero_velocity_drone1.publish(0.0)
pub_zero_velocity_drone2.publish(1.0)

pub_stage_two_drone_one.publish(1.0)
pub_stage_two_pdc_drone1.publish(1.0)
pub_velocity_estimator_drone2.publish(1.0)

time.sleep(15)
# time.sleep(10)
# pub_velocity_estimator_drone2.publish(0.0)
# time.sleep(1.5)
pub_finish_velocity_estimator_drone2.publish(1.0)
time.sleep(1)
pub_start_calculating_velocity_drone2.publish(1.0)
time.sleep(1)
pub_finish_velocity_estimator_drone2.publish(0.0)
# pub_zero_velocity_drone2.publish(0.0)
############################################################################

############################################################################
# stage 3
# pub_stage_three_drone_two.publish(1.0)
# pub_stage_three_pdc_drone2.publish(1.0)
############################################################################