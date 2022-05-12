#!/usr/bin/env python
PKG = 'testing'
import roslib; roslib.load_manifest(PKG)

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

def callback(data):
    # print rospy.get_name(), "I heard %s"%str(data.astpye(int).data)
    array = data.data
    print(array[0])

def listener():
    rospy.init_node('listener')
    rospy.Subscriber("floats", numpy_msg(Floats), callback)
    rospy.spin()

if __name__ == '__main__':
    listener()