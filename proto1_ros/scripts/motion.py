#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy

def joy_callback(msg):
    rospy.loginfo("received message")
    pass

if __name__ == '__main__':
    # setup ros node
    rospy.loginfo("setting up ros node")
    rospy.init_node('motion')
    sub = rospy.Subscriber('/joy', Joy, joy_callback)
    rate = rospy.Rate(20) # 20Hz

    while not rospy.is_shutdown():
        # list motions here
        
        rate.sleep()
