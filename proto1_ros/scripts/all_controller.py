#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy

import Adafruit_PCA9685

import time
from math import pi

SERVOMIN = 103
SERVOMAX = 490

head_pitch = 0
head_yaw = 0

def head_callback(msg):
    global head_pitch
    global head_yaw

    rospy.loginfo("received head orientation")

    # copying for simplicity
    position = msg.pose.position
    quat = msg.pose.orientation

    # convert to euler angles
    euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

    # update head angles
    head_pitch = euler[0]
    head_yaw   = euler[2]
    head_pitch = head_pitch * 180.0 / pi
    head_yaw   = head_yaw   * 180.0 / pi
    if head_pitch < -30.0:
        head_pitch = -30.0
    elif head_pitch > 30.0:
        head_pitch = 30.0
    if head_yaw < -80.0:
        head_yaw = -80.0
    elif head_yaw > 80.0:
        head_yaw = 80.0

    rospy.loginfo("pitch{}, yaw{}".format(head_pitch, head_yaw))

def body_callback(msg):
    pass

def conv_ang(ang):
    return int((SERVOMAX - SERVOMIN) * (ang + 90.0) / 180.0 + SERVOMIN)


if __name__ == '__main__':
    # setup ros node
    rospy.loginfo("setting up ros node")
    rospy.init_node('all_controller')
    sub = rospy.Subscriber('/camera/pose_stamped', PoseStamped, head_callback)
    rate = rospy.Rate(20) # 20Hz

    # setup servo motors
    rospy.loginfo("setting servo motors")
    pwm = Adafruit_PCA9685.PCA9685()
    pwm.set_pwm_freq(50)

    pwm.set_pwm(7, 0, conv_ang(0))
    pwm.set_pwm(8, 0, conv_ang(0))

    while not rospy.is_shutdown():
        pwm.set_pwm(7, 0, conv_ang(head_pitch))
        pwm.set_pwm(8, 0, conv_ang(head_yaw))

        rate.sleep()
