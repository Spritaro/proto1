#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped
from math import pi
import Adafruit_PCA9685


SERVOMIN = 103
SERVOMAX = 490
SERVONUM = 2

class Servo(object):
    id = 0
    end_angle = 0.0         # in degree
    current_angle = 0.0     # in degree
    start_angle = 90.0       # in degree
    end_time = 1            # in 10ms
    current_time = 0        # in 10ms
    start_time = 0          # always 0
    power = False

servos = [ Servo() for i in range(SERVONUM) ]


def conv_ang(ang):
    return int((SERVOMAX - SERVOMIN) * (ang + 90.0) / 180.0 + SERVOMIN)


def servo_update():
    global servos
    for i, servo in enumerate(servos):
        if(servo.power):
            if(servo.current_time < servo.end_time):
                servo.current_angle = (servo.end_angle - servo.start_angle) * (servo.current_time - servo.start_time) / (servo.end_time - servo.start_time) + servo.start_angle
                servo.current_time += 1 # 10ms
            if(servo.current_time >= servo.end_time):
                servo.current_angle = servo.end_angle
            # set updated angle
            pwm.set_pwm(servo.id, 0, conv_ang(servo.current_angle))
        else:
            # power off
            pwm.set_pwm(servo.id, 0, 0)


def head_callback(msg):
    # rospy.loginfo("received head orientation")

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

    servos[0].id = 0
    servos[0].end_time = 0
    servos[0].end_angle = head_pitch
    servos[0].power = True

    servos[1].id = 15
    servos[1].end_time = 0
    servos[1].end_angle = head_yaw
    servos[1].power = True

    servo_update()


if __name__ == '__main__':
    # setup ros node
    rospy.loginfo("setting up ros node")
    rospy.init_node('head')
    sub = rospy.Subscriber('/camera/pose_stamped', PoseStamped, head_callback)

    # setup servo motors
    rospy.loginfo("setting servo motors")
    pwm = Adafruit_PCA9685.PCA9685()
    pwm.set_pwm_freq(50)    # 50Hz -> 20ms

    rospy.spin()
