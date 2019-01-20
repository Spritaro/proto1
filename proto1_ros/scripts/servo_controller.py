#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from proto1_ros.msg import proto1_servo, proto1_servo_multi
import Adafruit_PCA9685
from math import pi

SERVOMIN = 103
SERVOMAX = 490
SERVONUM = 16

class Servo(object):
    end_angle = 0.0         # in degree
    current_angle = 0.0     # in degree
    start_angle = 0.0       # in degree
    end_time = 1            # in 10ms
    current_time = 0        # in 10ms
    start_time = 0          # always 0
    power = False

servos = [ Servo() for i in range(SERVONUM) ]

def servo_callback(msg):
    # rospy.loginfo("received message")
    for i in range(msg.count):
        servo_msg = msg.servos[i]
        id = msg.servos[i].id
        servos[id].end_angle = servo_msg.angle
        servos[id].start_angle = servos[id].current_angle
        servos[id].end_time = servo_msg.time
        servos[id].current_time = 0
        servos[id].power = servo_msg.power

def servo_update():
    for id, servo in enumerate(servos):
        if(servo.power):
            if(servo.current_time < servo.end_time):
                servo.current_angle = (servo.end_angle - servo.start_angle) * (servo.current_time - servo.start_time) / (servo.end_time - servo.start_time) + servo.start_angle
                servo.current_time += 2 # 20ms
            elif(servo.current_time >= servo.end_time):
                servo.current_angle = servo.end_angle
            # set updated angle
            pwm.set_pwm(id, 0, conv_ang(servo.current_angle))
        else:
            # power off
            pwm.set_pwm(id, 0, 0)

    print(servos[5].current_time)

def conv_ang(ang):
    return int((SERVOMAX - SERVOMIN) * (ang + 90.0) / 180.0 + SERVOMIN)


if __name__ == '__main__':
    # setup ros node
    rospy.loginfo("setting up ros node")
    rospy.init_node('servo_controller')
    sub = rospy.Subscriber('ServoAngles', proto1_servo_multi, servo_callback)
    rate = rospy.Rate(50) # 50Hz -> 20ms

    # setup servo motors
    rospy.loginfo("setting servo motors")
    pwm = Adafruit_PCA9685.PCA9685()
    pwm.set_pwm_freq(50)    # 50Hz -> 20ms

    while not rospy.is_shutdown():
        servo_update()
        rate.sleep()
