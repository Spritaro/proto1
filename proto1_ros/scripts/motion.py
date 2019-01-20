#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from proto1_ros.msg import proto1_servo, proto1_servo_multi
from math import sin, cos, acos, atan2, pi

# servo
SERVONUM = 14
servos_msg = proto1_servo_multi()
servos_msg.servos = [ proto1_servo() for i in range(SERVONUM) ]

# [ 10., 20.-20.,-130.,0.,0., -20.,-20.-20.,-135.,0.,0., -20.,-20.,50.,  20.,-20.,50.]
# [left foot x,y,z,roll,yaw, right foot x,y,z,roll,yaw, left shoulder pitch,roll, left elbow, right shoulder pitch,roll, right elbow]

# joy
joy_data = Joy()
joy_data.axes = [0.0] * 6
joy_data.buttons = [0] * 12


def joy_callback(msg):
    rospy.loginfo("received message")
    pass


# x y z (position of foot)
# a (ankle roll) and b (thigh yaw)
def ik(x, y, z, a, b):
  l1 = 20
  l2 = 50
  l3 = 12
  l4 = 50
  l5 = 20

  # degrees to radians
  a = a * pi / 180.
  b = b * pi / 180.

  # apply b (thigh yaw) rotation
  x_ = x*cos(b)-y*sin(b)
  y_ = x*sin(b)+y*cos(b)

  # get th1 (thigh roll) rotation
  th1 = atan2(-x, -z)

  # get z_ axis
  z_ = z / cos(th1)
  z_ -= -l1  -l3 -l5

  # get th2 (thigh pitch) and th3 (ankle pitch)
  ph1 = atan2(-y, -z)
  l = -z_ / cos(ph1)
  ph2 = acos((l*l + l2*l2 - l4*l4)/(2*l*l2))
  ph3 = acos((l*l + l4*l4 - l2*l2)/(2*l*l4))
  th2 = ph1 + ph2
  th3 = ph3 - ph1

  # other
  th0 = b
  th4 = th1 + a

  # radians to degrees
  th0 = th0 * 180. / pi
  th1 = th1 * 180. / pi
  th2 = th2 * 180. / pi
  th3 = th3 * 180. / pi
  th4 = th4 * 180. / pi
  ret = [th0, th1, th2, th3, th4]
  return ret


# move servos directly
# 0 head yaw 
# 1 right elbow
# 2 right shoulder roll
# 3 right shoulder pitch
# 4 right ankle roll
# 5 right ankle pitch
# 6 right thigh pitch
# 7 right thigh roll  
# 8 left thigh roll
# 9 left thigh pitch
# 10 left ankle pitch
# 11 left ankle roll
# 12 left shoulder pitch
# 13 left shoulder roll
# 14 left elbow 
# 15 head pitch

# left thigh roll, pitch, ankle pitch, roll, 
# right thigh roll, pitch, ankle pitch, roll,
# left shoulder pitch, roll,
# right shoulder pitch, roll,
ids = [8,9,10,11, 7,6,5,4, 12,13,14, 3,2,1]
def set_servo_directly(time, angle):
    global servos_msg
    for i in range(SERVONUM):
        servos_msg.servos[i].id = ids[i]
        servos_msg.servos[i].time = time
        servos_msg.servos[i].angle = angle[i]
        servos_msg.servos[i].power = True
    servos_msg.count = SERVONUM
    pub.publish(servos_msg)
    for i in range(time):
        rate.sleep()



if __name__ == '__main__':
    # setup ros node
    pub = rospy.Publisher('ServoAngles', proto1_servo_multi, queue_size=0)
    rospy.loginfo("setting up ros node")
    rospy.init_node('motion')
    sub = rospy.Subscriber('/joy', Joy, joy_callback)
    rate = rospy.Rate(50) # 50Hz -> 20ms

    while not rospy.is_shutdown():
        # list motions here

        set_servo_directly(500, [ -10.,-10.,-10.,-10., -10.,-10,-10.,-10., -10.,-10.,-10., -10.,-10.,-10.])
        set_servo_directly(500, [ 10.,10.,10.,10., 10.,10,10.,10., 10.,10.,10., 10.,10.,10.])

        rate.sleep()
