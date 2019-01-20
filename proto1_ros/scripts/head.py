#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped
from proto1_ros.msg import proto1_servo, proto1_servo_multi
from math import pi

def head_callback(msg):
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

    # create message
    servo_p = proto1_servo()
    servo_p.id = 0
    servo_p.time = 0
    servo_p.power = True
    servo_p.angle = head_pitch

    servo_y = proto1_servo()
    servo_y.id = 15
    servo_y.time = 0
    servo_y.power = True
    servo_y.angle = head_yaw

    servo_multi = proto1_servo_multi()
    servo_multi.count = 2               # number of servo motors
    servo_multi.servo.append(servo_p)   # pitch servo data
    servo_multi.servo.append(servo_y)   # yaw servo data

    # publish message
    pub.publish(servo_multi)

if __name__ == '__main__':
    # setup ros node
    pub = rospy.Publisher('ServoAngles', proto1_servo_multi, queue_size=0)
    rospy.loginfo("setting up ros node")
    rospy.init_node('head')
    sub = rospy.Subscriber('/camera/pose_stamped', PoseStamped, head_callback)
    rospy.spin()
