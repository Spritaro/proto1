#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from math import sin, cos, acos, atan2, pi
import Adafruit_PCA9685

# joy
joy_data = Joy()
joy_data.axes = [0.0] * 6
joy_data.buttons = [0] * 12
def joy_callback(msg):
    global joy_data
    joy_data = msg
    # rospy.loginfo("joy = {}".format(joy_data))


SERVOMIN = 103
SERVOMAX = 490
SERVONUM = 14

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


def servo_poweroff():
    for i in range(SERVONUM):
        pwm.set_pwm(i, 0, 0)
        rate.sleep()


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
    # CCW
    angle[1] *= -1   # left thigh pitch
    angle[6] *= -1   # right ankle pitch
    angle[9] *= -1  # left shoulder roll
    angle[10] *= -1  # left elbow

    # offsets
    angle[0] += 20. # left thigh roll
    angle[1] += 85. # left thigh pitch
    angle[2] -= 40. # left ankle pitch
    angle[3] += 18.+1. # left ankle roll

    angle[4] += 13. # right right roll
    angle[5] -= 30. # right thigh pitch
    angle[6] += 65. - 6. # right ankle pitch
    angle[7] += 17.+5. # right ankle roll

    angle[8] += 0. # left shoulder pitch
    angle[9] += 10. # left shoulder roll
    angle[10] += 15. # left elbow

    angle[11] += 0. # right shoulder pitch
    angle[12] += 10. # right shoulder roll
    angle[13] += 50. # right elbow

    for i in range(SERVONUM):
        servos[i].id = ids[i]
        servos[i].start_angle = servos[i].current_angle
        servos[i].end_angle = angle[i]
        servos[i].end_time = time
        servos[i].current_time = 0
        servos[i].start_time = 0    # always 0
        servos[i].power = True

    for i in range(time):
        servo_update()
        rate.sleep()


# move servos
def set_servo(time, target):
    target_angle = [0.0] * SERVONUM

    # 1ms -> 10ms
    time /= 10
    if time < 1:
        time = 1

    # invert axis
    target[1] *= -1
    target[6] *= -1
    target[10] *= -1

    # apply ik
    tgt_l = ik(target[0], target[1], target[2], target[3], target[4])
    tgt_r = ik(target[5], target[6], target[7], target[8], target[9])
    # rospy.loginfo("ik = %s" % tgt_r)

    target_angle[0] = tgt_l[1]
    target_angle[1] = tgt_l[2]
    target_angle[2] = tgt_l[3]
    target_angle[3] = tgt_l[4]

    target_angle[4] = tgt_r[1]
    target_angle[5] = tgt_r[2]
    target_angle[6] = tgt_r[3]
    target_angle[7] = tgt_r[4]

    target_angle[8] = target[10]
    target_angle[9] = target[11]
    target_angle[10] = target[12]
    target_angle[11] = target[13]
    target_angle[12] = target[14]
    target_angle[13] = target[15]

    # # CCW
    # target_angle[1] *= -1   # left thigh pitch
    # target_angle[6] *= -1   # right ankle pitch
    # target_angle[9] *= -1  # left shoulder roll
    # target_angle[10] *= -1  # left elbow

    set_servo_directly(time, target_angle)


if __name__ == '__main__':
    # setup ros node
    rospy.loginfo("setting up ros node")
    rospy.init_node('motion')
    sub = rospy.Subscriber('/joy', Joy, joy_callback)
    rate = rospy.Rate(100) # 100Hz = 10ms cycle

    # setup servo motors
    rospy.loginfo("setting servo motors")
    pwm = Adafruit_PCA9685.PCA9685()
    pwm.set_pwm_freq(50)    # 50Hz = 20ms cycle

    # button 0:X 1:A 2:B 3:Y 4:LB 5:RB 6:LT 7:RT 8:Back 9:Start 10:LStick 11:RStick
    # axes 0:R-X 1:R+Y 2:L-X 3:LY 4:P-X 5:PY
    BX = 0  # X
    BA = 1  # A
    BB = 2  # B
    BY = 3  # Y
    BLB = 4 # left button
    BRB = 5 # right button
    BLT = 6 # left trigger
    BRT = 7 # right trigger
    BBack = 8
    BStart = 9
    BLStick = 10
    BRStick = 11
    ARX = 0
    ARY = 1
    ALX = 2
    ALY = 3
    APX = 4
    APY = 5

    # side
    lx0, rx0 =  0.,   0.
    lx1, rx1 = 10., -10.
    lx2, rx2 = 15., -15.
    lxc, rxc = 40., -40.

    # forward
    ly0, ry0 = -25.-20., -25.-20.
    ly1, ry1 =   0.-20.,   0.-20.
    ly2, ry2 =  25.-20.,  25.-20.

    # upward
    lzf, rzf = -148, -148 # first height
    lz0, rz0 = -100, -100
    lz1, rz1 = -140, -140
    lz2, rz2 = -138, -138

    while not rospy.is_shutdown():

        if joy_data.axes[APY] >= 1.0 and not joy_data.buttons[BLT] == 1:
            # forward
            set_servo(    30,  [lx2,ly1,lzf,0.,0., rx2,ry1,rz1,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.]) # kick left
            set_servo(    30,  [lx1,ly1,lz0,0.,0., rx1,ry1,rz1,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.]) # lift left
            while True:
                if joy_data.buttons[BRB] == 1:
                    # right slalom
                    if not joy_data.axes[APY] >= 1.0 or joy_data.buttons[BLT] == 1: break
                    set_servo(30,  [lx2,ly2,lz1,0.,0., rx2,ry0,rz2,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.]) # back right
                    set_servo(30,  [lx1,ly1,lz1,0.,0., rx1,ry1,rz0,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.]) # lift right
                    if not joy_data.axes[APY] >= 1.0 or joy_data.buttons[BLT] == 1: break
                    set_servo(30,  [lx2,ly0,lz2,0.,0., rx2,ry2,rz1,0.,0.,  20.,-15.,-60., -20.,-15.,-60.]) # back left
                    set_servo(30,  [lx1,ly1,lz0,0.,0., rx1,ry1,rz1,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.]) # lift left
                    if not joy_data.axes[APY] >= 1.0 or joy_data.buttons[BLT] == 1: break
                elif joy_data.buttons[BLB] == 1:
                    # left slalom
                    if not joy_data.axes[APY] >= 1.0 or joy_data.buttons[BLT] == 1: break
                    set_servo(30,  [lx2,ly2,lz1,0.,0., rx2,ry0,rz2,0.,0., -20.,-15.,-60.,  20.,-15.,-60.]) # back right
                    set_servo(30,  [lx1,ly1,lz1,0.,0., rx1,ry1,rz0,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.]) # lift right
                    if not joy_data.axes[APY] >= 1.0 or joy_data.buttons[BLT] == 1: break
                    set_servo(30,  [lx2,ly0,lz2,0.,0., rx2,ry2,rz1,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.]) # back left
                    set_servo(30,  [lx1,ly1,lz0,0.,0., rx1,ry1,rz1,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.]) # lift left
                    if not joy_data.axes[APY] >= 1.0 or joy_data.buttons[BLT] == 1: break
                else:
                    # straight
                    if not joy_data.axes[APY] >= 1.0 or joy_data.buttons[BLT] == 1: break
                    set_servo(30,  [lx2,ly2,lz1,0.,0., rx2,ry0,rz2,0.,0.,   0.,-15.,-60.,  20.,-15.,-60.]) # back right
                    set_servo(30,  [lx1,ly1,lz1,0.,0., rx1,ry1,rz0,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.]) # lift right
                    if not joy_data.axes[APY] >= 1.0 or joy_data.buttons[BLT] == 1: break
                    set_servo(30,  [lx2,ly0,lz2,0.,0., rx2,ry2,rz1,0.,0.,  20.,-15.,-60.,   0.,-15.,-60.]) # back left
                    set_servo(30,  [lx1,ly1,lz0,0.,0., rx1,ry1,rz1,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.]) # lift left
                    if not joy_data.axes[APY] >= 1.0 or joy_data.buttons[BLT] == 1: break
            # stop
            set_servo(    50,  [lxc,ly1,lz0,0.,0., rxc,ly1,lz0,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.])
            set_servo(   100,  [lxc,ly1,lz0,0.,0., rxc,ly1,lz0,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.])

        elif joy_data.axes[5] <= -1.0:
            # backward
            set_servo(    30,  [lx2,ly1,lz1,0.,0., rx2,ry1,rzf,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.]) # kick right
            set_servo(    30,  [lx1,ly1,lz1,0.,0., rx1,ry1,rz0,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.]) # lift right
            while True:
                if not joy_data.axes[5] <= -1.0: break
                set_servo(30,  [lx2,ly2,lz2,0.,0., rx2,ry0,rz1,0.,0., -10.,-15.,-60.,  10.,-15.,-60.]) # front left
                set_servo(30,  [lx1,ly1,lz0,0.,0., rx1,ry1,rz1,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.]) # lift left
                if not joy_data.axes[5] <= -1.0: break
                set_servo(30,  [lx2,ly0,lz1,0.,0., rx2,ry2,rz2,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.]) # front right
                set_servo(30,  [lx1,ly1,lz1,0.,0., rx1,ry1,rz0,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.]) # lift right
                if not joy_data.axes[5] <= -1.0: break
            set_servo(    50,  [lx2,ly1,lz0,0.,0., rx2,ry1,lz0,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.])
            set_servo(   100,  [lx2,ly1,lz0,0.,0., rx2,ry1,lz0,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.])

        elif joy_data.axes[4] <= -1.0:
            # right
            while True:
                set_servo(40,  [10.,ly1,lz1,0.,0., -10.,ry1,rz2,-5.,0.,  0.,  0.,-60.,   0.,  0.,-60.]) # kick right
                set_servo(40,  [30.,ly1,lz1,0.,0., -35.,ry1,rz0,-5.,0.,  0.,  0.,-60.,   0.,  0.,-60.]) # lift right
                set_servo(40,  [30.,ly1,lz2,0.,0., -35.,ry1,rz1,-5.,0.,  0.,  0.,-60.,   0.,  0.,-60.]) # kick left
                set_servo(40,  [10.,ly1,lz0,0.,0., -10.,ry1,rz1,-5.,0.,  0.,  0.,-60.,   0.,  0.,-60.]) # lift left
                if not joy_data.axes[4] <= -1.0: break
            set_servo(    50,  [lx2,ly1,lz0,0.,0., rx2,ry1,rz0,0.,0.,  0.,-15.,-60.,   0.,-15.,-60.])
            set_servo(   100,  [lx2,ly1,lz0,0.,0., rx2,ry1,rz0,0.,0.,  0.,-15.,-60.,   0.,-15.,-60.])

        elif joy_data.axes[4] >= 1.0:
            # left
            while True:
                set_servo(40,  [ 10.,ly1,lz2,9.,0., -10.,ry1,rz1,0.,0.,   0.,  0.,-60.,   0.,  0.,-60.]) # kick right
                set_servo(40,  [ 30.,ly1,lz0,9.,0., -30.,ry1,rz1,0.,0.,   0.,  0.,-60.,   0.,  0.,-60.]) # lift right
                set_servo(40,  [ 30.,ly1,lz1,9.,0., -30.,ry1,rz2,0.,0.,   0.,  0.,-60.,   0.,  0.,-60.]) # kick left
                set_servo(40,  [ 10.,ly1,lz1,9.,0., -10.,ry1,rz0,0.,0.,   0.,  0.,-60.,   0.,  0.,-60.]) # lift left
                if not joy_data.axes[4] >= 1.0: break
            set_servo(    50,  [ lx2,ly1,lz0,0.,0.,  rx2,ry1,rz0,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.])
            set_servo(   100,  [ lx2,ly1,lz0,0.,0.,  rx2,ry1,rz0,0.,0.,   0.,-15.,-60.,   0.,-15.,-60.])

        elif joy_data.buttons[5] == 1:
            # turn right
            while True:
                set_servo( 50,  [lx0, 0-20,-120.,0.,0.,  rx0,  0-20,-120.,0.,0.,   0.,  0.,-60.,   0.,  0.,-60.])
                set_servo( 40,  [lx0,40-20,-120.,0.,0.,  rx0,-40-20,-120.,0.,0., -20.,  0.,-60.,  20.,  0.,-60.])
                set_servo( 40,  [lx0,40-20,-120.,0.,0.,  rx0,-40-20,-120.,0.,0., -20.,  0.,-60.,  20.,  0.,-60.])
                set_servo(200,  [lx0, 0-20,-120.,0.,0.,  rx0,  0-20,-120.,0.,0.,   0.,  0.,-60.,   0.,  0.,-60.])
                if not joy_data.buttons[5] == 1: break
            set_servo(    100,  [ 20.,ly1,-110.,0.,0.,  -20.,ly1,-110.,0.,0.,  0.,  0.,-60.,   0.,  0.,-60.])
            set_servo(    200,  [ 20.,ly1,-110.,0.,0.,  -20.,ly1,-110.,0.,0.,  0.,  0.,-60.,   0.,  0.,-60.])

        elif joy_data.buttons[4] == 1:
            # turn left
            while True:
                set_servo( 50,  [lx0,  0-20,-120.,0.,0.,  rx0, 0-20,-120.,0.,0.,   0.,  0.,-60.,   0.,  0.,-60.])
                set_servo( 40,  [lx0,-40-20,-120.,0.,0.,  rx0,40-20,-120.,0.,0.,  20.,  0.,-60., -20.,  0.,-60.])
                set_servo( 40,  [lx0,-40-20,-120.,0.,0.,  rx0,40-20,-120.,0.,0.,  20.,  0.,-60., -20.,  0.,-60.])
                set_servo(200,  [lx0,  0-20,-120.,0.,0.,  rx0, 0-20,-120.,0.,0.,   0.,  0.,-60.,   0.,  0.,-60.])
                if not joy_data.buttons[4] == 1: break
            set_servo(    100,  [ 20.,ly1,-110.,0.,0.,  -20.,ly1,-110.,0.,0.,  0.,  0.,-60.,   0.,  0.,-60.])
            set_servo(    200,  [ 20.,ly1,-110.,0.,0.,  -20.,ly1,-110.,0.,0.,  0.,  0.,-60.,   0.,  0.,-60.])

        elif joy_data.buttons[6] == 1 and joy_data.buttons[2] == 1:
            # get up front
            set_servo(200, [ 0.,0.,-60.,0.,0.,   0.,0.,-60.,0.,0.,    0., 90., 60.,     0., 90., 60.])   # ready arms
            set_servo(100, [ 0.,0.,-60.,0.,0.,   0.,0.,-60.,0.,0.,    0., 90., 60.,     0., 90., 60.])
            set_servo(200, [ 0.,0.,-60.,0.,0.,   0.,0.,-60.,0.,0.,   90., 90., 60.,    90., 90., 60.])  # get ready
            set_servo(100, [ 0.,0.,-60.,0.,0.,   0.,0.,-60.,0.,0.,   90., 90., 60.,    90., 90., 60.])
            set_servo(100, [30.,0.,-30.,0.,0., -30.,0.,-30.,0.,0.,   60.,-45.,-90.,    60.,-45.,-90.])   # get up
            set_servo(100, [30.,0.,-30.,0.,0., -30.,0.,-30.,0.,0.,   30.,-45.,-90.,    30.,-45.,-90.])   # get up
            set_servo(500, [30.,0.,-30.,0.,0., -30.,0.,-30.,0.,0.,   30.,-30.,-90.,    30.,-30.,-90.])
            set_servo(200, [10.,-30.,-130.,0.,0., -10.,-30.,-130.,0.,0.,   0.,-30.,-60.,   0.,-30.,-60.])    # stand up
            set_servo(100, [10.,-30.,-130.,0.,0., -10.,-30.,-130.,0.,0.,   0.,-30.,-60.,   0.,-30.,-60.])

        elif joy_data.buttons[6] == 1 and joy_data.buttons[1] == 1:
            # get up back
            set_servo(200, [ 0.,0.,-60.,0.,0.,   0.,0.,-60.,0.,0.,    0., 90., 60.,       0., 90., 60.])   # ready arms
            set_servo(100, [ 0.,0.,-60.,0.,0.,   0.,0.,-60.,0.,0.,    0., 90., 60.,       0., 90., 60.])
            set_servo(200, [ 0.,0.,-60.,0.,0.,   0.,0.,-60.,0.,0.,   -90., 90., 60.,    -90., 90., 60.])  # get ready
            set_servo(100, [ 0.,0.,-60.,0.,0.,   0.,0.,-60.,0.,0.,   -90., 90., 60.,    -90., 90., 60.])
            set_servo(100, [30.,0.,-30.,0.,0., -30.,0.,-30.,0.,0.,   -60.,-45.,-90.,    -60.,-45.,-90.])   # get up
            set_servo(100, [30.,0.,-30.,0.,0., -30.,0.,-30.,0.,0.,   -30.,-45.,-90.,    -30.,-45.,-90.])   # get up
            set_servo(500, [30.,0.,-30.,0.,0., -30.,0.,-30.,0.,0.,   -30.,-30.,-90.,    -30.,-30.,-90.])
            set_servo(200, [10.,-30.,-130.,0.,0., -10.,-30.,-130.,0.,0.,   0.,-30.,-60.,   0.,-30.,-60.])    # stand up
            set_servo(100, [10.,-30.,-130.,0.,0., -10.,-30.,-130.,0.,0.,   0.,-30.,-60.,   0.,-30.,-60.])

        elif joy_data.buttons[6] == 1 and joy_data.buttons[0] == 1:
            # T pose
            set_servo_directly(50, [0,0,0,0, 0,0,0,0, 0,40,-90, 0,40,-90])
            set_servo_directly(50, [0,0,0,0, 0,0,0,0, 0,40,-90, 0,40,-90])

        # elif joy_data.buttons[6] == 1 and joy_data.buttons[3] == 1:
        #     # throw dice
        #     # sit down
        #     set_servo(100, [  0.,-20.,-80.,0.,0., 0.,-20.,-80.,0.,0., 30.,0.,30.,  30.,0.,30.])
        #     set_servo(100, [  0.,-20.,-80.,0.,0., 0.,-20.,-80.,0.,0., 30.,0.,30.,  30.,0.,30.])
        #     # grab dice
        #     set_servo(100, [  0.,-20.,-80.,0.,0., 0.,-20.,-80.,0.,0., -30.,0.,-90.,  -30.,0.,-90.])
        #     set_servo(100, [  0.,-20.,-80.,0.,0., 0.,-20.,-80.,0.,0., -30.,0.,-90.,  -30.,0.,-90.])
        #     set_servo(100, [  0.,-20.,-80.,0.,0., 0.,-20.,-80.,0.,0., -30.,-60.,-90.,  -30.,-60.,-90.])
        #     set_servo(100, [  0.,-20.,-80.,0.,0., 0.,-20.,-80.,0.,0., -30.,-60.,-90.,  -30.,-60.,-90.])
        #     # stand up
        #     set_servo(100, [ 10.,-20.,-130.,0.,0., -10.,-20.,-130.,0.,0., -30.,-60.,-90.,  -30.,-60.,-90.])
        #     set_servo(100, [ 10.,-20.,-130.,0.,0., -10.,-20.,-130.,0.,0., -30.,-65.,-90.,  -30.,-65.,-90.])
        #     # throw dice
        #     set_servo( 10, [ 10.,-20.,-130.,0.,0., -10.,-20.,-130.,0.,0., -60.,-65.,-90.,  -60.,-65.,-90.])
        #     set_servo( 10, [ 10.,-20.,-130.,0.,0., -10.,-20.,-130.,0.,0., -80.,-50.,-90.,  -80.,-50.,-90.])
        #     set_servo(100, [ 10.,-20.,-130.,0.,0., -10.,-20.,-130.,0.,0., -80.,-50.,-90.,  -80.,-50.,-90.])
        #     # back to default pose
        #     set_servo(100, [ 10.,-20.,-130.,0.,0., -10.,-20.,-130.,0.,0., 30.,0.,-30.,  30.,0.,-30.])

        elif joy_data.buttons[2] == 1:
            # punch right
            set_servo( 20, [ lx1,ly2,-120,0.,0., rx1,ry0,-120,0.,0.,  0.,0.,-30.,   0.,-30.,-90.])
            set_servo( 20, [ lx1,ly2,-120,0.,0., rx1,ry0,-120,0.,0.,  0.,0.,-30.,  90.,-30.,-90.])
            set_servo(100, [ lx1,ly2,-120,0.,0., rx1,ry0,-120,0.,0.,  0.,0.,-30.,  90.,-45.,-90.])

        elif joy_data.buttons[0] == 1:
            # punch left
            set_servo( 20, [ lx1,ly0,-120,0.,0., rx1,ry2,-120,0.,0.,   0.,-30.,-60.,  0.,0.,-30.])
            set_servo( 20, [ lx1,ly0,-120,0.,0., rx1,ry2,-120,0.,0.,  90.,-30.,-90.,  0.,0.,-30.])
            set_servo(100, [ lx1,ly0,-120,0.,0., rx1,ry2,-120,0.,0.,  90.,-45.,-90.,  0.,0.,-30.])

        elif joy_data.buttons[1] == 1:
            # get down
            set_servo(50, [ lx2,ly1,lz0,0.,0., rx2,ry1,rz0,0.,0.,  0.,0.,-30.,   0.,0.,-30.])


        elif joy_data.buttons[8] == 1:
            # power off
            servo_poweroff()
            while not joy_data.buttons[9] == 1:
                pass

        else:
            # default pose
            set_servo(50, [ lx1,ly1,lz1,0.,0., rx1,ry1,rz1,0.,0.,  0.,0.,-30.,   0.,0.,-30.])
            # set_servo_directly(10, [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0])


        rate.sleep()
