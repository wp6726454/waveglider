#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from sensor_float_msgs.msg import WTST_msg
import WTST_Pro_msg
import math
from math import pi
import numpy as np
import copy


RADIUS = 4
WTSTinput = [0, 0, 0, 0, 0, 0, 0, 0, 0]

class Maker:
    def __init__(self, points):
        self.points = points
        self.s = points[0]
        self.e = points[1]
        self.pointer = 1

    def getdirection(self, state):
        x = state['x']
        y = state['y']
        phi = state['phi']
        x0, y0 = self.s
        x1, y1 = self.e

        if ((x1 - x) ** 2 + (y1 - y) ** 2) < RADIUS ** 2:
            if self.pointer == (len(self.points) - 1):
                return -1000
            self.s = self.points[self.pointer]
            self.e = self.points[self.pointer + 1]
            self.pointer += 1

        p = x1 - x0
        q = y1 - y0

        a = (x * (p ** 2) + y * p * q - y0 * p * q + (q ** 2) * x0) / (p ** 2 + q ** 2)
        b = (x * p * q + y * (q ** 2) + (p ** 2) * y0 - p * q * x0) / (p ** 2 + q ** 2)

        midpoint_x = (a + x1)/2
        midpoint_y = (b + y1)/2

        return math.atan2((midpoint_y - y), (midpoint_x - x))

def pretreat(wtstinput):
    wtst=copy.deepcopy(wtstinput)
    wtst[4] = wtst[4] * np.pi / 180
    wtst[5] = (wtst[5]+5.04 )* np.pi / 180
    wtst[6] = wtst[6] * np.pi / 180
    wtst[2] = wtst[2] * np.pi / 180
    wtst[3]=wtst[3]*0.5144
    wtst[7] = wtst[7] * np.pi / 180
    if wtst[6]>np.pi:
        wtst[6]=wtst[6]-2*np.pi
    if wtst[2]>np.pi:
        wtst[2]=wtst[2]-2*np.pi
    if wtst[7]>np.pi:
        wtst[7]=wtst[7]-2*np.pi
    return wtst

def wtst_callback(data):
    global WTSTinput,wtst_time,wtstnum
    #print ('start')
    #rospy.loginfo("I heard %f", data.roll)
    WTSTinput[0] = data.PosX
    WTSTinput[1] = data.PosY
    WTSTinput[2] = data.DegreeTrue
    WTSTinput[3] = data.SpeedKnots
    WTSTinput[4] = data.Roll
    WTSTinput[5] = data.Pitch
    WTSTinput[6] = data.Yaw
    WTSTinput[7] = data.WindAngle
    WTSTinput[8] = data.WindSpeed
    wtst_time=data.header.stamp
    # WTSTinput=copy.deepcopy([data.PosX,data.PosY,data.DegreeTrue,data.SpeedKnots,data.Roll,data.Pitch,data.Yaw,data.WindAngle,data.WindSpeed])
    # yyaw=data.Yaw
    # rospy.loginfo("I heard WTST %f", data.Yaw)
    # print('callback',WTSTinput[6])

def listener():
    rospy.init_node('Course', anonymous=True)
    rospy.Subscriber('wtst', WTST_msg, wtst_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
