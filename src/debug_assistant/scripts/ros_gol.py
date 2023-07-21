#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

debug_publisher = None

def pubDebugCMD(cmd, param_list):
    global debug_publisher
    msg      = Float64MultiArray()
    msg.data = [cmd]
    for item in param_list:
        msg.data.append(item)
    debug_publisher.publish(msg)

def pubSDFLayerVis(xmin, xmax, ymin, ymax, z):
    global debug_publisher
    msg = Float64MultiArray()
    msg.data = [2, xmin, xmax, ymin, ymax, z, 0.1]
    debug_publisher.publish(msg)