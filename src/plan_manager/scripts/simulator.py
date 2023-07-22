#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import pygame
import sys
from pygame.locals import * 
from nav_msgs.msg import Odometry

import random

import rospy

from utils.msg import debug
pygame.init()
pygame.display.set_caption('debug')

SCREEN_SIZE = SCREEN_WIDTH, SCREEN_HEIGHT = 300, 300  # 设置窗口大小
BASE_SCREEN = pygame.display.set_mode(SCREEN_SIZE)  # 底层窗口

# odom = [
#     [0,0,1],
#     [2,0,1],
#     [4,0,1],
# ]

# odom_pub = []

timer = None

# vy = 0.0

# def pubOdoms(e):
#     global odom_pub,odom, vy
#     odomt = Odometry()
#     odomt.header.frame_id = "map"
#     for i in range(0,3):
#         odomt.pose.pose.position.x = odom[i][0]
#         odomt.pose.pose.position.y = odom[i][1]
#         odomt.pose.pose.position.z = odom[i][2]
#         odom_pub[i].publish(odomt)
#         odom[i][1] = odom[i][1] + 0.05 * vy
#         if vy > 0.5:
#             odom[i][2] = odom[i][2] + random.randint(0,10)/400.0
    

def main():
    global obstalce_pub

    rospy.init_node("debug_node",anonymous=True)
    obstalce_pub=rospy.Publisher("/debug",debug,queue_size=10)


    # timer = rospy.Timer(rospy.Duration(0.05),pubOdoms)

    
    while True:  # 主循环

        for event in pygame.event.get():  # 遍历所有事件
            if event.type == pygame.KEYDOWN:
                if event.key == K_w:
                    print("!!!!!ENABLE DEBUG!!!!")
                elif event.key == K_0:
                    # sys.stdout.flush()
                    type=input("opteration type:")
                    msg=debug()
                    msg.operation_type=int(type)
                    msg.id1=int(type)
                    msg.id2=0
                    msg.id3=0
                    data1=float(input("data1:"))
                    data2=float(input("data2:"))
                    data3=float(input("data3:"))
                    data4=float(input("data4:"))
                    data5=float(input("data5:"))
                    data6=float(input("data6:"))
                    data7=float(input("data7:"))
                    data8=float(input("the last data7:"))
                    msg.data1=data1
                    msg.data2=data2
                    msg.data3=data3
                    msg.data4=data4
                    msg.data5=data5
                    msg.data6=data6
                    msg.data7=data7
                    msg.data8=data8
                    obstalce_pub.publish(msg)
                elif event.key == K_q:
                    pygame.quit()


        pygame.display.update()
        pygame.draw.rect(BASE_SCREEN,(0,0,0),((0,0) ,SCREEN_SIZE),0)
    
    rospy.spin()	

if __name__ == "__main__":
    main()