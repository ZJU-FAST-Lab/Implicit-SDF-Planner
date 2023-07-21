#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import pygame
from pygame.locals import * 
import sys

import rospy
from std_msgs.msg import Float32MultiArray

pygame.init()
pygame.display.set_caption('grad viewer')

SCREEN_SIZE = SCREEN_WIDTH, SCREEN_HEIGHT = 300, 300 
BASE_SCREEN = pygame.display.set_mode(SCREEN_SIZE)  

Z = 0.0
msg_pub = None
font_Basis_s = pygame.font.SysFont("SimHei",72)

def render():
    BASE_SCREEN.blit(font_Basis_s.render(str(Z), True, (255,255,100)), (SCREEN_WIDTH/2 , SCREEN_HEIGHT/2))

def pubTopic():
    global msg_pub,Z
    msg = Float32MultiArray()
    msg.data.append(Z)
    msg.data.append(0.08)
    msg.data.append(20)
    msg.data.append(40)
    msg.data.append(0)
    msg.data.append(20)
    msg_pub.publish(msg)

def main():
    global msg_pub,Z
    rospy.init_node("GRAD_VIEWER",anonymous=True)

    msg_pub = rospy.Publisher("/check_grad",Float32MultiArray,queue_size=10)

    while True:  

        for event in pygame.event.get():  
            if event.type == pygame.KEYDOWN:
                if event.key == K_w:
                    Z = Z + 0.1
                elif event.key == K_s:
                    Z = Z - 0.1
                elif event.key == K_SPACE:
                    pubTopic()

                elif event.key == K_q:
                    pygame.quit()


        pygame.display.update()
        pygame.time.wait(2)
        pygame.draw.rect(BASE_SCREEN,(0,0,0),((0,0) ,SCREEN_SIZE),0)
        render()
    
    rospy.spin()	

if __name__ == "__main__":
    main()
