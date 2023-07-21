#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

from tkinter import font
from turtle import down
import pygame
import sys
from pygame.locals import * 
from geometry_msgs.msg import Point

import random

import rospy

pygame.init()
pygame.display.set_caption('checkesd_pointer')

SCREEN_SIZE = SCREEN_WIDTH, SCREEN_HEIGHT = 400, 400  
BASE_SCREEN = pygame.display.set_mode(SCREEN_SIZE)  

font_base   = pygame.font.SysFont("SimHei",25)
font_big    = pygame.font.SysFont("SimHei",45)

px = 0
py = 0
pz = 0

point_pub = None
down_keys = []


def blitTextCenter( surface, text, font, text_center, color ):
    text_obj  = font.render(text, True, color)
    text_rect = text_obj.get_rect(center = text_center)
    surface.blit( text_obj, text_rect )

def blitTextLeft( surface, text, font, leftup_coord , color ):
    text_obj  = font.render(text, True, color)
    surface.blit( text_obj, leftup_coord )

def update():
    global down_keys, px,py,pz, point_pub

    step = 0.001
    if K_w in down_keys:
        px = px + step
    if K_s in down_keys:
        px = px - step
    if K_a in down_keys:
        py = py + step
    if K_d in down_keys:
        py = py - step
    if K_UP in down_keys:
        pz = pz + step
    if K_DOWN in down_keys:
        pz = pz - step
    
    if down_keys != []:
        pt_msg = Point()
        pt_msg.x = px
        pt_msg.y = py
        pt_msg.z = pz
        point_pub.publish( pt_msg )


def render():
    global px, py, pz, font_base
    blitTextCenter( BASE_SCREEN, "AWSD:moving on a plane, UP and DOWN", font_base, (200,300), (255,255,100))

    string_x = "x = " + str(round(px,2))
    string_y = "y = " + str(round(py,2))
    string_z = "z = " + str(round(pz,2))
    blitTextLeft( BASE_SCREEN, string_x, font_big, (100,100), (255,255,100))
    blitTextLeft( BASE_SCREEN, string_y, font_big, (100,150), (255,255,100))
    blitTextLeft( BASE_SCREEN, string_z, font_big, (100,200), (255,255,100))

def main():
    global point_pub

    rospy.init_node("pointer_node", anonymous=True)
    point_pub  =  rospy.Publisher("/check_grad2", Point, queue_size=10)

    while True:  # 主循环
        for event in pygame.event.get():  # 遍历所有事件
            if event.type == pygame.KEYDOWN:
                if event.key == K_w:
                    down_keys.append(event.key)
                elif event.key == K_a:
                    down_keys.append(event.key)
                elif event.key == K_s:  
                    down_keys.append(event.key)
                elif event.key == K_d:            
                    down_keys.append(event.key)
                elif event.key == K_UP:                  
                    down_keys.append(event.key)
                elif event.key == K_DOWN:
                    down_keys.append(event.key)
                elif event.key == K_ESCAPE:
                    pygame.quit()
            if event.type == pygame.KEYUP:
                if event.key == K_w:
                    down_keys.remove(event.key)
                elif event.key == K_a:
                    down_keys.remove(event.key)
                elif event.key == K_s:  
                    down_keys.remove(event.key)
                elif event.key == K_d:            
                    down_keys.remove(event.key)
                elif event.key == K_UP:                  
                    down_keys.remove(event.key)
                elif event.key == K_DOWN:
                    down_keys.remove(event.key)

        pygame.display.update()
        pygame.draw.rect(BASE_SCREEN,(0,0,0),((0,0) ,SCREEN_SIZE),0)
        update()
        render()
    
    rospy.spin()	

if __name__ == "__main__":
    main()