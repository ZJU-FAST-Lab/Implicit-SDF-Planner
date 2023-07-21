#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import pygame
import sys
import rospy
from std_msgs.msg import Float64MultiArray
WHITE = (0xFFFFFF)
BLACK = (0x000000)
RED = (0xFF0000)
GREEN = (0x00FF00)
BLUE = (0x0000FF)
YELLOW = (0xFFFF00)
CYAN = (0x00FFFF)
MAGENTA = (0xFF00FF)
ORANGE = (0xFFA500)
PINK = (0xFFC0CB)
PURPLE = (0x800080)
GRAY = (0x808080)
LIGHT_BLUE = (173, 216, 230)

pygame.init()

window_width = 1200
window_height = 600
screen=pygame.display.set_mode((window_width, window_height), pygame.HWSURFACE | pygame.DOUBLEBUF)


surface = pygame.Surface(screen.get_size())

surface.fill((0, 0, 0))


pygame.display.set_caption("SDF Visualization")
AXISOFFSET=40

Pointcolor1 = ORANGE
Pointcolor2 = PURPLE
point_radius = 5
data_points = []

Time_min=int(0)
Time_max=int(11)
SDF_max=int(25)
SDF_min=int(-5)
Time_mincandidate=int(0)
Time_maxcandidate=int(0)
SDF_maxcandidate=-int(555)
SDF_mincandidate=int(555)


axis_color = (255, 255, 255)  # 白色
axis_thickness = 2
tick_length = 10
tick_font = pygame.font.SysFont("SimHei",22)
lable_font = pygame.font.SysFont("SimHei",30)
data_font = pygame.font.SysFont("黑体",20)


def callback(msg):
    global screen,surface,Time_min,Time_max,SDF_max,SDF_min,data_points,Time_mincandidate,Time_maxcandidate,SDF_maxcandidate,SDF_mincandidate
    time = msg.data[0]
    sdf = msg.data[1]
    sdfneighbor=msg.data[2]
    if time<0 and sdf<0:
        print("Clear data now!")
        #重新更新范围以便更好可视化
        Time_max=Time_maxcandidate+1
        SDF_max=SDF_maxcandidate+5
        SDF_min=SDF_mincandidate-5
        print("Timemax:{}, SDF_max:{},SDF_min:{}".format(Time_max,SDF_max,SDF_min))
        data_points=[]
        surface.fill((0, 0, 0))
        screen.blit(surface,(0, 0))
  
        return
    
 
    if sdf<SDF_mincandidate:
        SDF_mincandidate=int(sdf)

    if sdf>SDF_maxcandidate:
        SDF_maxcandidate=int(sdf)
    if time>Time_maxcandidate:
        Time_maxcandidate=int(time)


    data_points.append((time, sdf,sdfneighbor))
    draw(data_points)

    lastx = int((time - Time_min) / (Time_max - Time_min) * (window_width-AXISOFFSET*2))  
    lasty = int(window_height-AXISOFFSET*2 - (sdf - SDF_min) / (SDF_max - SDF_min) * (window_height-AXISOFFSET*2))  
    lastyneighbor = int(window_height-AXISOFFSET*2 - (sdfneighbor - SDF_min) / (SDF_max - SDF_min) * (window_height-AXISOFFSET*2))  # 纵轴，反向显示

    lastx=lastx+AXISOFFSET
    lasty=lasty+AXISOFFSET
    lastyneighbor=lastyneighbor+AXISOFFSET
    pygame.draw.circle(screen, RED, (lastx,lasty), point_radius*1.5) 
    pygame.draw.circle(screen, RED, (lastx,lastyneighbor), point_radius*1.5)




def draw_axis():
    global screen,surface,Time_min,Time_max,SDF_max,SDF_min,data_points
    surface.fill((0, 0, 0))
    pygame.draw.line(surface, axis_color, (AXISOFFSET, window_height-AXISOFFSET), (window_width-AXISOFFSET,window_height-AXISOFFSET), axis_thickness)  # 绘制横轴
    pygame.draw.line(surface, axis_color, (AXISOFFSET, AXISOFFSET), (AXISOFFSET, window_height-AXISOFFSET), axis_thickness)  # 绘制纵轴

    for time in range(Time_min, Time_max + 1):
        x = int((time - Time_min) / (Time_max - Time_min) * (window_width-AXISOFFSET*2)+AXISOFFSET)
        pygame.draw.line(surface, axis_color, (x,window_height-AXISOFFSET - tick_length // 2),
                         (x, window_height-AXISOFFSET + tick_length // 2), axis_thickness)
        tick_text = tick_font.render(str(time), True, axis_color)
        surface.blit(tick_text, (x - tick_text.get_width() // 2, window_height-AXISOFFSET + tick_length // 2))

    tick_text = lable_font.render(str("Time(s)"), True, axis_color)
    surface.blit(tick_text, (window_width-80, window_height-80))


    for sdf in range(SDF_min, SDF_max + 1, 5):
        y = int(window_height-2*AXISOFFSET - (sdf - SDF_min) / (SDF_max - SDF_min) * (window_height-2*AXISOFFSET)+AXISOFFSET)
        pygame.draw.line(surface, axis_color, (AXISOFFSET - tick_length // 2, y),
                         (AXISOFFSET + tick_length // 2, y), axis_thickness)
        tick_text = tick_font.render(str(sdf), True, axis_color)
        surface.blit(tick_text, (AXISOFFSET - tick_length - tick_text.get_width(), y - tick_text.get_height() // 2))

    tick_text = lable_font.render(str("SDF(m)"), True, axis_color)
    surface.blit(tick_text, (10, 10))

def draw(data):
    global screen,surface,Time_min,Time_max,SDF_max,SDF_min,data_points
    draw_axis()
    for point in data:
            time,sdf,sdfneighbor=point
            x = int((time - Time_min) / (Time_max - Time_min) * (window_width-AXISOFFSET*2))  
            y = int(window_height-AXISOFFSET*2 - (sdf - SDF_min) / (SDF_max - SDF_min) * (window_height-AXISOFFSET*2))  
            yneighbor = int(window_height-AXISOFFSET*2 - (sdfneighbor - SDF_min) / (SDF_max - SDF_min) * (window_height-AXISOFFSET*2))  # 纵轴，反向显示

            x=x+AXISOFFSET
            y=y+AXISOFFSET
            yneighbor=yneighbor+AXISOFFSET
            pygame.draw.circle(surface, Pointcolor1, (x,y), point_radius)
            pygame.draw.circle(surface, PURPLE, (x,yneighbor), point_radius) 
    

    text_surface1 = data_font.render("P1 t: %.2f, SDF: %.2f" % (data_points[-1][0], data_points[-1][1]), True, LIGHT_BLUE) 
    text_surface2 = data_font.render("P2 t: %.2f, SDF: %.2f" % (data_points[-1][0], data_points[-1][2]), True, PURPLE)
    lastx = int((data_points[-1][0] - Time_min) / (Time_max - Time_min) * (window_width-AXISOFFSET*2))  
    lasty1 = int(window_height-AXISOFFSET*2 - (data_points[-1][1] - SDF_min) / (SDF_max - SDF_min) * (window_height-AXISOFFSET*2))  
    lasty2 = int(window_height-AXISOFFSET*2 - (data_points[-1][2] - SDF_min) / (SDF_max - SDF_min) * (window_height-AXISOFFSET*2)) 
    lastx=lastx+AXISOFFSET
    lasty1=lasty1+AXISOFFSET
    lasty2=lasty2+AXISOFFSET
    surface.blit(text_surface1, (lastx-160, lasty1- 6))
    surface.blit(text_surface2, (lastx-160, lasty2- 6))

    screen.blit(surface,(0, 0))
    



def main():
    global screen,surface,Time_min,Time_max,SDF_max,SDF_min,data_points
    running = True
    rospy.init_node("SDF_visualization" ,anonymous=True)
    rospy.Subscriber("/sdf_vis", Float64MultiArray, callback)

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_k:
                    pygame.display.flip()
                    print("Keydown now")


        pygame.display.flip()
             


if __name__ == "__main__":
    main()