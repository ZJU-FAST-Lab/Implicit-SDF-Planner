#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
from surfaces   import *
from algorithms import *
from input import input_manager

import json
import os
from ros_gol import pubDebugCMD, pubSDFLayerVis

SELECTOR_SIZE = 50

SEL_CLICK_NONE = 0
SEL_CLICK_LEFT = 1
SEL_CLICK_RIGHT = 2
SEL_CLICK_MID = 3

ITEM_HEIGHT = 30
def readItemsFromJson( json_file):
    with open(json_file, 'r') as f:
        data = json.load(f)

    pos_x = 0
    pos_y = 0
    btns = []
    dict = {}
    for item in data['operations']:
        name = item['name']
        iid  = item['id']
        title = str(iid) + ": " + name
        text_btn = TextButtonLeft( 0, (pos_x, pos_y),(250, ITEM_HEIGHT*0.9), iid, title )
        btns.append( text_btn )
        pos_y += ITEM_HEIGHT
        dict[iid] = name
    
    return btns, dict



class StaticTitle():
    def __init__(self, coord = (0,0), title = "title", color = COLOR_YELLOW):
        self.title = title
        self.coord = coord
        self.color = color

    def render( self, surface ):
        blitTextCenter(surface, self.title, font_scribe26, self.coord, self.color )
    
    def onMouseMotion(self, mouse_x, mouse_y ):
        pass
        
    def onMouseClick(self, mouse_x, mouse_y, btn ):
        pass

    def update(self):
        pass

class FlashTitle():
    def __init__(self, coord = (0,0), title = "title", color = COLOR_WHITE):
        self.title   = title
        self.coord   = coord
        self.color   = color
        self.alpha   = 0.7
        self.bgcolor = colorGradient(STATE_WINDOW_COLOR, self.color, self.alpha)

    def render( self, surface ):
        pygame.draw.rect( surface, self.bgcolor, ((0, self.coord[1] - 10 ), ( COMPONENT_WIDTH, 20)), 0 )
        blitTextCenter(surface, self.title, font_scribe26, self.coord, self.color )
    
    def onMouseMotion(self, mouse_x, mouse_y ):
        pass
        
    def onMouseClick(self, mouse_x, mouse_y, btn ):
        pass

    def reset(self, new_title, extraParam):
        self.title = new_title
        self.alpha = 0.7
        if len(extraParam) != 0:
            if extraParam[0] == 'e':
                self.color = COLOR_RED
        else:
            self.color = COLOR_WHITE

    def update(self):
        self.alpha -= 0.01
        if self.alpha < 0:
            self.alpha = 0
        self.bgcolor = colorGradient(STATE_WINDOW_COLOR, self.color, self.alpha)
    

class Selector():
    def __init__(self, coord = (0,0), size = (100,50), id = 0):
        self.id           = id
        self.color_static = SEL_COLOR
        self.color_hover  = SEL_COLOR_HOVER
        self.color        = self.color_static
        self.size         = size
        self.box          = size
        self.rect         = (coord, size)
        self.cscale       = 0
        self.cdir         = 0
        self.coord        = coord
        self.stroke       = False

    
    def render( self, surface ):
        pygame.draw.rect( surface, self.color, (self.coord, self.box) , 2, border_radius = 5 )
        if self.stroke == True:
            pygame.draw.rect( surface, SEL_COLOR_CUSTOM , (self.coord, self.box) , 2, border_radius = 5 )

    def onScroll(self, scroll_x, scroll_y):
        self.coord = (self.coord[0] + scroll_x, self.coord[1] + scroll_y)

    def onMouseMotion(self, mouse_x, mouse_y ):
        if isMouseInWindow( mouse_x, mouse_y, self.coord, self.box ) == True:
            self.cdir = 5
        else:
            self.cdir = -2
        
    
    def onMouseClick(self, mouse_x, mouse_y, btn ):
        if btn == MOUSEBUTTON_LEFT:
            if isMouseInWindow( mouse_x, mouse_y, self.coord, self.box ) == True:
                return SEL_CLICK_LEFT
        
        if btn == MOUSEBUTTON_RIGHT:
            if isMouseInWindow( mouse_x, mouse_y, self.coord, self.box ) == True:       
                return SEL_CLICK_RIGHT
        
        if btn == MOUSEBUTTON_MID:
            if isMouseInWindow( mouse_x, mouse_y, self.coord, self.box ) == True:       
                return SEL_CLICK_MID

        return SEL_CLICK_NONE

    def update(self):
        if self.cdir != 0:
            self.cscale = self.cscale + self.cdir * FRAME_GAP
            self.cscale = limitValue( self.cscale, 0.0, 1.0 )
            self.color  = colorGradient( self.color_static, self.color_hover, self.cscale )

class Button(Selector):
    def __init__(self, radius = 5 , coord = (0,0), size = (100,50), id = 0):
        super(Button, self).__init__(coord, size, id)
        self.radius = radius

    def render(self, surface):
        pygame.draw.rect( surface, self.color, self.rect ,width = 0, border_radius = self.radius )


class TextButton(Button):
    def __init__(self, radius = 5 ,coord = (0,0), size = (20,10), id = 0, title = "button"):
        super(TextButton, self).__init__(radius, coord, size, id)
        self.title        = title
        self.radius       = radius
        self.center_coord = (coord[0] + 0.5*size[0] , coord[1] + 0.5*size[1])
    
    def render( self, surface ):
        pygame.draw.rect( surface, self.color, self.rect , width = 0, border_radius = self.radius )
        blitTextCenter(surface, self.title, font_scribe26, self.center_coord, COLOR_WHITE)

class TextButtonLeft(Button):
    def __init__(self, radius = 5 ,coord = (0,0), size = (20,10), id = 0, title = "button"):
        super(TextButtonLeft, self).__init__(radius, coord, size, id)
        self.title        = title
        self.radius       = radius
        self.left_coord = (coord[0]+5 , coord[1] + 0.5*size[1] - 9)
    
    def onScroll(self, scroll_x, scroll_y):
        super().onScroll(scroll_x, scroll_y)
        self.left_coord = (self.coord[0]+5 , self.coord[1] + 0.5*self.size[1] - 9)
        self.rect         = (self.coord, self.size)
    
    def render( self, surface ):
        pygame.draw.rect( surface, self.color, self.rect , width = 0, border_radius = self.radius )
        blitTextLeft(surface, self.title, font_scribe20, self.left_coord, COLOR_WHITE)

class TextSwitchButton(Button):
    def __init__(self, radius = 5 ,coord = (0,0), size = (20,10), id = 0, title = "button"):
        super(TextSwitchButton, self).__init__(radius, coord, size, id)
        self.title        = title
        self.radius       = radius
        self.center_coord = (coord[0] + 0.5*size[0] , coord[1] + 0.5*size[1])
        self.on           = False
    
    def render( self, surface ):
        if self.on:
            self.color = self.color_hover
        pygame.draw.rect( surface, self.color, self.rect , width = 0, border_radius = self.radius )
        # st = self.title + " OFF"
        # if self.on:
        #     st = self.title + " ON"
        blitTextCenter(surface, self.title, font_scribe26, self.center_coord, COLOR_WHITE)
    
    def onMouseClick(self, mouse_x, mouse_y, btn ):
        if btn == MOUSEBUTTON_LEFT:
            if isMouseInWindow( mouse_x, mouse_y, self.coord, self.box ) == True:
                self.on = not self.on
                return SEL_CLICK_LEFT
        
        if btn == MOUSEBUTTON_RIGHT:
            if isMouseInWindow( mouse_x, mouse_y, self.coord, self.box ) == True:       
                return SEL_CLICK_RIGHT
        
        if btn == MOUSEBUTTON_MID:
            if isMouseInWindow( mouse_x, mouse_y, self.coord, self.box ) == True:       
                return SEL_CLICK_MID


class Item(TextButton):
    def __init__(self, radius = 5 ,coord = (0,0), size = (20,10), id = 0, title = "button", color = COLOR_WHITE):
        super(Item, self).__init__(radius, coord, size, id, title)
        self.item_color = color
        self.height_9   = size[1] * 0.9
        self.height_1   = size[1] * 0.1
        self.item_coord = (coord[0] + 10 ,  coord[1] + 5 * self.height_1)
        self.title_coord = (coord[0] + size[1] + self.height_1, coord[1] + self.height_1 + 2)
        self.on           = False
    
    def onScroll(self, scroll_x, scroll_y):
        super().onScroll(scroll_x, scroll_y)
        self.left_coord = (self.coord[0]+5 , self.coord[1] + 0.5*self.size[1] - 9)
        self.rect         = (self.coord, self.size)
        self.item_coord   = (self.coord[0] + 10 ,  self.coord[1] + 5 * self.height_1)
        self.title_coord  = (self.coord[0] + self.size[1] + self.height_1, self.coord[1] + self.height_1 + 2)
    
    def render( self, surface ):
        pygame.draw.rect( surface, self.color, self.rect , width = 0, border_radius = self.radius )
        if self.on:
            pygame.draw.rect( surface, COLOR_YELLOW, self.rect , width = 1, border_radius = self.radius )
        pygame.draw.circle( surface, self.item_color, self.item_coord, 3 , width = 0 )
        blitTextLeft( surface, self.title, font_scribe18, self.title_coord, COLOR_WHITE )
        # blitTextCenter(surface, self.title, font_scribe26, self.center_coord, COLOR_WHITE)
    
    def onMouseClick(self, mouse_x, mouse_y, btn ):
        if btn == MOUSEBUTTON_LEFT:
            if isMouseInWindow( mouse_x, mouse_y, self.coord, self.box ) == True:
                self.on = not self.on
                return SEL_CLICK_LEFT
        if btn == MOUSEBUTTON_RIGHT:
            if isMouseInWindow( mouse_x, mouse_y, self.coord, self.box ) == True:       
                return SEL_CLICK_RIGHT
        if btn == MOUSEBUTTON_MID:
            if isMouseInWindow( mouse_x, mouse_y, self.coord, self.box ) == True:       
                return SEL_CLICK_MID
        return SEL_CLICK_NONE

class SettingNum(TextButton):
    def __init__(self, radius = 5 ,coord = (0,0), size = (20,10), id = 0, title = "X:", value = 3.14):
        super(SettingNum, self).__init__(radius, coord, size, id, title)
        self.value       = value
        self.hold        = False
        self.title_coord = (coord[0] - 20, self.center_coord[1])
    
    def render( self, surface ):
        # pygame.draw.rect( surface, self.item_color, self.item_rect , width = 0, border_radius = 5 )
        blitTextCenter( surface, self.title, font_scribe26, self.title_coord, COLOR_WHITE )
        blitTextCenter( surface, "{:.2f}".format(self.value), font_scribe26, self.center_coord, COLOR_WHITE )
    
    def onMouseMotion(self, mouse_x, mouse_y ):
        if self.hold == True:
            self.value += 0.15498 * input_manager.mouse_x_vel


class ScrollBar(Selector):
    def __init__( self , box = ((0,0),(10,10)), value_low = 0, value_high = 10, value_page = 2 ,surface = COMPONENT_WINDOW ):
        super(ScrollBar, self).__init__()
        self.color_static = SCROLL_BORDER_COLOR
        self.color_hover  = SCROLL_HOVER_COLOR
        self.color = self.color_static
        self.surface = surface

        self.icon = None
        self.box = box
        self.act_box = box
        self.value_low  = value_low
        self.value_high = value_high
        self.value_page = value_page
        self.value = value_low
        self.enable = False
    
    def onMouseMotion(self, mouse_x, mouse_y ):
        if isMouseInWindow( mouse_x, mouse_y, self.act_box[0], self.act_box[1] ) == True:
            self.cdir = 5
        else:
            self.cdir = -2
    
    def changeRange(self, value_low, value_high, value):
        self.value_low  = value_low
        self.value_high = value_high
        self.value = value
    
    def render(self ):

        if self.value_high - self.value_low <= 5:
            self.act_box = self.box
        else:
            gap = (self.value_high - self.value_low)
            h  = (self.box[1][1] - self.box[1][0]) * ( self.value_page / (gap+self.value_page) ) 
            up = self.box[0][1] + (self.box[1][1] - h ) * ( 1 - (self.value - self.value_low) / gap  )
            self.act_box = ((self.box[0][0], up ) ,( self.box[1][0] , h ))

        pygame.draw.rect( self.surface, SCROLL_BORDER_COLOR, self.box, 1 )
        pygame.draw.rect( self.surface, self.color, self.act_box, 0 )

class VarMonitor:
    def __init__(self, coord_var = (0,0), size_var = (20,10), coord_grad = (0,50), size_grad = (20,10), id = 0, var = -3.14, last_var = 4.18, grad_ana = 20, var_change = -5):
        self.coord_var  = coord_var
        self.size_var   = size_var
        self.coord_grad = coord_grad
        self.size_grad  = size_grad
        self.id         = id
        self.var        = var
        self.last_var   = last_var
        self.grad_ana   = grad_ana
        self.var_change   = var_change
    
    def onScroll(self, scroll_x, scroll_y):
        self.coord_var  = (self.coord_var[0] + scroll_x, self.coord_var[1] + scroll_y)
        self.coord_grad = (self.coord_grad[0] + scroll_x, self.coord_grad[1] + scroll_y)
    
    def render( self, surface, max_var, max_grad ):
        if max_var == 0:
            max_var = 999999999
        if max_grad == 0:
            max_grad = 999999999

        coord_var = self.coord_var
        size_var  = (self.size_var[0], self.size_var[1] * abs(self.var) / max_var)
        if(self.var > 0):
            coord_var = (self.coord_var[0], self.coord_var[1] - self.size_var[1] * self.var / max_var)
        
        coord_grad = self.coord_grad
        size_grad  = (self.size_grad[0], self.size_grad[1] * abs(self.grad_ana) / max_grad)
        if(self.grad_ana > 0):
            coord_grad = (self.coord_grad[0], self.coord_grad[1] - self.size_grad[1] * self.grad_ana / max_grad)
        
        coord_last_var = self.coord_var
        size_last_var  = (self.size_var[0], self.size_var[1] * abs(self.last_var) / max_var)
        if(self.last_var > 0):
            coord_last_var = (self.coord_var[0], self.coord_var[1] - self.size_var[1] * self.last_var / max_var)
        
        coord_var_change = self.coord_grad
        size_var_change  = (self.size_grad[0], self.size_grad[1] * abs(self.var_change) / max_grad)
        if(self.var_change > 0):
            coord_var_change = (self.coord_grad[0], self.coord_grad[1] - self.size_grad[1] * self.var_change / max_grad)
        
        if( abs(self.var) < abs(self.last_var) ):
            pygame.draw.rect(surface, COLOR_DARK_WHITE, (coord_last_var, size_last_var), 0)
            pygame.draw.rect(surface, COLOR_DARK_RED, (coord_var, size_var), 0)
        
        else:
            pygame.draw.rect(surface, COLOR_DARK_RED, (coord_var, size_var), 0)
            pygame.draw.rect(surface, COLOR_DARK_WHITE, (coord_last_var, size_last_var), 0)
        
        if( abs(self.grad_ana) < abs(self.var_change) ):
            pygame.draw.rect(surface, COLOR_DARK_WHITE, (coord_var_change, size_var_change), 0)
            pygame.draw.rect(surface, COLOR_DARK_RED, (coord_grad, size_grad), 0)
        
        else:
            pygame.draw.rect(surface, COLOR_DARK_RED, (coord_grad, size_grad), 0)
            pygame.draw.rect(surface, COLOR_DARK_WHITE, (coord_var_change, size_var_change), 0)
        
        pygame.draw.line(surface, COLOR_BLUE,   self.coord_var, (self.coord_var[0] + self.size_var[0], self.coord_var[1]), 1)
        pygame.draw.line(surface, COLOR_ORANGE, self.coord_grad, (self.coord_grad[0] + self.size_grad[0], self.coord_grad[1]), 1)
            
    def onMouseMotion(self, mouse_x, mouse_y ):
        pass
        
    def onMouseClick(self, mouse_x, mouse_y, btn ):
        pass

    def updateData(self, var, grad_ana):
        self.last_var = self.var
        self.var      = var
        self.grad_ana = grad_ana
        self.var_change = var - self.last_var

    def update(self):
        pass


class VarMonitorManager:
    def __init__(self, size = (10,20), begin_x = 0, begin_y_var = 0, begin_y_grad = 0):
        self.max_var  = 100
        self.max_grad = 100
        self.monitor_var_size = size
        self.begin_x = begin_x
        self.begin_y_var  = begin_y_var
        self.begin_y_grad = begin_y_grad
        self.current_x    = 0
        self.monitor_list  = []
        self.monitor_count = 0
        self.id            = -100
    
    def updateMaxBound(self):
        max_var  = 0
        max_grad = 0
        for monitor in self.monitor_list:
            if(abs(monitor.var) > max_var):
                max_var = abs(monitor.var)
            if(abs(monitor.last_var) > max_var):
                max_var = abs(monitor.last_var)
            if(abs(monitor.grad_ana) > max_grad):
                max_grad = abs(monitor.grad_ana)
            if(abs(monitor.var_change) > max_grad):
                max_grad = abs(monitor.var_change)
        
        self.max_grad = max_grad
        self.max_var  = max_var
    
    def clearMonitor(self):
        self.monitor_list  = []
        self.current_x     = 0
        self.monitor_count = 0
    
    def onScroll(self, scroll_x, scroll_y):
        for monitor in self.monitor_list:
            monitor.onScroll(scroll_x, scroll_y)
    
    def appendMonitor(self, gap):
        self.current_x += gap
        monitor = VarMonitor((self.begin_x + self.current_x, self.begin_y_var), self.monitor_var_size, (self.begin_x + self.current_x, self.begin_y_grad), self.monitor_var_size, self.monitor_count)
        self.monitor_list.append(monitor)
        self.monitor_count += 1
        self.current_x += self.monitor_var_size[0]
    
    def updateData(self, var_list, grad_ana_list):
        if len(var_list) != self.monitor_count:
            print("Error: var_list length error")
            return
        if len(grad_ana_list) != self.monitor_count:
            print("Error: grad_ana_list length error")
            return

        for i in range(self.monitor_count):
            self.monitor_list[i].updateData(var_list[i], grad_ana_list[i])
        self.updateMaxBound()

    def render( self, surface ):
        for monitor in self.monitor_list:
            monitor.render(surface, self.max_var, self.max_grad)
    
    def onMouseMotion(self, mouse_x, mouse_y ):
        pass
        
    def onMouseClick(self, mouse_x, mouse_y, btn ):
        return SEL_CLICK_NONE, -1

    def update(self):
        pass

class Curve:
    def __init__(self, coord, size, max_count = 10000):
        self.coord = coord
        self.size  = size
        self.data_total  = []
        self.data_pos    = []
        self.data_other  = []

        self.data_count = 0
        self.max_count = max_count
        self.x_scale = 100  # 水平方向可以显示多少个数据
        self.x_pos   = 0
        self.y_max   = 30
        self.y_min  = 16777777777
        self.x_gap   = self.size[0] / self.x_scale
    
    def clear(self):
        self.data_total  = []
        self.data_pos    = []
        self.data_other  = []
        self.data_count = 0
        self.x_pos = 0
        self.y_max = 30
        self.y_min  = 16777777777
        self.x_gap = self.size[0] / self.x_scale
    
    def render( self, surface ):
        pygame.draw.rect(surface, COLOR_BLACK, (self.coord, self.size), 0)
        blitTextLeft(surface, str(self.y_max), font_scribe16,(self.coord[0] + 5, self.coord[1] ), COLOR_WHITE)
        blitTextLeft(surface, str(self.y_min), font_scribe16,(self.coord[0] + 5, self.coord[1] + self.size[1] - 10), COLOR_WHITE)
        for i in range(self.x_pos, min(self.data_count, self.x_pos + self.x_scale) ):
            if i == self.x_pos:
                continue
            start_x  = self.x_gap * (i - self.x_pos - 1)
            end_x    = self.x_gap * (i - self.x_pos)
            start_y  = self.size[1] - self.data_total[i-1] / self.y_max * self.size[1]
            end_y    = self.size[1] - self.data_total[i] / self.y_max * self.size[1]
            start_pos = (start_x + self.coord[0], start_y + self.coord[1])
            end_pos   = (end_x   + self.coord[0],   end_y + self.coord[1])
            pygame.draw.line(surface, COLOR_WHITE, start_pos, end_pos, 1)

            start_y  = self.size[1] - self.data_pos[i-1] / self.y_max * self.size[1]
            end_y    = self.size[1] - self.data_pos[i] / self.y_max * self.size[1]
            start_pos = (start_x + self.coord[0], start_y + self.coord[1])
            end_pos   = (end_x   + self.coord[0],   end_y + self.coord[1])
            pygame.draw.line(surface, COLOR_YELLOW, start_pos, end_pos, 1)

            start_y  = self.size[1] - self.data_other[i-1] / self.y_max * self.size[1]
            end_y    = self.size[1] - self.data_other[i] / self.y_max * self.size[1]
            start_pos = (start_x + self.coord[0], start_y + self.coord[1])
            end_pos   = (end_x   + self.coord[0],   end_y + self.coord[1])
            pygame.draw.line(surface, COLOR_BLUE, start_pos, end_pos, 1)

        
        pygame.draw.line(surface, COLOR_ORANGE, (self.coord[0], self.coord[1] + self.size[1]),  (self.coord[0] + self.size[0], self.coord[1] + self.size[1]), 1)
        
        

    def appendData(self, data_list):
        if len(data_list) != 3:
            return
        
        data0 = data_list[0]
        data1 = data_list[1]
        data2 = data_list[2]
        if self.data_count < self.max_count:
            self.data_total.append(data0)
            self.data_pos.append(data1)
            self.data_other.append(data2)
            self.data_count += 1
        else:
            self.data_total.pop(0)
            self.data_total.append(data0)
            self.data_pos.pop(0)
            self.data_pos.append(data1)
            self.data_other.pop(0)
            self.data_other.append(data2)
        
        if self.data_count > self.x_scale:
            self.x_scale = self.data_count
            self.x_gap = self.size[0] / self.x_scale
        
        if data0 > self.y_max:
            self.y_max = data0
        if data0 < self.y_min:
            self.y_min = data0
        
        
        if data1 > self.y_max:
            self.y_max = data1
        if data1 < self.y_min:
            self.y_min = data1
        
        
        if data2 > self.y_max:
            self.y_max = data2
        if data2 < self.y_min:
            self.y_min = data2
        # self.y_max = self.data_total[0]
        # self.y_min = self.data_total[-1]
    
    def onMouseMotion(self, mouse_x, mouse_y ):
        pass
        
    def onMouseClick(self, mouse_x, mouse_y, btn ):
        return SEL_CLICK_NONE, -1

    def update(self):
        pass


class FuncSDFGraph:
    def __init__(self, coord, size, id = 10000):
        self.coord  = coord
        self.size   = size
        self.times  = []
        self.datas  = []
        self.data_count = 0
        self.time_count = 0
        self.x_scale = 100  # 水平方向可以显示多少个数据
        self.x_pos   = 0
        self.y_max   = 0.1
        self.y_min  = 16777777777
        self.id     = id
        self.x_gap   = self.size[0] / self.x_scale
        self.ready   = False
        self.rendering = False
        self.traj_duration = 0.0
    
    def clear(self):
        self.times  = []
        self.datas  = []
        self.data_count = 0
        self.x_pos = 0
        self.y_max = 0.1
        self.y_min  = 16777777777
        self.x_gap = self.size[0] / self.x_scale
        self.traj_duration = 0.0
    
    def render( self, surface ):     
        pygame.draw.rect(surface, COLOR_BLACK, (self.coord, self.size), 0)  
        if self.ready == False:
            return

        # for i in range(self.x_pos,  self.data_count-1) :
        self.rendering = True
        i = self.x_pos
        while(i < self.data_count):
            if i == self.x_pos:
                i +=1 
                continue
            start_x  = self.x_gap * (i - self.x_pos - 1)
            end_x    = self.x_gap * (i - self.x_pos)
            start_y  = self.size[1] - self.datas[i-1] / self.y_max * self.size[1]
            end_y    = self.size[1] - self.datas[i] / self.y_max * self.size[1]
            start_pos = (start_x + self.coord[0], start_y + self.coord[1])
            end_pos   = (end_x   + self.coord[0],   end_y + self.coord[1])
            pygame.draw.line(surface, COLOR_WHITE, start_pos, end_pos, 1)
            i += 1
        
        i = 0
        while (i < self.time_count):
            start_x   = self.size[0] * (self.times[i] / self.traj_duration)
            dat_cor   = math.ceil(self.times[i] * self.data_count / self.traj_duration)
            start_y   = self.size[1] - self.datas[dat_cor] / self.y_max * self.size[1]
            start_pos = (start_x + self.coord[0], start_y + self.coord[1])
            # pygame.draw.circle(surface, COLOR_RED, start_pos, 2, 0)
            pygame.draw.line(surface, COLOR_RED, start_pos, (start_pos[0], self.coord[1] + self.size[1]), 1)
            i += 1
        
        pygame.draw.line(surface, COLOR_ORANGE, (self.coord[0], self.coord[1] + self.size[1]),  (self.coord[0] + self.size[0], self.coord[1] + self.size[1]), 1)
        self.rendering = False
        
        
    def updateData(self, traj_dur, data_list, time_list):
        if self.rendering == True:
            return
        self.ready = False
        self.clear()
        self.traj_duration = traj_dur
        ind = 0
        for dat in data_list:
            self.datas.append(dat)
            ind += 1
            if dat > self.y_max:
                self.y_max = dat
            if dat < self.y_min:
                self.y_min = dat
        
        self.data_count = len(self.datas)
        self.x_scale    = max(100, self.data_count)
        if self.data_count > self.x_scale:
            self.x_scale = self.data_count
            self.x_gap = self.size[0] / self.x_scale

        self.times = time_list
        self.time_count = len(self.times)
        self.ready = True



    def onMouseMotion(self, mouse_x, mouse_y ):
        pass
        
    def onMouseClick(self, mouse_x, mouse_y, btn ):
        return SEL_CLICK_NONE, -1

    def update(self):
        pass
    
        

class Canvas:
    def __init__(self, parent_canvas, coord = (10,10), size = (300,300), color = COLOR_WHITE ):
        # create a child canvas with given coordinates
        self.child_surf    = pygame.Surface(size,  pygame.SRCALPHA)
        self.coord         = coord
        self.size          = size
        self.height        = size[1]
        self.parent_canvas = parent_canvas
        self.color         = color
        self.total_height  = 0
        self.begin_y       = 0
        self.vel_y         = 0
        self.need_scroll   = False
        self.begin_y_min   = 0
        self.begin_y_max   = 0
        self.last_begin_y  = 0
        self.components    = []
        
    
    def render(self):
        pygame.draw.rect(self.child_surf, self.color, ((0, 0), self.size),0)
        for component in self.components:
            component.render(self.child_surf)
        
        if self.need_scroll:
            scroll_bar_height = self.height * self.height / self.total_height
            scroll_bar_posy   = -self.height * self.begin_y / self.total_height
            pygame.draw.rect(self.child_surf, SCROLL_BORDER_COLOR, ((self.size[0]-10, scroll_bar_posy), (10, scroll_bar_height)), 0)
        
        self.parent_canvas.blit(self.child_surf, self.coord)
    
    def appendComponent(self, component, component_height):
        self.components.append(component)
        self.total_height += component_height
        if self.total_height > self.height:
            self.need_scroll = True
            self.begin_y_min = self.height - self.total_height
        else:
            self.need_scroll = False
    
    def onMouseMotion(self, mouse_x, mouse_y ):
        rel_x = mouse_x - self.coord[0]
        rel_y = mouse_y - self.coord[1]
        # if isPointInWindow( (rel_x, rel_y), self.size ) == False:
        #     return
        for component in self.components:
            component.onMouseMotion( rel_x, rel_y )

    def onMouseClick(self, mouse_x, mouse_y, btn ):
        rel_x = mouse_x - self.coord[0]
        rel_y = mouse_y - self.coord[1]
        
        if isPointInWindow( (rel_x, rel_y), self.size ) == False:
            return SEL_CLICK_NONE, -1

        if self.need_scroll == True:
            if btn == MOUSEBUTTON_WHEELUP:
                self.vel_y += 5
                if self.vel_y > 50:
                    self.vel_y = 50
            
            elif btn == MOUSEBUTTON_WHEELDOWN:
                self.vel_y -= 5
                if self.vel_y < -50:
                    self.vel_y = -50

        for component in self.components:
            ret =  component.onMouseClick( rel_x, rel_y, btn )
            if ret != SEL_CLICK_NONE:
                return ret, component.id
        return SEL_CLICK_NONE, -1

    def update(self):
        if self.need_scroll == True:
            self.begin_y += self.vel_y * 0.1
            self.vel_y *= 0.95

            if abs(self.vel_y) < 0.1:
                self.vel_y = 0
            
            if self.begin_y < self.begin_y_min:
                self.begin_y = self.begin_y_min
                self.vel_y = 0
            
            if self.begin_y > self.begin_y_max:
                self.begin_y = self.begin_y_max
                self.vel_y = 0
            
            if self.last_begin_y != self.begin_y:
                for component in self.components:
                    component.onScroll( 0, self.begin_y - self.last_begin_y )
            
            self.last_begin_y = self.begin_y

        for component in self.components:
            component.update()


class CanvasHorizon(Canvas):
    def __init__(self, parent_canvas, coord = (10,10), size = (300,300), color = COLOR_WHITE ):
        Canvas.__init__(self, parent_canvas, coord, size, color)
        self.begin_x       = 0
        self.vel_x         = 0
        self.begin_x_min   = 0
        self.begin_x_max   = 0
        self.last_begin_x  = 0
        self.width         = size[0]
    
    def render(self):
        pygame.draw.rect(self.child_surf, self.color, ((0, 0), self.size),0)
        for component in self.components:
            component.render(self.child_surf)
        
        if self.need_scroll:
            scroll_bar_width = self.width * self.width / self.total_height
            scroll_bar_posx   = -self.width * self.begin_x / self.total_height
            pygame.draw.rect(self.child_surf, SCROLL_BORDER_COLOR, ((scroll_bar_posx, self.size[1] - 5), (scroll_bar_width, 5)), 0)
        
        self.parent_canvas.blit(self.child_surf, self.coord)
    
    def appendComponent(self, component, component_width):
        self.components.append(component)
        self.total_width += component_width
        if self.total_width > self.width:
            self.need_scroll = True
            self.begin_x_min = self.width - self.total_width
        else:
            self.need_scroll = False
    
    def onMouseMotion(self, mouse_x, mouse_y ):
        rel_x = mouse_x - self.coord[0]
        rel_y = mouse_y - self.coord[1]
        for component in self.components:
            component.onMouseMotion( rel_x, rel_y )

    def onMouseClick(self, mouse_x, mouse_y, btn ):
        rel_x = mouse_x - self.coord[0]
        rel_y = mouse_y - self.coord[1]
        if isPointInWindow( (rel_x, rel_y), self.size ) == False:
            return SEL_CLICK_NONE, -1

        if self.need_scroll == True:
            if btn == MOUSEBUTTON_WHEELUP:
                self.vel_x += 8
                if self.vel_x > 120:
                    self.vel_x = 120
            
            elif btn == MOUSEBUTTON_WHEELDOWN:
                self.vel_x -= 8
                if self.vel_x < -120:
                    self.vel_x = -120

        for component in self.components:
            ret =  component.onMouseClick( rel_x, rel_y, btn )
            if ret != SEL_CLICK_NONE:
                return ret, component.id
        return SEL_CLICK_NONE, -1

    def update(self):
        if self.need_scroll == True:
            self.begin_x += self.vel_x * 0.1
            self.vel_x *= 0.95

            if abs(self.vel_x) < 0.1:
                self.vel_x = 0
            
            if self.begin_x < self.begin_x_min:
                self.begin_x = self.begin_x_min
                self.vel_x = 0
            
            if self.begin_x > self.begin_x_max:
                self.begin_x = self.begin_x_max
                self.vel_x = 0
            
            if self.last_begin_x != self.begin_x:
                for component in self.components:
                    component.onScroll(self.begin_x - self.last_begin_x, 0 )
            
            self.last_begin_x = self.begin_x

        for component in self.components:
            component.update()


class ComponentManager():
    def __init__(self):
        self.components = []
        self.canvases   = []

    def initComponents(self):

        self.components.append( StaticTitle( (90, 60), "Test Point", COLOR_YELLOW) )
        self.components.append(TextButton( 5, (180,90)  , (120,30), 1,  "Nothing"))
        self.visSDF      = TextSwitchButton( 5, (180,140) , (120,30), 2,  "VisSDF")
        self.components.append(self.visSDF)
        self.point_testX = SettingNum(0, (80,90), (60,20), 10, "X:", 5.0)
        self.point_testY = SettingNum(0, (80,120), (60,20), 11, "Y:", 5.0)
        self.point_testZ = SettingNum(0, (80,150), (60,20), 12, "Z:", 5.0)
        self.components.append(self.point_testX)
        self.components.append(self.point_testY)
        self.components.append(self.point_testZ)

        self.components.append( StaticTitle( (90, 220), "SDF Layer", COLOR_YELLOW) )
        self.components.append(TextButton( 5, (180,210) , (120,30), 3,  "Publish"))
        self.sdf_layerZ    = SettingNum(0, (80,250), (60,20), 13, "Z:", 3.0)
        self.sdf_layerXmin = SettingNum(0, (80,280), (60,20), 14, "X:", 3.0)
        self.sdf_layerXmax = SettingNum(0, (150,280), (60,20), 15, "     --", 3.0)
        self.sdf_layerYmin = SettingNum(0, (80,310), (60,20), 16, "Y:", 3.0)
        self.sdf_layerYmax = SettingNum(0, (150,310), (60,20), 17, "     --", 3.0)
        self.components.append(self.sdf_layerZ)
        self.components.append(self.sdf_layerXmin)
        self.components.append(self.sdf_layerXmax)
        self.components.append(self.sdf_layerYmin)
        self.components.append(self.sdf_layerYmax)

        self.components.append( StaticTitle( (120, 370), "Other Operations", COLOR_YELLOW) )
        self.components.append( StaticTitle((COMPONENT_WIDTH - 140,30), "Log titles", COLOR_YELLOW))
        self.components.append( StaticTitle((COMPONENT_WIDTH - 305,30), "Monitor vars", COLOR_YELLOW))
        self.components.append( StaticTitle( (COMPONENT_WIDTH - 305,370), "Opt monitor", COLOR_YELLOW) )

        self.other_operation_canvas = Canvas(COMPONENT_WINDOW, (50,390),(250,200), COLOR_BLACK )
        # Call readItemsFromJson and pass in the operations.json file in the current directory
        dir_path = os.path.dirname(os.path.realpath(__file__))
        other_oper_btns, other_oper_dict = readItemsFromJson(dir_path + "/operations.json")
        self.other_oper_dict  = other_oper_dict
        self.other_operation_canvas.components   = other_oper_btns
        self.other_operation_canvas.total_height = len(other_oper_btns) * ITEM_HEIGHT
        if self.other_operation_canvas.total_height < 200:
            self.other_operation_canvas.need_scroll = False
        else:
            self.other_operation_canvas.need_scroll = True
            self.other_operation_canvas.begin_y_min = 200 - self.other_operation_canvas.total_height

        self.canvases.append( self.other_operation_canvas )

        self.cost_curve = Curve((COMPONENT_WIDTH - 360,250), (350,100), 1000)
        self.components.append( self.cost_curve )

        self.opt_monitor_canvas = CanvasHorizon(COMPONENT_WINDOW, (COMPONENT_WIDTH - 360,390),(350,200), COLOR_BLACK )
        self.opt_monitor        = FuncSDFGraph((0,5),(350,180))
        self.opt_monitor_canvas.total_height = 0

        # for i in range(20):
        #     self.opt_monitor.appendMonitor(2)

        # if self.opt_monitor_canvas.total_height < 350:
        #     self.opt_monitor_canvas.need_scroll = False
        # else:
        #     self.opt_monitor_canvas.need_scroll = True
        #     self.opt_monitor_canvas.begin_x_min = 350 - self.opt_monitor_canvas.total_height

        # dats = []
        # ts   =  []
        # for i in range(100):
        #     if(i % 10 == 0):
        #         ts.append(-1)
        #     else:
        #         ts.append(1)
        #     dats.append(i)
            
        # self.opt_monitor.updateData(dats,ts)

        self.opt_monitor_canvas.components.append( self.opt_monitor )
        self.canvases.append( self.opt_monitor_canvas )


        # opti_stream control
        self.opti_stop_btn = TextButton( 5, (COMPONENT_WIDTH - 100, 360), (24,24), 21,  "S")
        self.opti_pause_btn = TextSwitchButton( 5, (COMPONENT_WIDTH - 70, 360), (24,24), 22,  "P")
        self.opti_step_btn = TextButton( 5, (COMPONENT_WIDTH - 40, 360), (24,24), 23,  "T")
        self.components.append( self.opti_stop_btn )
        self.components.append( self.opti_pause_btn )
        self.components.append( self.opti_step_btn )

        self.flash_state = FlashTitle((COMPONENT_WIDTH/2, COMPONENT_HEIGHT - 20), "ready", COLOR_WHITE )
        self.components.append( self.flash_state )

 
        


    def pubState(self, cont, extraParam = []):
        self.flash_state.reset(cont, extraParam)

    def leftButtonHandler(self, id):
        if id == 1:
            param_list =[]
            self.pubState("Nothing")
            param_list.append(self.point_testX.value)
            param_list.append(self.point_testY.value)
            param_list.append(self.point_testZ.value)
            pubDebugCMD(id,param_list) #debug
        if id == 2:
            if self.visSDF.on:
                self.pubState("Open test point SDF vis")
            else:
                self.pubState("Close test point SDF vis")
        if id == 3:
            xmin = self.sdf_layerXmin.value
            xmax = self.sdf_layerXmax.value
            ymin = self.sdf_layerYmin.value
            ymax = self.sdf_layerYmax.value
            z    = self.sdf_layerZ.value
            pubSDFLayerVis(xmin, xmax, ymin, ymax, z)
            self.pubState("Topic Published")
        
        if id == 10:
            self.point_testX.hold = True
        if id == 11:
            self.point_testY.hold = True
        if id == 12:
            self.point_testZ.hold = True
        if id == 13:
            self.sdf_layerZ.hold = True
        if id == 14:
            self.sdf_layerXmin.hold = True
        if id == 15:
            self.sdf_layerXmax.hold = True
        if id == 16:
            self.sdf_layerYmin.hold = True
        if id == 17:
            self.sdf_layerYmax.hold = True
        

        if id == 21:
            self.pubState("Stop optimization")
            pubDebugCMD(21, [])
        if id == 22:
            state = 1
            if self.opti_pause_btn.on:
                self.pubState("Optimization Paused.")
            else:
                self.pubState("Optimization Continue.")
                state = 0
            param = [state]
            pubDebugCMD(22, param)
        
        if id == 23:
            if self.opti_pause_btn.on:
                self.pubState("Next Step.")
                pubDebugCMD(23, [])
            else:
                self.pubState("Pause First.", ['e'])

        if id >= 100:
            param_list = []
            self.pubState(self.other_oper_dict[id])
            if id == 106 or id==109:
                param_list.append( self.sdf_layerZ.value)
            if id == 118 or id ==113: 
                param_list.append( self.sdf_layerXmin.value) 
                param_list.append( self.sdf_layerYmin.value)
                param_list.append( self.sdf_layerZ.value) 
                param_list.append( self.sdf_layerXmax.value) 
            pubDebugCMD(id,param_list)


    def rightButtonHandler(self, id):
        pass

    
    def onMouseMotion(self,rel_x, rel_y):
        for component in self.components:
            component.onMouseMotion( rel_x, rel_y )
        for canvas in self.canvases:
            canvas.onMouseMotion( rel_x, rel_y )

    def onMouseClick(self, mouse_x, mouse_y, btn ):
        for component in self.components:
            ret =  component.onMouseClick( mouse_x, mouse_y, btn )
            if ret == SEL_CLICK_LEFT:
                self.leftButtonHandler( component.id )
            if ret == SEL_CLICK_RIGHT:
                self.rightButtonHandler( component.id )
        for canvas in self.canvases:
            ret, cid = canvas.onMouseClick( mouse_x, mouse_y, btn )
            if ret == SEL_CLICK_LEFT:
                self.leftButtonHandler( cid )
            if ret == SEL_CLICK_RIGHT:
                self.rightButtonHandler( cid )
    
    def onMouseUp(self, mouse_x, mouse_y, btn ):
        if btn == MOUSEBUTTON_LEFT:
            self.point_testX.hold = False
            self.point_testY.hold = False
            self.point_testZ.hold = False
            self.sdf_layerZ.hold  = False
            self.sdf_layerXmin.hold = False
            self.sdf_layerXmax.hold = False
            self.sdf_layerYmin.hold = False
            self.sdf_layerYmax.hold = False
    
    def render(self, surface):
        for component in self.components:
            component.render( surface )
            
        for canvas in self.canvases:
            canvas.render()
    
    def update(self):
        for component in self.components:
            component.update( )
        
        for canvas in self.canvases:
            canvas.update()
        

component_manager = ComponentManager()
component_manager.initComponents()

news_component_manager = ComponentManager()
news_component_manager.components.append( StaticTitle((35,20), "Logs", COLOR_YELLOW))



