#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import pygame
from surfaces import *
from algorithms import *

class InputManager():
    def __init__(self):
        self.down_push_buttons = []
        self.down_push_keys    = []
        self.mouse_x           = 0
        self.mouse_y           = 0
        self.mouse_x_vel       = 0
        self.mouse_y_vel       = 0
        self.intransition      = 0
    
    def relativeMouseCoordInWindow(self, coord):
        x = self.mouse_x
        y = self.mouse_y
        window_pos_x = coord[0]
        window_pos_y = coord[1]
        relative_x = x - window_pos_x
        relative_y = y - window_pos_y
        return (relative_x , relative_y )

    def relativeMouseCoord(self):
        x = self.mouse_x
        y = self.mouse_y
        window_pos_x = 0
        window_pos_y = 0
        mouse_in_window = False
        if isMouseInWindow( x,y, (0,0), (GUIDE_WIDTH, GUIDE_HEIGHT) ):
            mouse_in_window = True
            window_pos_x = 0
            window_pos_y = 0

        elif isMouseInWindow( x,y, (0,GUIDE_HEIGHT), (ITEM_WIDTH, ITEM_HEIGHT) ):
            mouse_in_window = True
            window_pos_x = 0
            window_pos_y = GUIDE_HEIGHT
        
        elif isMouseInWindow( x,y, (ITEM_WIDTH, 0), (TAG_WIDTH, TAG_HEIGHT) ):
            mouse_in_window = True
            window_pos_x = ITEM_WIDTH
            window_pos_y = 0

        elif isMouseInWindow( x,y, (ITEM_WIDTH, TAG_HEIGHT), (INFO_WIDTH, INFO_HEIGHT) ):
            mouse_in_window = True
            window_pos_x = ITEM_WIDTH
            window_pos_y = TAG_HEIGHT
        
        
        elif isMouseInWindow( x,y, (0, ITEM_HEIGHT + GUIDE_HEIGHT + COMPONENT_HEIGHT), (STATE_WIDTH, STATE_HEIGHT) ):
            mouse_in_window = True
            window_pos_x = 0
            window_pos_y = ITEM_HEIGHT + GUIDE_HEIGHT + COMPONENT_HEIGHT
        
        elif isMouseInWindow( x,y, (0,GUIDE_HEIGHT + ITEM_HEIGHT), (COMPONENT_WIDTH, COMPONENT_HEIGHT) ):
            mouse_in_window = True
            window_pos_x = 0
            window_pos_y = ITEM_HEIGHT + GUIDE_HEIGHT
        
        relative_x = x - window_pos_x
        relative_y = y - window_pos_y
        return (relative_x , relative_y )
    
    def absoluteMouseCoord(self):
        return (self.mouse_x ,self.mouse_y)
    
    def isKeyDown(self, key):
        if key in self.down_push_keys:
            return True
        else:
            return False
    
    def isButtonDown(self, button):
        if button in self.down_push_buttons:
            return True
        else:
            return False
    
    def inputEventCallback(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 5 or event.button == 4:
                return 
            if self.isButtonDown(event.button):
                return
            self.down_push_buttons.append(event.button)

        if event.type == pygame.MOUSEMOTION:
            self.mouse_x_vel = event.pos[0] - self.mouse_x
            self.mouse_y_vel = event.pos[1] - self.mouse_y
            self.mouse_x = event.pos[0]
            self.mouse_y = event.pos[1]
            return 

        if event.type == pygame.MOUSEBUTTONUP:
            if event.button == 4 or event.button == 5:
                return 
            if self.isButtonDown(event.button):
                self.down_push_buttons.remove(event.button)
            return

        if event.type == pygame.KEYDOWN:  
            if self.isKeyDown(event.key):
                return
            self.down_push_keys.append(event.key)
            return

        if event.type == pygame.KEYUP:
            if self.isKeyDown(event.key):
                self.down_push_keys.remove(event.key)
            return 

input_manager = InputManager()