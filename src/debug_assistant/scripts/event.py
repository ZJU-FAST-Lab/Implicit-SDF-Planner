#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
from surfaces import *
import pygame
import sys
from pygame.locals import *
from algorithms import *


from input import input_manager
from component import component_manager, news_component_manager
from news import news_manager


class EventHandler():
    def __init__(self):
        self.MOUSE_IN_NEWS      = 1
        self.MOUSE_IN_COMPONENT = 2

        self.mouse_in_which_window = 0
    
    def relativeMouseCoord(self, x,y):
        window_pos_x = 0
        window_pos_y = 0
        mouse_in_window = False
        self.mouse_in_which_window = 0 
        if isMouseInWindow( x,y, (0,0), (COMPONENT_WIDTH, COMPONENT_HEIGHT) ):
            mouse_in_window = True
            window_pos_x = 0
            window_pos_y = 0
            self.mouse_in_which_window = self.MOUSE_IN_COMPONENT

        elif isMouseInWindow( x,y, (COMPONENT_WIDTH, 0), (NEWS_WIDTH, NEWS_HEIGHT) ):
            mouse_in_window = True
            window_pos_x = COMPONENT_WIDTH
            window_pos_y = 0
            self.mouse_in_which_window = self.MOUSE_IN_NEWS
        
        relative_x = x - window_pos_x
        relative_y = y - window_pos_y
        return (relative_x , relative_y )


    def eventHandle(self, e ):
        input_manager.inputEventCallback( e )
        if e.type == pygame.KEYDOWN:
            self.keyDownHandle( e.key )
        if e.type == pygame.MOUSEMOTION:
            self.mouseMotionHandle(e.pos[0], e.pos[1])
        if e.type == pygame.MOUSEBUTTONDOWN:
            self.mouseClickHandle( e.button )
        if e.type == pygame.MOUSEBUTTONUP:
            self.mouseUpHandle( e.button )
            
    def mouseMotionHandle(self, x,y):
        (rel_x, rel_y) = self.relativeMouseCoord(x,y)
        if self.mouse_in_which_window == self.MOUSE_IN_NEWS:
            pass
        if self.mouse_in_which_window == self.MOUSE_IN_COMPONENT:
            component_manager.onMouseMotion(rel_x, rel_y)
            news_component_manager.onMouseMotion(rel_x, rel_y)
            news_manager.onMouseMotionInComponentWindow(rel_x, rel_y)
        

    def mouseClickHandle(self, btn ):
        x = input_manager.mouse_x
        y = input_manager.mouse_y
        (rel_x, rel_y) = self.relativeMouseCoord(x,y)
        if self.mouse_in_which_window == self.MOUSE_IN_NEWS:
            news_manager.onMouseClick(rel_x, rel_y, btn)
        if self.mouse_in_which_window == self.MOUSE_IN_COMPONENT:
            component_manager.onMouseClick(rel_x, rel_y, btn)
            news_component_manager.onMouseClick(rel_x, rel_y, btn)
            news_manager.onMouseClickInComponentWindow(rel_x, rel_y, btn)
    
    def mouseUpHandle(self, btn ):
        x = input_manager.mouse_x
        y = input_manager.mouse_y
        (rel_x, rel_y) = self.relativeMouseCoord(x,y)
        if self.mouse_in_which_window == self.MOUSE_IN_NEWS:
            pass
        if self.mouse_in_which_window == self.MOUSE_IN_COMPONENT:
            component_manager.onMouseUp(rel_x, rel_y, btn)
            news_component_manager.onMouseMotion(rel_x, rel_y)

    def keyDownHandle(self, key):
        if key == K_ESCAPE: # test
            sys.exit()

event_handler = EventHandler()