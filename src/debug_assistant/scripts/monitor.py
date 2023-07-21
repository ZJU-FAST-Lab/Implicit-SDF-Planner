#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
from surfaces import *
from algorithms import *
from component import Canvas, Item, SEL_CLICK_LEFT, SEL_CLICK_RIGHT, SEL_CLICK_NONE


class VarTable():
    pass

 
class Monitor:
    def __init__(self):
        self.var_tables = []
        self.vars       = {}
        self.shown_vars = []

        self.var_btns     = []
        self.var_btn_y    = 0
        self.var_btn_id   = 20000
        self.var_btn_can  = Canvas(COMPONENT_WINDOW, (COMPONENT_WIDTH - 360, 50 ), (170,190), COLOR_BLACK)
    
    def getBtnById(self,id):
        for btn in self.var_btns:
            if btn.id == id:
                return btn
        return None

    
    def leftButtonHandler(self, id):
        if id >= 20000:
            pass
            
    def rightButtonHandler(self, id):
        if id >= 20000:
            pass

    def onMouseMotionInComponentWindow(self,rel_x, rel_y):
        self.var_btn_can.onMouseMotion( rel_x, rel_y )
    
    def onMouseClickInComponentWindow(self, mouse_x, mouse_y, btn ):
        ret, cid = self.var_btn_can.onMouseClick( mouse_x, mouse_y, btn )
        if ret == SEL_CLICK_LEFT:
            self.leftButtonHandler( cid )
        if ret == SEL_CLICK_RIGHT:
            self.rightButtonHandler( cid )



    def insertVar(self, title, value, time_stamp = 0):
        pass
    
    def render(self, surface):
        for var_table in self.var_tables:
            var_table.render(surface)

        self.var_btn_can.render()
 
    def update(self):
        self.var_btn_can.update()


monitor = Monitor()



