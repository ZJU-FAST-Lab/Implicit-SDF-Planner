#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import pygame
pygame.init()
pygame.display.set_caption('Monitor')
from toros import *
initRosPart()


from render import windows_manager
from component import component_manager, news_component_manager
from event import event_handler
from news import news_manager

def update():
    component_manager.update()
    news_component_manager.update()
    news_manager.update()
while True: 
    events = pygame.event.get()
    for event in events: 
        event_handler.eventHandle( event )

    pygame.display.update()
    update()
    pygame.time.wait(2)
    windows_manager.renderScreen()

pygame.quit()  
