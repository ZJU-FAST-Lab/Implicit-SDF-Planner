#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
from surfaces import *
from news import news_manager
from component import component_manager, news_component_manager
from monitor import monitor



class WindowsManager:
    def __init__(self):
        pass

    def clearScreen(self):
        pygame.draw.rect( BASE_WINDOW , BASE_WINDOW_COLOR, ( (0,0) ,( APP_WIDTH, APP_HEIGHT)),0)

    def renderScreen(self):
        self.clearScreen()
        
        # 依次绘制各窗口
        pygame.draw.rect( COMPONENT_WINDOW , COMPONENT_WINDOW_COLOR , ( (0,0) ,( COMPONENT_WIDTH, COMPONENT_HEIGHT)),0)
        pygame.draw.rect( NEWS_WINDOW , STATE_WINDOW_COLOR , ( (0,0) ,( NEWS_WIDTH, NEWS_HEIGHT)),0)

        news_manager.render(NEWS_WINDOW)
        monitor.render(COMPONENT_WINDOW)
        component_manager.render(COMPONENT_WINDOW)
        news_component_manager.render(NEWS_WINDOW)

        BASE_WINDOW.blit( COMPONENT_WINDOW, (0, 0) )
        BASE_WINDOW.blit( NEWS_WINDOW, (COMPONENT_WIDTH, 0) )


windows_manager = WindowsManager() 
