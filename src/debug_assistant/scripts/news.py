#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
from surfaces import *
from algorithms import *
from component import Canvas, Item, SEL_CLICK_LEFT, SEL_CLICK_RIGHT, SEL_CLICK_NONE

Split = 30
line_height = 20

TOP_PADDIG    = 40
BOTTOM_PADDIG = 20

class News:
    def __init__(self, title, content, icolor = COLOR_GREEN, time_stamp = 0):
        self.title   = title
        self.content = content
        self.words   = len(content)
        self.cols    = math.ceil( self.words / Split )
        self.lines   = [content[i:i+Split] for i in range(0, self.words, Split)]
        self.alpha   = 0.9
        self.icolor  = icolor
        self.bgcolor = colorGradient(COLOR_WHITE, STATE_WINDOW_COLOR, self.alpha)
        self.height  = line_height * self.cols
        self.in_screen    = True

        self.time_stamp = time_stamp


        
    def render(self, surface, current_y):
        pygame.draw.rect( surface, self.bgcolor, ((20,current_y), ( NEWS_WIDTH, self.height)), 0 )
        pygame.draw.circle( surface, self.icolor, (10,current_y+10), 3, 0 )
        pygame.draw.line( surface, self.icolor, (NEWS_WIDTH - 100, current_y+8), (NEWS_WIDTH - 90, current_y+8), 2 )
        for line in self.lines:
            if current_y >= TOP_PADDIG-5 and current_y < NEWS_HEIGHT - BOTTOM_PADDIG:
                blitTextLeft(surface, line, font_scribe26, (20, current_y), COLOR_WHITE )
            current_y += line_height

    def update(self):
        self.alpha -= 0.01
        if self.alpha < 0:
            self.alpha = 0
        self.bgcolor = colorGradient(STATE_WINDOW_COLOR, COLOR_WHITE, self.alpha)

 
    
class NewsManager:
    def __init__(self):
        self.news    = []
        self.titles  = {}
        self.titles_count = {}
        self.shown_titles = []
        self.begin_y = TOP_PADDIG
        self.total_height = 0

        self.need_scroll   = False
        self.window_height = NEWS_HEIGHT - TOP_PADDIG - BOTTOM_PADDIG
        self.begin_y       = TOP_PADDIG
        self.last_y        = TOP_PADDIG
        self.begin_y_min   = TOP_PADDIG
        self.begin_y_max   = TOP_PADDIG

        self.vel_y = 0

        self.title_btns    = []
        self.title_btn_y    = 0
        self.title_btn_id   = 10000
        self.title_btn_can = Canvas(COMPONENT_WINDOW, (COMPONENT_WIDTH - 180, 50 ), (170,190), COLOR_BLACK)
    
    def getBtnById(self,id):
        for btn in self.title_btns:
            if btn.id == id:
                return btn
        return None

    def onScroll(self, y):
        self.begin_y += y
    
    def leftButtonHandler(self, id):
        if id >= 10000:
            btn = self.getBtnById(id)
            if btn != None:
                if btn.on == False:
                    self.shown_titles.remove(btn.title)
                else:
                    self.shown_titles.append(btn.title)
            

    def rightButtonHandler(self, id):
        if id >= 10000:
            btn = self.getBtnById(id)
            if btn != None:
                btn.on = True
                self.shown_titles = []
                self.shown_titles.append(btn.title)
                for btn in self.title_btns:
                    if btn.id != id:
                        btn.on = False
    
    def onMouseMotionInComponentWindow(self,rel_x, rel_y):
        self.title_btn_can.onMouseMotion( rel_x, rel_y )
    
    def onMouseClickInComponentWindow(self, mouse_x, mouse_y, btn ):
        ret, cid = self.title_btn_can.onMouseClick( mouse_x, mouse_y, btn )
        if ret == SEL_CLICK_LEFT:
            self.leftButtonHandler( cid )
        if ret == SEL_CLICK_RIGHT:
            self.rightButtonHandler( cid )

    def onMouseClick(self, mouse_x, mouse_y, btn ):
        if self.need_scroll == True:
            if btn == MOUSEBUTTON_WHEELUP:
                self.vel_y += 5
                if self.vel_y < 0:
                    self.vel_y = 5
                if self.vel_y > 500:
                    self.vel_y = 500
            
            elif btn == MOUSEBUTTON_WHEELDOWN:
                self.vel_y -= 5
                if self.vel_y > 0:
                    self.vel_y = -5
                if self.vel_y < -500:
                    self.vel_y = -500

    def insertNew(self, title, content, time_stamp = 0):
        icolor = None
        if title not in self.titles:
            icolor = getRandomColor()
        else:
            icolor = self.titles[title]

        n = News(title, content, icolor, time_stamp)
        self.news.append( n )

        if title not in self.titles:
            self.titles[title] = icolor
            new_title_btn = Item(0, (0,self.title_btn_y), (160,20), self.title_btn_id,title, icolor)
            new_title_btn.on = True
            self.title_btn_id += 1
            self.title_btn_y  += 25
            self.shown_titles.append(title)
            self.title_btns.append(new_title_btn)
            self.title_btn_can.appendComponent(new_title_btn, 25)
            self.titles_count[title] = n.height
        else:
            self.titles_count[title] += n.height

        self.total_height += n.height
        if(self.total_height > self.window_height):
            self.need_scroll = True
            self.begin_y_min -= n.height
            self.begin_y -= n.height
    
    def render(self, surface):
        self.title_btn_can.render()

        total_height = 0
        for shown_title in self.shown_titles:
            total_height += self.titles_count[shown_title]

        self.total_height = total_height
        if(self.total_height > self.window_height):
            self.need_scroll = True
            self.begin_y_min = TOP_PADDIG - self.total_height + self.window_height
        else:
            self.need_scroll = False
            self.begin_y_min = TOP_PADDIG
            self.begin_y     = TOP_PADDIG

        current_y = self.begin_y
        last_news = None
        for n in self.news:
            if n.title not in self.shown_titles:
                n.in_screen = False
                continue
            if (current_y + n.height) < 0 or current_y > NEWS_HEIGHT:
                current_y += n.height
                n.in_screen = False
                continue
            n.in_screen = True
            n.render(surface, current_y)
            current_y += n.height

            if last_news != None:
                dtick = n.time_stamp - last_news.time_stamp
                unit  = "ms"
                if dtick > 1000:
                    dtick /= 1000
                    unit = "s"

                dtick = "{:.4f}".format(dtick)
                time_str = str(dtick) + " " + unit
                blitTextLeft(surface, time_str, font_scribe16, (NEWS_WIDTH - 80, current_y - n.height - 0.4 * last_news.height), COLOR_WHITE )
            last_news = n
        
        lby = self.begin_y + 8
        if self.begin_y < -8:
            lby = 0

        pygame.draw.line( surface, COLOR_GREEN, (NEWS_WIDTH - 90, lby), (NEWS_WIDTH - 90, NEWS_HEIGHT), 2 )
        if self.need_scroll:
            scroll_bar_height = NEWS_HEIGHT * NEWS_HEIGHT / self.total_height
            scroll_bar_posy   = (-self.begin_y + self.begin_y_max)  / (self.begin_y_max - self.begin_y_min)
            scroll_bar_posy   = scroll_bar_posy * (NEWS_HEIGHT - scroll_bar_height)
            pygame.draw.rect( surface, SCROLL_BORDER_COLOR, ((NEWS_WIDTH - 10, scroll_bar_posy), ( 10, scroll_bar_height)), 0 )
        
        
    
    def update(self):

        self.title_btn_can.update()

        if self.need_scroll:
            self.begin_y += self.vel_y * 0.1
            self.vel_y *= 0.99

            if self.begin_y > self.begin_y_max:
                self.begin_y = self.begin_y_max
                self.vel_y = 0

            if self.begin_y < self.begin_y_min:
                self.begin_y = self.begin_y_min
                self.vel_y = 0

            if self.begin_y != self.last_y:
                self.onScroll( self.begin_y - self.last_y )
            self.last_y = self.begin_y

        for n in self.news:
            if n.in_screen == True:
                n.update()

news_manager = NewsManager()
