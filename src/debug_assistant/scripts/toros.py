#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import ros_gol
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from news import news_manager
from component import component_manager



def NewsCbk(data):
    content = data.data 
    # Splitting the content string by '@' symbol and storing the result in a list called content_list.
    content_list = content.split('@') 
    title = None
    text  = None
    if len(content_list) == 1:
        title = "unknown"
        text  = content_list[0]
    else:
        title = content_list[0]
        text  = content_list[1]
    
    news_manager.insertNew(title, text, rospy.Time.now().to_sec()*1000 )

def OptiStepCbk(msg):
    data = msg.data
    var_count  = int(data[0])
    if(var_count < 0):
        return

    time_count = int(data[1])
    traj_dur  = data[2]

    dat_list   = []
    times_list = []
    for i in range(var_count):
        dat_list.append(data[i+3])
    for i in range(time_count):
        times_list.append(data[i+3+var_count])
    component_manager.opt_monitor.updateData(traj_dur, dat_list, times_list)


def LogCostCbk(msg):
    if msg.data[0] < 0:
        component_manager.cost_curve.clear()
        return
    component_manager.cost_curve.appendData(msg.data)



################################################
######################   INIT   ################
################################################
def initRosPart():
    rospy.init_node("debug_terminal" ,anonymous=True)
    rospy.Subscriber("/debug_receive_news", String, NewsCbk)
    rospy.Subscriber("/debug_receive_opti_step", Float64MultiArray, OptiStepCbk, queue_size=100)
    rospy.Subscriber("/debug_receive_log_cost" , Float64MultiArray, LogCostCbk)
    ros_gol.debug_publisher = rospy.Publisher('/debug_cmd', Float64MultiArray, queue_size=10)
    print("调试终端启动")

    # 获取所有节点信息
    node_dict = rospy.get_node_uri()
    print(node_dict)






