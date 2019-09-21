#!/usr/bin/env python3
import actionlib
from main_sequence.msg import *
import rospy

control_mode = 0

def callback(attitude):
    control_mode = attitude.control_mode


navigationmode = 0
if __name__ == '__main__':
    rospy.init_node('control_hub')
    rospy.Subscriber('control_hub_listener',attitude,callback)
    client = actionlib.SimpleActionClient('control_hub_nav_client',uas_navigationAction)
    client.wait_for_server()
    while not rospy.is_shutdown():
        if(control_mode == 0):#遥控模式
            pass
        elif(control_mode == 1):#自动模式
            goal.navigation_mode = navigationmode
            client.send_goal(goal)
            result = client.get_result()
            while(control_mode == 1):
                pass
            client.cancel_goal()
        elif(control_mode == 2):#采点模式
            pass
