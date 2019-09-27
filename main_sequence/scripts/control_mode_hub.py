#!/usr/bin/env python3
import actionlib
from constant_params import *
from main_sequence.msg import *
import rospy

control_mode = 0

def callback(attitude):
    control_mode = attitude.control_mode


#this param is used to define the strategy uas will take
prvNavmode = 0

if __name__ == '__main__':
    rospy.init_node('control_hub')
    rospy.Subscriber('control_hub_listener',attitude,callback)
    client = actionlib.SimpleActionClient('GPS_nav',uas_navigationAction)
    print('FSM:Control mode switcher awating action server')
    client.wait_for_server()
    rate = rospy.Rate(20)
    timeCounter = 0
    print('FSM:Control mode switcher successfully started up.')
    while not rospy.is_shutdown():
        if(control_mode == control_manual):#遥控模式
            pass
        elif(control_mode == control_auto):#自动模式
            goal.control_mode = control_mode
            goal.navigation_mode = prvNavmode
            client.send_goal(goal)
            while True:
                client.waitForResult(rospy.Duration(5))
                if(client.getState() == actionlib.SimpleClinetGoalState.SUCCEEDED):
                    print("FSM:Target location arrived.")
                    while control_mode == control_auto:
                        pass
                    break
                if(control_mode != control_auto):
                    client.cancel_goal()
                    print("FSM:Navgation has been cancelled")
                    break
        elif(control_mode == control_record):#采点模式
            goal.control_mode = control_mode
            client.send_goal(goal)
            while(control_mode == control_record):
                pass
            client.cancel_goal()
        if(timeCounter >= 40):
            print('FSM:Current control mode:',control_mode_dict.get(control_mode,'Exception'))
            timeCounter = 0
        timeCounter += 1
        rate.sleep()
