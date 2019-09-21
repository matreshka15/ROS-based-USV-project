#!/usr/bin/env python3
NAME = 'navigation_action'
FILE_LOCATION = '../LatLong.txt'
from library import AziFromPos
import actionlib
from main_sequence.msg import *
import rospy
import math

#define global variables
class navigation_data():
    latitude = 0
    longitude = 0
    fixtype = 0
nav_action_data = navigation_data()
  #路线规划描述：        
class Route:
    yaw = 0 #单位：°
    distance = 0#单位：厘米,u16型数据
    EndOfNav = 0#导航结束标志位
    def __init__(self):
        self.yaw = 0
        self.distance = 0
        self.EndOfNav = 0           
prvRoute = Route()

#define general function
def acquire_mapdata():
    try:
        f = open(FILE_LOCATION,'r')
        Mapdata = {}
        if(f.read() != ''):
            for line in f:
                    startIndex = line.find(':')
                    endIndex = line.find(',')
                    name = int(line[:startIndex])
                    Position0 = float(line[startIndex+1:endIndex])
                    startIndex = endIndex
                    line = line[endIndex+1:]
                    endIndex = line.find(',')
                    Position1 = float(line[:endIndex])
                    Mapdata[name] = [Position0,Position1]
            for key in Mapdata.keys():
                print("Point#%d:%s"%(key,str(Mapdata[key])))
            print("%d Points In Total."%len(Mapdata.keys()))
        else:
            print('Warning:Mapdata Empty!')
    except FileNotFoundError:
        print("Map Data Not Found!")
    return Mapdata

       
#define subscriber of vehicle's attitude
  #define callback of subscriber
def subscriber_callback(attitude_data):
    nav_action_data.latitude = attitude_data.latitude
    nav_action_data.longitude = attitude_data.longitude
    nav_action_data.fixtype = attitude_data.fixtype


#define publisher part
nav_route = route()

#define some macros
mode_start_from_the_beginning = 0
mode_start_from_the_nearest = 1
MaxDistance = 100
           
#define action 
#how to get current GPS location? using a publisher&subscriber?

nav_result = uas_navigationResult()
nav_feedback = uas_navigationFeedback()
def navigation(goal):
    if(nav_action_data.fixtype):
        MapData = acquire_mapdata()
        if(goal.mode == mode_start_from_the_beginning):
            nextPoint = min(Mapdata.keys())
        elif(goal.mode == mode_start_from_the_nearest):
            for key in Mapdata.keys():
                distance = AziFromPos.distanceFromCoordinate(nav_action_data.longitude,nav_action_data.latitude,Mapdata[key][0],Mapdata[key][1])
                if(key==1):
                    minimum = distance
                    indexMin = 1
                else:
                    if(distance < minimum):
                        minimum = distance
                        indexMin = key
        nextPoint = indexMin
        while True:
            if(server.is_preempt_requested()):
                server.set_preempted(nav_feedback,'GPS Navigation Halt')
                break
            
            #After this part of function,initial start point will be selected
            distance = AziFromPos.distanceFromCoordinate(nav_action_data.longitude,nav_action_data.latitude,Mapdata[nextPoint][0],Mapdata[nextPoint][1])
            if distance >=255:
                distance = 255
            angle =  AziFromPos.angleFromCoordinate(nav_action_data.longitude,nav_action_data.latitude,Mapdata[nextPoint][0],Mapdata[nextPoint][1])
            prvRoute.yaw = round(angle)
            prvRoute.distance = round(distance)
            if(distance > MaxDistance):
                #如果超出范围
                pass 
                #----根据夹角调转机头----#
            elif(nextPoint == len(Mapdata.keys())):
                #未超出范围，则已达到目标点。
                nextPoint = nextPoint
                prvRoute.EndOfNav = 1
                server.set_succeeded(nav_result)
                while not server.is_preempt_requested():
                    pass
                else:
                    server.set_preempted(nav_result,'GPS Navigation Halt')
                    break
                break
            else:
                prvRoute.EndOfNav = 0
                nextPoint += 1
                    
            nav_feedback.current_index = nextPoint
            nav_feedback.current_latitude = nav_action_data.latitude
            nav_feedback.current_longitude = nav_action_data.longitude
            server.publish_feedback(nav_feedback)
            
            nav_route.EndOfNav = prvRoute.EndOfNav
            nav_route.nav_yaw = prvRoute.yaw
            nav_route.nav_distance = prvRoute.distance
            pub.publish(nav_route)
    else:
        print("GPS not fixed")
    rate.sleep()
                #发现的问题：一个串口在同一时间只能在一个脚本里打开
                #那么串口的收发都要经过同一个脚本！
                #把route写成一个msg文件,然后这个脚本里还要再加一个publisher!
                #SendNavMessege(ser,route)


if __name__ == '__main__':
    rospy.init_node('main_execute_module',anonymous=True)
    #start nav data listener
    rospiy.Subscriber('nav_action_listener',attitude,subscriber_callback)
    pub = rospy.Publisher('navigation_route',route,queue_size=5)
    rate = rospy.Rate(5) #5Hz
    server = actionlib.SimpleActionServer('GPS_nav',uas_navigation,navigation,False)
    server.start()
    rospy.spin()
