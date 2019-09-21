#!/usr/bin/env python3
NAME = 'navigation_action'
FILE_LOCATION = 'LatLong.txt'
import sys
sys.path.append('../src')
import AziFromPos
import actionlib
import main_sequence.msg
import rospy
import math

#general function
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
mode_start_from_the_beginning = 0
mode_start_from_the_nearest = 1

#how to get current GPS location? using a publisher&subscriber?
def navigation(goal):
	MapData = acquire_mapdata()
	if(goal.mode == mode_start_from_the_beginning):
		nextPoint = min(Mapdata.keys())
	elif(goal.mode == mode_start_from_the_nearest):
		for key in Mapdata.keys():
                    distance = AziFromPos.distanceFromCoordinate(longitude,latitude,Mapdata[key][0],Mapdata[key][1])
                    if(key==1):
                        minimum = distance
                        indexMin = 1
                    else:
                        if(distance < minimum):
                            minimum = distance
                            indexMin = key
                nextPoint = indexMin

	#Initial start point selected
	


