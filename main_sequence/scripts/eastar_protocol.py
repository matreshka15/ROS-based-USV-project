#!/usr/bin/env python3
import rospy
import serial
#message type:include almost all GPS\Imu data needed
from main_sequence.msg import *
#configuration
baudrate = 115200
frame_length = 22

class attitude_buff():
    latitude = 0
    longitude = 0
    fixtype = 0
    hAcc = 0
    yaw = 0
    pitch = 0
    roll = 0
    speed = 0
    angleSpeed = 0
    dataProcessed = 0
    control_mode = 0


def callback(Route):
    sendFile = list()
    sumup = 0
    for cnt in range(20):
        sendFile.append(0) #初始化数组
    sendFile[0]=0x63
    sendFile[1]=0x73
    sendFile[2]=(Route.yaw&0xff00)>>8
    sendFile[3]=Route.yaw&0xff
    sendFile[4]=(Route.distance&0xff00)>>8
    sendFile[5]=Route.distance&0xff
    sendFile[6]=(Route.EndOfNav << 7)&0xff
    #这里用来后续填充
    for cnt in range(17):
        sumup+=sendFile[2+cnt]
    sumup = sumup & 0xff
    sendFile[19] = sumup
    sendBytes = bytes()
    for cnt in range(20):
        sendBytes += bytes([sendFile[cnt]])
    ser.write(sendBytes)	

#initialize data needed
revData = []
int_revData = []
attitude_buff = attitude_buff()
real_attitude = attitude()
one_frame_received = 0
CRC_validation_passed = 0
for cnt in range(frame_length):
    revData.append(0)
    int_revData.append(0)


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('attitude_info',attitude,queue_size=10)
        rospy.init_node('serial_port_hub',anonymous=True)
        rospy.Subscriber('navigation_data_publisher',route,callback)
        rate = rospy.Rate(5) #5Hz
        print("Initiating phase 1")
        #timeout protection
        ser = serial.Serial('/dev/ttyAMA0',baudrate,timeout=1)
        print(ser)
        portNotFound = 0
        while not rospy.is_shutdown():
            revData[0] = ser.read(1)
            if revData[0] == b'' or portNotFound == 0:
                portNotFound = 1
                print("No data coming in serial port.")
                print("Please check port:",ser.name)
            else:
                portNotFound = 0
            one_frame_received = 0
            if(revData[0] == b'0x73'):
                revData[1] = ser.read(1)
                if(revData[1] ==b'0x63'):
                    for cnt in range(frame_length - 2):
                        revData[2+cnt] = ser.read(1)
                    one_frame_received = 1
                    for cnt in range(frame_length):
                        int_revData[cnt] = int.from_bytes(revData[cnt],byteorder='big',signed=False)
            else:
                pass
                
            if(one_frame_received == True):
                #CRCvalidating prosedure
                sumUp = 0
                CRC_validation_passed = 0
                for cnt in range(frame_length - 3):
                    sumUp += int_revData[2+cnt]
                sumUp = sumUp & 0xff
                if(sumUp == int_revData[frame_length - 1]):
                    CRC_validation_passed = True
                else:
                    CRC_validation_passed = False
                                        
            if(CRC_validation_passed == True):
                #Interpret data into NavSatFix
                attitude_buff.longitude = bytes()
                attitude_buff.latitude = bytes()
                attitude_buff.yaw = bytes()
                attitude_buff.pitch = bytes()
                attitude_buff.roll = bytes()
                attitude_buff.hAcc = revData[16]
                #control field
                attitude_buff.fixtype = revData[20]
                attitude.control_mode = revData[20]
                for cnt in range(4):
                    attitude_buff.longitude += revData[cnt+2]
                    attitude_buff.latitude += revData[cnt+2+4]
                for cnt in range(2):
                    attitude_buff.yaw += revData[cnt+10]
                    attitude_buff.pitch += revData[cnt+12]
                    attitude_buff.roll += revData[cnt+14]
                real_attitude.yaw = int.from_bytes(attitude_buff.yaw,byteorder='big',signed=False)
                real_attitude.pitch = int.from_bytes(attitude_buff.pitch,byteorder='big',signed=True)
                real_attitude.roll = int.from_bytes(attitude_buff.roll,byteorder='big',signed=True)
                real_attitude.fixtype = (0xC0 & int.from_bytes(attitude_buff.fixtype,byteorder='big',signed=False)) >> 6
                real_attitude.control_mode = (0x30 & int.from_bytes(attitude_buff.control_mode,byteorder='big',signed=False)) >> 4
                real_attitude.hAcc = int.from_bytes(attitude_buff.hAcc,byteorder='big',signed=False)/10
                real_attitude.longitude = int.from_bytes(attitude_buff.longitude,byteorder='big',signed=False)/1e7
                real_attitude.latitude = int.from_bytes(attitude_buff.latitude,byteorder='big',signed=False)/1e7
                pub.publish(real_attitude)
            #rate.sleep()
    except rospy.ROSInterruptException:
        ser.close()
        print("Node shuting down")


