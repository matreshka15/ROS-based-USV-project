无人载具上位机端程序
================
此package为使用ROS为框架开发的无人载具上位机程序。目前正在开发阶段，功能尚未完成。使用ROS的主要目的在于试图消除因不同硬件平台带来的低通用性。在Kinetic、Melodic等版本的ROS、在Nvidia TX系列、树莓派、PC等Ubuntu系统平台无差别运行。

-------

## 概述
本package运行在运行ROS的上位机中，通过串口与下位机(STM32F103ZE)相连接；下位机负责动力控制、姿态解算与遥控信号接收等功能，上位机负责自动导航模式的路径规划。
* [下位机端工程文件Github仓库](https://github.com/matreshka15/UAS-Project-STM32)
## 开发环境
初期在Nvidia Jetson TX1开发板上进行开发。(系统：Ubuntu 18.04；ROS版本：melodic)，与ROS kinetic兼容性有待测试。但实际开发过程中发现在Kinetic平台未报错误。
开发中期使用Nvidia Jetson Nano, 由于Nvidia官方社区支持Ubuntu版本为18.04，因此本Project将专注在ROS melodic平台上开发。

## 相关问题
* Nvidia 在TX1开发板上使用USB串口(CH341，/dev/ttyUSB0)时接收到的数据不正确，但串口设置正常。实验相同的硬件连接插在windows平台上可接收到正确数据。
  * 解决方案：使用TX1开发板上的ttyTHS2串口进行数据通信。ttyTHS1和ttyTHS3分别是控制台串口和蓝牙模块，不宜使用。
* 当波特率很高时，误码率也会大幅提高，因而出现丢包、错包等情况。此外由于使用的串口为TTL电平，且连接线较长，波特率高时会发生干扰。等待解决。
  * 解决方案：波特率降低为115200。
* rospy中没有像roscpp的ros::spinOnce()方法。rospy中的回调函数是在一个单独的线程中进行的，不需spin()。还请注意。

## 开发进度
* main_sequence package
包内主要涵盖：路径规划、上位机与下位机的通信（基于自创Eastar protocol协议，协议处于测试阶段。若需要协议框图请联系开发者）
  * control_mode_hub.py--驱动系统状态切换的switcher.遥控器可选择手动操作模式、自动导航模式、计点模式，switcher的作用即接收遥控器模式的切换，并且通知相应server进行相关操作。
  * eastar_protocol.py--系统的串口操作hub.所有串口的输入输出脚本均在此脚本中进行
  * navigation_action_server.py--系统的core组件。负责实际路径的规划、GPS坐标点的记录等核心操作。
  * constant_params.py--系统常量存放处。
  * 注意--串口等设置参数。均在参数服务器中，在launch文件内修改。

## 开发者的话
****

|Author|Estello|
|---|---
|E-mail|matreshka15@icloud.com

Estello.club-- A self-supported newly-born geek club
****
