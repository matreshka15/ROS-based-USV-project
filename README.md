无人船上位机端程序
================
此package为使用ROS为框架开发的无人船上位机程序。目前正在开发阶段，功能尚未完成。使用ROS的主要目的在于试图消除因不同硬件平台带来的低通用性。在Kinetic、Melodic等版本的ROS上；在Nvidia TX系列、树莓派、PC等Ubuntu系统的平台上无差别运行。

****

|Author|Estello|
|---|---
|E-mail|matreshka15@icloud.com


****

## 开发环境
初期在Nvidia TX1开发板上进行开发。(系统：Ubuntu 18.04；ROS版本：melodic)，与ROS kinetic兼容性有待测试。但实际开发过程中发现在Kinetic平台未报错误。

## 相关问题
* Nvidia 在TX1开发板上使用USB串口(CH341，/dev/ttyUSB0)时接收到的数据不正确，但串口设置正常。实验相同的硬件连接插在windows平台上可接收到正确数据。
  * 解决方案：使用TX1开发板上的ttyTHS2串口进行数据通信。ttyTHS1和ttyTHS3分别是控制台串口和蓝牙模块，不宜使用。
* 当波特率很高时，误码率也会大幅提高，因而出现丢包、错包等情况。等待解决。

## 开发进度
* main_sequence package
包内主要涵盖：路径规划、上位机与下位机的通信（基于Eastar protocol协议，若需要协议框图请联系开发者）
  * control_mode_hub.py--驱动系统状态切换的switcher.遥控器可选择手动操作模式、自动导航模式、计点模式，switcher的作用即接收遥控器模式的切换，并且通知相应server进行相关操作。
  * eastar_protocol.py--系统的串口操作hub.所有串口的输入输出脚本均在此脚本中进行
  * navigation_action_server.py--系统的core组件。负责实际路径的规划、GPS坐标点的记录等核心操作。
  * constant_params.py--系统常量存放处。
  * variable.py--系统变量存放处。用于更改串口等设置参数。

## 开发者的话
* Estello.club-- A self-supported newly-born geek club
