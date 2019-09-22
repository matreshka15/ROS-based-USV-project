# 无人船上位机端程序
此package为使用ROS为框架开发的无人船上位机程序。目前正在开发阶段，功能尚未完成。
# 开发环境
初期在Nvidia TX1开发板上进行开发。(系统：Ubuntu 18.04；ROS版本：melodic)，与ROS kinetic兼容性有待测试。但实际开发过程中发现在Kinetic平台未报错误。
# 开发进度
* main_sequence package:包内主要涵盖：路径规划、上位机与下位机的通信（基于Eastar protocol协议，若需要协议框图请联系开发者）
** control_mode_hub.py--驱动系统状态切换的switcher.遥控器可选择手动操作模式、自动导航模式、计点模式，switcher的作用即接收遥控器模式的切换，并且通知相应server进行相关操作。
** eastar_protocol.py--系统的串口操作hub.所有串口的输入输出脚本均在此脚本中进行
** navigation_action_server--系统的core组件。负责实际路径的规划、GPS坐标点的记录等核心操作。
# 开发者
* Estello.club-- A self-supported newly-born geek club
* Email:8523429@qq.com
