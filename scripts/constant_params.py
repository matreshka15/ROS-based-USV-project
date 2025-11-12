#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
USV控制常量参数定义
基于ROS 2 Humble
"""

# 控制模式定义
CONTROL_MANUAL = 0
CONTROL_AUTO = 1
CONTROL_RECORD = 2

# 导航模式定义
NAV_MODE_START_FROM_BEGINNING = 0
NAV_MODE_START_FROM_NEAREST = 1

# 控制模式字典
CONTROL_MODE_DICT = {
    CONTROL_MANUAL: 'manual',
    CONTROL_AUTO: 'auto', 
    CONTROL_RECORD: 'recording'
}

# 串口配置参数
SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 0.1

# 雷达配置参数
RADAR_PORT = '/dev/ttyUSB1'
RADAR_BAUDRATE = 921600

# 控制参数
MAX_THRUST = 100.0          # 最大推力百分比
MAX_YAW_RATE = 45.0         # 最大偏航率 (度/秒)
CONTROL_FREQUENCY = 50.0    # 控制频率 (Hz)

# 导航参数
WAYPOINT_TOLERANCE = 1.0    # 航点到达 tolerance (米)
MAX_SPEED = 2.0             # 最大速度 (米/秒)
SAFETY_DISTANCE = 3.0       # 安全距离 (米)

# ROS话题名称
TOPIC_CONTROL_CMD = '/usv/control/cmd_vel'
TOPIC_CONTROL_MODE = '/usv/control/mode'
TOPIC_NAVIGATION_WAYPOINT = '/usv/navigation/waypoint'
TOPIC_NAVIGATION_STATUS = '/usv/navigation/status'
TOPIC_SENSOR_GPS = '/usv/sensor/gps'
TOPIC_SENSOR_IMU = '/usv/sensor/imu'
TOPIC_SENSOR_RADAR = '/usv/sensor/radar'
TOPIC_SENSOR_BATTERY = '/usv/sensor/battery'

# ROS服务名称
SERVICE_SET_MODE = '/usv/control/set_mode'
SERVICE_GO_TO_WAYPOINT = '/usv/navigation/go_to_waypoint'
SERVICE_START_NAVIGATION = '/usv/navigation/start'
SERVICE_STOP_NAVIGATION = '/usv/navigation/stop'

# 动作名称
ACTION_NAVIGATION = '/usv/navigation/navigate'

# 日志配置
LOG_LEVEL = 'INFO'  # DEBUG, INFO, WARNING, ERROR, CRITICAL

# 颜色定义（用于终端输出）
class Colors:
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    PURPLE = '\033[95m'
    CYAN = '\033[96m'
    RESET = '\033[0m'

def print_color(text, color=Colors.WHITE):
    """带颜色的打印函数"""
    print(f"{color}{text}{Colors.RESET}")

if __name__ == '__main__':
    print("USV控制常量参数定义文件")
    print(f"控制模式: {CONTROL_MODE_DICT}")
    print(f"串口配置: {SERIAL_PORT} @ {SERIAL_BAUDRATE}")
    print(f"控制频率: {CONTROL_FREQUENCY} Hz")