#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
USV控制模式中枢
基于ROS 2 Humble
管理无人船的各种控制模式：手动、自动、记录
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Int32, String, Bool
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType, ParameterValue, Parameter
import threading
import time
from enum import Enum
import constant_params as cp

class ControlMode(Enum):
    """控制模式枚举"""
    MANUAL = cp.CONTROL_MANUAL
    AUTO = cp.CONTROL_AUTO
    RECORD = cp.CONTROL_RECORD

class ControlModeHub(Node):
    def __init__(self):
        super().__init__('control_mode_hub')
        
        # 控制模式状态
        self.current_mode = ControlMode.MANUAL
        self.mode_lock = threading.Lock()
        
        # 创建回调组
        self.callback_group = ReentrantCallbackGroup()
        
        # 订阅器
        self.manual_control_sub = self.create_subscription(
            Twist,
            '/cmd_vel',  # 标准的cmd_vel话题
            self.manual_control_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.mode_command_sub = self.create_subscription(
            Int32,
            cp.TOPIC_CONTROL_MODE,
            self.mode_command_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 发布器
        self.control_output_pub = self.create_publisher(
            Twist,
            cp.TOPIC_CONTROL_CMD,
            10
        )
        
        self.mode_status_pub = self.create_publisher(
            String,
            f'{cp.TOPIC_CONTROL_MODE}/status',
            10
        )
        
        # 服务
        self.set_mode_service = self.create_service(
            SetParameters,
            cp.SERVICE_SET_MODE,
            self.set_mode_callback,
            callback_group=self.callback_group
        )
        
        # 自动控制定时器
        self.auto_control_timer = self.create_timer(
            1.0 / cp.CONTROL_FREQUENCY,
            self.auto_control_callback,
            callback_group=self.callback_group
        )
        
        # 记录模式相关
        self.recording = False
        self.recorded_waypoints = []
        
        self.get_logger().info(f"控制模式中枢已启动，当前模式: {self.current_mode.name}")

    def get_current_mode(self):
        """获取当前控制模式"""
        with self.mode_lock:
            return self.current_mode

    def set_control_mode(self, new_mode):
        """设置控制模式"""
        if not isinstance(new_mode, ControlMode):
            try:
                new_mode = ControlMode(new_mode)
            except ValueError:
                self.get_logger().error(f"无效的控制模式: {new_mode}")
                return False
        
        with self.mode_lock:
            if self.current_mode != new_mode:
                self.get_logger().info(f"切换控制模式: {self.current_mode.name} -> {new_mode.name}")
                self.current_mode = new_mode
                
                # 模式切换处理
                if new_mode == ControlMode.MANUAL:
                    self.handle_manual_mode_enter()
                elif new_mode == ControlMode.AUTO:
                    self.handle_auto_mode_enter()
                elif new_mode == ControlMode.RECORD:
                    self.handle_record_mode_enter()
                
                # 发布模式状态
                self.publish_mode_status()
                return True
        return False

    def handle_manual_mode_enter(self):
        """进入手动模式处理"""
        self.get_logger().info("进入手动模式")

    def handle_auto_mode_enter(self):
        """进入自动模式处理"""
        self.get_logger().info("进入自动模式")

    def handle_record_mode_enter(self):
        """进入记录模式处理"""
        self.get_logger().info("进入记录模式")
        self.recording = True

    def manual_control_callback(self, msg):
        """手动控制回调函数"""
        if self.get_current_mode() == ControlMode.MANUAL:
            self.control_output_pub.publish(msg)
            self.get_logger().debug(f"手动控制指令: 线速度={msg.linear.x}, 角速度={msg.angular.z}")

    def mode_command_callback(self, msg):
        """模式命令回调函数"""
        self.set_control_mode(msg.data)

    def set_mode_callback(self, request, response):
        """设置模式服务回调函数"""
        for param in request.parameters:
            if param.name == 'control_mode':
                if param.type == ParameterType.PARAMETER_INTEGER:
                    success = self.set_control_mode(param.value.integer_value)
                    response.results.append(ParameterValue(bool_value=success))
                    return response
        
        response.results.append(ParameterValue(bool_value=False))
        return response

    def auto_control_callback(self):
        """自动控制回调函数"""
        if self.get_current_mode() == ControlMode.AUTO:
            # 在实际应用中，这里应该从导航系统获取控制指令
            # 这里只是一个示例
            control_cmd = Twist()
            control_cmd.linear.x = 0.5  # 示例：前进速度
            control_cmd.angular.z = 0.0 # 示例：无转向
            self.control_output_pub.publish(control_cmd)

    def publish_mode_status(self):
        """发布模式状态"""
        status_msg = String()
        status_msg.data = self.current_mode.name
        self.mode_status_pub.publish(status_msg)

    def record_waypoint(self, waypoint):
        """记录航点"""
        if self.get_current_mode() == ControlMode.RECORD and self.recording:
            self.recorded_waypoints.append(waypoint)
            self.get_logger().info(f"记录航点: {waypoint}, 总数: {len(self.recorded_waypoints)}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        control_mode_hub = ControlModeHub()
        executor = MultiThreadedExecutor()
        executor.add_node(control_mode_hub)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            control_mode_hub.get_logger().info("收到键盘中断，正在关闭...")
        finally:
            executor.shutdown()
            control_mode_hub.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()