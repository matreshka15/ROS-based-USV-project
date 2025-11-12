#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
USV主控制器
基于ROS 2 Humble
整合所有控制组件，提供统一的无人船控制接口
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Int32, String, Bool, Float32
from geometry_msgs.msg import Twist, Point, PoseStamped
from sensor_msgs.msg import NavSatFix, Imu, BatteryState
from rcl_interfaces.srv import SetParameters, GetParameters
import threading
import time
import constant_params as cp

class USVController(Node):
    def __init__(self):
        super().__init__('usv_controller')
        
        # 系统状态
        self.system_status = "starting"
        self.controller_lock = threading.Lock()
        
        # 创建回调组
        self.callback_group = ReentrantCallbackGroup()
        
        # 订阅器
        self.battery_sub = self.create_subscription(
            BatteryState,
            cp.TOPIC_SENSOR_BATTERY,
            self.battery_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            cp.TOPIC_SENSOR_IMU,
            self.imu_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            cp.TOPIC_SENSOR_GPS,
            self.gps_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.radar_sub = self.create_subscription(
            String,
            cp.TOPIC_SENSOR_RADAR,
            self.radar_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.mode_status_sub = self.create_subscription(
            String,
            f'{cp.TOPIC_CONTROL_MODE}/status',
            self.mode_status_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.navigation_status_sub = self.create_subscription(
            String,
            cp.TOPIC_NAVIGATION_STATUS,
            self.navigation_status_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 发布器
        self.system_status_pub = self.create_publisher(
            String,
            '/usv/system/status',
            10
        )
        
        self.heartbeat_pub = self.create_publisher(
            Bool,
            '/usv/heartbeat',
            10
        )
        
        self.debug_info_pub = self.create_publisher(
            String,
            '/usv/debug/info',
            10
        )
        
        # 服务客户端
        self.set_mode_client = self.create_client(
            SetParameters,
            cp.SERVICE_SET_MODE,
            callback_group=self.callback_group
        )
        
        # 系统状态
        self.battery_voltage = 0.0
        self.battery_percentage = 0.0
        self.current_mode = "manual"
        self.navigation_status = "idle"
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        
        # GPS状态
        self.latitude = 0.0
        self.longitude = 0.0
        self.gps_fix = False
        
        # IMU状态
        self.linear_acceleration = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 0.0]
        
        # 系统监控定时器
        self.monitor_timer = self.create_timer(
            1.0,
            self.system_monitor_callback,
            callback_group=self.callback_group
        )
        
        # 心跳定时器
        self.heartbeat_timer = self.create_timer(
            0.5,
            self.heartbeat_callback,
            callback_group=self.callback_group
        )
        
        # 状态报告定时器
        self.status_report_timer = self.create_timer(
            5.0,
            self.status_report_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("USV主控制器已启动")
        
        # 启动系统自检
        self.system_self_test()

    def system_self_test(self):
        """系统自检"""
        self.get_logger().info("开始系统自检...")
        
        # 检查必要的服务是否可用
        services_available = True
        
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("控制模式服务不可用")
            services_available = False
        
        if services_available:
            self.system_status = "ready"
            self.get_logger().info("系统自检完成，状态: ready")
        else:
            self.system_status = "error"
            self.get_logger().error("系统自检失败，状态: error")
        
        self.publish_system_status()

    def battery_callback(self, msg):
        """电池状态回调函数"""
        self.battery_voltage = msg.voltage
        self.battery_percentage = msg.percentage
        
        # 检查电池电量
        if self.battery_percentage < 0.2:
            self.get_logger().warning(f"低电量警告: {self.battery_percentage:.1%}")
            if self.battery_percentage < 0.1:
                self.get_logger().error("电量严重不足，建议立即返航")

    def imu_callback(self, msg):
        """IMU数据回调函数"""
        self.linear_acceleration = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]
        self.angular_velocity = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]
        self.orientation = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

    def gps_callback(self, msg):
        """GPS数据回调函数"""
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.gps_fix = (msg.status.status >= msg.status.STATUS_FIX)
        
        if not self.gps_fix:
            self.get_logger().warning("GPS信号弱或无定位")

    def radar_callback(self, msg):
        """雷达数据回调函数"""
        try:
            data = msg.data.split(',')
            if len(data) >= 2:
                self.obstacle_distance = float(data[0])
                self.obstacle_detected = self.obstacle_distance < cp.SAFETY_DISTANCE
        except Exception as e:
            self.get_logger().warning(f"解析雷达数据错误: {str(e)}")

    def mode_status_callback(self, msg):
        """控制模式状态回调函数"""
        self.current_mode = msg.data

    def navigation_status_callback(self, msg):
        """导航状态回调函数"""
        self.navigation_status = msg.data

    def set_control_mode(self, mode):
        """设置控制模式"""
        if mode not in ['manual', 'auto', 'record']:
            self.get_logger().error(f"无效的控制模式: {mode}")
            return False
        
        # 转换模式名称为数值
        mode_map = {
            'manual': cp.CONTROL_MANUAL,
            'auto': cp.CONTROL_AUTO,
            'record': cp.CONTROL_RECORD
        }
        
        request = SetParameters.Request()
        request.parameters.append(
            Parameter(
                name='control_mode',
                value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=mode_map[mode])
            )
        )
        
        future = self.set_mode_client.call_async(request)
        return future

    def system_monitor_callback(self, msg=None):
        """系统监控回调函数"""
        with self.controller_lock:
            # 检查系统状态
            if self.system_status == "error":
                self.get_logger().error("系统错误状态，请检查")
                return
            
            # 检查关键传感器数据
            self.check_sensor_health()

    def check_sensor_health(self):
        """检查传感器健康状态"""
        # 检查GPS连接
        if not self.gps_fix:
            self.get_logger().warning("GPS无定位")
        
        # 检查电池状态
        if self.battery_percentage < 0.2:
            self.get_logger().warning(f"电池电量低: {self.battery_percentage:.1%}")

    def heartbeat_callback(self):
        """心跳回调函数"""
        heartbeat_msg = Bool()
        heartbeat_msg.data = True
        self.heartbeat_pub.publish(heartbeat_msg)

    def status_report_callback(self):
        """状态报告回调函数"""
        # 发布详细的系统状态报告
        status_report = (
            f"System Status: {self.system_status}\n"
            f"Control Mode: {self.current_mode}\n"
            f"Navigation Status: {self.navigation_status}\n"
            f"Battery: {self.battery_voltage:.2f}V ({self.battery_percentage:.1%})\n"
            f"GPS: {'Fix' if self.gps_fix else 'No Fix'} "
            f"({self.latitude:.6f}, {self.longitude:.6f})\n"
            f"Obstacle: {'Detected' if self.obstacle_detected else 'None'} "
            f"(Distance: {self.obstacle_distance:.2f}m)"
        )
        
        debug_msg = String()
        debug_msg.data = status_report
        self.debug_info_pub.publish(debug_msg)
        
        self.get_logger().info(f"\n{status_report}")

    def publish_system_status(self):
        """发布系统状态"""
        status_msg = String()
        status_msg.data = self.system_status
        self.system_status_pub.publish(status_msg)

    def destroy_node(self):
        """销毁节点"""
        self.system_status = "shutdown"
        self.publish_system_status()
        self.get_logger().info("USV主控制器已关闭")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        usv_controller = USVController()
        executor = MultiThreadedExecutor()
        executor.add_node(usv_controller)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            usv_controller.get_logger().info("收到键盘中断，正在关闭...")
        finally:
            executor.shutdown()
            usv_controller.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()