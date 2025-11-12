#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
USV雷达数据处理器
基于ROS 2 Humble
处理IWR1443毫米波雷达数据，提供障碍物检测功能
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import LaserScan
import serial
import threading
import time
import struct
import binascii
import math
import constant_params as cp

class RadarDataProcessor(Node):
    def __init__(self):
        super().__init__('radar_data_processor')
        
        # 雷达配置
        self.radar_port = cp.RADAR_PORT
        self.radar_baudrate = cp.RADAR_BAUDRATE
        
        # 串口对象
        self.radar_serial = None
        self.radar_connected = False
        
        # 创建回调组
        self.callback_group = ReentrantCallbackGroup()
        
        # 发布器
        self.radar_data_pub = self.create_publisher(
            String,
            cp.TOPIC_SENSOR_RADAR,
            10
        )
        
        self.obstacle_detected_pub = self.create_publisher(
            Bool,
            '/usv/obstacle/detected',
            10
        )
        
        self.obstacle_distance_pub = self.create_publisher(
            Float32,
            '/usv/obstacle/distance',
            10
        )
        
        self.laser_scan_pub = self.create_publisher(
            LaserScan,
            '/usv/scan',
            10
        )
        
        self.radar_status_pub = self.create_publisher(
            String,
            '/usv/radar/status',
            10
        )
        
        # 雷达状态
        self.obstacle_distance = float('inf')
        self.obstacle_angle = 0.0
        self.obstacle_detected = False
        self.radar_points = []
        
        # 连接状态定时器
        self.connection_timer = self.create_timer(
            5.0,
            self.check_radar_connection,
            callback_group=self.callback_group
        )
        
        # 数据处理定时器
        self.processing_timer = self.create_timer(
            0.1,  # 10Hz处理频率
            self.process_radar_data,
            callback_group=self.callback_group
        )
        
        # 数据读取线程
        self.read_thread = threading.Thread(target=self.read_radar_data)
        self.read_thread.daemon = True
        
        # 初始化雷达连接
        self.init_radar_connection()
        
        # 启动读取线程
        self.read_thread.start()
        
        self.get_logger().info("雷达数据处理器已启动")

    def init_radar_connection(self):
        """初始化雷达串口连接"""
        try:
            self.radar_serial = serial.Serial(
                port=self.radar_port,
                baudrate=self.radar_baudrate,
                timeout=0.1,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            
            if self.radar_serial.is_open:
                self.radar_connected = True
                self.get_logger().info(f"成功连接到雷达: {self.radar_port}")
                self.publish_radar_status("connected")
                
                # 发送雷达配置命令（根据IWR1443协议）
                self.configure_radar()
            else:
                self.radar_connected = False
                self.get_logger().error(f"雷达连接失败: {self.radar_port}")
                
        except Exception as e:
            self.radar_connected = False
            self.get_logger().error(f"雷达初始化错误: {str(e)}")

    def check_radar_connection(self):
        """检查雷达连接状态"""
        if self.radar_serial and self.radar_serial.is_open:
            self.radar_connected = True
            self.publish_radar_status("connected")
        else:
            self.radar_connected = False
            self.publish_radar_status("disconnected")
            self.get_logger().warning("雷达连接丢失，尝试重新连接...")
            self.init_radar_connection()

    def publish_radar_status(self, status):
        """发布雷达状态"""
        status_msg = String()
        status_msg.data = status
        self.radar_status_pub.publish(status_msg)

    def configure_radar(self):
        """配置雷达参数"""
        if not self.radar_connected:
            return
            
        try:
            # IWR1443配置命令示例（需要根据实际协议修改）
            # 这里只是示例，实际命令需要参考IWR1443的数据手册
            
            # 配置检测范围
            config_cmd = "sensorStop\n"
            self.radar_serial.write(config_cmd.encode())
            time.sleep(0.1)
            
            # 设置检测距离范围
            config_cmd = "profileCfg 0 77 6 7 100 1 0 8 1 128 30 0 0 40 1 0 0\n"
            self.radar_serial.write(config_cmd.encode())
            time.sleep(0.1)
            
            # 设置检测模式
            config_cmd = "detectCfg -1 0 0 1 1\n"
            self.radar_serial.write(config_cmd.encode())
            time.sleep(0.1)
            
            # 启动传感器
            config_cmd = "sensorStart\n"
            self.radar_serial.write(config_cmd.encode())
            time.sleep(0.1)
            
            self.get_logger().info("雷达配置完成")
            
        except Exception as e:
            self.get_logger().error(f"雷达配置错误: {str(e)}")

    def read_radar_data(self):
        """读取雷达数据线程"""
        while rclpy.ok():
            try:
                if self.radar_connected and self.radar_serial and self.radar_serial.is_open:
                    if self.radar_serial.in_waiting > 0:
                        line = self.radar_serial.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            self.process_raw_radar_data(line)
                else:
                    time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"读取雷达数据错误: {str(e)}")
                self.radar_connected = False
                time.sleep(1.0)

    def process_raw_radar_data(self, raw_data):
        """处理原始雷达数据"""
        # IWR1443输出格式示例：
        # "Detected objects: 1"
        # "Target 0: Distance: 2.5m, Angle: 15.0deg, Speed: 0.0m/s"
        
        try:
            if "Target" in raw_data:
                # 解析目标数据
                parts = raw_data.split(':')
                if len(parts) >= 4:
                    distance_str = parts[2].split(',')[0].strip().replace('m', '')
                    angle_str = parts[3].split(',')[0].strip().replace('deg', '')
                    
                    distance = float(distance_str)
                    angle = float(angle_str)
                    
                    self.radar_points.append((distance, angle))
                    self.get_logger().debug(f"雷达目标: 距离={distance:.2f}m, 角度={angle:.1f}°")
                    
        except Exception as e:
            self.get_logger().warning(f"解析雷达数据错误: {str(e)}, 原始数据: {raw_data}")

    def process_radar_data(self):
        """处理雷达数据并发布结果"""
        if not self.radar_connected:
            return
            
        if self.radar_points:
            # 找到最近的障碍物
            min_distance = float('inf')
            min_angle = 0.0
            
            for distance, angle in self.radar_points:
                if distance < min_distance and distance > 0.1:  # 过滤近距离噪声
                    min_distance = distance
                    min_angle = angle
            
            self.obstacle_distance = min_distance
            self.obstacle_angle = min_angle
            self.obstacle_detected = min_distance < cp.SAFETY_DISTANCE
            
            # 发布障碍物信息
            self.publish_obstacle_info()
            
            # 发布激光扫描消息（模拟）
            self.publish_laser_scan()
            
            # 清空雷达点列表
            self.radar_points.clear()

    def publish_obstacle_info(self):
        """发布障碍物信息"""
        # 发布障碍物检测状态
        detected_msg = Bool()
        detected_msg.data = self.obstacle_detected
        self.obstacle_detected_pub.publish(detected_msg)
        
        # 发布障碍物距离
        distance_msg = Float32()
        distance_msg.data = self.obstacle_distance
        self.obstacle_distance_pub.publish(distance_msg)
        
        # 发布雷达数据字符串
        radar_msg = String()
        radar_msg.data = f"{self.obstacle_distance:.2f},{self.obstacle_angle:.1f}"
        self.radar_data_pub.publish(radar_msg)
        
        if self.obstacle_detected:
            self.get_logger().warning(f"检测到障碍物: 距离={self.obstacle_distance:.2f}m, 角度={self.obstacle_angle:.1f}°")

    def publish_laser_scan(self):
        """发布激光扫描消息（模拟）"""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "radar_link"
        
        # 毫米波雷达通常只有几个检测点，这里模拟为激光扫描
        scan_msg.angle_min = math.radians(-90.0)
        scan_msg.angle_max = math.radians(90.0)
        scan_msg.angle_increment = math.radians(1.0)
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 50.0
        
        # 创建空的距离数组
        num_ranges = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
        scan_msg.ranges = [float('inf')] * num_ranges
        
        # 如果检测到障碍物，在对应角度位置设置距离
        if self.obstacle_detected and self.obstacle_distance < scan_msg.range_max:
            angle_index = int((math.radians(self.obstacle_angle) - scan_msg.angle_min) / scan_msg.angle_increment)
            if 0 <= angle_index < num_ranges:
                scan_msg.ranges[angle_index] = self.obstacle_distance
        
        self.laser_scan_pub.publish(scan_msg)

    def destroy_node(self):
        """销毁节点时关闭雷达"""
        if self.radar_serial and self.radar_serial.is_open:
            # 停止雷达
            try:
                self.radar_serial.write("sensorStop\n".encode())
                time.sleep(0.1)
            except:
                pass
            self.radar_serial.close()
            self.get_logger().info("雷达已关闭")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        radar_processor = RadarDataProcessor()
        executor = MultiThreadedExecutor()
        executor.add_node(radar_processor)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            radar_processor.get_logger().info("收到键盘中断，正在关闭...")
        finally:
            executor.shutdown()
            radar_processor.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()