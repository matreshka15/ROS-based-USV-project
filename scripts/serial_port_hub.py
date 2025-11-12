#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
USV串口通信中枢
基于ROS 2 Humble
负责与STM32下位机的串口通信
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, NavSatFix, BatteryState
from std_msgs.msg import Float32, Bool, String
import serial
import threading
import time
import struct
import binascii
from enum import Enum
import constant_params as cp

class SerialCommand(Enum):
    """串口命令枚举"""
    SET_MOTOR_SPEED = 0x01
    SET_STEERING = 0x02
    SET_CONTROL_MODE = 0x03
    REQUEST_SENSOR_DATA = 0x04
    SENSOR_DATA_RESPONSE = 0x05
    SET_PID_PARAMS = 0x06
    SYSTEM_STATUS = 0x07

class SerialPortHub(Node):
    def __init__(self):
        super().__init__('serial_port_hub')
        
        # 串口配置
        self.serial_port = cp.SERIAL_PORT
        self.baudrate = cp.SERIAL_BAUDRATE
        self.timeout = cp.SERIAL_TIMEOUT
        
        # 串口对象
        self.ser = None
        self.serial_connected = False
        
        # 创建回调组
        self.callback_group = ReentrantCallbackGroup()
        
        # 订阅器
        self.control_cmd_sub = self.create_subscription(
            Twist,
            cp.TOPIC_CONTROL_CMD,
            self.control_cmd_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 发布器
        self.imu_pub = self.create_publisher(
            Imu,
            cp.TOPIC_SENSOR_IMU,
            10
        )
        
        self.gps_pub = self.create_publisher(
            NavSatFix,
            cp.TOPIC_SENSOR_GPS,
            10
        )
        
        self.battery_pub = self.create_publisher(
            BatteryState,
            cp.TOPIC_SENSOR_BATTERY,
            10
        )
        
        self.serial_status_pub = self.create_publisher(
            String,
            '/usv/serial/status',
            10
        )
        
        # 连接状态定时器
        self.connection_timer = self.create_timer(
            5.0,
            self.check_connection,
            callback_group=self.callback_group
        )
        
        # 数据读取线程
        self.read_thread = threading.Thread(target=self.read_serial_data)
        self.read_thread.daemon = True
        
        # 初始化串口连接
        self.init_serial_connection()
        
        # 启动读取线程
        self.read_thread.start()
        
        self.get_logger().info("串口通信中枢已启动")

    def init_serial_connection(self):
        """初始化串口连接"""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            
            if self.ser.is_open:
                self.serial_connected = True
                self.get_logger().info(f"成功连接到串口: {self.serial_port}")
                self.publish_serial_status("connected")
            else:
                self.serial_connected = False
                self.get_logger().error(f"串口连接失败: {self.serial_port}")
                
        except Exception as e:
            self.serial_connected = False
            self.get_logger().error(f"串口初始化错误: {str(e)}")

    def check_connection(self):
        """检查串口连接状态"""
        if self.ser and self.ser.is_open:
            self.serial_connected = True
            self.publish_serial_status("connected")
        else:
            self.serial_connected = False
            self.publish_serial_status("disconnected")
            self.get_logger().warning("串口连接丢失，尝试重新连接...")
            self.init_serial_connection()

    def publish_serial_status(self, status):
        """发布串口状态"""
        status_msg = String()
        status_msg.data = status
        self.serial_status_pub.publish(status_msg)

    def control_cmd_callback(self, msg):
        """控制指令回调函数"""
        if not self.serial_connected:
            self.get_logger().warning("串口未连接，无法发送控制指令")
            return
        
        try:
            # 将Twist消息转换为串口命令
            # 线速度转换为电机速度百分比 (-100 到 100)
            linear_speed = max(min(msg.linear.x * 100.0, 100.0), -100.0)
            # 角速度转换为转向角度 (-45 到 45 度)
            steering_angle = max(min(msg.angular.z * 45.0, 45.0), -45.0)
            
            # 发送电机速度命令
            self.send_motor_command(int(linear_speed))
            # 发送转向命令
            self.send_steering_command(int(steering_angle))
            
        except Exception as e:
            self.get_logger().error(f"发送控制指令错误: {str(e)}")

    def send_motor_command(self, speed):
        """发送电机速度命令"""
        if not self.serial_connected:
            return
            
        # 构建电机速度命令包
        # 格式: [0xAA, 0xBB, 0x01, speed_high, speed_low, checksum]
        speed_bytes = struct.pack('h', speed)  # 2字节有符号整数
        command_packet = bytearray([0xAA, 0xBB, SerialCommand.SET_MOTOR_SPEED.value])
        command_packet.extend(speed_bytes)
        
        # 计算校验和
        checksum = sum(command_packet) & 0xFF
        command_packet.append(checksum)
        
        self.send_serial_data(command_packet)

    def send_steering_command(self, angle):
        """发送转向命令"""
        if not self.serial_connected:
            return
            
        # 构建转向命令包
        angle_bytes = struct.pack('h', angle)  # 2字节有符号整数
        command_packet = bytearray([0xAA, 0xBB, SerialCommand.SET_STEERING.value])
        command_packet.extend(angle_bytes)
        
        checksum = sum(command_packet) & 0xFF
        command_packet.append(checksum)
        
        self.send_serial_data(command_packet)

    def send_serial_data(self, data):
        """发送串口数据"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(data)
                self.get_logger().debug(f"发送串口数据: {binascii.hexlify(data).decode()}")
        except Exception as e:
            self.get_logger().error(f"发送串口数据错误: {str(e)}")
            self.serial_connected = False

    def read_serial_data(self):
        """读取串口数据线程"""
        while rclpy.ok():
            try:
                if self.serial_connected and self.ser and self.ser.is_open:
                    # 读取数据
                    if self.ser.in_waiting > 0:
                        data = self.ser.read(self.ser.in_waiting)
                        self.process_serial_data(data)
                else:
                    time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"读取串口数据错误: {str(e)}")
                self.serial_connected = False
                time.sleep(1.0)

    def process_serial_data(self, data):
        """处理串口数据"""
        # 简单的数据包解析示例
        # 数据包格式: [0xAA, 0xBB, command, data_length, data..., checksum]
        self.get_logger().debug(f"收到串口数据: {binascii.hexlify(data).decode()}")
        
        # 这里需要根据实际的通信协议来解析数据
        # 以下是示例代码，需要根据实际协议修改
        if len(data) >= 5 and data[0] == 0xAA and data[1] == 0xBB:
            command = data[2]
            data_length = data[3]
            
            if len(data) >= 5 + data_length:
                payload = data[4:4+data_length]
                checksum = data[4+data_length]
                
                # 验证校验和
                calculated_checksum = sum(data[:4+data_length]) & 0xFF
                if checksum == calculated_checksum:
                    self.parse_command_packet(command, payload)
                else:
                    self.get_logger().warning("校验和错误")

    def parse_command_packet(self, command, payload):
        """解析命令数据包"""
        try:
            if command == SerialCommand.SENSOR_DATA_RESPONSE.value:
                self.parse_sensor_data(payload)
            elif command == SerialCommand.SYSTEM_STATUS.value:
                self.parse_system_status(payload)
            else:
                self.get_logger().info(f"收到未知命令: {command}")
        except Exception as e:
            self.get_logger().error(f"解析命令数据包错误: {str(e)}")

    def parse_sensor_data(self, payload):
        """解析传感器数据"""
        # 根据实际的传感器数据格式解析
        # 这里是示例代码
        if len(payload) >= 24:  # 假设IMU数据占24字节
            # 解析IMU数据 (示例格式)
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"
            
            # 假设前12字节是加速度 (3个float)
            accel_x, accel_y, accel_z = struct.unpack('fff', payload[0:12])
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z
            
            # 接下来12字节是角速度 (3个float)
            gyro_x, gyro_y, gyro_z = struct.unpack('fff', payload[12:24])
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z
            
            self.imu_pub.publish(imu_msg)

        # 解析GPS数据 (示例)
        if len(payload) >= 32:
            # 假设GPS数据从第24字节开始
            gps_msg = NavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = "gps_link"
            
            # 解析经纬度 (假设是double类型)
            latitude, longitude = struct.unpack('dd', payload[24:40])
            gps_msg.latitude = latitude
            gps_msg.longitude = longitude
            
            self.gps_pub.publish(gps_msg)

    def parse_system_status(self, payload):
        """解析系统状态"""
        if len(payload) >= 4:
            battery_voltage = struct.unpack('f', payload[0:4])[0]
            
            battery_msg = BatteryState()
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            battery_msg.voltage = battery_voltage
            battery_msg.percentage = min(max((battery_voltage - 10.8) / (12.6 - 10.8), 0.0), 1.0)
            
            self.battery_pub.publish(battery_msg)

    def destroy_node(self):
        """销毁节点时关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("串口已关闭")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        serial_port_hub = SerialPortHub()
        executor = MultiThreadedExecutor()
        executor.add_node(serial_port_hub)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            serial_port_hub.get_logger().info("收到键盘中断，正在关闭...")
        finally:
            executor.shutdown()
            serial_port_hub.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()