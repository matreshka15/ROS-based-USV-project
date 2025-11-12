#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
USV导航动作服务器
基于ROS 2 Humble
实现无人船的自动导航功能
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from geometry_msgs.msg import Twist, Point, PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String, Float32
import action_msgs.msg._goal_status
import math
import threading
import time
from enum import Enum
import constant_params as cp

# 导航动作定义（需要先创建对应的.action文件）
from usv_control.action import NavigateToWaypoint

class NavigationState(Enum):
    """导航状态枚举"""
    IDLE = 0
    NAVIGATING = 1
    WAYPOINT_REACHED = 2
    GOAL_COMPLETED = 3
    FAILED = 4

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')
        
        # 导航状态
        self.navigation_state = NavigationState.IDLE
        self.current_goal_handle = None
        self.goal_lock = threading.Lock()
        
        # 创建回调组
        self.callback_group = ReentrantCallbackGroup()
        
        # 订阅器
        self.gps_sub = self.create_subscription(
            NavSatFix,
            cp.TOPIC_SENSOR_GPS,
            self.gps_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.imu_sub = self.create_subscription(
            PoseStamped,  # 假设使用PoseStamped来获取航向
            '/usv/pose',
            self.pose_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.radar_sub = self.create_subscription(
            String,  # 假设雷达数据是字符串格式
            cp.TOPIC_SENSOR_RADAR,
            self.radar_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 发布器
        self.control_cmd_pub = self.create_publisher(
            Twist,
            cp.TOPIC_CONTROL_CMD,
            10
        )
        
        self.navigation_status_pub = self.create_publisher(
            String,
            cp.TOPIC_NAVIGATION_STATUS,
            10
        )
        
        self.target_waypoint_pub = self.create_publisher(
            Point,
            cp.TOPIC_NAVIGATION_WAYPOINT,
            10
        )
        
        # 动作服务器
        self.action_server = ActionServer(
            self,
            NavigateToWaypoint,
            cp.ACTION_NAVIGATION,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        
        # 当前状态
        self.current_latitude = 0.0
        self.current_longitude = 0.0
        self.current_heading = 0.0  # 弧度
        self.current_speed = 0.0
        
        # 目标状态
        self.target_latitude = 0.0
        self.target_longitude = 0.0
        self.current_waypoint_index = 0
        self.waypoints = []
        
        # 控制参数
        self.Kp_distance = 0.5      # 距离控制器比例系数
        self.Kp_heading = 2.0       # 航向控制器比例系数
        self.max_angular_velocity = math.radians(30.0)  # 最大角速度 (弧度/秒)
        
        # 避障相关
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        
        # 导航定时器
        self.navigation_timer = self.create_timer(
            1.0 / cp.CONTROL_FREQUENCY,
            self.navigation_control_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("导航动作服务器已启动")

    def goal_callback(self, goal_request):
        """目标请求回调函数"""
        self.get_logger().info("收到导航目标请求")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """取消请求回调函数"""
        self.get_logger().info("收到导航取消请求")
        with self.goal_lock:
            self.navigation_state = NavigationState.IDLE
            self.current_goal_handle = None
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """执行导航目标"""
        self.get_logger().info("开始执行导航目标")
        
        with self.goal_lock:
            self.current_goal_handle = goal_handle
            self.waypoints = goal_handle.request.waypoints
            self.current_waypoint_index = 0
            self.navigation_state = NavigationState.NAVIGATING
        
        feedback_msg = NavigateToWaypoint.Feedback()
        result_msg = NavigateToWaypoint.Result()
        
        try:
            while rclpy.ok() and self.navigation_state == NavigationState.NAVIGATING:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result_msg.success = False
                    result_msg.message = "导航已取消"
                    return result_msg
                
                if self.current_waypoint_index < len(self.waypoints):
                    # 获取当前目标航点
                    waypoint = self.waypoints[self.current_waypoint_index]
                    self.target_latitude = waypoint.latitude
                    self.target_longitude = waypoint.longitude
                    
                    # 发布目标航点
                    target_point = Point()
                    target_point.x = self.target_latitude
                    target_point.y = self.target_longitude
                    self.target_waypoint_pub.publish(target_point)
                    
                    # 计算到目标的距离和方位
                    distance_to_target = self.calculate_distance(
                        self.current_latitude, self.current_longitude,
                        self.target_latitude, self.target_longitude
                    )
                    
                    bearing_to_target = self.calculate_bearing(
                        self.current_latitude, self.current_longitude,
                        self.target_latitude, self.target_longitude
                    )
                    
                    # 更新反馈
                    feedback_msg.current_latitude = self.current_latitude
                    feedback_msg.current_longitude = self.current_longitude
                    feedback_msg.distance_to_waypoint = distance_to_target
                    feedback_msg.waypoint_index = self.current_waypoint_index
                    feedback_msg.total_waypoints = len(self.waypoints)
                    goal_handle.publish_feedback(feedback_msg)
                    
                    # 检查是否到达当前航点
                    if distance_to_target < cp.WAYPOINT_TOLERANCE:
                        self.get_logger().info(f"到达航点 {self.current_waypoint_index + 1}/{len(self.waypoints)}")
                        self.current_waypoint_index += 1
                        
                        if self.current_waypoint_index >= len(self.waypoints):
                            self.navigation_state = NavigationState.GOAL_COMPLETED
                            break
                    
                    # 发布导航状态
                    self.publish_navigation_status(
                        f"Navigating to waypoint {self.current_waypoint_index + 1}, "
                        f"Distance: {distance_to_target:.2f}m"
                    )
                    
                else:
                    self.navigation_state = NavigationState.GOAL_COMPLETED
                    break
                
                await rclpy.task.sleep(0.1)
            
            if self.navigation_state == NavigationState.GOAL_COMPLETED:
                goal_handle.succeed()
                result_msg.success = True
                result_msg.message = "导航目标完成"
                self.publish_navigation_status("Navigation completed successfully")
            else:
                goal_handle.abort()
                result_msg.success = False
                result_msg.message = "导航失败"
                self.publish_navigation_status("Navigation failed")
                
            return result_msg
            
        except Exception as e:
            self.get_logger().error(f"导航执行错误: {str(e)}")
            goal_handle.abort()
            result_msg.success = False
            result_msg.message = f"导航执行错误: {str(e)}"
            return result_msg

    def navigation_control_callback(self):
        """导航控制回调函数"""
        if self.navigation_state != NavigationState.NAVIGATING:
            return
        
        if self.current_waypoint_index < len(self.waypoints):
            # 获取当前目标航点
            waypoint = self.waypoints[self.current_waypoint_index]
            target_lat = waypoint.latitude
            target_lon = waypoint.longitude
            
            # 计算到目标的距离和方位
            distance_to_target = self.calculate_distance(
                self.current_latitude, self.current_longitude,
                target_lat, target_lon
            )
            
            bearing_to_target = self.calculate_bearing(
                self.current_latitude, self.current_longitude,
                target_lat, target_lon
            )
            
            # 计算航向误差
            heading_error = self.normalize_angle(bearing_to_target - self.current_heading)
            
            # 避障处理
            if self.obstacle_detected and self.obstacle_distance < cp.SAFETY_DISTANCE:
                self.get_logger().warning(f"检测到障碍物，距离: {self.obstacle_distance:.2f}m")
                self.avoid_obstacle()
                return
            
            # 计算控制指令
            control_cmd = Twist()
            
            # 距离控制 - 基于距离调整速度
            if distance_to_target > cp.WAYPOINT_TOLERANCE * 2:
                # 远距模式 - 全速前进
                control_cmd.linear.x = cp.MAX_SPEED
            else:
                # 近距模式 - 减速接近
                control_cmd.linear.x = min(cp.MAX_SPEED, self.Kp_distance * distance_to_target)
            
            # 航向控制
            angular_velocity = self.Kp_heading * heading_error
            angular_velocity = max(min(angular_velocity, self.max_angular_velocity), -self.max_angular_velocity)
            control_cmd.angular.z = angular_velocity
            
            # 发布控制指令
            self.control_cmd_pub.publish(control_cmd)
            
            self.get_logger().debug(
                f"导航控制: 距离={distance_to_target:.2f}m, "
                f"航向误差={math.degrees(heading_error):.1f}°, "
                f"线速度={control_cmd.linear.x:.2f}m/s, "
                f"角速度={math.degrees(control_cmd.angular.z):.1f}°/s"
            )

    def avoid_obstacle(self):
        """避障行为"""
        # 简单的避障策略：停止并转向
        control_cmd = Twist()
        control_cmd.linear.x = 0.0  # 停止前进
        control_cmd.angular.z = self.max_angular_velocity  # 向右转向
        
        self.control_cmd_pub.publish(control_cmd)
        self.publish_navigation_status("Avoiding obstacle...")

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """计算两点之间的距离 (米)"""
        # 使用简化的距离计算公式 (适用于近距离)
        R = 6371000  # 地球半径 (米)
        
        dLat = math.radians(lat2 - lat1)
        dLon = math.radians(lon2 - lon1)
        
        a = math.sin(dLat/2) * math.sin(dLat/2) + \
            math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * \
            math.sin(dLon/2) * math.sin(dLon/2)
        
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = R * c
        
        return distance

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """计算从点1到点2的方位角 (弧度)"""
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)
        
        dLon = lon2 - lon1
        
        y = math.sin(dLon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - \
            math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
        
        bearing = math.atan2(y, x)
        return self.normalize_angle(bearing)

    def normalize_angle(self, angle):
        """将角度归一化到 [-π, π] 范围"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def gps_callback(self, msg):
        """GPS数据回调函数"""
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude

    def pose_callback(self, msg):
        """位姿数据回调函数"""
        # 从四元数提取航向角
        orientation = msg.pose.orientation
        self.current_heading = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                                         1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))

    def radar_callback(self, msg):
        """雷达数据回调函数"""
        # 假设雷达数据格式为 "distance,angle"
        try:
            data = msg.data.split(',')
            if len(data) >= 1:
                self.obstacle_distance = float(data[0])
                self.obstacle_detected = self.obstacle_distance < cp.SAFETY_DISTANCE
        except Exception as e:
            self.get_logger().warning(f"解析雷达数据错误: {str(e)}")

    def publish_navigation_status(self, status):
        """发布导航状态"""
        status_msg = String()
        status_msg.data = status
        self.navigation_status_pub.publish(status_msg)

    def destroy_node(self):
        """销毁节点"""
        self.action_server.destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        navigation_server = NavigationActionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(navigation_server)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            navigation_server.get_logger().info("收到键盘中断，正在关闭...")
        finally:
            executor.shutdown()
            navigation_server.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()