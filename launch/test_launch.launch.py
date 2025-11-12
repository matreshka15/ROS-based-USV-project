#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
USV控制测试启动文件
用于测试各个组件的功能
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess

def generate_launch_description():
    """生成测试启动描述"""
    
    # 包路径
    package_dir = FindPackageShare('usv_control')
    
    # 声明启动参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # 启动控制模式中枢测试
    control_mode_test_node = Node(
        package='usv_control',
        executable='control_mode_hub.py',
        name='control_mode_hub_test',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # 启动串口通信测试（使用虚拟串口）
    serial_test_node = Node(
        package='usv_control',
        executable='serial_port_hub.py',
        name='serial_port_hub_test',
        output='screen',
        parameters=[
            {'serial.port': '/dev/pts/1'},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # 启动导航测试
    navigation_test_node = Node(
        package='usv_control',
        executable='navigation_action_server.py',
        name='navigation_action_server_test',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # 启动雷达测试
    radar_test_node = Node(
        package='usv_control',
        executable='radar_data_processor.py',
        name='radar_data_processor_test',
        output='screen',
        parameters=[
            {'radar.port': '/dev/pts/2'},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # 启动主控制器测试
    controller_test_node = Node(
        package='usv_control',
        executable='usv_controller.py',
        name='usv_controller_test',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # 启动ROS 2话题查看器
    topic_echo_node = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '/usv/system/status'],
        output='screen',
        shell=True
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加启动参数
    ld.add_action(declare_use_sim_time_cmd)
    
    # 添加测试节点
    ld.add_action(control_mode_test_node)
    ld.add_action(serial_test_node)
    ld.add_action(navigation_test_node)
    ld.add_action(radar_test_node)
    ld.add_action(controller_test_node)
    
    # 添加话题查看器
    ld.add_action(topic_echo_node)
    
    return ld