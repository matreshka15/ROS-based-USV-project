#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
USV控制主启动文件
基于ROS 2 Humble
启动所有必要的节点和组件
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    config_file = LaunchConfiguration('config_file')
    
    # 包路径
    package_dir = FindPackageShare('usv_control')
    
    # 默认配置文件路径
    default_config_file = PathJoinSubstitution(
        [package_dir, 'config', 'usv_config.yaml']
    )
    
    # 声明启动参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Full path to the ROS 2 parameters file'
    )
    
    # 启动控制模式中枢
    control_mode_hub_node = Node(
        package='usv_control',
        executable='control_mode_hub.py',
        name='control_mode_hub',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/cmd_vel', '/usv/manual/cmd_vel')
        ]
    )
    
    # 启动串口通信中枢
    serial_port_hub_node = Node(
        package='usv_control',
        executable='serial_port_hub.py',
        name='serial_port_hub',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 启动导航动作服务器
    navigation_server_node = Node(
        package='usv_control',
        executable='navigation_action_server.py',
        name='navigation_action_server',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 启动雷达数据处理器
    radar_processor_node = Node(
        package='usv_control',
        executable='radar_data_processor.py',
        name='radar_data_processor',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 启动USV主控制器
    usv_controller_node = Node(
        package='usv_control',
        executable='usv_controller.py',
        name='usv_controller',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 启动TF发布器
    tf_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link']
    )
    
    tf_publisher_node2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_gps_link',
        arguments=['0.5', '0', '0.2', '0', '0', '0', 'base_link', 'gps_link']
    )
    
    tf_publisher_node3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_radar_link',
        arguments=['-0.3', '0', '0.15', '0', '0', '0', 'base_link', 'radar_link']
    )
    
    # 启动rviz2（可选）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([package_dir, 'config', 'usv.rviz'])],
        condition=LaunchConfigurationEquals('use_rviz', 'true')
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加启动参数
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_file_cmd)
    
    # 添加节点
    ld.add_action(control_mode_hub_node)
    ld.add_action(serial_port_hub_node)
    ld.add_action(navigation_server_node)
    ld.add_action(radar_processor_node)
    ld.add_action(usv_controller_node)
    
    # 添加TF发布器
    ld.add_action(tf_publisher_node)
    ld.add_action(tf_publisher_node2)
    ld.add_action(tf_publisher_node3)
    
    # 添加rviz节点（默认不启动）
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='false'))
    ld.add_action(rviz_node)
    
    return ld

class LaunchConfigurationEquals:
    """条件类：检查启动配置是否等于指定值"""
    def __init__(self, launch_configuration_name, value):
        self.launch_configuration_name = launch_configuration_name
        self.value = value

    def evaluate(self, context):
        """评估条件"""
        config_value = context.launch_configurations.get(self.launch_configuration_name)
        return config_value == self.value