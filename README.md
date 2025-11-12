# USV Control System for ROS 2 Humble

基于ROS 2 Humble的无人船控制系统，适配Ubuntu 22.04。

## 项目概述

这个项目是从原始ROS 1项目迁移并优化到ROS 2 Humble的无人船控制系统。
该系统实现了无人船的自主导航、手动控制、障碍物检测等核心功能。通过串口与下位机(STM32或ESP32等)相连接；下位机负责动力控制、姿态解算与遥控信号接收等功能，上位机负责自动导航模式的路径规划。

本项目基于原始的ROS-based-USV-project (https://github.com/matreshka15/ROS-based-USV-project)，从ROS 1 (Kinetic/Melodic) 迁移到了ROS 2 Humble，并适配现代Ubuntu 22.04系统。原项目使用STM32F103ZE作为下位机，通过自定义的Eastar协议进行串口通信，上位机负责自动导航模式的路径规划。

* [下位机端工程文件Github仓库](https://github.com/matreshka15/UAS-Project-STM32)

### 重要！所有开发下位机过程中的开发日志以及手册均已存放在下面地址

* [开发无人船过程中参考的传感器手册以及算法资料](https://github.com/matreshka15/unmanned-ship-datasheets)
* 开发日志记录了项目从一开始的立项到后面一步步测试成功的大大小小细节。前后由于放弃了旧的姿态算法、选取了新的姿态算法，因此前期关于姿态的说明仅供参考用。

### 主要功能

- **多模式控制**: 手动控制、自动导航、路径记录
- **串口通信**: 与STM32下位机的可靠通信 (基于Eastar协议)
- **自主导航**: 基于GPS的航点导航
- **障碍物检测**: 集成IWR1443毫米波雷达
- **系统监控**: 实时监控系统状态和传感器数据
- **坐标变换**: 支持TF2坐标变换系统
- **动作服务器**: 使用ROS 2标准动作接口进行导航

## 硬件要求

- **上位机**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **下位机**: STM32F103ZE (或兼容型号)
- **传感器**: GPS模块、IMU、IWR1443毫米波雷达
- **执行器**: 推进电机、转向机构

## 软件依赖

```bash
# ROS 2 Humble基础包
sudo apt install ros-humble-desktop

# 额外依赖
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-action-msgs
sudo apt install ros-humble-rcl-interfaces
sudo apt install ros-humble-tf2-ros
sudo apt install python3-serial
sudo apt install python3-pip
pip3 install pynmea2
```

## 安装步骤

### 1. 创建工作空间

```bash
mkdir -p ~/usv_ros2_ws/src
cd ~/usv_ros2_ws
```

### 2. 克隆代码

```bash
cd src
git clone <repository-url> usv_control
cd ..
```

### 3. 构建项目

```bash
# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 构建
colcon build --packages-select usv_control

# 刷新环境
source install/setup.bash
```

## 配置文件

主要配置文件位于 `config/usv_config.yaml`，可以根据实际硬件配置进行修改：

```yaml
# 串口配置
serial:
  port: "/dev/ttyUSB0"    # 下位机串口
  baudrate: 115200        # 波特率
  timeout: 0.1            # 串口超时时间

# 雷达配置
radar:
  port: "/dev/ttyUSB1"    # 雷达串口
  baudrate: 921600        # 雷达波特率
  safety_distance: 3.0    # 安全距离 (米)

# 控制参数
control:
  frequency: 50.0         # 控制频率 (Hz)
  max_thrust: 100.0       # 最大推力百分比
  max_yaw_rate: 45.0      # 最大偏航率 (度/秒)
  max_speed: 2.0          # 最大速度 (米/秒)

# 导航参数
navigation:
  waypoint_tolerance: 1.0 # 航点到达容差 (米)
  kp_distance: 0.5        # 距离控制器比例系数
  kp_heading: 2.0         # 航向控制器比例系数
  max_angular_velocity: 30.0  # 最大角速度 (度/秒)

# PID控制器参数
pid:
  linear:
    p: 0.5
    i: 0.1
    d: 0.05
  angular:
    p: 2.0
    i: 0.5
    d: 0.1

# 坐标系配置
tf:
  base_link: "base_link"
  imu_link: "imu_link"
  gps_link: "gps_link"
  radar_link: "radar_link"
  camera_link: "camera_link"
```

## 启动系统

### 启动完整系统

```bash
ros2 launch usv_control usv_control.launch.py
```

### 启动带RViz的系统

```bash
ros2 launch usv_control usv_control.launch.py use_rviz:=true
```

### 启动测试模式

```bash
ros2 launch usv_control test_launch.launch.py
```

## 节点说明

### 1. control_mode_hub (控制模式中枢)

- **功能**: 管理控制模式切换（手动/自动/记录）
- **订阅**:
  - `/cmd_vel` - 手动控制指令
  - `/usv/control/mode` - 模式切换指令
- **发布**:
  - `/usv/control/cmd_vel` - 控制输出
  - `/usv/control/mode/status` - 模式状态
- **服务**:
  - `/usv/control/set_mode` - 设置控制模式

### 2. serial_port_hub (串口通信中枢)

- **功能**: 与下位机的串口通信 (基于Eastar协议)
- **订阅**: `/usv/control/cmd_vel` - 控制指令
- **发布**:
  - `/usv/sensor/imu` - IMU数据
  - `/usv/sensor/gps` - GPS数据
  - `/usv/sensor/battery` - 电池状态

### 3. navigation_action_server (导航动作服务器)

- **功能**: 实现自主导航功能
- **动作**: `/usv/navigation/navigate` - 导航到航点
- **订阅**:
  - `/usv/sensor/gps` - GPS数据
  - `/usv/pose` - 位姿数据
  - `/usv/sensor/radar` - 雷达数据
- **发布**:
  - `/usv/navigation/status` - 导航状态
  - `/usv/navigation/waypoint` - 目标航点

### 4. radar_data_processor (雷达数据处理器)

- **功能**: 处理IWR1443雷达数据
- **订阅**: 雷达原始数据
- **发布**:
  - `/usv/sensor/radar` - 处理后的雷达数据
  - `/usv/obstacle/detected` - 障碍物检测状态
  - `/usv/scan` - 激光扫描消息（模拟）

### 5. usv_controller (USV主控制器)

- **功能**: 系统总控和状态监控
- **订阅**: 所有传感器数据和状态信息
- **发布**:
  - `/usv/system/status` - 系统状态
  - `/usv/heartbeat` - 心跳信号
  - `/usv/debug/info` - 调试信息

## 服务接口

### 设置控制模式

```bash
ros2 service call /usv/control/set_mode rcl_interfaces/srv/SetParameters "{parameters: [{name: 'control_mode', value: {type: 2, integer_value: 1}}]}"
```

- 0: 手动模式
- 1: 自动模式
- 2: 记录模式

### 导航到航点

```bash
ros2 action send_goal /usv/navigation/navigate usv_control/action/NavigateToWaypoint "{waypoints: [{x: 39.9042, y: 116.4074}, {x: 39.9052, y: 116.4084}]}"
```

## 话题接口

### 发布话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/usv/control/cmd_vel` | geometry_msgs/Twist | 控制指令 |
| `/usv/control/mode/status` | std_msgs/String | 控制模式状态 |
| `/usv/sensor/imu` | sensor_msgs/Imu | IMU数据 |
| `/usv/sensor/gps` | sensor_msgs/NavSatFix | GPS数据 |
| `/usv/sensor/radar` | std_msgs/String | 雷达数据 |
| `/usv/navigation/status` | std_msgs/String | 导航状态 |
| `/usv/system/status` | std_msgs/String | 系统状态 |

### 订阅话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/cmd_vel` | geometry_msgs/Twist | 手动控制指令 |
| `/usv/control/mode` | std_msgs/Int32 | 模式切换指令 |

## 通信协议

### 串口通信协议 (Eastar Protocol)

**数据包格式**: `[0xAA, 0xBB, command, data_length, data..., checksum]`

**命令定义**:
- `0x01`: 设置电机速度
- `0x02`: 设置转向角度
- `0x03`: 设置控制模式
- `0x04`: 请求传感器数据
- `0x05`: 传感器数据响应
- `0x06`: 设置PID参数
- `0x07`: 系统状态

### 雷达数据格式

IWR1443雷达输出格式:
```
"Target X: Distance: X.Xm, Angle: X.Xdeg, Speed: X.Xm/s"
```

## 性能优化

### 实时性优化
- 使用实时内核
- 优化进程优先级
- 减少不必要的日志输出

### 通信优化
- 合理设置话题队列大小
- 使用合适的消息频率
- 考虑使用DDS优化配置

### 电源管理
- 监控系统功耗
- 实现智能电源管理
- 设置低电量保护机制

## 故障排除

### 常见问题

1. **串口连接失败**
   - 检查串口权限: `sudo chmod 666 /dev/ttyUSB0`
   - 确认串口设备号正确

2. **GPS无定位**
   - 检查GPS天线连接
   - 确保在开阔环境下测试
   - 等待GPS模块冷启动完成

3. **雷达无数据**
   - 检查雷达电源和串口连接
   - 确认雷达波特率设置正确

4. **ROS 2节点无法通信**
   - 检查ROS_DOMAIN_ID设置
   - 确保网络配置正确

### 调试命令

```bash
# 查看节点状态
ros2 node list
ros2 node info /node_name

# 查看话题
ros2 topic list
ros2 topic echo /topic_name

# 查看服务
ros2 service list
ros2 service type /service_name

# 查看动作
ros2 action list
ros2 action info /action_name

# 查看参数
ros2 param list
ros2 param get /node_name param_name
```

## 项目历史

本项目基于原始的ROS-based-USV-project (https://github.com/matreshka15/ROS-based-USV-project) 进行开发。原始项目使用ROS 1 (Kinetic/Melodic) 框架，在Nvidia TX1/TX2、树莓派、PC等Ubuntu平台上运行。本项目将其成功迁移到ROS 2 Humble，并适配Ubuntu 22.04系统，同时保持了与原始硬件平台的兼容性。

原始项目特点：
- 使用STM32F103ZE作为下位机
- 基于自定义Eastar协议的串口通信
- 上位机负责自动导航模式的路径规划
- 支持在多种硬件平台上运行 (Nvidia TX系列、树莓派、PC)


## 联系信息

如有问题或建议，请联系:
- 邮箱: matreshka999@icloud.com
