#!/bin/bash
# USV控制系统项目打包脚本
# 创建可分发的项目包

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=============================================${NC}"
echo -e "${GREEN}     USV控制系统项目打包脚本                  ${NC}"
echo -e "${GREEN}=============================================${NC}"

# 项目信息
PROJECT_NAME="usv_control"
VERSION="1.0.0"
DATE=$(date +"%Y%m%d_%H%M%S")
PACKAGE_NAME="${PROJECT_NAME}_v${VERSION}_${DATE}.tar.gz"

# 检查项目目录
check_project_structure() {
    echo -e "${YELLOW}检查项目结构...${NC}"
    
    REQUIRED_FILES=(
        "package.xml"
        "CMakeLists.txt"
        "README.md"
        "scripts/control_mode_hub.py"
        "scripts/serial_port_hub.py"
        "scripts/navigation_action_server.py"
        "scripts/radar_data_processor.py"
        "scripts/usv_controller.py"
        "config/usv_config.yaml"
        "launch/usv_control.launch.py"
    )
    
    for file in "${REQUIRED_FILES[@]}"; do
        if [ ! -f "$file" ]; then
            echo -e "${RED}错误: 必需文件缺失: $file${NC}"
            exit 1
        fi
    done
    
    echo -e "${GREEN}✓ 项目结构检查通过${NC}"
}

# 运行代码检查
run_code_checks() {
    echo -e "${YELLOW}运行代码检查...${NC}"
    
    # 检查Python语法
    echo -e "${YELLOW}检查Python语法...${NC}"
    for script in scripts/*.py; do
        if ! python3 -m py_compile "$script"; then
            echo -e "${RED}错误: Python语法错误: $script${NC}"
            exit 1
        fi
    done
    
    # 检查Shell脚本语法
    echo -e "${YELLOW}检查Shell脚本语法...${NC}"
    for script in *.sh install_service.sh test_system.sh; do
        if [ -f "$script" ]; then
            if ! bash -n "$script"; then
                echo -e "${RED}错误: Shell脚本语法错误: $script${NC}"
                exit 1
            fi
        fi
    done
    
    echo -e "${GREEN}✓ 代码检查通过${NC}"
}

# 创建临时打包目录
create_temp_directory() {
    echo -e "${YELLOW}创建临时打包目录...${NC}"
    
    TEMP_DIR=$(mktemp -d -t "${PROJECT_NAME}_package_XXXXXX")
    PACKAGE_DIR="${TEMP_DIR}/${PROJECT_NAME}"
    
    mkdir -p "$PACKAGE_DIR"
    
    echo -e "${GREEN}✓ 临时目录创建完成: $PACKAGE_DIR${NC}"
}

# 复制文件到打包目录
copy_files() {
    echo -e "${YELLOW}复制文件...${NC}"
    
    # 复制主要代码文件
    cp -r \
        package.xml \
        CMakeLists.txt \
        README.md \
        LICENSE \
        scripts/ \
        config/ \
        launch/ \
        action/ \
        systemd/ \
        "$PACKAGE_DIR/"
    
    # 复制安装和测试脚本
    cp -r \
        install.sh \
        install_service.sh \
        test_system.sh \
        package_project.sh \
        "$PACKAGE_DIR/"
    
    # 设置执行权限
    chmod +x "$PACKAGE_DIR"/*.sh
    chmod +x "$PACKAGE_DIR/scripts"/*.py
    
    echo -e "${GREEN}✓ 文件复制完成${NC}"
}

# 创建版本信息文件
create_version_info() {
    echo -e "${YELLOW}创建版本信息...${NC}"
    
    VERSION_FILE="${PACKAGE_DIR}/VERSION.txt"
    cat > "$VERSION_FILE" <<EOF
USV Control System
版本: $VERSION
打包日期: $(date)
ROS版本: Humble Hawksbill
操作系统: Ubuntu 22.04 LTS

项目描述:
基于ROS 2 Humble的无人船控制系统，实现自主导航、手动控制、障碍物检测等功能。

主要特性:
- 多模式控制（手动/自动/记录）
- 串口通信与STM32下位机
- 基于GPS的自主导航
- IWR1443毫米波雷达障碍物检测
- 系统状态监控与故障处理

安装说明:
1. 运行 install.sh 进行自动安装
2. 编辑 config/usv_config.yaml 配置硬件参数
3. 使用 ros2 launch usv_control usv_control.launch.py 启动系统
EOF
    
    echo -e "${GREEN}✓ 版本信息创建完成${NC}"
}

# 创建使用示例
create_examples() {
    echo -e "${YELLOW}创建使用示例...${NC}"
    
    EXAMPLES_DIR="${PACKAGE_DIR}/examples"
    mkdir -p "$EXAMPLES_DIR"
    
    # 创建简单的控制示例
    cat > "${EXAMPLES_DIR}/simple_control.py" <<EOF
#!/usr/bin/env python3
"""
USV控制示例
发送简单的控制指令
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class SimpleUSVControl(Node):
    def __init__(self):
        super().__init__('simple_usv_control')
        
        # 创建发布器
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.mode_pub = self.create_publisher(
            Int32,
            '/usv/control/mode',
            10
        )
        
        self.get_logger().info("简单USV控制器已启动")
        
        # 发送控制指令
        self.send_control_commands()
    
    def send_control_commands(self):
        """发送控制指令"""
        # 设置为手动模式
        mode_msg = Int32()
        mode_msg.data = 0  # 0: 手动模式
        self.mode_pub.publish(mode_msg)
        self.get_logger().info("已设置为手动模式")
        
        # 前进指令
        twist_msg = Twist()
        twist_msg.linear.x = 0.5  # 前进速度 0.5 m/s
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info("发送前进指令: 0.5 m/s")
        
        # 运行5秒后停止
        import time
        time.sleep(5)
        
        # 停止指令
        twist_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info("发送停止指令")

def main(args=None):
    rclpy.init(args=args)
    controller = SimpleUSVControl()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
    
    # 创建导航示例
    cat > "${EXAMPLES_DIR}/waypoint_navigation.py" <<EOF
#!/usr/bin/env python3
"""
USV航点导航示例
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from usv_control.action import NavigateToWaypoint
from geometry_msgs.msg import Point
from std_msgs.msg import Int32

class WaypointNavigation(Node):
    def __init__(self):
        super().__init__('waypoint_navigation')
        
        # 创建动作客户端
        self.action_client = ActionClient(
            self,
            NavigateToWaypoint,
            '/usv/navigation/navigate'
        )
        
        # 创建模式发布器
        self.mode_pub = self.create_publisher(
            Int32,
            '/usv/control/mode',
            10
        )
        
        self.get_logger().info("航点导航客户端已启动")
        
        # 等待动作服务器
        if not self.action_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("导航动作服务器不可用")
            return
        
        # 开始导航
        self.start_navigation()
    
    def start_navigation(self):
        """开始导航"""
        # 设置为自动模式
        mode_msg = Int32()
        mode_msg.data = 1  # 1: 自动模式
        self.mode_pub.publish(mode_msg)
        self.get_logger().info("已设置为自动模式")
        
        # 创建导航目标
        goal_msg = NavigateToWaypoint.Goal()
        
        # 添加航点（示例坐标，需要根据实际情况修改）
        waypoint1 = Point()
        waypoint1.x = 39.9042  # 纬度
        waypoint1.y = 116.4074  # 经度
        goal_msg.waypoints.append(waypoint1)
        
        waypoint2 = Point()
        waypoint2.x = 39.9052  # 纬度  
        waypoint2.y = 116.4084  # 经度
        goal_msg.waypoints.append(waypoint2)
        
        self.get_logger().info(f"开始导航到 {len(goal_msg.waypoints)} 个航点")
        
        # 发送目标
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """目标响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("导航目标被拒绝")
            return
        
        self.get_logger().info("导航目标已接受")
        
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        """反馈回调"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"导航进度: 航点 {feedback.waypoint_index + 1}/{feedback.total_waypoints}, "
            f"距离: {feedback.distance_to_waypoint:.2f}m"
        )
    
    def get_result_callback(self, future):
        """结果回调"""
        result = future.result().result
        if result.success:
            self.get_logger().info("导航完成!")
        else:
            self.get_logger().error(f"导航失败: {result.message}")
        
        # 关闭节点
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigation()
    rclpy.spin(navigator)

if __name__ == '__main__':
    main()
EOF
    
    chmod +x "$EXAMPLES_DIR"/*.py
    
    echo -e "${GREEN}✓ 使用示例创建完成${NC}"
}

# 打包项目
package_project() {
    echo -e "${YELLOW}正在打包项目...${NC}"
    
    # 进入临时目录
    cd "$TEMP_DIR"
    
    # 创建压缩包
    tar -czf "$PACKAGE_NAME" "$PROJECT_NAME/"
    
    # 移动到当前目录
    mv "$PACKAGE_NAME" "$OLDPWD/"
    
    echo -e "${GREEN}✓ 项目打包完成: $PACKAGE_NAME${NC}"
}

# 清理临时文件
cleanup() {
    echo -e "${YELLOW}清理临时文件...${NC}"
    
    if [ -d "$TEMP_DIR" ]; then
        rm -rf "$TEMP_DIR"
    fi
    
    echo -e "${GREEN}✓ 清理完成${NC}"
}

# 显示打包信息
show_package_info() {
    echo -e "\n${GREEN}=============================================${NC}"
    echo -e "${GREEN}           打包完成信息                       ${NC}"
    echo -e "${GREEN}=============================================${NC}"
    echo -e "项目名称: $PROJECT_NAME"
    echo -e "版本: $VERSION"
    echo -e "打包日期: $(date)"
    echo -e "包文件: $PACKAGE_NAME"
    echo -e "文件大小: $(du -h "$PACKAGE_NAME" | awk '{print $1}')"
    echo -e ""
    echo -e "${YELLOW}使用说明:${NC}"
    echo -e "1. 解压: tar -xzf $PACKAGE_NAME"
    echo -e "2. 进入目录: cd $PROJECT_NAME"
    echo -e "3. 运行安装: ./install.sh"
    echo -e "4. 查看文档: cat README.md"
    echo -e ""
    echo -e "${GREEN}✓ 项目打包成功!${NC}"
}

# 主打包流程
main() {
    check_project_structure
    run_code_checks
    create_temp_directory
    copy_files
    create_version_info
    create_examples
    package_project
    cleanup
    show_package_info
}

# 开始打包
main