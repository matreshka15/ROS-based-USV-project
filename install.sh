#!/bin/bash
# USV控制系统安装脚本
# 适用于Ubuntu 22.04和ROS 2 Humble

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=============================================${NC}"
echo -e "${GREEN}     USV控制系统安装脚本                      ${NC}"
echo -e "${GREEN}     适用于Ubuntu 22.04 + ROS 2 Humble       ${NC}"
echo -e "${GREEN}=============================================${NC}"

# 检查操作系统版本
check_os_version() {
    if [ ! -f /etc/lsb-release ]; then
        echo -e "${RED}错误: 不支持的操作系统${NC}"
        exit 1
    fi
    
    source /etc/lsb-release
    if [ "$DISTRIB_ID" != "Ubuntu" ] || [ "$DISTRIB_RELEASE" != "22.04" ]; then
        echo -e "${RED}错误: 仅支持Ubuntu 22.04 LTS${NC}"
        echo -e "${YELLOW}当前系统: $DISTRIB_ID $DISTRIB_RELEASE${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}✓ 操作系统检查通过${NC}"
}

# 检查ROS 2安装
check_ros2_installation() {
    if ! command -v ros2 &> /dev/null; then
        echo -e "${YELLOW}警告: 未检测到ROS 2安装${NC}"
        read -p "是否自动安装ROS 2 Humble? (y/n): " install_ros
        if [ "$install_ros" = "y" ] || [ "$install_ros" = "Y" ]; then
            install_ros2
        else
            echo -e "${RED}错误: 必须安装ROS 2才能继续${NC}"
            exit 1
        fi
    else
        echo -e "${GREEN}✓ ROS 2已安装${NC}"
    fi
}

# 安装ROS 2 Humble
install_ros2() {
    echo -e "${YELLOW}正在安装ROS 2 Humble...${NC}"
    
    # 设置locale
    sudo apt update && sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    
    # 添加ROS 2源
    sudo apt install -y software-properties-common
    sudo add-apt-repository -y universe
    
    # 添加ROS 2 GPG key
    sudo apt update && sudo apt install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    # 添加源列表
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # 安装ROS 2
    sudo apt update
    sudo apt install -y ros-humble-desktop
    
    # 设置环境变量
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source /opt/ros/humble/setup.bash
    
    echo -e "${GREEN}✓ ROS 2 Humble安装完成${NC}"
}

# 安装依赖包
install_dependencies() {
    echo -e "${YELLOW}正在安装依赖包...${NC}"
    
    # ROS 2依赖
    sudo apt update
    sudo apt install -y \
        ros-humble-geometry-msgs \
        ros-humble-sensor-msgs \
        ros-humble-action-msgs \
        ros-humble-rcl-interfaces \
        ros-humble-tf2-ros \
        ros-humble-ament-cmake \
        ros-humble-rosidl-default-generators
    
    # Python依赖
    sudo apt install -y python3-serial python3-pip
    pip3 install --user pynmea2
    
    # 系统工具
    sudo apt install -y git build-essential cmake
    
    echo -e "${GREEN}✓ 依赖包安装完成${NC}"
}

# 创建工作空间
create_workspace() {
    echo -e "${YELLOW}正在创建工作空间...${NC}"
    
    WORKSPACE_DIR=~/usv_ros2_ws
    if [ -d "$WORKSPACE_DIR" ]; then
        read -p "工作空间已存在，是否覆盖? (y/n): " overwrite
        if [ "$overwrite" = "y" ] || [ "$overwrite" = "Y" ]; then
            rm -rf "$WORKSPACE_DIR"
        else
            echo -e "${YELLOW}使用现有工作空间${NC}"
            return
        fi
    fi
    
    mkdir -p "$WORKSPACE_DIR/src"
    echo -e "${GREEN}✓ 工作空间创建完成: $WORKSPACE_DIR${NC}"
}

# 克隆代码
clone_repository() {
    echo -e "${YELLOW}正在克隆代码...${NC}"
    
    WORKSPACE_DIR=~/usv_ros2_ws
    cd "$WORKSPACE_DIR/src"
    
    # 这里应该替换为实际的仓库URL
    if [ -d "usv_control" ]; then
        echo -e "${YELLOW}代码已存在，跳过克隆${NC}"
        return
    fi
    
    # 注意：这里使用当前目录的代码，实际使用时应该从Git仓库克隆
    echo -e "${YELLOW}提示: 请将代码克隆到 $WORKSPACE_DIR/src/usv_control${NC}"
    echo -e "${YELLOW}例如: git clone <repository-url> usv_control${NC}"
    
    echo -e "${GREEN}✓ 代码准备完成${NC}"
}

# 构建项目
build_project() {
    echo -e "${YELLOW}正在构建项目...${NC}"
    
    WORKSPACE_DIR=~/usv_ros2_ws
    cd "$WORKSPACE_DIR"
    
    # 安装ROS依赖
    rosdep install --from-paths src --ignore-src -r -y
    
    # 构建
    colcon build --packages-select usv_control
    
    # 设置环境变量
    echo "source $WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc
    source "$WORKSPACE_DIR/install/setup.bash"
    
    echo -e "${GREEN}✓ 项目构建完成${NC}"
}

# 配置串口权限
configure_serial() {
    echo -e "${YELLOW}正在配置串口权限...${NC}"
    
    # 添加用户到dialout组
    sudo usermod -aG dialout $USER
    
    # 创建udev规则
    UDEV_RULE="/etc/udev/rules.d/99-usv-serial.rules"
    if [ ! -f "$UDEV_RULE" ]; then
        sudo tee "$UDEV_RULE" > /dev/null <<EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttyUSV"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", SYMLINK+="ttyRadar"
KERNEL=="ttyUSB*", MODE="0666"
EOF
    fi
    
    # 重新加载udev规则
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
    echo -e "${GREEN}✓ 串口权限配置完成${NC}"
    echo -e "${YELLOW}注意: 需要重新登录才能生效串口权限${NC}"
}

# 系统检查
system_check() {
    echo -e "${YELLOW}正在进行系统检查...${NC}"
    
    # 检查Python版本
    PYTHON_VERSION=$(python3 --version | awk '{print $2}')
    echo -e "${GREEN}✓ Python版本: $PYTHON_VERSION${NC}"
    
    # 检查ROS 2版本
    ROS2_VERSION=$(ros2 --version | head -n1 | awk '{print $4}')
    echo -e "${GREEN}✓ ROS 2版本: $ROS2_VERSION${NC}"
    
    # 检查串口设备
    if ls /dev/ttyUSB* 1> /dev/null 2>&1; then
        echo -e "${GREEN}✓ 检测到串口设备: $(ls /dev/ttyUSB*)${NC}"
    else
        echo -e "${YELLOW}警告: 未检测到串口设备${NC}"
    fi
    
    echo -e "${GREEN}✓ 系统检查完成${NC}"
}

# 创建快捷启动脚本
create_launch_scripts() {
    echo -e "${YELLOW}正在创建快捷启动脚本...${NC}"
    
    # 创建启动脚本
    LAUNCH_SCRIPT=~/start_usv_control.sh
    cat > "$LAUNCH_SCRIPT" <<EOF
#!/bin/bash
# USV控制系统启动脚本

# 设置环境
source /opt/ros/humble/setup.bash
source ~/usv_ros2_ws/install/setup.bash

# 启动USV控制系统
ros2 launch usv_control usv_control.launch.py
EOF
    
    chmod +x "$LAUNCH_SCRIPT"
    
    # 创建测试启动脚本
    TEST_SCRIPT=~/test_usv_control.sh
    cat > "$TEST_SCRIPT" <<EOF
#!/bin/bash
# USV控制系统测试脚本

# 设置环境
source /opt/ros/humble/setup.bash
source ~/usv_ros2_ws/install/setup.bash

# 启动测试模式
ros2 launch usv_control test_launch.launch.py
EOF
    
    chmod +x "$TEST_SCRIPT"
    
    echo -e "${GREEN}✓ 快捷启动脚本创建完成${NC}"
    echo -e "${YELLOW}启动系统: ./start_usv_control.sh${NC}"
    echo -e "${YELLOW}测试系统: ./test_usv_control.sh${NC}"
}

# 安装完成
installation_complete() {
    echo -e "\n${GREEN}=============================================${NC}"
    echo -e "${GREEN}        安装完成!                            ${NC}"
    echo -e "${GREEN}=============================================${NC}"
    echo -e ""
    echo -e "${YELLOW}重要提示:${NC}"
    echo -e "1. 请重新登录以应用串口权限设置"
    echo -e "2. 编辑配置文件: ~/usv_ros2_ws/src/usv_control/config/usv_config.yaml"
    echo -e "3. 根据实际硬件修改串口和传感器参数"
    echo -e "4. 启动系统: ./start_usv_control.sh"
    echo -e ""
    echo -e "${YELLOW}使用帮助:${NC}"
    echo -e "查看README.md获取详细使用说明"
    echo -e "运行rqt_graph查看节点关系"
    echo -e "运行ros2 topic list查看可用话题"
    echo -e ""
    echo -e "${GREEN}祝您使用愉快!${NC}"
}

# 主安装流程
main() {
    check_os_version
    check_ros2_installation
    install_dependencies
    create_workspace
    clone_repository
    build_project
    configure_serial
    system_check
    create_launch_scripts
    installation_complete
}

# 开始安装
main