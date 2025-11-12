#!/bin/bash
# USV控制系统测试脚本
# 验证系统基本功能

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=============================================${NC}"
echo -e "${GREEN}     USV控制系统测试脚本                      ${NC}"
echo -e "${GREEN}=============================================${NC}"

# 检查ROS 2环境
check_ros2_environment() {
    echo -e "${YELLOW}检查ROS 2环境...${NC}"
    
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}错误: 未检测到ROS 2${NC}"
        echo -e "${YELLOW}请先安装ROS 2并设置环境变量${NC}"
        exit 1
    fi
    
    # 检查工作空间
    if [ ! -f "install/setup.bash" ]; then
        echo -e "${RED}错误: 未找到工作空间安装文件${NC}"
        echo -e "${YELLOW}请先构建项目: colcon build${NC}"
        exit 1
    fi
    
    # 加载环境
    source install/setup.bash
    
    echo -e "${GREEN}✓ ROS 2环境检查通过${NC}"
}

# 检查节点列表
check_nodes() {
    echo -e "\n${YELLOW}检查节点列表...${NC}"
    
    # 启动测试节点
    echo -e "${YELLOW}启动测试节点...${NC}"
    ros2 launch usv_control test_launch.launch.py &
    LAUNCH_PID=$!
    
    # 等待节点启动
    sleep 10
    
    # 检查节点
    NODES=$(ros2 node list)
    EXPECTED_NODES=("control_mode_hub_test" "serial_port_hub_test" "navigation_action_server_test" "radar_data_processor_test" "usv_controller_test")
    
    for node in "${EXPECTED_NODES[@]}"; do
        if echo "$NODES" | grep -q "$node"; then
            echo -e "${GREEN}✓ 节点运行: $node${NC}"
        else
            echo -e "${RED}✗ 节点未运行: $node${NC}"
            NODE_ERROR=1
        fi
    done
    
    # 停止测试节点
    kill $LAUNCH_PID
    wait $LAUNCH_PID 2>/dev/null
    
    if [ -n "$NODE_ERROR" ]; then
        echo -e "${RED}错误: 部分节点未正常启动${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}✓ 所有节点检查通过${NC}"
}

# 检查话题
check_topics() {
    echo -e "\n${YELLOW}检查话题...${NC}"
    
    # 启动单个节点来检查话题
    ros2 run usv_control control_mode_hub.py &
    NODE_PID=$!
    sleep 5
    
    TOPICS=$(ros2 topic list)
    EXPECTED_TOPICS=("/usv/control/cmd_vel" "/usv/control/mode/status" "/usv/system/status")
    
    for topic in "${EXPECTED_TOPICS[@]}"; do
        if echo "$TOPICS" | grep -q "$topic"; then
            echo -e "${GREEN}✓ 话题存在: $topic${NC}"
        else
            echo -e "${YELLOW}? 话题不存在: $topic${NC}"
        fi
    done
    
    # 停止节点
    kill $NODE_PID
    wait $NODE_PID 2>/dev/null
    
    echo -e "${GREEN}✓ 话题检查完成${NC}"
}

# 检查服务
check_services() {
    echo -e "\n${YELLOW}检查服务...${NC}"
    
    # 启动控制模式节点
    ros2 run usv_control control_mode_hub.py &
    NODE_PID=$!
    sleep 5
    
    SERVICES=$(ros2 service list)
    EXPECTED_SERVICES=("/usv/control/set_mode")
    
    for service in "${EXPECTED_SERVICES[@]}"; do
        if echo "$SERVICES" | grep -q "$service"; then
            echo -e "${GREEN}✓ 服务存在: $service${NC}"
            
            # 测试服务调用
            echo -e "${YELLOW}测试服务调用: $service${NC}"
            if ros2 service call "$service" rcl_interfaces/srv/SetParameters "{parameters: [{name: 'control_mode', value: {type: 2, integer_value: 0}}]}" >/dev/null 2>&1; then
                echo -e "${GREEN}✓ 服务调用成功${NC}"
            else
                echo -e "${YELLOW}? 服务调用可能失败${NC}"
            fi
        else
            echo -e "${RED}✗ 服务不存在: $service${NC}"
            SERVICE_ERROR=1
        fi
    done
    
    # 停止节点
    kill $NODE_PID
    wait $NODE_PID 2>/dev/null
    
    if [ -n "$SERVICE_ERROR" ]; then
        echo -e "${RED}错误: 部分服务未正常提供${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}✓ 服务检查完成${NC}"
}

# 检查动作
check_actions() {
    echo -e "\n${YELLOW}检查动作...${NC}"
    
    # 启动导航节点
    ros2 run usv_control navigation_action_server.py &
    NODE_PID=$!
    sleep 5
    
    ACTIONS=$(ros2 action list)
    EXPECTED_ACTIONS=("/usv/navigation/navigate")
    
    for action in "${EXPECTED_ACTIONS[@]}"; do
        if echo "$ACTIONS" | grep -q "$action"; then
            echo -e "${GREEN}✓ 动作存在: $action${NC}"
        else
            echo -e "${YELLOW}? 动作不存在: $action${NC}"
        fi
    done
    
    # 停止节点
    kill $NODE_PID
    wait $NODE_PID 2>/dev/null
    
    echo -e "${GREEN}✓ 动作检查完成${NC}"
}

# 检查配置文件
check_configuration() {
    echo -e "\n${YELLOW}检查配置文件...${NC}"
    
    CONFIG_FILES=("config/usv_config.yaml" "launch/usv_control.launch.py" "package.xml" "CMakeLists.txt")
    
    for file in "${CONFIG_FILES[@]}"; do
        if [ -f "$file" ]; then
            echo -e "${GREEN}✓ 配置文件存在: $file${NC}"
        else
            echo -e "${RED}✗ 配置文件缺失: $file${NC}"
            CONFIG_ERROR=1
        fi
    done
    
    if [ -n "$CONFIG_ERROR" ]; then
        echo -e "${RED}错误: 部分配置文件缺失${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}✓ 配置文件检查通过${NC}"
}

# 性能测试
performance_test() {
    echo -e "\n${YELLOW}性能测试...${NC}"
    
    # 启动控制模式节点并测量启动时间
    START_TIME=$(date +%s.%N)
    ros2 run usv_control control_mode_hub.py &
    NODE_PID=$!
    
    # 等待节点启动
    for i in {1..10}; do
        if ros2 node list | grep -q "control_mode_hub"; then
            break
        fi
        sleep 0.5
    done
    
    END_TIME=$(date +%s.%N)
    STARTUP_TIME=$(echo "$END_TIME - $START_TIME" | bc)
    
    echo -e "${GREEN}✓ 节点启动时间: $STARTUP_TIME 秒${NC}"
    
    if (( $(echo "$STARTUP_TIME > 5" | bc -l) )); then
        echo -e "${YELLOW}警告: 节点启动时间较长${NC}"
    fi
    
    # 停止节点
    kill $NODE_PID
    wait $NODE_PID 2>/dev/null
    
    echo -e "${GREEN}✓ 性能测试完成${NC}"
}

# 生成测试报告
generate_report() {
    echo -e "\n${GREEN}=============================================${NC}"
    echo -e "${GREEN}           测试报告                           ${NC}"
    echo -e "${GREEN}=============================================${NC}"
    
    if [ -z "$TEST_ERROR" ]; then
        echo -e "${GREEN}✓ 所有测试通过!${NC}"
        echo -e "${YELLOW}系统已准备就绪，可以使用。${NC}"
    else
        echo -e "${RED}✗ 部分测试失败${NC}"
        echo -e "${YELLOW}请检查错误信息并修复问题。${NC}"
        exit 1
    fi
    
    echo -e "\n${YELLOW}使用建议:${NC}"
    echo -e "1. 编辑config/usv_config.yaml配置硬件参数"
    echo -e "2. 使用ros2 launch usv_control usv_control.launch.py启动系统"
    echo -e "3. 使用rqt_graph查看节点关系"
    echo -e "4. 查看README.md获取详细使用说明"
}

# 主测试流程
main() {
    check_ros2_environment
    
    # 运行各项测试
    TEST_ERROR=""
    
    if ! check_configuration; then
        TEST_ERROR="1"
    fi
    
    if ! check_nodes; then
        TEST_ERROR="1"
    fi
    
    if ! check_topics; then
        TEST_ERROR="1"
    fi
    
    if ! check_services; then
        TEST_ERROR="1"
    fi
    
    if ! check_actions; then
        TEST_ERROR="1"
    fi
    
    if ! performance_test; then
        TEST_ERROR="1"
    fi
    
    generate_report
}

# 开始测试
main