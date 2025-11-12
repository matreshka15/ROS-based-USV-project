#!/bin/bash
# USV控制系统服务安装脚本
# 设置systemd服务自动启动

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

SERVICE_NAME="usv-control"
SERVICE_FILE="systemd/usv-control.service"
DEST_SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME.service"

echo -e "${GREEN}=============================================${NC}"
echo -e "${GREEN}     USV控制系统服务安装脚本                  ${NC}"
echo -e "${GREEN}=============================================${NC}"

# 检查权限
check_permissions() {
    if [ "$(id -u)" -ne 0 ]; then
        echo -e "${RED}错误: 需要root权限${NC}"
        echo -e "${YELLOW}请使用sudo运行: sudo ./install_service.sh${NC}"
        exit 1
    fi
}

# 检查服务文件
check_service_file() {
    if [ ! -f "$SERVICE_FILE" ]; then
        echo -e "${RED}错误: 服务文件不存在: $SERVICE_FILE${NC}"
        exit 1
    fi
    echo -e "${GREEN}✓ 服务文件检查通过${NC}"
}

# 安装服务
install_service() {
    echo -e "${YELLOW}正在安装服务...${NC}"
    
    # 复制服务文件
    cp "$SERVICE_FILE" "$DEST_SERVICE_FILE"
    
    # 更新文件权限
    chmod 644 "$DEST_SERVICE_FILE"
    
    # 重新加载systemd配置
    systemctl daemon-reload
    
    echo -e "${GREEN}✓ 服务安装完成${NC}"
}

# 启用服务
enable_service() {
    echo -e "${YELLOW}正在启用服务...${NC}"
    
    # 启用服务
    systemctl enable "$SERVICE_NAME"
    
    echo -e "${GREEN}✓ 服务已启用，将在系统启动时自动运行${NC}"
}

# 启动服务
start_service() {
    echo -e "${YELLOW}正在启动服务...${NC}"
    
    # 启动服务
    systemctl start "$SERVICE_NAME"
    
    # 检查服务状态
    sleep 2
    if systemctl is-active --quiet "$SERVICE_NAME"; then
        echo -e "${GREEN}✓ 服务启动成功${NC}"
    else
        echo -e "${RED}错误: 服务启动失败${NC}"
        echo -e "${YELLOW}查看日志: journalctl -u $SERVICE_NAME -f${NC}"
        exit 1
    fi
}

# 显示服务状态
show_service_status() {
    echo -e "\n${YELLOW}服务状态:${NC}"
    systemctl status "$SERVICE_NAME" --no-pager
}

# 显示使用说明
show_usage() {
    echo -e "\n${GREEN}=============================================${NC}"
    echo -e "${GREEN}          服务管理命令                        ${NC}"
    echo -e "${GREEN}=============================================${NC}"
    echo -e "${YELLOW}启动服务:${NC} sudo systemctl start $SERVICE_NAME"
    echo -e "${YELLOW}停止服务:${NC} sudo systemctl stop $SERVICE_NAME"
    echo -e "${YELLOW}重启服务:${NC} sudo systemctl restart $SERVICE_NAME"
    echo -e "${YELLOW}查看状态:${NC} sudo systemctl status $SERVICE_NAME"
    echo -e "${YELLOW}查看日志:${NC} sudo journalctl -u $SERVICE_NAME -f"
    echo -e "${YELLOW}禁用服务:${NC} sudo systemctl disable $SERVICE_NAME"
    echo -e "${YELLOW}启用服务:${NC} sudo systemctl enable $SERVICE_NAME"
    echo -e "${GREEN}=============================================${NC}"
}

# 主安装流程
main() {
    check_permissions
    check_service_file
    install_service
    enable_service
    start_service
    show_service_status
    show_usage
    
    echo -e "\n${GREEN}服务安装完成!${NC}"
    echo -e "${YELLOW}系统将在启动时自动运行USV控制系统${NC}"
}

# 开始安装
main