#!/bin/bash

# 确保脚本在出错时停止
set -e

# 获取根目录绝对路径
PROJECT_ROOT=$(pwd)

echo "========================================================="
echo "   AUV Zit6 - Integrated Interface Build Script"
echo "========================================================="

# 1. 生成单片机静态库 (micro-ROS Static Library)
echo "[1/2] Generating micro-ROS static library for STM32..."
cd micro_ros_stm32cubemx_utils/microros_static_library_ide/library_generation

# 执行 micro-ROS 官方生成脚本
# 注意：这通常需要本地已配置好 arm-none-eabi-gcc 路径或在 Docker 环境中运行
./library_generation.sh

echo ">>> micro-ROS library generation finished."
cd $PROJECT_ROOT

# 2. 编译上位机接口 (ROS 2 Host Install)
echo "[2/2] Building ROS 2 interfaces for Host..."

# 检查是否已 source ROS 2 环境
if [ -z "$ROS_DISTRO" ]; then
    echo "Warning: ROS_DISTRO is not set. Trying to source ROS 2 Jazzy..."
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    else
        echo "Error: ROS 2 environment not found. Please source your ROS 2 setup.bash first."
        exit 1
    fi
fi

# 使用 colcon 编译
colcon build --packages-select zit6_interfaces --symlink-install

echo "========================================================="
echo "   Build Complete! "
echo "   - MCU Lib: micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros"
echo "   - Host Install: install/zit6_interfaces"
echo "========================================================="
