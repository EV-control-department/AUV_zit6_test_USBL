#!/bin/bash

# 确保脚本在出错时停止
set -e

# 获取当前目录
PROJECT_ROOT=$(pwd)

echo "========================================================="
echo "   AUV Zit6 - Integrated Build Script"
echo "========================================================="

# 0. 准备工作
echo "[0/3] Preparing environment..."
# 仅清理库文件，不清理整个 build 目录（除非 build 目录损坏）
sudo rm -rf micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros

# 同步接口定义 (解决 Docker 内部软链接失效问题)
EXTRA_PKG_DIR="micro_ros_stm32cubemx_utils/microros_static_library_ide/library_generation/extra_packages"
mkdir -p "$EXTRA_PKG_DIR"
if [ -L "$EXTRA_PKG_DIR/zit6_interfaces" ]; then rm "$EXTRA_PKG_DIR/zit6_interfaces"; fi
cp -rL zit6_interfaces "$EXTRA_PKG_DIR/"

# 1. 生成 micro-ROS 静态库 (使用 Docker)
echo "[1/3] Generating micro-ROS library via Docker..."
# 恢复使用官方镜像以防本地镜像有问题
docker run --rm --network host --name microros_builder \
  -v "${PROJECT_ROOT}:/project" \
  --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide \
  --env http_proxy=http://127.0.0.1:7897 \
  --env https_proxy=http://127.0.0.1:7897 \
  --env all_proxy=socks5://127.0.0.1:7897 \
  microros/micro_ros_static_library_builder:humble

echo ">>> micro-ROS library generation finished."

# 2. 编译 STM32 固件
echo "[2/3] Building STM32 firmware..."
mkdir -p build
# 使用项目自带的 ARM 工具链文件重新生成配置
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build

# 3. 编译上位机接口
echo "[3/3] Building ROS 2 host interfaces..."
if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; fi
colcon build --packages-select zit6_interfaces --symlink-install

echo "========================================================="
echo "   Build Success!"
echo "========================================================="
