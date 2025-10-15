#!/bin/bash
# setup_vcan.sh - 创建并启动虚拟 CAN 接口 can0

# 检查是否以 root 身份运行
if [ "$EUID" -ne 0 ]; then
  echo "请使用 sudo 运行该脚本"
  exit 1
fi

# 加载 vcan 模块
modprobe vcan

# 检查模块是否加载成功
if ! lsmod | grep -q vcan; then
  echo "❌ vcan 模块加载失败，请确认内核支持"
  exit 1
fi

# 如果 can0 已存在，先删除
if ip link show can0 >/dev/null 2>&1; then
  echo "can0 已存在，正在删除..."
  ip link delete can0
fi

# 创建并启动虚拟 can0
echo "正在创建虚拟 CAN 接口 can0 ..."
ip link add dev can0 type vcan

echo "启动 can0 接口 ..."
ip link set up can0

# 显示结果
echo "✅ can0 创建成功！当前状态："
ip -details link show can0

