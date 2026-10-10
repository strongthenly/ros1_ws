#!/bin/bash
# ============================================================
#  阶段⑤ 一键启动: FUEL 自主探索（含控制桥）
#
#  前置（按顺序）:
#    1. Windows: AirSim Blocks 已 Play
#    2. twin 链路已跑:  bash ~/twin_livo_fast.sh   （PX4+桥+FAST-LIVO2）
#    3. MAVROS 已连 PX4: 见 FUEL_阶段5_操作说明.md
#    4. 无人机已起飞（可选，FUEL 轨迹也能带它上去）
#
#  用法: bash ~/fuel_ws/src/fuel_bridge/scripts/start_fuel_explore.sh [ctrl_mode]
#        ctrl_mode: position(默认,稳) | raw(带速度/加速度前馈)
# ============================================================
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/noetic/setup.bash
source ~/twin_ws/devel/setup.bash
source ~/fuel_ws/devel/setup.bash

MODE=${1:-position}

echo "================ 前置检查 ================"
if timeout 3 rosnode list >/dev/null 2>&1; then
  echo "  [✓] ROS master (11311)"
else
  echo "  [✗] 无 master —— 请先跑: bash ~/twin_livo_fast.sh"; exit 1
fi

if timeout 6 rostopic hz /cloud_registered 2>&1 | grep -q "average"; then
  echo "  [✓] FAST-LIVO2 在出数据 (/cloud_registered)"
else
  echo "  [⚠] /cloud_registered 无数据 —— FAST-LIVO2 可能还在初始化(30-60s)，或没启动"
fi

if timeout 5 rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q "connected: True"; then
  echo "  [✓] MAVROS 已连 PX4"
else
  echo "  [⚠] MAVROS 未连 PX4 —— 控制桥将无法切 OFFBOARD（见操作说明）"
fi

echo ""
echo "================ 启动 FUEL (ctrl_mode=$MODE) ================"
echo "  提示: 另开终端发目标即可触发探索（见操作说明）"
echo ""
exec roslaunch fuel_bridge fuel_real.launch enable_ctrl:=true ctrl_mode:=$MODE
