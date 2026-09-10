#!/bin/bash
# ============================================================
#  一键启动: PX4 + 仿真桥 + FAST-LIVO2 + RViz
#  用法:  bash ~/twin_livo_rviz.sh
#  前提:  Windows 侧 Blocks 已在 Play（AirSim 运行）
# ============================================================
source /opt/ros/noetic/setup.bash
source ~/twin_ws/devel/setup.bash

ROOT=~/twin_ws/src/FAST-LIVO2
CFG=$ROOT/config/TwinSim.yaml
CAM=$ROOT/config/TwinSim_camera.yaml
RVIZ_CFG=$ROOT/rviz_cfg/fast_livo2.rviz
PX4_DIR=~/PX4-AirSim

echo "[1/7] 清理旧节点 ..."
pkill -f fastlivo_mapping 2>/dev/null
pkill -f "rosbag play" 2>/dev/null
pkill -x rviz 2>/dev/null
pkill -f roslaunch 2>/dev/null
pkill -f bridge.py 2>/dev/null
pkill -f rosmaster 2>/dev/null
pkill -f roscore 2>/dev/null
tmux kill-session -t px4 2>/dev/null
sleep 2

echo "[2/7] 检查/启动 PX4 (tmux: px4) ..."
if pgrep -f "px4_sitl" >/dev/null 2>&1 || pgrep -x px4 >/dev/null 2>&1; then
  echo "      ✅ PX4 已在跑"
else
  if [ -d "$PX4_DIR" ]; then
    tmux new-session -d -s px4 "cd $PX4_DIR && make px4_sitl_default none_iris 2>&1 | tee /tmp/px4_run.log"
    echo "      -> 已启动 PX4 (tmux attach -t px4 可查看)"
    sleep 12
  else
    echo "      ⚠️  没找到 $PX4_DIR，跳过"
  fi
fi

echo "[3/7] 启动 roscore ..."
setsid roscore > /tmp/roscore.log 2>&1 < /dev/null &
sleep 5

echo "[4/7] 检查/启动仿真桥 (/twin) ..."
if timeout 6 rostopic echo -n 1 /twin/lidar/points_preview/header >/dev/null 2>&1; then
  echo "      ✅ 桥已在跑"
else
  setsid roslaunch twin_bridge bridge.launch host:=127.0.0.1 imu_rate:=220 rgb_rate:=30 depth_rate:=5 \
      > /tmp/bridge.log 2>&1 < /dev/null &
  echo "      -> 已启动桥，等待数据 ..."
  sleep 12
  if timeout 6 rostopic echo -n 1 /twin/lidar/points_preview/header >/dev/null 2>&1; then
    echo "      ✅ /twin 出数据了"
  else
    echo "      ⚠️  桥起了但没数据 —— 检查 Blocks 是否 Play + PX4 是否连上 (Simulator connected)，看 /tmp/bridge.log"
  fi
fi

echo "[5/7] 加载配置 + 启动 FAST-LIVO2 ..."
rosparam load $CFG
rosparam load $CAM /laserMapping
setsid rosrun fast_livo fastlivo_mapping __name:=laserMapping > /tmp/fastlivo.log 2>&1 < /dev/null &
sleep 6

echo "[6/7] 启动 RViz ..."
setsid rviz -d $RVIZ_CFG > /tmp/rviz.log 2>&1 < /dev/null &
sleep 6

echo "[7/7] 状态 ..."
echo "----------------------------------------"
rosnode list 2>/dev/null
echo "----------------------------------------"
timeout 6 rostopic hz /aft_mapped_to_init 2>&1 | head -3
echo "----------------------------------------"
echo "✅ 完成"
echo "  看坐标: rostopic echo /aft_mapped_to_init/pose/pose/position"
echo "  日志: /tmp/px4_run.log /tmp/bridge.log /tmp/fastlivo.log /tmp/rviz.log"
