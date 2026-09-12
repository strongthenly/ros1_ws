#!/bin/bash
# ============================================================
#  一键启动: PX4 + 仿真桥 + FAST-LIVO2 + RViz   (v3)
#  用法:  bash ~/twin_livo_rviz.sh
#  前提:  Windows 侧 Blocks 已在 Play
#  v2: 不无条件杀 PX4；等 TCP4560 连接
#  v3: 彻底清理残留 roslaunch/roscore（修 run_id 冲突）；
#      roscore 起来后等 master 真正就绪
# ============================================================
source /opt/ros/noetic/setup.bash
source ~/twin_ws/devel/setup.bash

ROOT=~/twin_ws/src/FAST-LIVO2
PX4_DIR=~/PX4-AirSim

echo "[1/7] 彻底清理 ROS 侧（不动 PX4 进程）..."
pkill -f fastlivo_mapping       2>/dev/null
pkill -f "rosbag play"          2>/dev/null
pkill -x rviz                   2>/dev/null
pkill -f bridge.py              2>/dev/null
pkill -f roslaunch              2>/dev/null
pkill -f rosout                 2>/dev/null
pkill -f rosmaster              2>/dev/null
pkill -f roscore                2>/dev/null
# 等它们真正退出
for i in $(seq 1 10); do
  if ! pgrep -f "rosmaster|roslaunch" >/dev/null 2>&1; then break; fi
  sleep 1
done
echo "      残留检查: $(pgrep -f 'rosmaster|roslaunch' | wc -l) 个"
sleep 2

echo "[2/7] 检查/启动 PX4 ..."
if pgrep -f "bin/px4" >/dev/null 2>&1; then
  echo "      ✅ PX4 已在跑（不重启）"
else
  tmux kill-session -t px4 2>/dev/null
  tmux new-session -d -s px4 "cd $PX4_DIR && make px4_sitl_default none_iris 2>&1 | tee /tmp/px4_run.log"
  echo "      -> 已启动 PX4 (tmux: px4)"
fi

echo "[3/7] 启动干净的 roscore 并等它就绪 ..."
setsid roscore > /tmp/roscore.log 2>&1 < /dev/null &
READY=0
for i in $(seq 1 30); do
  if timeout 5 rosnode list >/dev/null 2>&1; then READY=1; echo "      ✅ master 就绪（第 ${i}s）"; break; fi
  sleep 1
done
[ "$READY" = "1" ] || { echo "      ⚠️ master 未就绪，看 /tmp/roscore.log"; tail -5 /tmp/roscore.log; }

echo "[4/7] 等待 PX4 <-> AirSim TCP4560 连接（最多 120s）..."
CONNECTED=0
for i in $(seq 1 40); do
  if ss -tn 2>/dev/null | grep -q ':4560'; then
    echo "      ✅ 4560 已连接（第 $((i*3))s）"; CONNECTED=1; break
  fi
  [ $((i % 5)) -eq 0 ] && echo "      ... 等待中 $((i*3))s（确认 Blocks 处于 Play）"
  sleep 3
done
[ "$CONNECTED" = "1" ] || echo "      ⚠️ 120s 未连上，检查 Blocks 是否 Play；日志: /tmp/px4_run.log"

echo "[5/7] 启动仿真桥 ..."
setsid roslaunch twin_bridge bridge.launch host:=127.0.0.1 imu_rate:=220 rgb_rate:=30 depth_rate:=5 \
    > /tmp/bridge.log 2>&1 < /dev/null &
BRIDGE_OK=0
for i in $(seq 1 20); do
  if timeout 4 rostopic echo -n 1 /twin/lidar/points_preview/header >/dev/null 2>&1; then
    echo "      ✅ /twin 有数据（第 $((i*3))s）"; BRIDGE_OK=1; break
  fi
  [ $((i % 5)) -eq 0 ] && echo "      ... 等桥出数据 $((i*3))s"
  sleep 3
done
if [ "$BRIDGE_OK" != "1" ]; then
  echo "      ⚠️ 桥没出数据，日志末尾:"
  tail -12 /tmp/bridge.log 2>/dev/null
fi

echo "[6/7] 加载配置 + FAST-LIVO2 + RViz ..."
rosparam load $ROOT/config/TwinSim.yaml
rosparam load $ROOT/config/TwinSim_camera.yaml /laserMapping
setsid rosrun fast_livo fastlivo_mapping __name:=laserMapping > /tmp/fastlivo.log 2>&1 < /dev/null &
sleep 6
setsid rviz -d $ROOT/rviz_cfg/fast_livo2.rviz > /tmp/rviz.log 2>&1 < /dev/null &
sleep 6

echo "[7/7] 状态 ..."
echo "----------------------------------------"
rosnode list 2>/dev/null
echo "----------------------------------------"
timeout 6 rostopic hz /aft_mapped_to_init 2>&1 | head -3
echo "----------------------------------------"
echo "✅ 完成  看坐标: rostopic echo /aft_mapped_to_init/pose/pose/position"
echo "  日志: /tmp/px4_run.log /tmp/bridge.log /tmp/fastlivo.log /tmp/rviz.log"
