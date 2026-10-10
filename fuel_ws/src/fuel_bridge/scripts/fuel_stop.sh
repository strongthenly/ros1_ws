#!/bin/bash
# ============================================================
#  一键关闭: FUEL + 控制桥 + 输入桥 + MAVROS + FAST-LIVO2 + 仿真桥 + PX4 + RViz
#
#  默认: 保留 ROS master（常驻，重启只需 4s）
#         FAST-LIVO2 用 SIGINT 关闭 → 若启动时 PCD_SAVE=1 会保存 PCD
#  --all      : 连 ROS master 一起停
#  --no-save  : FAST-LIVO2 用 SIGKILL（不等保存、更干脆）
#  --keep-px4 : 不停 PX4
#
#  用法: bash ~/fuel_stop.sh [--all] [--no-save] [--keep-px4]
# ============================================================
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/noetic/setup.bash 2>/dev/null

ALL=0; NOSAVE=0; KEEPPX4=0
for a in "$@"; do
  case "$a" in
    --all) ALL=1 ;;
    --no-save) NOSAVE=1 ;;
    --keep-px4) KEEPPX4=1 ;;
  esac
done

ROOT=~/twin_ws/src/FAST-LIVO2
stopped=""

echo "==================== 关闭 ===================="

# 1) FUEL（exploration_node / traj_server / waypoint_generator / 两个桥）
echo "[1/7] 停 FUEL（探索 + 规划 + 桥）..."
pkill -f "roslaunch fuel_bridge"   2>/dev/null
pkill -f "exploration_node"        2>/dev/null
pkill -f "traj_server"             2>/dev/null
pkill -f "waypoint_generator"      2>/dev/null
pkill -f "fuel_ctrl_bridge"        2>/dev/null
pkill -f "fuel_input_bridge"       2>/dev/null
stopped="$stopped FUEL"
sleep 1

# 2) MAVROS（先停它，避免它继续和 PX4 通信）
echo "[2/7] 停 MAVROS ..."
pkill -f "roslaunch mavros" 2>/dev/null
pkill -f "mavros_node"      2>/dev/null
stopped="$stopped MAVROS"
sleep 1

# 3) FAST-LIVO2（SIGINT → 触发 savePCD；不要用 -9）
echo "[3/7] 停 FAST-LIVO2（PCD保存=$([ $NOSAVE -eq 1 ] && echo 关 || echo 开)）..."
if pgrep -f "fastlivo_mapping" >/dev/null 2>&1; then
  if [ $NOSAVE -eq 1 ]; then
    pkill -KILL -f "fastlivo_mapping" 2>/dev/null
  else
    pkill -INT -f "fastlivo_mapping" 2>/dev/null
    echo "       等它保存 PCD 并退出（最多 25s）..."
    for i in $(seq 1 25); do
      pgrep -f "fastlivo_mapping" >/dev/null 2>&1 || break
      sleep 1
    done
    if pgrep -f "fastlivo_mapping" >/dev/null 2>&1; then
      echo "       [!] 未退出，用 SIGTERM"
      pkill -TERM -f "fastlivo_mapping" 2>/dev/null
    else
      if [ -d "$ROOT/Log/pcd" ]; then
        echo "       PCD 目录: $ROOT/Log/pcd"
        ls -lh "$ROOT/Log/pcd" 2>/dev/null | tail -3
      fi
    fi
  fi
  stopped="$stopped FAST-LIVO2"
else
  echo "       （未在跑）"
fi

# 4) 仿真桥
echo "[4/7] 停仿真桥 ..."
pkill -f "roslaunch twin_bridge" 2>/dev/null
pkill -f "bridge.py"             2>/dev/null
stopped="$stopped 桥"
sleep 1

# 5) RViz
echo "[5/7] 停 RViz ..."
pkill -x rviz 2>/dev/null
stopped="$stopped RViz"

# 6) PX4（LockStep 源；停了 AirSim 就冻结）
if [ $KEEPPX4 -eq 0 ]; then
  echo "[6/7] 停 PX4 ..."
  tmux kill-session -t px4 2>/dev/null
  pkill -f "bin/px4"           2>/dev/null
  pkill -f "px4_sitl_default"  2>/dev/null
  stopped="$stopped PX4"
else
  echo "[6/7] 保留 PX4（--keep-px4）"
fi

# 7) ROS master（默认保留）
if [ $ALL -eq 1 ]; then
  echo "[7/7] 停 ROS master（--all）..."
  pkill -f "rosmaster --core -p 11311" 2>/dev/null
  pkill -f "rosout" 2>/dev/null
  stopped="$stopped master"
  sleep 1
else
  echo "[7/7] 保留 ROS master 常驻（要全停加 --all）"
fi

echo ""
echo "已停:${stopped:-（无）}"
echo "提示: PCD 保存在 $ROOT/Log/pcd/（需启动时 PCD_SAVE=1）"
