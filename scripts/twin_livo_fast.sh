#!/bin/bash
# ============================================================
#  启动脚本（合并版）  v3
#   默认:      只补"缺失"的组件（常驻时 0-2 秒）
#   --full:    先清残留 roslaunch，启动后等待并验证链路
#              （用于异常时：run_id 冲突 / 桥没数据）
#  用法: bash ~/twin_livo_fast.sh [--full]
# ============================================================
START=$(date +%s)
FULL=0
[ "$1" = "--full" ] && FULL=1

export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/noetic/setup.bash
source ~/twin_ws/devel/setup.bash
ROOT=~/twin_ws/src/FAST-LIVO2
started=""
el(){ echo $(( $(date +%s) - START )); }

if [ $FULL -eq 1 ]; then
  echo "  [full] 清理残留 roslaunch（防 run_id 冲突）..."
  pkill -f roslaunch 2>/dev/null
  sleep 2
fi

# 1) ROS master（绕开慢的 roscore：它 140s，rosmaster+rosout 只要 ~4s）
if ! timeout 2 rosnode list >/dev/null 2>&1; then
  echo "  [1/5] 启动 ROS master（rosmaster+rosout，约 4 秒）..."
  setsid rosmaster --core -p 11311 > /tmp/rosmaster.log 2>&1 < /dev/null &
  for i in $(seq 1 30); do
    timeout 2 rosnode list >/dev/null 2>&1 && break
    sleep 1
  done
  rosparam set /rosdistro noetic 2>/dev/null
  rosparam set /rosversion 1.17.4 2>/dev/null
  setsid rosrun rosout rosout __name:=rosout > /tmp/rosout.log 2>&1 < /dev/null &
  sleep 2
  started="$started master($(el)s)"
else
  echo "  [1/5] master 已在跑 ✓"
fi

# 2) PX4
if ! pgrep -f "bin/px4" >/dev/null 2>&1; then
  echo "  [2/5] 启动 PX4 ..."
  tmux kill-session -t px4 2>/dev/null
  tmux new-session -d -s px4 "cd ~/PX4-AirSim && make px4_sitl_default none_iris 2>&1 | tee /tmp/px4_run.log"
  started="$started PX4"
else
  echo "  [2/5] PX4 已在跑 ✓"
fi

# 3) 桥
if ! pgrep -f "bridge.py" >/dev/null 2>&1; then
  echo "  [3/5] 启动仿真桥 ..."
  setsid roslaunch twin_bridge bridge.launch host:=127.0.0.1 imu_rate:=220 rgb_rate:=30 depth_rate:=5 \
      > /tmp/bridge.log 2>&1 < /dev/null &
  started="$started 桥"
else
  echo "  [3/5] 桥已在跑 ✓"
fi

# 4) FAST-LIVO2
if ! pgrep -f "fastlivo_mapping" >/dev/null 2>&1; then
  echo "  [4/5] 启动 FAST-LIVO2 ..."
  rosparam load $ROOT/config/TwinSim.yaml
  rosparam load $ROOT/config/TwinSim_camera.yaml /laserMapping
  setsid rosrun fast_livo fastlivo_mapping __name:=laserMapping > /tmp/fastlivo.log 2>&1 < /dev/null &
  started="$started FAST-LIVO2"
else
  echo "  [4/5] FAST-LIVO2 已在跑 ✓"
fi

# 5) RViz
if ! pgrep -x rviz >/dev/null 2>&1; then
  echo "  [5/5] 启动 RViz ..."
  setsid rviz -d $ROOT/rviz_cfg/fast_livo2.rviz > /tmp/rviz.log 2>&1 < /dev/null &
  started="$started RViz"
else
  echo "  [5/5] RViz 已在跑 ✓"
fi

# --full: 等待并验证链路
if [ $FULL -eq 1 ]; then
  echo ""
  echo "  [full] 等 PX4 <-> AirSim TCP4560（最多 120s）..."
  for i in $(seq 1 40); do
    ss -tn 2>/dev/null | grep -q ':4560' && { echo "         ✅ 4560 已连接"; break; }
    [ $((i % 5)) -eq 0 ] && echo "         ... 等 4560（已 $(el)s）"
    sleep 3
  done
  echo "  [full] 等 /twin 出数据（最多 60s）..."
  for i in $(seq 1 20); do
    timeout 4 rostopic echo -n 1 /twin/lidar/points_preview/header >/dev/null 2>&1 && { echo "         ✅ /twin 有数据"; break; }
    [ $((i % 5)) -eq 0 ] && echo "         ... 等数据（已 $(el)s）"
    sleep 3
  done
fi

echo ""
echo "本次启动: ${started:-（无需启动, 全部已在跑）}"
echo "耗时: $(el) 秒"
