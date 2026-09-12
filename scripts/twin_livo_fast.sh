#!/bin/bash
# ============================================================
#  快速启动 (fast v2)
#  关键优化: 不用 roscore（本机要 140s），直接起 rosmaster+rosout（~4s）
#  用法: bash ~/twin_livo_fast.sh
# ============================================================
START=$(date +%s)
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/noetic/setup.bash
source ~/twin_ws/devel/setup.bash
ROOT=~/twin_ws/src/FAST-LIVO2
started=""
el(){ echo $(( $(date +%s) - START )); }

# 1) ROS master（绕开慢的 roscore）
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

echo ""
echo "本次启动: ${started:-（无需启动, 全部已在跑）}"
echo "耗时: $(el) 秒"
