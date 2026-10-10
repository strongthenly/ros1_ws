#!/bin/bash
# ============================================================
#  关闭脚本 v3
#   默认: 停 桥 / FAST-LIVO2 / RViz / PX4
#         → 保留 ROS master 常驻（它重启要 140s，留着下次秒开）
#   --all: 连 ROS master 一起停（彻底清场）
#  用法: bash ~/twin_livo_stop.sh [--all]
# ============================================================
ALL=0
[ "$1" = "--all" ] && ALL=1

echo "=== 停算法 + 仿真 ==="
pkill -f fastlivo_mapping        2>/dev/null && echo "  - FAST-LIVO2"
pkill -x rviz                    2>/dev/null && echo "  - RViz"
pkill -f "roslaunch twin_bridge" 2>/dev/null && echo "  - 桥(roslaunch)"
pkill -f bridge.py               2>/dev/null && echo "  - 桥(bridge.py)"
pkill -f "rosbag play"           2>/dev/null && echo "  - rosbag play"
tmux kill-session -t px4         2>/dev/null && echo "  - PX4 (tmux 会话)"
pkill -f "bin/px4"               2>/dev/null && echo "  - PX4 进程"

if [ "$ALL" = "1" ]; then
  echo "=== --all: 连 ROS master 一起停 ==="
  pkill -f roslaunch 2>/dev/null
  pkill -f rosout    2>/dev/null && echo "  - rosout"
  pkill -f rosmaster 2>/dev/null && echo "  - rosmaster"
  pkill -f roscore   2>/dev/null
  echo ""
  echo "⚠️  已彻底清场（连 master 也停了）"
  echo "   下次启动会慢一点：master ~6s + PX4 ~20-40s"
else
  echo ""
  echo "✅ 已停：算法 + PX4；ROS master 保留常驻"
  echo "   → 下次: bash ~/twin_livo_fast.sh"
  echo "     （重起 PX4 + 桥 + 算法，约 20-40 秒，PX4 启动+连 AirSim 需时间）"
  echo "   → 要连 master 一起停: bash ~/twin_livo_stop.sh --all"
fi

sleep 2
echo ""
echo "=== 剩余进程 ==="
pgrep -af "fastlivo_mapping|bridge.py|bin/px4|rosmaster" 2>/dev/null | head -6 || echo "  （无）"
pgrep -ax rviz 2>/dev/null
