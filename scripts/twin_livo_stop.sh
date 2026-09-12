#!/bin/bash
# ============================================================
#  关闭脚本 v2
#   默认: 只停 桥/FAST-LIVO2/RViz  → 保留 roscore+PX4 常驻,
#         下次 twin_livo_fast.sh 秒开（roscore 重启要 2 分钟！）
#   --all: 连 roscore/PX4 一起停（彻底清场，下次需完整启动）
#  用法: bash ~/twin_livo_stop.sh [--all]
# ============================================================
ALL=0
[ "$1" = "--all" ] && ALL=1

echo "=== 停 ROS 侧节点 ==="
pkill -f fastlivo_mapping 2>/dev/null && echo "  - FAST-LIVO2"
pkill -x rviz             2>/dev/null && echo "  - RViz"
pkill -f "roslaunch twin_bridge" 2>/dev/null && echo "  - 桥(roslaunch)"
pkill -f bridge.py        2>/dev/null && echo "  - 桥(bridge.py)"
pkill -f "rosbag play"    2>/dev/null && echo "  - rosbag play"

if [ "$ALL" = "1" ]; then
  echo "=== --all: 连 PX4 / roscore 一起停 ==="
  tmux kill-session -t px4 2>/dev/null && echo "  - PX4 (tmux)"
  pkill -f "bin/px4" 2>/dev/null && echo "  - PX4 进程"
  pkill -f roslaunch 2>/dev/null
  pkill -f rosout    2>/dev/null
  pkill -f rosmaster 2>/dev/null && echo "  - rosmaster"
  pkill -f roscore   2>/dev/null && echo "  - roscore"
  echo ""
  echo "⚠️  已彻底清场：下次启动需完整等待（roscore 约 2 分钟）"
else
  echo ""
  echo "✅ 算法已关闭；roscore 与 PX4 保留常驻"
  echo "   → 下次用 bash ~/twin_livo_fast.sh 可在几秒内恢复"
  echo "   → 要连 roscore/PX4 一起停: bash ~/twin_livo_stop.sh --all"
fi

sleep 2
echo ""
echo "=== 剩余相关进程 ==="
pgrep -af "fastlivo_mapping|bridge.py|rosmaster|bin/px4" 2>/dev/null | head -6 || echo "  （无）"
pgrep -ax rviz 2>/dev/null
