#!/bin/bash
# ============================================================
#  关闭 FAST-LIVO2 全套: FAST-LIVO2 / RViz / 仿真桥 / PX4 / roscore
#  用法:  bash ~/twin_livo_stop.sh
# ============================================================
echo "=== 关闭 FAST-LIVO2 相关进程 ==="

pkill -f fastlivo_mapping 2>/dev/null && echo "  - FAST-LIVO2"
pkill -x rviz             2>/dev/null && echo "  - RViz"
pkill -f twin_bridge      2>/dev/null && echo "  - 传感器桥"
pkill -f "rosbag play"    2>/dev/null && echo "  - rosbag play"

if tmux kill-session -t px4 2>/dev/null; then echo "  - PX4 (tmux 会话)"; fi
pkill -x px4              2>/dev/null && echo "  - PX4 进程"

pkill -f rosmaster        2>/dev/null && echo "  - ROS master"
pkill -f roscore          2>/dev/null && echo "  - roscore"

sleep 2

echo ""
echo "=== 检查残留 ==="
LEFT="$(pgrep -a -f 'fastlivo_mapping|twin_bridge|rosmaster|roscore|rosbag play' 2>/dev/null)"
LEFT="$LEFT$(pgrep -ax rviz 2>/dev/null)"
LEFT="$LEFT$(pgrep -ax px4 2>/dev/null)"
if [ -z "$LEFT" ]; then
  echo "  ✅ 全部已关闭"
else
  echo "$LEFT"
fi
echo ""
echo "提示: AirSim(Blocks) / QGC 需在 Windows 侧手动关闭"
