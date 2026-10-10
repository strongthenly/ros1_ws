#!/bin/bash
# ============================================================
#  一键启动: FAST-LIVO2 + MAVROS + 起飞 1.5m + FUEL 自主探索
#
#  前置(唯一手动项): Windows 上 AirSim Blocks 已点 Play
#
#  用法: bash ~/fuel_all.sh [seed_z] [mode] [goal_x] [pcd]
#        seed_z  起飞/悬停高度   默认 1.5
#        mode    控制模式        position(默认,稳) | raw(带速度/加速度前馈)
#        goal_x  触发点 x       默认 3.0
#        pcd     1=退出时保存PCD  0=不保存(默认)
#
#  例:  bash ~/fuel_all.sh 1.5 position 3.0 1     # 开 PCD 保存
# ============================================================
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/noetic/setup.bash
source ~/twin_ws/devel/setup.bash
source ~/fuel_ws/devel/setup.bash

SEED_Z=${1:-1.5}
MODE=${2:-position}
GOAL_X=${3:-3.0}
PCD=${4:-0}
export PCD_SAVE=$PCD

say(){ echo "[fuel_all] $*"; }

# ---------- 0) AirSim ----------
say "0/6 检查 AirSim (RPC 41451)..."
if timeout 3 bash -c "echo > /dev/tcp/127.0.0.1/41451" 2>/dev/null; then
  say "     [OK] AirSim 在 Play"
else
  say "     [X] AirSim 不可连 —— 请先在 UE4 Blocks 窗口点绿色 Play，再重跑"; exit 1
fi

# ---------- 1) twin 链路（--full：含 4560 / /clock / /twin / /cloud_registered 检查 + 自动重启 FK）----------
say "1/6 启动 twin 链路（--full 会验证链路并对 FAST-LIVO2 自动重启）..."
bash ~/twin_livo_fast.sh --full

# 硬检查：/clock 是否在步进（这是"点云有没有"的总开关）
say "     复核 /clock 步进（LockStep 冻结检测）..."
if timeout 6 rostopic hz /clock 2>/dev/null | grep -q "average"; then
  say "     [OK] /clock 在走"
else
  say "     [X] /clock 不走 → AirSim 未步进（PX4 没连上 4560 或 AirSim 没 Play）"
  say "         这是"点云有时没有"的真正原因：不是等不够久，而是仿真冻住了。"
  say "         检查: ss -tn | grep 4560 ; tail -20 /tmp/px4_run.log"
  exit 1
fi

# 硬检查：FAST-LIVO2 是否真出数据
if timeout 6 rostopic hz /cloud_registered 2>/dev/null | grep -q "average"; then
  say "     [OK] /cloud_registered 有数据"
else
  say "     [X] FAST-LIVO2 无输出 —— 查 tail -20 /tmp/fastlivo.log"; exit 1
fi

# ---------- 2) PX4 专用 MAVLink ----------
say "2/6 开 PX4 专用 MAVLink (14610/14611)..."
tmux send-keys -t px4 "mavlink start -u 14610 -o 14611" Enter
sleep 3

# ---------- 3) MAVROS ----------
if pgrep -f mavros_node >/dev/null 2>&1; then
  say "3/6 MAVROS 已在跑"
else
  say "3/6 启动 MAVROS..."
  setsid roslaunch mavros px4.launch fcu_url:=udp://:14611@127.0.0.1:14610 > /tmp/mavros.log 2>&1 < /dev/null &
fi
ok=0
for i in $(seq 1 25); do
  if timeout 5 rostopic echo -n1 /mavros/state 2>/dev/null | grep -q "connected: True"; then ok=1; break; fi
  sleep 2
done
if [ $ok -eq 1 ]; then say "     [OK] MAVROS 已连 PX4"; else
  say "     [X] MAVROS 未连上 —— 查 tail /tmp/mavros.log（多半是 PX4 的 14610 没开成功）"; exit 1
fi

# ---------- 4) FUEL（控制桥 seed 悬停 → 自动起飞）----------
say "4/6 启动 FUEL（seed_z=$SEED_Z, mode=$MODE, PCD=$PCD）..."
pkill -f exploration_node   2>/dev/null
pkill -f fuel_input_bridge  2>/dev/null
pkill -f fuel_ctrl_bridge   2>/dev/null
sleep 2
setsid roslaunch fuel_bridge fuel_real.launch enable_ctrl:=true \
  ctrl_mode:=$MODE seed_z:=$SEED_Z > /tmp/fuel_real.log 2>&1 < /dev/null &

# ---------- 5) 等起飞（带诊断 + 稳定检测，不干等）----------
say "5/6 等起飞到 ${SEED_Z} m ..."
ok=0; lastz=""; stable=0
for i in $(seq 1 30); do
  z=$(timeout 4 rostopic echo -n1 /mavros/local_position/pose/pose/position/z 2>/dev/null | head -1)
  st=$(timeout 4 rostopic echo -n1 /mavros/state 2>/dev/null | grep -oE 'mode: "[^"]*"|armed: (True|False)' | tr '\n' ' ')
  if [ -n "$z" ] && awk -v a="$z" -v b="$SEED_Z" 'BEGIN{exit !(a > b-0.35)}'; then
    ok=1; say "     [OK] 高度 ${z} m | $st"; break
  fi
  # 稳定检测：连续 5 次高度几乎不变且 >0.3m → 认为已悬停（不必死等目标高度）
  if [ -n "$z" ] && [ -n "$lastz" ] && awk -v a="$z" -v b="$lastz" 'BEGIN{exit !(a>b-0.05 && a<b+0.05)}'; then
    stable=$((stable+1))
  else
    stable=0
  fi
  lastz="$z"
  if [ $stable -ge 5 ] && awk -v a="$z" 'BEGIN{exit !(a>0.3)}'; then
    ok=2; say "     [OK] 已在 ${z} m 稳定悬停（目标 ${SEED_Z}m）| $st"; break
  fi
  [ $((i % 5)) -eq 0 ] && say "     ... 高度=${z:-?} | $st （已 $((i*2))s）"
  sleep 2
done
if [ $ok -eq 0 ]; then
  say "     [X] 未起飞 —— 诊断："
  say "         当前: z=${z:-无数据} | $st"
  say "         控制桥日志: tail -15 /tmp/fuel_real.log"
  say "         常见原因: (a) 不是 OFFBOARD → 控制桥 set_mode 失败（MAVROS 没连）"
  say "                   (b) 一直 armed=False → PX4 拒绝 arm（查飞控状态）"
  say "                   (c) 无 setpoint → /mavros/local_position/pose 没数据（EKF 未就绪）"
  exit 1
fi

# ---------- 6) 触发探索 ----------
say "6/6 触发探索（发 2D Nav Goal）..."
timeout 6 rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: ${GOAL_X}, y: 0.0, z: ${SEED_Z}}, orientation: {w: 1.0}}}" >/dev/null 2>&1
sleep 6

if grep -qE "Triggered|PLAN_TRAJ" /tmp/fuel_real.log 2>/dev/null; then
  say "     [OK] 探索已触发"
else
  say "     [!] 未见 Triggered —— 查 tail -20 /tmp/fuel_real.log"
fi

echo ""
say "==================== 完成 ===================="
say "FUEL 日志  : tail -f /tmp/fuel_real.log"
say "各节点日志 : tail -f ~/.ros/log/latest/fuel_ctrl_bridge-2.log   # 对齐等信息在这"
say "对齐验证   : bash ~/fuel_align_check.sh       # 只读，实时校验方向"
say "无人机位置 : rostopic echo /mavros/local_position/pose/pose/position"
say "探索耗时   : grep -E 'Cost mat|Total time' /tmp/fuel_real.log"
say "关闭       : bash ~/fuel_stop.sh          # 默认留 master"
say "             bash ~/fuel_stop.sh --all    # 全停"
[ "$PCD" = "1" ] && say "PCD        : 关闭时存到 ~/twin_ws/src/FAST-LIVO2/Log/pcd/"
