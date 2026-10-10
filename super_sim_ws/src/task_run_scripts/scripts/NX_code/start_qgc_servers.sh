#!/bin/bash

# ==============================================================================
# QGC 远程操控终端 - NX 端核心自启脚本
# 负责：等待图形桌面 -> 启动 roscore -> 启动推流/触控/指令三大微服务
# ==============================================================================

# 1. 延时 2 秒，确保 Ubuntu 桌面环境、虚拟 HDMI 信号和网络完全就绪
sleep 2

# 2. 导出图形界面环境变量 (极其关键！这是 mss 截屏和 pynput 模拟鼠标的前提)
export DISPLAY=:0
# 终极护航】：关闭本地图形界面的安全拦截！允许所有后台进程在桌面上画图！
xhost +local:

# 3. 加载 ROS 环境变量 (假设使用的是 Noetic，如果是 Melodic 请自行更改)
source ~/.bashrc
source /home/fouruav/catkin_ws/devel/setup.bash

# 4. 定义日志目录：按启动时间分目录，便于排障和回溯
BASE_DIR="/home/fouruav/catkin_ws/src/task_run_scripts/scripts/NX_code"
LOG_ROOT="/home/fouruav/catkin_ws/src/task_run_scripts/log"
LOG_RETENTION_COUNT=10
RUN_TAG="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$LOG_ROOT/$RUN_TAG"

mkdir -p "$LOG_DIR"
ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"

prune_old_logs() {
	mapfile -t RUN_DIRS < <(find "$LOG_ROOT" -maxdepth 1 -mindepth 1 -type d -regextype posix-extended -regex '.*/[0-9]{8}_[0-9]{6}' | sort)
	if [ "${#RUN_DIRS[@]}" -le "$LOG_RETENTION_COUNT" ]; then
		return
	fi

	LOCAL_DELETE_COUNT=$(( ${#RUN_DIRS[@]} - LOG_RETENTION_COUNT ))
	for OLD_DIR in "${RUN_DIRS[@]:0:LOCAL_DELETE_COUNT}"; do
		rm -rf "$OLD_DIR"
	done
}

prune_old_logs

START_LOG="$LOG_DIR/start_qgc_servers.log"
ROSCORE_LOG="$LOG_DIR/roscore.log"
TOUCH_LOG="$LOG_DIR/nx_touch_server2.log"
COMMAND_LOG="$LOG_DIR/nx_command_server2.log"
STREAM_LOG="$LOG_DIR/rviz_full_stream2.log"

exec > >(tee -a "$START_LOG") 2>&1

echo "[$(date +'%F %T')] start_qgc_servers.sh launched"
echo "[$(date +'%F %T')] logs are stored in: $LOG_DIR"

# 5. 在后台静默启动 roscore，为所有 ROS 节点打好基础
#    防止 rviz_full_stream.py 里的 rospy.init_node() 因找不到核心而死锁卡住
nohup roscore > "$ROSCORE_LOG" 2>&1 &
ROSCORE_PID=$!

# 给 roscore 3 秒钟的启动时间，确保它的 Master 节点完全 ready
sleep 3 

# ==============================================================================
# 6. 后台并发启动微服务
# ==============================================================================

nohup python3 -u "$BASE_DIR/nx_touch_server2.py" > "$TOUCH_LOG" 2>&1 &
TOUCH_PID=$!
nohup python3 -u "$BASE_DIR/nx_command_server2.py" > "$COMMAND_LOG" 2>&1 &
COMMAND_PID=$!
nohup python3 -u "$BASE_DIR/rviz_full_stream2.py" > "$STREAM_LOG" 2>&1 &
STREAM_PID=$!

cleanup() {
	echo "[$(date +'%F %T')] stopping all modules..."
	kill "$TOUCH_PID" "$COMMAND_PID" "$STREAM_PID" "$ROSCORE_PID" 2>/dev/null
}

trap cleanup SIGINT SIGTERM

echo "[$(date +'%F %T')] background pids: roscore=$ROSCORE_PID touch=$TOUCH_PID command=$COMMAND_PID stream=$STREAM_PID"
echo "[$(date +'%F %T')] press Ctrl+C to stop and keep logs"

wait
exit 0