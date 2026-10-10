#!/bin/bash

# ============================================
# Mid360 激光雷达 (Fast-LIVO2) 启动脚本
# 用法:
#   bash start_fastlivo2.sh [true|false]
#     true  = 带 MindCloud：fastlivo2 录制数据并输出 pcd
#     false = 不带 MindCloud（默认）：仅建图不录制
# 说明:
#   通常由 nx_command_server.py / Radar_Launcher.yaml 以独立进程组
#   方式调用，脚本保持前台阻塞(wait)，便于外部通过进程组(PGID)统一清理。
# ============================================

# 1. 解析参数：$1=是否带 MindCloud 录制数据；$2=建图质量模式(可选，覆盖自动选择)
#    自动选择：带 MindCloud(建图) -> 高精度点云配置 MID360_Fly_Lidar.yaml
#              不带 MindCloud(仅飞行) -> 飞行定位配置 MID360_Fly.yaml
WITH_CLOUD="${1:-false}"
case "$WITH_CLOUD" in
  true|True|TRUE|1|yes|YES) WITH_CLOUD="true" ;;
  *) WITH_CLOUD="false" ;;
esac

# 按 WITH_CLOUD 自动选择点云模式
if [ "$WITH_CLOUD" = "true" ]; then
  CONFIG_NAME="MID360_Fly.yaml"   # 🗺️ 高质量建图（全量点云，PCD 完整）
else
  CONFIG_NAME="MID360_Fly.yaml"         # 🚁 飞行定位（降采样，PCD 点数少）
fi

# $2 可手动覆盖（lidar=建图 / fly=飞行）
MODE="${2:-}"
case "$MODE" in
  lidar|Lidar|LIDAR|建图) CONFIG_NAME="MID360_Fly.yaml" ;;
  fly|Fly|FLY|飞行) CONFIG_NAME="MID360_Fly.yaml" ;;
esac

# 2. 加载 ROS 环境
source /opt/ros/noetic/setup.bash
source /home/fouruav/catkin_ws/devel/setup.bash
export DISABLE_ROS1_EOL_WARNINGS=1

# 2.1 设置图形显示环境（rviz 需要），与 start_qgc_servers.sh 保持一致
#     仅在本地桌面会话下生效；无桌面环境(如纯SSH)时 rviz 无法弹出窗口
export DISPLAY=:0
xhost +local: 2>/dev/null || true

# 3. 定义日志目录（每次启动独立目录，便于排障）
LOG_ROOT="/home/fouruav/catkin_ws/src/task_run_scripts/log"
LOG_RETENTION_COUNT=10
RUN_TAG="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$LOG_ROOT/fastlivo2_${RUN_TAG}_${WITH_CLOUD}"
mkdir -p "$LOG_DIR"

# 清理旧日志：与 start_qgc_servers.sh 一致，只保留最近 LOG_RETENTION_COUNT 个
prune_old_logs() {
	mapfile -t RUN_DIRS < <(find "$LOG_ROOT" -maxdepth 1 -mindepth 1 -type d \
		-regextype posix-extended \
		-regex '.*/fastlivo2_[0-9]{8}_[0-9]{6}_(true|false)' | sort)
	if [ "${#RUN_DIRS[@]}" -le "$LOG_RETENTION_COUNT" ]; then
		return
	fi

	LOCAL_DELETE_COUNT=$(( ${#RUN_DIRS[@]} - LOG_RETENTION_COUNT ))
	for OLD_DIR in "${RUN_DIRS[@]:0:LOCAL_DELETE_COUNT}"; do
		rm -rf "$OLD_DIR"
	done
}

prune_old_logs

LIVOX_LOG="$LOG_DIR/livox.log"
IMU_TF_LOG="$LOG_DIR/imu_tf.log"
FASTLIVO_LOG="$LOG_DIR/fastlivo.log"
PX4_OTHERS_LOG="$LOG_DIR/px4_and_others.log"
RS_LOG="$LOG_DIR/rs_d435.log"
RVIZ_LOG="$LOG_DIR/rviz.log"

# 4. 后台启动辅助函数：$1=名称 $2=命令 $3=日志文件，回显 PID
#    注意：提示信息输出到 stderr，stdout 只保留 PID，供 $(...) 捕获
start_node() {
	local name="$1" cmd="$2" log="$3"
	echo "[$(date +'%F %T')] starting $name -> $log" >&2
	nohup $cmd > "$log" 2>&1 &
	echo $!
}

echo "[$(date +'%F %T')] ============ Fast-LIVO2 启动 (MindCloud=$WITH_CLOUD, 模式=$CONFIG_NAME) ============"

# 5. 依次启动：Realsense D435 -> Livox 驱动 -> IMU TF -> rviz(提前) -> Fast-LIVO2 建图
#    每个之间仅等待 1 秒；rviz 提前启动让窗口先弹出，建图就绪后立即可视化
RS_PID=$(start_node "realsense_d435" "roslaunch task_run_scripts rs_d435_camera.launch" "$RS_LOG")

LIVOX_PID=$(start_node "livox_driver" "roslaunch task_run_scripts msg_MID360s.launch" "$LIVOX_LOG")
sleep 1

IMU_PID=$(start_node "imu_tf" "roslaunch task_run_scripts mid360_imu_tf.launch" "$IMU_TF_LOG")
sleep 1

# 🌟 提前启动 rviz（窗口立即弹出），建图节点启动时不再重复启动
RVIZ_PID=$(start_node "rviz" "rosrun rviz rviz -d /home/fouruav/catkin_ws/src/task_run_scripts/rviz/slam_rviz.rviz" "$RVIZ_LOG")
sleep 1

FASTLIVO_PID=$(start_node "fast_livo" "roslaunch task_run_scripts mapping_MID360.launch pcd_save:=$WITH_CLOUD rviz:=false config_name:=$CONFIG_NAME" "$FASTLIVO_LOG")
sleep 3

# 建图就绪后再启动 PX4 / odom_to_mavros / realman 等外设
PX4_PID=$(start_node "fastlivo2_px4_and_others" "roslaunch task_run_scripts fastlivo2_px4_and_others.launch" "$PX4_OTHERS_LOG")

echo "[$(date +'%F %T')] pids: rs=$RS_PID livox=$LIVOX_PID imu_tf=$IMU_PID rviz=$RVIZ_PID fast_livo=$FASTLIVO_PID px4_others=$PX4_PID"
echo "[$(date +'%F %T')] 日志目录: $LOG_DIR"

# 6. 优雅退出：收到 SIGINT(SIGTERM) 时按序关闭所有节点
#    流程：SIGTERM -> 等待最多 3 秒 -> 残留则 SIGKILL 兜底
cleanup() {
	echo "[$(date +'%F %T')] 正在关闭 Fast-LIVO2 (MindCloud=$WITH_CLOUD)..."
	# ① 建图进程先收 SIGINT（等价 Ctrl+C）触发 savePCD。
	#    X2：savePCD 已 fork 子进程(pcd_saver)后台写盘，主进程秒退；
	#    此处用进程名 -x 精确匹配主进程(fastlivo_mappi=15字符截断)，不误杀 pcd_saver。
	#    fork 失败退化为同步写盘时，主进程停留至写完，下方等待仍兜底。
	kill -INT "$FASTLIVO_PID" 2>/dev/null
	pkill -INT -x fastlivo_mappi 2>/dev/null
	for _ in $(seq 1 120); do
		pgrep -x fastlivo_mappi >/dev/null 2>&1 || break
		sleep 1
	done
	# ② 其余进程照常 SIGTERM
	kill -TERM "$PX4_PID" "$RVIZ_PID" "$IMU_PID" "$LIVOX_PID" "$RS_PID" 2>/dev/null
	for _ in 1 2 3; do
		kill -0 "$PX4_PID" "$RVIZ_PID" "$IMU_PID" "$LIVOX_PID" "$RS_PID" 2>/dev/null || break
		sleep 1
	done
	# ③ SIGKILL 兜底
	kill -KILL "$PX4_PID" "$FASTLIVO_PID" "$RVIZ_PID" "$IMU_PID" "$LIVOX_PID" "$RS_PID" 2>/dev/null
	pkill -KILL -x fastlivo_mappi 2>/dev/null
	echo "[$(date +'%F %T')] Fast-LIVO2 已关闭"
}
trap cleanup SIGINT SIGTERM

# 7. 保持前台阻塞，使外部可通过进程组(PGID)统一追踪与清理
wait
exit 0