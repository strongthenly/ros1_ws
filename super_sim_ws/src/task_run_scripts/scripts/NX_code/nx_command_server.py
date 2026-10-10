#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket
import subprocess
import signal
import shutil
import time
import threading
import os
import sys
import re
import json
import uuid
import errno
import zipfile
import shlex
import glob
from urllib.parse import urlparse, parse_qs
import yaml  # 🌟 新增：用于解析 yaml 文件

# 🌟 引入多线程 HTTP 服务器，解决并发阻塞导致内存查询超时的问题
try:
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
except ImportError:
    import socketserver
    from http.server import BaseHTTPRequestHandler, HTTPServer
    class ThreadingHTTPServer(socketserver.ThreadingMixIn, HTTPServer): pass

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image  # 🌟 导入图像消息类型

# ==========================================
# 配置文件与路径定义
# ==========================================
def _first_existing_path(candidates, fallback):
    for candidate in candidates:
        if not candidate:
            continue
        candidate = os.path.expanduser(candidate)
        if os.path.exists(candidate):
            return candidate
    return os.path.expanduser(fallback)

HOME_DIR = os.path.expanduser("~")
YAML_PATH = os.environ.get(
    "LIYUMEN_RECORDDATA_YAML",
    _first_existing_path(
        [
            "~/catkin_ws/src/odin_ros_driver/config/control_command.yaml",
            "~/odin_ws/src/odin_ros_driver/config/control_command.yaml",
            "/home/nv/catkin_ws/src/odin_ros_driver/config/control_command.yaml",
            "/home/nv/odin_ws/src/odin_ros_driver/config/control_command.yaml",
            "/home/fouruav/catkin_ws/src/odin_ros_driver/config/control_command.yaml",
            "/home/fouruav/odin_ws/src/odin_ros_driver/config/control_command.yaml",
        ],
        "~/catkin_ws/src/odin_ros_driver/config/control_command.yaml",
    ),
)
PATH_FILE = os.environ.get(
    "LIYUMEN_PATH_FILE",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "My_Flight_Path.txt"),
)


# 🌟 动态获取当前脚本所在目录，拼接用户自定义 yaml 路径
USER_DEFINE_YAML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "User_define_commands.yaml")

# 🌟 图像话题配置（优先级：环境变量 LIYUMEN_IMAGE_TOPIC > 雷达 profile 配置 > 默认值）
# Odin 相机: "/odin1/image/undistorted"
# D435 相机: "/camera/color/image_raw"
DEFAULT_IMAGE_TOPIC = "/odin1/image/undistorted"
IMAGE_TOPIC = DEFAULT_IMAGE_TOPIC


def _refresh_image_topic():
    """
    按 环境变量/当前雷达 profile 刷新全局图像话题。
    优先级: LIYUMEN_IMAGE_TOPIC > profile.image_topic > DEFAULT_IMAGE_TOPIC
    """
    global IMAGE_TOPIC
    env_topic = os.environ.get("LIYUMEN_IMAGE_TOPIC", "").strip()
    if env_topic:
        IMAGE_TOPIC = env_topic
        return IMAGE_TOPIC

    _, profile = _get_radar_profile()
    if isinstance(profile, dict):
        topic = (profile.get("image_topic") or "").strip()
        if topic:
            IMAGE_TOPIC = topic
            return IMAGE_TOPIC

    IMAGE_TOPIC = DEFAULT_IMAGE_TOPIC
    return IMAGE_TOPIC

# ==========================================
# 雷达统一配置路径
# ==========================================
RADAR_LAUNCHER_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Radar_Launcher.yaml")

# 全局缓存：当前激活的雷达 profile 配置（加载后不可变）
_radar_profile_config = {}     # 完整的 profile 字典
_radar_profile_name = ""       # 当前 profile 名称
_radar_profile_loaded = False  # 是否已加载

def _get_radar_profile():
    """
    读取 Radar_Launcher.yaml，根据 active_profile 和环境变量
    返回 (profile_name, profile_dict)。失败时返回 ("", {})。
    """
    global _radar_profile_config, _radar_profile_name, _radar_profile_loaded
    if _radar_profile_loaded:
        return _radar_profile_name, _radar_profile_config

    _radar_profile_loaded = True
    config = {}
    try:
        with open(RADAR_LAUNCHER_YAML, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f) or {}
    except Exception as e:
        print(f"⚠️ 读取雷达配置失败 ({RADAR_LAUNCHER_YAML}): {e}", flush=True)
        return "", {}

    profiles = config.get("profiles", {})
    if not isinstance(profiles, dict):
        return "", {}

    # 优先级：环境变量 > YAML active_profile > 默认 "odin"
    profile_name = (
        os.environ.get("LIYUMEN_RADAR_PROFILE")
        or config.get("active_profile")
        or "odin"
    )
    profile = profiles.get(profile_name)
    if not profile:
        print(f"⚠️ 雷达配置中未找到 profile '{profile_name}'，可用: {list(profiles.keys())}", flush=True)
        return "", {}

    _radar_profile_name = profile_name
    _radar_profile_config = profile
    print(f"📡 雷达配置已加载: profile='{profile_name}' -> {profile.get('display_name', profile_name)}", flush=True)
    return profile_name, profile


def _get_radar_cleanup_patterns():
    """从当前 profile 获取清理 pattern 列表，失败时返回空列表"""
    _, profile = _get_radar_profile()
    cleanup = profile.get("cleanup", {}) if isinstance(profile, dict) else {}
    pkill = cleanup.get("pkill_patterns", [])
    rosnode = cleanup.get("rosnode_patterns", [])
    return pkill, rosnode


global_current_pose = None
global_last_pose_time = 0.0
global_replay_process = None # 追踪复飞子进程状态

# 🌟 雷达启动状态追踪
global_radar_starting = False
global_radar_start_time = 0.0
global_radar_start_result = ""   # "starting" / "ok" / "fail"
global_radar_start_message = ""

# 🌟 追踪雷达子进程组（pgid, 描述, 启动时间），用于精准关闭
_radar_process_groups = []
_radar_process_groups_lock = threading.Lock()

def pose_callback(msg):
    global global_current_pose, global_last_pose_time
    global_current_pose = msg
    global_last_pose_time = time.time()

def get_free_space():
    total, used, free = shutil.disk_usage("/")
    return free / (1024 ** 3)


def _read_sysfs_number(path):
    try:
        with open(path, "r") as f:
            return float(f.read().strip())
    except (OSError, ValueError):
        return None


def _health_gpu_percent():
    for path in (
        "/sys/devices/platform/17000000.ga10b/load",
        "/sys/devices/17000000.ga10b/load",
        "/sys/devices/gpu.0/load",
    ):
        load = _read_sysfs_number(path)
        if load is not None:
            return load / 10.0
    return None


def _health_cpu_temp_c():
    for zone in glob.glob("/sys/class/thermal/thermal_zone*"):
        try:
            with open(os.path.join(zone, "type"), "r") as f:
                if f.read().strip() != "CPU-therm":
                    continue
        except OSError:
            continue
        temp = _read_sysfs_number(os.path.join(zone, "temp"))
        if temp is not None:
            return temp / 1000.0
    return None


def _health_metrics():
    cpu_percent = mem_percent = None
    try:
        import psutil
    except Exception:
        psutil = None
    if psutil is not None:
        try:
            cpu_percent = psutil.cpu_percent(interval=0.1)
        except Exception:
            pass
        try:
            mem_percent = psutil.virtual_memory().percent
        except Exception:
            pass
    try:
        disk_free_gb = get_free_space()
    except Exception:
        disk_free_gb = None
    return {
        "status": "degraded",
        "cpuPercent": cpu_percent,
        "memPercent": mem_percent,
        "gpuPercent": _health_gpu_percent(),
        "cpuTempC": _health_cpu_temp_c(),
        "diskFreeGB": disk_free_gb,
        "time": time.time(),
    }

def set_recorddata(value):
    if not os.path.exists(YAML_PATH):
        print(f"⚠️ recorddata 配置不存在，跳过写入但继续启动雷达: {YAML_PATH}", flush=True)
        return True
    try:
        with open(YAML_PATH, 'r', encoding='utf-8') as f:
            content = f.read()
        new_content = re.sub(r"(recorddata:\s*)\d", rf"\g<1>{value}", content)
        with open(YAML_PATH, 'w', encoding='utf-8') as f:
            f.write(new_content)
        return True
    except Exception as e:
        print(f"⚠️ recorddata 写入失败，跳过写入但继续启动雷达: {e}", flush=True)
        return True

def _ros_source_prefix():
    ros_setup = os.environ.get("LIYUMEN_ROS_SETUP", "/opt/ros/noetic/setup.bash")
    workspace_setup = os.environ.get(
        "LIYUMEN_WORKSPACE_SETUP",
        _first_existing_path(
            [
                "~/catkin_ws/devel/setup.bash",
                "~/odin_ws/devel/setup.bash",
                "/home/nv/catkin_ws/devel/setup.bash",
                "/home/nv/odin_ws/devel/setup.bash",
                "/home/fouruav/catkin_ws/devel/setup.bash",
                "/home/fouruav/odin_ws/devel/setup.bash",
            ],
            "",
        ),
    )

    parts = []
    if ros_setup and os.path.exists(os.path.expanduser(ros_setup)):
        parts.append(f"source {shlex.quote(os.path.expanduser(ros_setup))}")
    if workspace_setup and os.path.exists(os.path.expanduser(workspace_setup)):
        parts.append(f"source {shlex.quote(os.path.expanduser(workspace_setup))}")
    return " && ".join(parts) + (" && " if parts else "")

def _build_lidar_launch_cmd():
    log_dir = os.environ.get("LIYUMEN_LOG_DIR", os.path.join(HOME_DIR, ".cache", "liyumen", "log"))
    os.makedirs(log_dir, exist_ok=True)
    odin_log = os.path.join(log_dir, "odin_launch.log")
    px4_log = os.path.join(log_dir, "px4_others_launch.log")
    odin_launch = os.environ.get("LIYUMEN_ODIN_LAUNCH", "roslaunch odin_ros_driver odin1_ros1.launch")
    px4_launch = os.environ.get("LIYUMEN_PX4_LAUNCH", "roslaunch task_run_scripts px4_and_others.launch")
    delay = os.environ.get("LIYUMEN_PX4_LAUNCH_DELAY", "9")
    prefix = _ros_source_prefix()
    command = (
        f"{prefix}{odin_launch} > {shlex.quote(odin_log)} 2>&1 & "
        f"sleep {shlex.quote(str(delay))} && "
        f"{px4_launch} > {shlex.quote(px4_log)} 2>&1 &"
    )
    return command, odin_log, px4_log

# ==========================================
# 雷达进程组追踪与精准关闭
# ==========================================
def _track_radar_pgid(pgid, description=""):
    """记录一个雷达进程组 PGID，用于后续精准关闭"""
    with _radar_process_groups_lock:
        _radar_process_groups.append((pgid, description, time.time()))
        # 保留最近 10 条记录
        if len(_radar_process_groups) > 10:
            _radar_process_groups.pop(0)

def _kill_tracked_radar_groups():
    """
    通过进程组追踪精准关闭雷达进程。
    先 SIGTERM，等待 2 秒，再 SIGKILL 残留。
    返回 True 表示有关闭动作。
    """
    with _radar_process_groups_lock:
        groups = list(_radar_process_groups)
        _radar_process_groups.clear()

    if not groups:
        return False

    print(f"🧹 通过进程组追踪精准关闭 {len(groups)} 个雷达进程组...", flush=True)
    for pgid, desc, _ in groups:
        try:
            os.killpg(pgid, signal.SIGINT)
            print(f"  -> SIGINT 进程组 {pgid} ({desc})", flush=True)
        except ProcessLookupError:
            pass
        except PermissionError:
            print(f"  ⚠️ 无权限终止进程组 {pgid} ({desc})", flush=True)
        except Exception as e:
            print(f"  ⚠️ 终止进程组 {pgid} 异常: {e}", flush=True)

    # 隐患2修复：直接对 fastlivo_mapping 主进程补发 SIGINT（双保险）
    # 防止 roslaunch 异常/提前退出导致信号没转发到建图节点，PCD 无法保存。
    # X2：savePCD 已 fork 子进程(pcd_saver)后台写盘，主进程秒退；
    #     用进程名 -x 精确匹配主进程(fastlivo_mappi=15字符截断)，不误杀 pcd_saver。
    try:
        subprocess.Popen(["pkill", "-INT", "-x", "fastlivo_mappi"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).wait()
        print("  -> 双保险: pkill -INT fastlivo_mappi", flush=True)
    except Exception as e:
        print(f"  ⚠️ 双保险补发 SIGINT 异常: {e}", flush=True)

    # X2：只等 fastlivo_mapping 主进程退出（主进程秒退；PCD 由 pcd_saver 后台写盘），最长 60 秒
    # 不再等整个进程组消失（rviz/livox 等不响应 SIGINT 的会拖后腿）
    for _ in range(60):
        try:
            out = subprocess.check_output(["pgrep", "-x", "fastlivo_mappi"],
                                          stderr=subprocess.DEVNULL)
            if not out.strip():
                break
        except Exception:
            break
        time.sleep(1)

    # PCD 已写完（或超时兜底），立即 SIGKILL 进程组，绝不残留
    for pgid, desc, _ in groups:
        try:
            os.killpg(pgid, signal.SIGKILL)
            print(f"  -> SIGKILL 进程组 {pgid} ({desc})", flush=True)
        except ProcessLookupError:
            pass
        except Exception as e:
            print(f"  ⚠️ 强制终止进程组 {pgid} 异常: {e}", flush=True)

    return True

def _launch_and_track_radar(launch_cmd_str, description="radar", env=None):
    """
    启动雷达进程（创建独立进程组）并追踪 PGID。
    返回 (subprocess.Popen, pgid)
    """
    my_env = (env or os.environ).copy()
    my_env.setdefault("DISPLAY", ":0")
    my_env.setdefault("DISABLE_ROS1_EOL_WARNINGS", "1")

    proc = subprocess.Popen(
        launch_cmd_str,
        shell=True,
        executable='/bin/bash',
        env=my_env,
        preexec_fn=os.setsid,   # 创建新会话 => 独立进程组
    )
    pgid = os.getpgid(proc.pid)
    _track_radar_pgid(pgid, description)
    print(f"🚀 已启动雷达: {description} (pid={proc.pid}, pgid={pgid})", flush=True)
    return proc, pgid

# ==========================================
# 关闭雷达：混合策略（进程组优先 + pattern 回退）
# ==========================================
# 硬编码的默认清理 pattern（当 YAML 配置缺失时使用）
_DEFAULT_PKILL_PATTERNS = [
    # --- Fast-LIVO2 核心 ---
    "mid360_imu_tf.launch", "roslaunch task_run_scripts", "imu_tf_node",
    "msg_MID360s.launch", "msg_MID360.launch",
    "livox_ros_driver2", "roslaunch livox_ros_driver2",
    "fast_livo mapping_MID360.launch", "republish", "lib/rviz",
    # --- px4_and_others 相关 ---
    "px4_and_others.launch", "px4.launch",
    "roslaunch mavros", "mavros_node",
    "odom_to_mavros", "realman_monitor",
    "odom_to_map_broadcaster", "static_transform_publisher",
    # --- Odin / 相机 ---
    "odin1_ros1.launch", "rs_camera.launch", "realsense2_camera",
]
_DEFAULT_ROSNODE_PATTERNS = [
    "imu_tf", "livox", "host_sdk_sample", "pcd2depth_node",
    "livox_lidar_publisher2", "laserMapping", "republish",
    "mavros", "odom_to_mavros", "odom_throttle_node",
    "odom_to_map_broadcaster", "realman_monitor", "/rviz",
    "realsense2_camera",
]

_kill_lidar_lock = threading.Lock()  # 全局互斥：同一时间只允许一个清理流程

def kill_lidar_processes():
    """关闭雷达进程（互斥保护 + 代际保护）"""
    if not _kill_lidar_lock.acquire(blocking=False):
        print("⏳ 已有清理流程在运行，跳过本次清理", flush=True)
        return
    try:
        _kill_lidar_processes_impl()
    finally:
        _kill_lidar_lock.release()

def _kill_lidar_processes_impl():
    """关闭雷达进程 — 先精确杀进程组，再 pattern 回退清理残留"""
    print("🧹 正在清理历史雷达进程...", flush=True)

    # ----- 第一阶段：通过追踪的进程组精准关闭 -----
    _kill_tracked_radar_groups()

    # ----- 第二阶段：从配置读取 pattern 回退规则 -----
    pkill_patterns, rosnode_patterns = _get_radar_cleanup_patterns()
    if not pkill_patterns:
        pkill_patterns = _DEFAULT_PKILL_PATTERNS
    if not rosnode_patterns:
        rosnode_patterns = _DEFAULT_ROSNODE_PATTERNS

    print(f"  -> 回退清理策略: pkill={len(pkill_patterns)}条 rosnode={len(rosnode_patterns)}条", flush=True)

    # rosnode 清理
    _rosnode_kill_by_patterns(rosnode_patterns)

    # SIGTERM 优雅退出
    for pat in pkill_patterns:
        try:
            subprocess.Popen(["pkill", "-SIGINT", "-f", pat],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).wait()
        except Exception:
            pass

    # 动态等待：检测到建图主进程还在（漏网进程）才等，最多 15 秒；没有则立即继续。
    # 注意用 -x 进程名匹配，pcd_saver 后台写盘不算建图残留。
    for _ in range(15):
        alive = False
        try:
            out = subprocess.check_output(
                ["pgrep", "-x", "fastlivo_mappi"],
                stderr=subprocess.DEVNULL)
            if out.strip():
                alive = True
        except Exception:
            alive = False
        if not alive:
            break
        time.sleep(1)

    # SIGKILL 强制清理
    for pat in pkill_patterns:
        try:
            subprocess.Popen(["pkill", "-9", "-f", pat],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).wait()
        except Exception:
            pass

    # 最终 rosnode 清理
    _rosnode_kill_by_patterns(rosnode_patterns)

    # 清理 master 上的僵尸节点注册（强杀后 rviz 等节点残留）
    _rosnode_cleanup_zombies()

    # PCD 由 FAST-LIVO2 直接保存为会话时间戳文件名（如 20260807_150000.pcd），无需事后重命名
    print("✅ 雷达进程清理完毕。", flush=True)

def _rosnode_kill_by_patterns(node_patterns):
    """通过 rosnode kill 杀掉匹配的 ROS 节点"""
    try:
        node_list = subprocess.check_output(["rosnode", "list"]).decode().splitlines()
    except Exception:
        return
    for n in node_list:
        if any(p in n for p in node_patterns):
            try:
                subprocess.Popen(["rosnode", "kill", n],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).wait()
            except Exception:
                pass


def _rosnode_cleanup_zombies():
    """清理 ROS master 上无响应的僵尸节点注册（强杀后残留）"""
    try:
        # rosnode cleanup 会列出所有无法联系的节点并询问一次，自动应答 y
        subprocess.Popen(
            "echo y | rosnode cleanup",
            shell=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        ).wait()
    except Exception:
        pass


# ==========================================
# 统一雷达启动分发（从 Radar_Launcher.yaml 读取配置）
# ==========================================
def _handle_radar_launch(with_cloud, env=None):
    """
    根据 Radar_Launcher.yaml 的 active_profile 启动雷达。
    with_cloud=True  -> 使用 launch_with_cloud（录数据）
    with_cloud=False -> 使用 launch_without_cloud（不录数据）
    """
    profile_name, profile = _get_radar_profile()
    if not profile:
        print("⚠️ 未加载雷达配置，使用旧版硬编码逻辑...", flush=True)
        _handle_radar_launch_fallback(with_cloud, env)
        return

    variant = "launch_with_cloud" if with_cloud else "launch_without_cloud"
    launch_cfg = profile.get(variant, {}) if isinstance(profile, dict) else {}
    if not launch_cfg or not isinstance(launch_cfg, dict):
        print(f"⚠️ 雷达 profile '{profile_name}' 缺少 '{variant}' 配置，使用旧版硬编码逻辑...", flush=True)
        _handle_radar_launch_fallback(with_cloud, env)
        return

    command = launch_cfg.get("command", "").strip()
    if not command:
        print(f"⚠️ 雷达 profile '{profile_name}' '{variant}' command 为空", flush=True)
        return

    display_name = profile.get("display_name", profile_name)
    description = f"{profile_name}{'+cloud' if with_cloud else ''}"

    # 可选：设置 recorddata
    recorddata_yaml = launch_cfg.get("recorddata_yaml", "")
    recorddata_val = launch_cfg.get("recorddata")
    if recorddata_val is not None and recorddata_yaml:
        # 动态修改 YAML 路径，让 set_recorddata 写到正确的位置
        global YAML_PATH
        saved_yaml = YAML_PATH
        try:
            YAML_PATH = recorddata_yaml
            set_recorddata(int(recorddata_val))
        finally:
            YAML_PATH = saved_yaml
    elif recorddata_val is not None and profile_name == "odin":
        # 兼容旧版：使用默认 YAML_PATH
        set_recorddata(int(recorddata_val))

    # 先关闭旧雷达：必须先等上一轮异步关闭线程彻底结束，否则它会误杀刚启动的新进程
    waited_prev = False
    for _t in threading.enumerate():
        if _t.name == "bg_kill_lidar" and _t.is_alive():
            print("⏳ 等待上一轮关闭收尾完成，避免误杀新进程...", flush=True)
            _t.join(timeout=120)
            waited_prev = True
    if waited_prev:
        # 上一轮后台清理已完整执行（进程已清干净），只做快速残留验证，跳过重复完整清理
        # 注意排除 pcd_saver 后台写盘子进程（不算雷达残留）
        try:
            out = subprocess.check_output(
                "pgrep -x fastlivo_mappi 2>/dev/null; "
                "pgrep -f livox_ros_driver2 2>/dev/null; "
                "pgrep -f 'roslaunch task_run_scripts' 2>/dev/null",
                shell=True, stderr=subprocess.DEVNULL)
            if out.strip():
                print("⚠️ 仍有残留进程，执行完整清理", flush=True)
                kill_lidar_processes()
            else:
                print("✅ 上一轮已清理干净，跳过重复清理", flush=True)
        except Exception:
            print("✅ 上一轮已清理干净，跳过重复清理", flush=True)
    else:
        kill_lidar_processes()

    # 启动新雷达并追踪
    _launch_and_track_radar(command, description, env=env)
    # 🌟 启动雷达后按 profile 刷新图像话题
    _refresh_image_topic()
    print(f"✅ [{display_name}] 已提交启动 (cloud={with_cloud})", flush=True)


def _handle_radar_launch_fallback(with_cloud, env):
    """旧版硬编码回退：当 YAML 配置缺失时使用"""
    if os.environ.get("LIYUMEN_USE_MID360", "").lower() in ("true", "1", "yes"):
        # Mid360 硬编码
        if with_cloud:
            cmd = (
                "bash /home/fouruav/catkin_ws/start_fastlivo2.sh true"
                " > /home/fouruav/catkin_ws/src/task_run_scripts/log/fastlivo2_launch.log 2>&1"
            )
            _launch_and_track_radar(cmd, "mid360+cloud(fallback)", env=env)
        else:
            cmd = (
                "bash /home/fouruav/catkin_ws/start_fastlivo2.sh false"
                " > /home/fouruav/catkin_ws/src/task_run_scripts/log/fastlivo2_launch.log 2>&1"
            )
            _launch_and_track_radar(cmd, "mid360(fallback)", env=env)
    else:
        # Odin 硬编码
        if with_cloud:
            set_recorddata(1)
        else:
            set_recorddata(0)
        launch_cmd, _, _ = _build_lidar_launch_cmd()
        tag = "odin+cloud(fallback)" if with_cloud else "odin(fallback)"
        _launch_and_track_radar(launch_cmd, tag, env=env)


def check_pose_condition():
    global global_current_pose, global_last_pose_time
    if global_current_pose is None or (time.time() - global_last_pose_time) > 1.5:
        return "NO_TOPIC"
    x = global_current_pose.pose.position.x
    y = global_current_pose.pose.position.y
    z = global_current_pose.pose.position.z
    dist = (x**2 + y**2)**0.5
    
    if z < 0.4: return f"ERR_Z_LOW:高度过低 ({z:.2f}m)。要求: >0.4m"
    if z > 1.6: return f"ERR_Z_HIGH:高度过高 ({z:.2f}m)。要求: <1.5m"
    if dist > 0.4: return f"ERR_DIST:偏离原点过远 (偏离 {dist:.2f}m)。要求: <0.20m"
    return "OK"

def check_path_file():
    if os.path.exists(PATH_FILE) and os.path.getsize(PATH_FILE) > 20:
        return "EXISTS"
    return "EMPTY"

# ==========================================
# 文件传输服务：复用 6004 HTTP 端口，避免和现有监测服务冲突
# ==========================================
FILE_TRANSFER_HOME_ROOT = os.path.abspath(os.environ.get("LIYUMEN_HOME_ROOT") or os.path.expanduser("~"))
FILE_TRANSFER_DESKTOP_ROOT = os.environ.get("LIYUMEN_DESKTOP_ROOT")
FILE_TRANSFER_USB_ROOT = os.environ.get("LIYUMEN_USB_ROOT")
FILE_TRANSFER_SERVER_VERSION = "2026-07-02-file-tools-v3"
STREAM_SETTINGS_VERSION = "2026-07-02-stream-settings-v1"
STREAM_SETTINGS_PATH = os.environ.get(
    "LIYUMEN_STREAM_SETTINGS_PATH",
    os.path.join(os.path.expanduser("~"), ".config", "liyumen", "stream_settings.json"),
)
STREAM_DEFAULT_SETTINGS = {
    "enabled": True,
    "overlayEnabled": False,
    "width": 960,
    "height": 540,
    "jpegQuality": 50,
    "fps": 25,
}
STREAM_ALLOWED_RESOLUTIONS = {
    (640, 360),
    (960, 540),
    (1280, 720),
    (1600, 900),
    (1920, 1080),
}
file_transfer_jobs = {}
file_transfer_processes = {}
file_transfer_jobs_lock = threading.Lock()
file_transfer_usb_root_cache = None
file_transfer_usb_error_until = 0
file_transfer_usb_error_message = ""
file_transfer_desktop_root_cache = None
FAT32_MAX_FILE_SIZE = (4 * 1024 * 1024 * 1024) - 1

def _safe_json_value(value):
    if isinstance(value, str):
        return value.encode("utf-8", "replace").decode("utf-8")
    if isinstance(value, dict):
        return {
            _safe_json_value(str(key)): _safe_json_value(item)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [_safe_json_value(item) for item in value]
    return value

def _json_bytes(payload):
    return json.dumps(_safe_json_value(payload), ensure_ascii=False, separators=(',', ':')).encode('utf-8')

def _stream_settings_public(settings):
    payload = dict(settings)
    payload.update({
        "overlay_enabled": bool(settings.get("overlayEnabled")),
        "jpeg_quality": int(settings.get("jpegQuality")),
        "version": STREAM_SETTINGS_VERSION,
        "configPath": STREAM_SETTINGS_PATH,
        "config_path": STREAM_SETTINGS_PATH,
    })
    return payload

def _sanitize_stream_settings(candidate, base=None):
    settings = dict(STREAM_DEFAULT_SETTINGS)
    if isinstance(base, dict):
        settings.update(base)
    if not isinstance(candidate, dict):
        candidate = {}

    if "enabled" in candidate:
        settings["enabled"] = bool(candidate.get("enabled"))
    if "overlayEnabled" in candidate or "overlay_enabled" in candidate:
        settings["overlayEnabled"] = bool(candidate.get("overlayEnabled", candidate.get("overlay_enabled")))

    width = candidate.get("width", settings["width"])
    height = candidate.get("height", settings["height"])
    try:
        width = int(width)
        height = int(height)
    except Exception:
        width = int(settings["width"])
        height = int(settings["height"])
    if (width, height) not in STREAM_ALLOWED_RESOLUTIONS:
        width, height = int(settings["width"]), int(settings["height"])
        if (width, height) not in STREAM_ALLOWED_RESOLUTIONS:
            width, height = STREAM_DEFAULT_SETTINGS["width"], STREAM_DEFAULT_SETTINGS["height"]
    settings["width"] = width
    settings["height"] = height

    quality = candidate.get("jpegQuality", candidate.get("jpeg_quality", settings["jpegQuality"]))
    try:
        quality = int(quality)
    except Exception:
        quality = int(settings["jpegQuality"])
    settings["jpegQuality"] = max(35, min(85, quality))

    fps = candidate.get("fps", settings["fps"])
    try:
        fps = int(fps)
    except Exception:
        fps = int(settings["fps"])
    settings["fps"] = max(8, min(25, fps))
    return settings

def _load_stream_settings():
    settings = dict(STREAM_DEFAULT_SETTINGS)
    try:
        with open(STREAM_SETTINGS_PATH, "r", encoding="utf-8") as settings_file:
            stored = json.load(settings_file)
        settings = _sanitize_stream_settings(stored, settings)
    except FileNotFoundError:
        pass
    except Exception:
        pass
    return settings

def _save_stream_settings(updates):
    current = _load_stream_settings()
    settings = _sanitize_stream_settings(updates, current)
    config_dir = os.path.dirname(STREAM_SETTINGS_PATH)
    if config_dir:
        os.makedirs(config_dir, exist_ok=True)
    temp_path = STREAM_SETTINGS_PATH + ".tmp"
    with open(temp_path, "w", encoding="utf-8") as settings_file:
        json.dump(settings, settings_file, ensure_ascii=False, indent=2, sort_keys=True)
        settings_file.write("\n")
    os.replace(temp_path, STREAM_SETTINGS_PATH)
    return settings

def _path_for_client(abs_path, root):
    rel_path = os.path.relpath(abs_path, root)
    if rel_path == ".":
        return "/"
    return "/" + rel_path.replace(os.sep, "/")

def _parent_for_client(abs_path, root):
    root = os.path.abspath(root)
    abs_path = os.path.abspath(abs_path)
    if abs_path == root:
        return None
    return _path_for_client(os.path.dirname(abs_path), root)

def _format_file_size(size):
    value = float(size)
    units = ["B", "KB", "MB", "GB"]
    index = 0
    while value >= 1024 and index < len(units) - 1:
        value /= 1024.0
        index += 1
    if index == 0:
        return f"{int(value)} {units[index]}"
    return f"{value:.1f} {units[index]}"

def _storage_usage(path):
    try:
        usage = shutil.disk_usage(path)
    except Exception:
        return None

    used = usage.total - usage.free
    used_percent = 0 if usage.total <= 0 else used / float(usage.total)
    return {
        "total": usage.total,
        "used": used,
        "free": usage.free,
        "totalText": _format_file_size(usage.total),
        "total_text": _format_file_size(usage.total),
        "usedText": _format_file_size(used),
        "used_text": _format_file_size(used),
        "freeText": _format_file_size(usage.free),
        "free_text": _format_file_size(usage.free),
        "usedPercent": used_percent,
        "used_percent": used_percent,
    }

def _is_safe_child(root, target):
    root = os.path.abspath(root)
    target = os.path.abspath(target)
    try:
        return os.path.commonpath([root, target]) == root
    except ValueError:
        return False

def _default_desktop_root():
    global file_transfer_desktop_root_cache

    if FILE_TRANSFER_DESKTOP_ROOT and os.path.isdir(FILE_TRANSFER_DESKTOP_ROOT):
        return os.path.abspath(FILE_TRANSFER_DESKTOP_ROOT)

    if file_transfer_desktop_root_cache and os.path.isdir(file_transfer_desktop_root_cache):
        return file_transfer_desktop_root_cache

    home_root = os.path.abspath(FILE_TRANSFER_HOME_ROOT)
    candidates = []
    try:
        result = subprocess.run(
            ["xdg-user-dir", "DESKTOP"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2,
        )
        if result.returncode == 0:
            xdg_path = result.stdout.strip()
            if xdg_path:
                candidates.append(xdg_path)
    except Exception:
        pass

    candidates.extend([
        os.path.join(home_root, "Desktop"),
        os.path.join(home_root, "桌面"),
        home_root,
    ])

    for candidate in candidates:
        abs_candidate = os.path.abspath(os.path.expanduser(candidate))
        if os.path.isdir(abs_candidate) and _is_safe_child(home_root, abs_candidate):
            file_transfer_desktop_root_cache = abs_candidate
            return abs_candidate

    file_transfer_desktop_root_cache = home_root
    return home_root

def _decode_mount_path(path):
    return (path
        .replace("\\040", " ")
        .replace("\\011", "\t")
        .replace("\\012", "\n")
        .replace("\\134", "\\"))

def _is_writable_dir(path):
    return os.path.isdir(path) and os.access(path, os.W_OK)

def _is_io_error(error):
    return isinstance(error, OSError) and error.errno == errno.EIO

def _invalidate_usb_root(message="", cooldown=12):
    global file_transfer_usb_root_cache, file_transfer_usb_error_until, file_transfer_usb_error_message
    file_transfer_usb_root_cache = None
    file_transfer_usb_error_until = 0
    file_transfer_usb_error_message = message or "U盘 I/O 异常，请重新插拔或更换 U 盘"

def _raise_usb_io_error(error, context):
    message = f"{context}: NX 系统返回 U盘 I/O 异常。程序已停止本次传输并清理未完成的目标文件；请检查 U 盘、USB 供电/接口或 dmesg 中的内核写入错误。原始错误: {error}"
    _invalidate_usb_root(message, cooldown=0)
    raise OSError(errno.EIO, message) from error

def _is_probable_usb_mount(mount_point, device, fs_type, mount_options):
    mount_point = os.path.abspath(mount_point)
    ignored_fs = {
        "autofs", "binfmt_misc", "bpf", "cgroup", "cgroup2", "configfs",
        "debugfs", "devpts", "devtmpfs", "efivarfs", "fusectl", "hugetlbfs",
        "mqueue", "overlay", "proc", "pstore", "rpc_pipefs", "securityfs",
        "squashfs", "sysfs", "tmpfs", "tracefs",
    }
    if fs_type in ignored_fs or "rw" not in mount_options.split(","):
        return False

    preferred_roots = ("/media/", "/run/media/", "/mnt/")
    if mount_point.startswith(preferred_roots):
        return _is_writable_dir(mount_point)

    removable_devices = ("/dev/sd", "/dev/disk/by-id/usb", "/dev/disk/by-uuid/")
    removable_filesystems = ("vfat", "exfat", "ntfs", "fuseblk")
    if device.startswith(removable_devices) or fs_type in removable_filesystems:
        return _is_writable_dir(mount_point)

    return False

def _mounted_usb_candidates():
    candidates = []
    try:
        with open("/proc/self/mountinfo", "r", encoding="utf-8") as mount_file:
            for line in mount_file:
                fields = line.strip().split()
                if "-" not in fields or len(fields) < 10:
                    continue
                separator = fields.index("-")
                if separator + 2 >= len(fields):
                    continue

                mount_point = _decode_mount_path(fields[4])
                mount_options = fields[5]
                fs_type = fields[separator + 1]
                device = _decode_mount_path(fields[separator + 2])

                if _is_probable_usb_mount(mount_point, device, fs_type, mount_options):
                    try:
                        score = os.path.getmtime(mount_point)
                    except Exception:
                        score = 0
                    candidates.append((score, mount_point))
    except Exception:
        pass

    return candidates

def _scan_usb_root():
    global file_transfer_usb_root_cache
    if time.time() < file_transfer_usb_error_until:
        raise OSError(errno.EIO, file_transfer_usb_error_message)

    if FILE_TRANSFER_USB_ROOT and os.path.isdir(FILE_TRANSFER_USB_ROOT):
        return os.path.abspath(FILE_TRANSFER_USB_ROOT)

    if file_transfer_usb_root_cache and os.path.ismount(file_transfer_usb_root_cache) and _is_writable_dir(file_transfer_usb_root_cache):
        return os.path.abspath(file_transfer_usb_root_cache)

    file_transfer_usb_root_cache = None
    candidates = _mounted_usb_candidates()

    user_name = os.environ.get("USER") or os.path.basename(os.path.expanduser("~"))
    search_roots = [
        os.path.join("/media", user_name),
        "/media",
        os.path.join("/run/media", user_name),
        "/run/media",
        "/mnt",
    ]

    seen = set()
    for base in search_roots:
        if not os.path.isdir(base):
            continue
        for path in [base]:
            if path in seen:
                continue
            seen.add(path)
            if os.path.ismount(path) and _is_writable_dir(path):
                try:
                    score = os.path.getmtime(path)
                except Exception:
                    score = 0
                candidates.append((score, path))

        try:
            names = os.listdir(base)
        except Exception:
            continue

        for name in names:
            path = os.path.join(base, name)
            if path in seen or not os.path.isdir(path):
                continue
            seen.add(path)
            if os.path.ismount(path) and _is_writable_dir(path):
                try:
                    score = os.path.getmtime(path)
                except Exception:
                    score = 0
                candidates.append((score, path))

    if not candidates:
        return None
    candidates.sort(reverse=True)
    file_transfer_usb_root_cache = os.path.abspath(candidates[0][1])
    return file_transfer_usb_root_cache

def _get_transfer_root(side):
    if side == "desktop":
        return os.path.abspath(FILE_TRANSFER_HOME_ROOT)
    if side == "usb":
        return _scan_usb_root()
    raise ValueError("未知文件区域")

def _resolve_transfer_path(side, client_path):
    root = _get_transfer_root(side)
    if not root or not os.path.isdir(root):
        raise FileNotFoundError("U盘未挂载" if side == "usb" else "Home 不存在")

    if side == "desktop" and client_path in (None, ""):
        target = _default_desktop_root()
    else:
        normalized = (client_path or "/").replace("\\", "/").lstrip("/")
        target = os.path.abspath(os.path.join(root, normalized))
    if not _is_safe_child(root, target):
        raise ValueError("路径越界")
    return root, target

def _file_entry(abs_path, root):
    stat_result = os.stat(abs_path)
    is_dir = os.path.isdir(abs_path)
    return {
        "name": os.path.basename(abs_path),
        "path": _path_for_client(abs_path, root),
        "isDir": is_dir,
        "is_dir": is_dir,
        "size": 0 if is_dir else stat_result.st_size,
        "sizeText": "" if is_dir else _format_file_size(stat_result.st_size),
        "size_text": "" if is_dir else _format_file_size(stat_result.st_size),
        "modified": int(stat_result.st_mtime),
    }

def _list_transfer_dir(side, client_path):
    root, target = _resolve_transfer_path(side, client_path)
    if not os.path.isdir(target):
        raise NotADirectoryError("目标不是文件夹")

    entries = []
    try:
        names = os.listdir(target)
    except OSError as e:
        if side == "usb" and _is_io_error(e):
            _raise_usb_io_error(e, "读取U盘目录失败")
        raise

    for name in names:
        if name.startswith(".") or name.startswith("LYM_TMP_"):
            continue
        abs_path = os.path.join(target, name)
        try:
            entries.append(_file_entry(abs_path, root))
        except Exception:
            pass

    entries.sort(key=lambda item: (not item["isDir"], item["name"].lower()))
    usage = _storage_usage(target)
    return {
        "ok": True,
        "side": side,
        "path": _path_for_client(target, root),
        "parent": _parent_for_client(target, root),
        "rootName": os.path.basename(root) or root,
        "mounted": True,
        "usbMounted": True if side == "usb" else None,
        "storage": usage,
        "storageUsage": usage,
        "entries": entries,
    }

def _dedupe_destination(path):
    if not os.path.exists(path):
        return path

    base, ext = os.path.splitext(path)
    index = 1
    while True:
        candidate = f"{base}_{index}{ext}"
        if not os.path.exists(candidate):
            return candidate
        index += 1

def _job_cancel_requested(job_id):
    with file_transfer_jobs_lock:
        job = file_transfer_jobs.get(job_id, {})
        return bool(job.get("cancelRequested") or job.get("cancel_requested"))

def _source_size(source_abs, cancel_checker=None):
    if os.path.isfile(source_abs):
        return os.path.getsize(source_abs)

    total = 0
    for current_root, dir_names, file_names in os.walk(source_abs, followlinks=False):
        if cancel_checker and cancel_checker():
            raise RuntimeError("传输已取消")
        dir_names[:] = [name for name in dir_names if not os.path.islink(os.path.join(current_root, name))]
        for file_name in file_names:
            if cancel_checker and cancel_checker():
                raise RuntimeError("传输已取消")
            source_file = os.path.join(current_root, file_name)
            if os.path.isfile(source_file):
                total += os.path.getsize(source_file)
    return total

def _path_size(path):
    if not os.path.exists(path) and not os.path.islink(path):
        return 0
    return _source_size(path)

def _max_regular_file_size(path, cancel_checker=None):
    if os.path.isfile(path):
        return os.path.getsize(path)

    max_size = 0
    for current_root, dir_names, file_names in os.walk(path, followlinks=False):
        if cancel_checker and cancel_checker():
            raise RuntimeError("传输已取消")
        dir_names[:] = [name for name in dir_names if not os.path.islink(os.path.join(current_root, name))]
        for file_name in file_names:
            if cancel_checker and cancel_checker():
                raise RuntimeError("传输已取消")
            source_file = os.path.join(current_root, file_name)
            if os.path.isfile(source_file):
                max_size = max(max_size, os.path.getsize(source_file))
    return max_size

def _is_fat32_filesystem(fs_type):
    return (fs_type or "").lower() in ("vfat", "msdos", "fat", "fat16", "fat32")

def _validate_usb_file_size_limits(usb_root, transfer_items, job_id):
    mount_info = _mount_info_for_path(usb_root) or {}
    fs_type = mount_info.get("fsType", "")
    if not _is_fat32_filesystem(fs_type):
        return

    for item in transfer_items:
        source_abs = item[2]
        clean_name = item[3]
        largest_file = _max_regular_file_size(source_abs, lambda: _job_cancel_requested(job_id))
        if largest_file > FAT32_MAX_FILE_SIZE:
            raise RuntimeError(
                f"U盘文件系统是 {fs_type}，单个文件最大只能约 4GB；{clean_name} 中存在 {_format_file_size(largest_file)} 的文件。"
                "请把U盘格式化为 exFAT/NTFS/ext4 后再传输大文件。"
            )

def _safe_remove(path):
    try:
        if os.path.isdir(path) and not os.path.islink(path):
            shutil.rmtree(path, ignore_errors=True)
        elif os.path.exists(path) or os.path.islink(path):
            os.remove(path)
    except Exception:
        pass

def _validate_usb_ready(usb_root, destination_abs, required_bytes=0):
    if not FILE_TRANSFER_USB_ROOT and not os.path.ismount(usb_root):
        raise OSError(errno.EIO, "U盘挂载点已经失效，请重新插拔 U 盘")
    if not _is_writable_dir(destination_abs):
        raise PermissionError("U盘目标目录不可写")

    try:
        usage = shutil.disk_usage(destination_abs)
    except OSError as e:
        if _is_io_error(e):
            _raise_usb_io_error(e, "检查U盘剩余空间失败")
        raise

    if required_bytes > 0 and usage.free < required_bytes:
        raise OSError(errno.ENOSPC, f"U盘剩余空间不足，需要 {_format_file_size(required_bytes)}，可用 {_format_file_size(usage.free)}")

def _copy_file_to_usb(source_abs, target_abs, progress_callback=None):
    os.makedirs(os.path.dirname(target_abs), exist_ok=True)
    source_file = None
    target_file = None
    try:
        source_file = open(source_abs, "rb")
        target_file = open(target_abs, "wb")
        while True:
            chunk = source_file.read(4 * 1024 * 1024)
            if not chunk:
                break
            target_file.write(chunk)
            if progress_callback:
                progress_callback(len(chunk))
        if progress_callback:
            progress_callback(0, True, "closing", f"正在关闭文件: {os.path.basename(target_abs)}")
        target_file.close()
        target_file = None
        source_file.close()
        source_file = None
    except OSError as e:
        if _is_io_error(e):
            _raise_usb_io_error(e, f"写入U盘失败: {os.path.basename(target_abs)}")
        raise
    finally:
        if target_file is not None:
            try:
                target_file.close()
            except Exception:
                pass
        if source_file is not None:
            try:
                source_file.close()
            except Exception:
                pass

def _copy_dir_to_usb(source_abs, target_abs, progress_callback=None):
    source_abs = os.path.abspath(source_abs)
    target_abs = os.path.abspath(target_abs)
    try:
        os.makedirs(target_abs, exist_ok=True)

        for current_root, dir_names, file_names in os.walk(source_abs, followlinks=False):
            rel_root = os.path.relpath(current_root, source_abs)
            target_root = target_abs if rel_root == "." else os.path.join(target_abs, rel_root)
            os.makedirs(target_root, exist_ok=True)

            for dir_name in dir_names:
                source_dir = os.path.join(current_root, dir_name)
                if os.path.islink(source_dir):
                    continue
                os.makedirs(os.path.join(target_root, dir_name), exist_ok=True)

            for file_name in file_names:
                source_file = os.path.join(current_root, file_name)
                if not os.path.isfile(source_file):
                    continue
                _copy_file_to_usb(source_file, os.path.join(target_root, file_name), progress_callback)
    except OSError as e:
        if _is_io_error(e):
            _raise_usb_io_error(e, f"写入U盘文件夹失败: {os.path.basename(target_abs)}")
        raise

def _copy_item_to_usb(source_abs, target_abs, progress_callback=None):
    if os.path.isdir(source_abs) and not os.path.islink(source_abs):
        _copy_dir_to_usb(source_abs, target_abs, progress_callback)
    else:
        _copy_file_to_usb(source_abs, target_abs, progress_callback)

def _fsync_path(path):
    fd = None
    try:
        fd = os.open(path, os.O_RDONLY)
        os.fsync(fd)
    finally:
        if fd is not None:
            try:
                os.close(fd)
            except Exception:
                pass

def _fsync_parent_dir(path):
    parent = os.path.dirname(os.path.abspath(path)) or "/"
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    fd = None
    try:
        fd = os.open(parent, flags)
        os.fsync(fd)
    except OSError as e:
        # Some removable filesystems do not support fsync on directories.
        if e.errno not in (errno.EINVAL, errno.ENOTSUP, errno.EOPNOTSUPP):
            raise
    finally:
        if fd is not None:
            try:
                os.close(fd)
            except Exception:
                pass

def _sync_filesystem_for_path(path):
    try:
        result = subprocess.run(
            ["sync", "-f", path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            timeout=1800,
        )
    except FileNotFoundError:
        os.sync()
        return
    except subprocess.TimeoutExpired:
        raise RuntimeError("等待U盘写入落盘超时，请检查U盘或USB供电稳定性")

    if result.returncode == 0:
        return

    error_text = (result.stderr or "").strip()
    if "invalid option" in error_text or "unrecognized option" in error_text:
        os.sync()
        return
    raise RuntimeError(f"U盘写入落盘失败: {error_text or 'sync -f 返回失败'}")

def _verify_usb_mount_alive(usb_root):
    if not FILE_TRANSFER_USB_ROOT and not os.path.ismount(usb_root):
        raise OSError(errno.EIO, "U盘在传输过程中已断开或挂载点失效")
    if not os.path.isdir(usb_root):
        raise OSError(errno.EIO, "U盘目录在传输过程中已不可访问")

def _flush_and_verify_transfer_target(job_id, usb_root, target_abs, expected_size, clean_name):
    _verify_usb_mount_alive(usb_root)
    current_size = _path_size(target_abs)
    if current_size != expected_size:
        raise RuntimeError(f"大小校验失败: {clean_name}，源大小 {_format_file_size(expected_size)}，目标大小 {_format_file_size(current_size)}")

    try:
        if os.path.isfile(target_abs):
            _fsync_path(target_abs)
        _fsync_parent_dir(target_abs)
        _set_file_transfer_job(job_id, phase="syncing", progress=0.99, message=f"正在确认U盘写入完成: {clean_name}")
        _sync_filesystem_for_path(target_abs)
    except OSError as e:
        if _is_io_error(e):
            _raise_usb_io_error(e, f"确认U盘写入失败: {clean_name}")
        raise

    _verify_usb_mount_alive(usb_root)
    final_size = _path_size(target_abs)
    if final_size != expected_size:
        raise RuntimeError(f"落盘后校验失败: {clean_name}，目标大小 {_format_file_size(final_size)}，应为 {_format_file_size(expected_size)}")

def _set_file_transfer_job(job_id, **updates):
    with file_transfer_jobs_lock:
        job = file_transfer_jobs.get(job_id, {})
        job.update(updates)
        file_transfer_jobs[job_id] = job

def _job_public_copy(job):
    return {key: value for key, value in job.items() if not key.startswith("_")}

def _register_transfer_process(job_id, process):
    with file_transfer_jobs_lock:
        file_transfer_processes[job_id] = process

def _unregister_transfer_process(job_id):
    with file_transfer_jobs_lock:
        file_transfer_processes.pop(job_id, None)

def _read_process_log(log_path):
    try:
        with open(log_path, "r", encoding="utf-8", errors="replace") as log_file:
            return log_file.read().strip()
    except Exception:
        return ""

def _copy_command_candidates(source_abs, target_abs):
    commands = []
    if os.path.isfile(source_abs):
        # 优先走 GIO，行为更接近 Ubuntu 文件管理器右键“复制到...”。
        commands.append(("gio", target_abs, ["gio", "copy", "-p", source_abs, target_abs]))
    # 普通 U 盘通常不支持 Linux ownership/xattr 等元数据；cp 仅作为稳定回退。
    commands.append(("cp", target_abs, ["cp", "-R", "--", source_abs, target_abs]))
    return commands

def _log_transfer_event(job_id, message):
    print(f"[file-transfer {job_id}] {message}", flush=True)

def _remove_source_after_move(source_abs):
    result = subprocess.run(
        ["rm", "-rf", "--", source_abs],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
        timeout=None,
    )
    if result.returncode != 0:
        raise RuntimeError((result.stderr or "删除源文件失败").strip())

def _resolve_desktop_items(sources, action_text):
    if not isinstance(sources, list) or not sources:
        raise ValueError(f"请选择要{action_text}的文件或文件夹")

    items = []
    seen = set()
    for source_path in sources:
        home_root, source_abs = _resolve_transfer_path("desktop", source_path)
        source_abs = os.path.abspath(source_abs)
        if source_abs == os.path.abspath(home_root):
            raise ValueError(f"不能{action_text} Home 根目录")
        if not os.path.lexists(source_abs):
            raise FileNotFoundError(f"源文件不存在: {source_path}")
        if source_abs in seen:
            continue
        seen.add(source_abs)
        clean_name = os.path.basename(source_abs.rstrip(os.sep))
        items.append((source_path, source_abs, clean_name))
    return items

def _job_speed(bytes_done, started_at):
    return bytes_done / max(time.time() - started_at, 0.001)

def _set_size_job_progress(job_id, bytes_done, bytes_total, phase, message, started_at, max_progress=0.98):
    progress = 0 if bytes_total <= 0 else min(max_progress, bytes_done / float(bytes_total))
    speed_bps = _job_speed(bytes_done, started_at)
    _set_file_transfer_job(
        job_id,
        bytesDone=bytes_done,
        bytes_done=bytes_done,
        bytesTotal=bytes_total,
        bytes_total=bytes_total,
        progress=progress,
        phase=phase,
        speedBps=speed_bps,
        speed_bps=speed_bps,
        heartbeatAt=time.time(),
        message=message,
    )

def _archive_file_name(items):
    if len(items) == 1:
        base_name = items[0][2] or "archive"
        if base_name.lower().endswith(".zip"):
            base_name = base_name[:-4]
        return f"{base_name}.zip"
    return f"liyumen_archive_{time.strftime('%Y%m%d_%H%M%S')}.zip"

def _zip_write_regular_file(job_id, archive, file_abs, arcname, bytes_done, bytes_total, started_at):
    stat_result = os.stat(file_abs)
    zip_info = zipfile.ZipInfo(arcname, time.localtime(stat_result.st_mtime)[:6])
    zip_info.compress_type = zipfile.ZIP_DEFLATED
    zip_info.external_attr = (stat_result.st_mode & 0xFFFF) << 16

    last_update = 0
    with open(file_abs, "rb") as source_file:
        with archive.open(zip_info, "w", force_zip64=True) as target_file:
            while True:
                if _job_cancel_requested(job_id):
                    raise RuntimeError("传输已取消")
                chunk = source_file.read(4 * 1024 * 1024)
                if not chunk:
                    break
                target_file.write(chunk)
                bytes_done += len(chunk)
                now = time.time()
                if now - last_update >= 0.8:
                    _set_size_job_progress(
                        job_id,
                        bytes_done,
                        bytes_total,
                        "zipping",
                        f"正在压缩: {os.path.basename(file_abs)}",
                        started_at,
                    )
                    last_update = now
    return bytes_done

def _zip_source_item(job_id, archive, source_abs, clean_name, target_abs, bytes_done, bytes_total, started_at):
    if os.path.isfile(source_abs):
        return _zip_write_regular_file(job_id, archive, source_abs, clean_name, bytes_done, bytes_total, started_at)

    if not os.path.isdir(source_abs):
        return bytes_done

    wrote_anything = False
    for current_root, dir_names, file_names in os.walk(source_abs, followlinks=False):
        if _job_cancel_requested(job_id):
            raise RuntimeError("传输已取消")
        dir_names[:] = [name for name in dir_names if not os.path.islink(os.path.join(current_root, name))]
        rel_root = os.path.relpath(current_root, source_abs)
        archive_root = clean_name if rel_root == "." else clean_name + "/" + rel_root.replace(os.sep, "/")
        if not file_names and not dir_names:
            archive.writestr(archive_root.rstrip("/") + "/", "")
            wrote_anything = True

        for file_name in file_names:
            file_abs = os.path.abspath(os.path.join(current_root, file_name))
            if file_abs == target_abs or not os.path.isfile(file_abs):
                continue
            rel_file = file_name if rel_root == "." else rel_root.replace(os.sep, "/") + "/" + file_name
            arcname = clean_name + "/" + rel_file
            bytes_done = _zip_write_regular_file(job_id, archive, file_abs, arcname, bytes_done, bytes_total, started_at)
            wrote_anything = True

    if not wrote_anything:
        archive.writestr(clean_name.rstrip("/") + "/", "")
    return bytes_done

def _zip_worker(job_id, sources, destination):
    operation_text = "压缩"
    started_at = time.time()
    target_abs = None
    try:
        _set_file_transfer_job(
            job_id,
            status="running",
            phase="preparing",
            progress=0,
            speedBps=0,
            speed_bps=0,
            message="正在准备压缩任务...",
            startedAt=started_at,
        )
        items = _resolve_desktop_items(sources, operation_text)
        desktop_root, destination_abs = _resolve_transfer_path("desktop", destination or "")
        if not os.path.isdir(destination_abs):
            raise NotADirectoryError("压缩包目标路径不是文件夹")
        target_abs = _dedupe_destination(os.path.join(destination_abs, _archive_file_name(items)))
        if not _is_safe_child(desktop_root, target_abs):
            raise ValueError("压缩包目标路径越界")

        bytes_total = 0
        for _, source_abs, _ in items:
            if _job_cancel_requested(job_id):
                raise RuntimeError("传输已取消")
            bytes_total += _source_size(source_abs, lambda: _job_cancel_requested(job_id))

        _set_file_transfer_job(
            job_id,
            bytesDone=0,
            bytes_done=0,
            bytesTotal=bytes_total,
            bytes_total=bytes_total,
            targetAbsPaths=[target_abs],
            target_abs_paths=[target_abs],
            targetPaths=[_path_for_client(target_abs, desktop_root)],
            target_paths=[_path_for_client(target_abs, desktop_root)],
            phase="zipping",
            message=f"正在压缩 0/{len(items)}",
        )

        bytes_done = 0
        with zipfile.ZipFile(target_abs, "w", compression=zipfile.ZIP_DEFLATED, allowZip64=True) as archive:
            for index, (_, source_abs, clean_name) in enumerate(items):
                _set_file_transfer_job(job_id, current=clean_name, copied=index, message=f"正在压缩 {index + 1}/{len(items)}: {clean_name}")
                bytes_done = _zip_source_item(job_id, archive, source_abs, clean_name, target_abs, bytes_done, bytes_total, started_at)

        _set_file_transfer_job(job_id, phase="finalizing", progress=0.99, message="正在保存压缩包...")
        _fsync_path(target_abs)
        _fsync_parent_dir(target_abs)
        if not os.path.isfile(target_abs) or os.path.getsize(target_abs) <= 0:
            raise RuntimeError("压缩包生成失败")

        _set_file_transfer_job(
            job_id,
            status="done",
            done=True,
            phase="done",
            progress=1,
            copied=len(items),
            current="",
            bytesDone=bytes_total,
            bytes_done=bytes_total,
            bytesTotal=bytes_total,
            bytes_total=bytes_total,
            speedBps=_job_speed(bytes_total, started_at),
            speed_bps=_job_speed(bytes_total, started_at),
            finishedAt=time.time(),
            message=f"压缩完成: {os.path.basename(target_abs)}",
        )
    except Exception as e:
        if target_abs and not os.path.isdir(target_abs):
            _safe_remove(target_abs)
        if str(e) == "传输已取消":
            _set_file_transfer_job(job_id, status="canceled", done=True, canceled=True, phase="canceled", message="压缩已取消", finishedAt=time.time())
        else:
            _set_file_transfer_job(job_id, status="error", done=True, phase="error", error=str(e), message=f"压缩失败: {e}", finishedAt=time.time())

def _delete_one_path(job_id, source_abs, bytes_done, bytes_total, started_at):
    if _job_cancel_requested(job_id):
        raise RuntimeError("传输已取消")

    if os.path.isdir(source_abs) and not os.path.islink(source_abs):
        for current_root, dir_names, file_names in os.walk(source_abs, topdown=False, followlinks=False):
            for file_name in file_names:
                if _job_cancel_requested(job_id):
                    raise RuntimeError("传输已取消")
                file_abs = os.path.join(current_root, file_name)
                try:
                    file_size = os.path.getsize(file_abs) if os.path.isfile(file_abs) else 0
                    os.remove(file_abs)
                    bytes_done += file_size
                    _set_size_job_progress(job_id, bytes_done, bytes_total, "deleting", f"正在删除: {file_name}", started_at)
                except FileNotFoundError:
                    pass

            for dir_name in dir_names:
                dir_abs = os.path.join(current_root, dir_name)
                try:
                    if os.path.islink(dir_abs):
                        os.remove(dir_abs)
                    else:
                        os.rmdir(dir_abs)
                except FileNotFoundError:
                    pass
            try:
                os.rmdir(current_root)
            except FileNotFoundError:
                pass
        return bytes_done

    file_size = os.path.getsize(source_abs) if os.path.isfile(source_abs) else 0
    os.remove(source_abs)
    bytes_done += file_size
    _set_size_job_progress(job_id, bytes_done, bytes_total, "deleting", f"正在删除: {os.path.basename(source_abs)}", started_at)
    return bytes_done

def _delete_worker(job_id, sources):
    operation_text = "删除"
    started_at = time.time()
    try:
        _set_file_transfer_job(
            job_id,
            status="running",
            phase="preparing",
            progress=0,
            message="正在准备删除任务...",
            startedAt=started_at,
        )
        items = _resolve_desktop_items(sources, operation_text)
        bytes_total = 0
        for _, source_abs, _ in items:
            bytes_total += _source_size(source_abs, lambda: _job_cancel_requested(job_id))

        _set_file_transfer_job(
            job_id,
            bytesDone=0,
            bytes_done=0,
            bytesTotal=bytes_total,
            bytes_total=bytes_total,
            phase="deleting",
            message=f"正在删除 0/{len(items)}",
        )

        bytes_done = 0
        for index, (_, source_abs, clean_name) in enumerate(items):
            _set_file_transfer_job(job_id, current=clean_name, copied=index, message=f"正在删除 {index + 1}/{len(items)}: {clean_name}")
            bytes_done = _delete_one_path(job_id, source_abs, bytes_done, bytes_total, started_at)

        _set_file_transfer_job(
            job_id,
            status="done",
            done=True,
            phase="done",
            progress=1,
            copied=len(items),
            current="",
            bytesDone=bytes_total,
            bytes_done=bytes_total,
            bytesTotal=bytes_total,
            bytes_total=bytes_total,
            speedBps=_job_speed(bytes_total, started_at),
            speed_bps=_job_speed(bytes_total, started_at),
            finishedAt=time.time(),
            message=f"删除完成: {len(items)} 项",
        )
    except Exception as e:
        if str(e) == "传输已取消":
            _set_file_transfer_job(job_id, status="canceled", done=True, canceled=True, phase="canceled", message="删除已取消", finishedAt=time.time())
        else:
            _set_file_transfer_job(job_id, status="error", done=True, phase="error", error=str(e), message=f"删除失败: {e}", finishedAt=time.time())

def _wait_for_system_copy(job_id, process, temp_abs, base_done_bytes, total_bytes, started_at, operation_text, clean_name):
    log_path = f"/tmp/liyumen_transfer_{job_id}.log"
    last_update = 0
    last_bytes = base_done_bytes
    last_speed_time = time.time()
    speed_bps = 0

    while True:
        if _job_cancel_requested(job_id):
            _set_file_transfer_job(job_id, status="canceling", phase="canceling", message="正在取消传输...")
            try:
                process.terminate()
            except Exception:
                pass

        return_code = process.poll()
        now = time.time()
        if now - last_update >= 1.0 or return_code is not None:
            current_bytes = base_done_bytes + _path_size(temp_abs)
            delta_time = max(now - last_speed_time, 0.001)
            speed_bps = max(0, (current_bytes - last_bytes) / delta_time)
            last_bytes = current_bytes
            last_speed_time = now
            progress = 0 if total_bytes <= 0 else min(0.97, current_bytes / float(total_bytes))
            _set_file_transfer_job(
                job_id,
                bytesDone=current_bytes,
                bytes_done=current_bytes,
                bytesTotal=total_bytes,
                bytes_total=total_bytes,
                progress=progress,
                phase="writing",
                speedBps=speed_bps,
                speed_bps=speed_bps,
                heartbeatAt=now,
                message=f"{operation_text}中: {clean_name}",
            )
            last_update = now

        if return_code is not None:
            return return_code, speed_bps

        time.sleep(0.5)

def _terminate_process_tree(process):
    try:
        process.terminate()
    except Exception:
        pass
    try:
        process.wait(timeout=3)
    except Exception:
        try:
            process.kill()
        except Exception:
            pass

def _transfer_worker(job_id, sources, destination, operation):
    operation_text = "剪切" if operation == "move" else "复制"
    started_at = time.time()
    _set_file_transfer_job(
        job_id,
        status="running",
        version=FILE_TRANSFER_SERVER_VERSION,
        copied=0,
        total=len(sources),
        bytesDone=0,
        bytes_done=0,
        bytesTotal=0,
        bytes_total=0,
        progress=0,
        phase="preparing",
        speedBps=0,
        speed_bps=0,
        error="",
        message=f"正在准备{operation_text}任务...",
        startedAt=started_at
    )
    log_path = f"/tmp/liyumen_transfer_{job_id}.log"
    try:
        usb_root, destination_abs = _resolve_transfer_path("usb", destination or "/")
        if not os.path.isdir(destination_abs):
            raise NotADirectoryError("U盘目标路径不是文件夹")
        mount_info = _mount_info_for_path(usb_root) or {}
        _log_transfer_event(job_id, f"start operation={operation} usb_root={usb_root} destination={destination_abs} fs={mount_info.get('fsType', '')}")

        transfer_items = []
        required_bytes = 0
        for index, source_path in enumerate(sources):
            _set_file_transfer_job(job_id, phase="preparing", message=f"正在统计文件大小 {index + 1}/{len(sources)}")
            if _job_cancel_requested(job_id):
                raise RuntimeError("传输已取消")
            desktop_root, source_abs = _resolve_transfer_path("desktop", source_path)
            if not os.path.exists(source_abs):
                raise FileNotFoundError(f"源文件不存在: {source_path}")

            clean_name = os.path.basename(source_abs.rstrip(os.sep))
            target_abs = _dedupe_destination(os.path.join(destination_abs, clean_name))
            if not _is_safe_child(usb_root, target_abs):
                raise ValueError("目标路径越界")
            work_abs = target_abs
            item_size = _source_size(source_abs, lambda: _job_cancel_requested(job_id))
            required_bytes += item_size
            transfer_items.append((index, source_path, source_abs, clean_name, target_abs, work_abs, item_size))

        _validate_usb_ready(usb_root, destination_abs, required_bytes)
        _validate_usb_file_size_limits(usb_root, transfer_items, job_id)
        _set_file_transfer_job(
            job_id,
            bytesDone=0,
            bytes_done=0,
            bytesTotal=required_bytes,
            bytes_total=required_bytes,
            progress=0,
            phase="writing",
            message=f"{operation_text}中 0/{len(sources)}",
            usbRoot=usb_root,
            usb_root=usb_root,
            targetAbsPaths=[item[4] for item in transfer_items],
            target_abs_paths=[item[4] for item in transfer_items],
            targetPaths=[_path_for_client(item[4], usb_root) for item in transfer_items],
            target_paths=[_path_for_client(item[4], usb_root) for item in transfer_items],
        )

        bytes_completed = 0
        for index, source_path, source_abs, clean_name, target_abs, temp_abs, item_size in transfer_items:
            if _job_cancel_requested(job_id):
                raise RuntimeError("传输已取消")
            if os.path.exists(temp_abs) or os.path.islink(temp_abs):
                raise FileExistsError(f"目标已存在，已停止避免覆盖: {temp_abs}")

            _set_file_transfer_job(
                job_id,
                current=clean_name,
                copied=index,
                destination=_path_for_client(destination_abs, usb_root),
                message=f"{operation_text}中 {index}/{len(sources)}: {clean_name}",
            )

            log_file = None
            process = None
            command_temp_paths = []
            successful_temp_abs = None
            target_completed = False
            try:
                command_errors = []
                return_code = None
                speed_bps = 0
                for command_name, command_target_abs, command in _copy_command_candidates(source_abs, temp_abs):
                    command_temp_paths.append(command_target_abs)
                    _safe_remove(command_target_abs)
                    if _job_cancel_requested(job_id):
                        break
                    _log_transfer_event(job_id, f"run {command_name}: source={source_abs} target={command_target_abs}")
                    log_file = open(log_path, "a", encoding="utf-8", errors="replace")
                    log_file.write(f"\n--- {command_name}: {' '.join(command)} ---\n")
                    log_file.flush()
                    try:
                        process = subprocess.Popen(
                            command,
                            stdout=subprocess.DEVNULL,
                            stderr=log_file,
                            start_new_session=True,
                        )
                    except FileNotFoundError as e:
                        command_errors.append(f"{command_name}: 未找到命令 {command[0]}")
                        _log_transfer_event(job_id, command_errors[-1])
                        if log_file:
                            log_file.close()
                            log_file = None
                        continue

                    _register_transfer_process(job_id, process)
                    return_code, speed_bps = _wait_for_system_copy(
                        job_id,
                        process,
                        command_target_abs,
                        bytes_completed,
                        required_bytes,
                        started_at,
                        operation_text,
                        clean_name,
                    )
                    _unregister_transfer_process(job_id)
                    if log_file:
                        log_file.close()
                        log_file = None

                    if _job_cancel_requested(job_id):
                        break
                    if return_code == 0:
                        _log_transfer_event(job_id, f"{command_name} finished ok: {clean_name}")
                        successful_temp_abs = command_target_abs
                        break

                    error_text = _read_process_log(log_path)
                    command_errors.append(f"{command_name}: 退出码 {return_code}; {error_text[-800:]}")
                    _log_transfer_event(job_id, f"{command_name} failed code={return_code}; will try fallback")
                    _safe_remove(command_target_abs)
                    if os.path.exists(command_target_abs) or os.path.islink(command_target_abs):
                        _log_transfer_event(job_id, f"warning: failed target remains after cleanup: {command_target_abs}")

                if _job_cancel_requested(job_id):
                    for cleanup_path in command_temp_paths:
                        _safe_remove(cleanup_path)
                    _set_file_transfer_job(
                        job_id,
                        status="canceled",
                        done=True,
                        canceled=True,
                        phase="canceled",
                        message="传输已取消",
                        finishedAt=time.time(),
                    )
                    return

                if return_code != 0:
                    error_text = " | ".join(command_errors) or _read_process_log(log_path) or f"系统复制命令退出码 {return_code}"
                    raise RuntimeError(f"{error_text}；详细日志: {log_path}")
                if not successful_temp_abs:
                    raise RuntimeError("复制命令没有返回可校验的目标")

                _set_file_transfer_job(
                    job_id,
                    phase="verifying",
                    progress=0.98,
                    message=f"正在校验: {clean_name}",
                    speedBps=speed_bps,
                    speed_bps=speed_bps,
                )
                _flush_and_verify_transfer_target(job_id, usb_root, target_abs, item_size, clean_name)
                target_completed = True

                if operation == "move":
                    _set_file_transfer_job(job_id, phase="finalizing", progress=0.99, message=f"正在删除源文件: {clean_name}")
                    _remove_source_after_move(source_abs)
            except Exception:
                if process and process.poll() is None:
                    _terminate_process_tree(process)
                _unregister_transfer_process(job_id)
                if log_file:
                    try:
                        log_file.close()
                    except Exception:
                        pass
                if not target_completed:
                    for cleanup_path in command_temp_paths or [temp_abs]:
                        _safe_remove(cleanup_path)
                raise

            bytes_completed += item_size
            progress = 1 if required_bytes <= 0 else min(0.99, bytes_completed / float(required_bytes))
            _set_file_transfer_job(job_id, copied=index + 1, message=f"{operation_text}中 {index + 1}/{len(sources)}")

        target_visible = all(os.path.exists(item[4]) or os.path.islink(item[4]) for item in transfer_items)
        if not target_visible:
            raise RuntimeError("目标文件完成后不可见，请检查U盘状态")
        _verify_usb_mount_alive(usb_root)

        _set_file_transfer_job(
            job_id,
            status="done",
            done=True,
            version=FILE_TRANSFER_SERVER_VERSION,
            current="",
            copied=len(sources),
            bytesDone=required_bytes,
            bytes_done=required_bytes,
            bytesTotal=required_bytes,
            bytes_total=required_bytes,
            progress=1,
            phase="done",
            speedBps=required_bytes / max(time.time() - started_at, 0.001),
            speed_bps=required_bytes / max(time.time() - started_at, 0.001),
            finishedAt=time.time(),
            message=f"{operation_text}完成，可点击弹出U盘"
        )
    except Exception as e:
        if str(e) == "传输已取消":
            _log_transfer_event(job_id, "canceled")
            _set_file_transfer_job(job_id, status="canceled", done=True, canceled=True, phase="canceled", message="传输已取消", finishedAt=time.time())
        else:
            _log_transfer_event(job_id, f"failed: {e}")
            _set_file_transfer_job(job_id, status="error", done=True, phase="error", error=str(e), message=f"{operation_text}失败: {e}", finishedAt=time.time())
    finally:
        _unregister_transfer_process(job_id)

def _start_copy_transfer_job(sources, destination, operation="copy"):
    if not isinstance(sources, list) or not sources:
        raise ValueError("请选择要复制的文件或文件夹")
    if operation not in ("copy", "move"):
        operation = "copy"

    job_id = uuid.uuid4().hex
    operation_text = "剪切" if operation == "move" else "复制"
    with file_transfer_jobs_lock:
        file_transfer_jobs[job_id] = {
            "id": job_id,
            "job_id": job_id,
            "version": FILE_TRANSFER_SERVER_VERSION,
            "status": "queued",
            "done": False,
            "operation": operation,
            "sources": sources,
            "destination": destination or "/",
            "copied": 0,
            "total": len(sources),
            "bytesDone": 0,
            "bytes_done": 0,
            "bytesTotal": 0,
            "bytes_total": 0,
            "progress": 0,
            "phase": "queued",
            "speedBps": 0,
            "speed_bps": 0,
            "current": "",
            "error": "",
            "cancelRequested": False,
            "cancel_requested": False,
            "message": f"{operation_text}任务已排队",
            "createdAt": time.time(),
        }

    threading.Thread(target=_transfer_worker, args=(job_id, sources, destination, operation), daemon=True).start()
    return _job_public_copy(file_transfer_jobs[job_id])

def _start_zip_job(sources, destination):
    if not isinstance(sources, list) or not sources:
        raise ValueError("请选择要压缩的文件或文件夹")

    job_id = uuid.uuid4().hex
    with file_transfer_jobs_lock:
        file_transfer_jobs[job_id] = {
            "id": job_id,
            "job_id": job_id,
            "version": FILE_TRANSFER_SERVER_VERSION,
            "status": "queued",
            "done": False,
            "operation": "zip",
            "sources": sources,
            "destination": destination or "",
            "copied": 0,
            "total": len(sources),
            "bytesDone": 0,
            "bytes_done": 0,
            "bytesTotal": 0,
            "bytes_total": 0,
            "progress": 0,
            "phase": "queued",
            "speedBps": 0,
            "speed_bps": 0,
            "current": "",
            "error": "",
            "cancelRequested": False,
            "cancel_requested": False,
            "message": "压缩任务已排队",
            "createdAt": time.time(),
        }

    threading.Thread(target=_zip_worker, args=(job_id, sources, destination), daemon=True).start()
    return _job_public_copy(file_transfer_jobs[job_id])

def _start_delete_job(sources):
    if not isinstance(sources, list) or not sources:
        raise ValueError("请选择要删除的文件或文件夹")

    job_id = uuid.uuid4().hex
    with file_transfer_jobs_lock:
        file_transfer_jobs[job_id] = {
            "id": job_id,
            "job_id": job_id,
            "version": FILE_TRANSFER_SERVER_VERSION,
            "status": "queued",
            "done": False,
            "operation": "delete",
            "sources": sources,
            "destination": "",
            "copied": 0,
            "total": len(sources),
            "bytesDone": 0,
            "bytes_done": 0,
            "bytesTotal": 0,
            "bytes_total": 0,
            "progress": 0,
            "phase": "queued",
            "speedBps": 0,
            "speed_bps": 0,
            "current": "",
            "error": "",
            "cancelRequested": False,
            "cancel_requested": False,
            "message": "删除任务已排队",
            "createdAt": time.time(),
        }

    threading.Thread(target=_delete_worker, args=(job_id, sources), daemon=True).start()
    return _job_public_copy(file_transfer_jobs[job_id])

def _get_copy_transfer_job(job_id):
    with file_transfer_jobs_lock:
        job = file_transfer_jobs.get(job_id)
        return _job_public_copy(job) if job else None

def _cancel_transfer_job(job_id):
    with file_transfer_jobs_lock:
        job = file_transfer_jobs.get(job_id)
        if not job:
            raise KeyError("任务不存在")

        if job.get("done") or job.get("status") in ("done", "error", "canceled"):
            return _job_public_copy(job)

        job["cancelRequested"] = True
        job["cancel_requested"] = True
        job["status"] = "canceling"
        job["phase"] = "canceling"
        job["message"] = "正在取消任务..."
        process = file_transfer_processes.get(job_id)

    if process and process.poll() is None:
        _terminate_process_tree(process)

    with file_transfer_jobs_lock:
        job = file_transfer_jobs.get(job_id, {})
        return _job_public_copy(job)

def _has_running_transfer_jobs():
    with file_transfer_jobs_lock:
        for job in file_transfer_jobs.values():
            if job.get("status") in ("queued", "running", "canceling") and not job.get("done"):
                return True
    return False

def _mount_info_for_path(path):
    path = os.path.abspath(path)
    try:
        with open("/proc/self/mountinfo", "r", encoding="utf-8") as mount_file:
            for line in mount_file:
                fields = line.strip().split()
                if "-" not in fields or len(fields) < 10:
                    continue
                separator = fields.index("-")
                if separator + 2 >= len(fields):
                    continue
                mount_point = os.path.abspath(_decode_mount_path(fields[4]))
                if mount_point != path:
                    continue
                return {
                    "mountPoint": mount_point,
                    "device": _decode_mount_path(fields[separator + 2]),
                    "fsType": fields[separator + 1],
                }
    except Exception:
        pass
    return None

def _run_unmount_command(usb_root):
    mount_info = _mount_info_for_path(usb_root) or {}
    device = mount_info.get("device", "")
    commands = []
    if device.startswith("/dev/"):
        commands.append(["udisksctl", "unmount", "-b", device])
    commands.append(["gio", "mount", "-u", usb_root])
    commands.append(["umount", usb_root])

    last_error = ""
    for command in commands:
        try:
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=20)
            if result.returncode == 0:
                return command
            last_error = (result.stderr or result.stdout or "").strip()
        except FileNotFoundError:
            last_error = f"未找到命令: {command[0]}"
        except subprocess.TimeoutExpired:
            last_error = f"命令超时: {' '.join(command)}"
    raise RuntimeError(last_error or "U盘卸载失败")

def _eject_usb():
    global file_transfer_usb_root_cache
    if _has_running_transfer_jobs():
        raise RuntimeError("正在传输中，不能弹出U盘")

    usb_root = _scan_usb_root()
    if not usb_root:
        return {
            "ok": True,
            "mounted": False,
            "usbMounted": False,
            "message": "未检测到已挂载的U盘",
        }

    try:
        subprocess.run(["sync"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
        command = _run_unmount_command(usb_root)
    except OSError as e:
        if _is_io_error(e):
            _raise_usb_io_error(e, "弹出U盘失败")
        raise

    file_transfer_usb_root_cache = None
    return {
        "ok": True,
        "mounted": False,
        "usbMounted": False,
        "path": "/",
        "message": f"U盘已安全弹出: {' '.join(command)}",
    }

# ==========================================
# HTTP 微服务 (完美解决 Keep-Alive 挂起与死锁版本)
# ==========================================
class SpaceHandler(BaseHTTPRequestHandler):
    def _send_bytes(self, status, content_type, response_data):
        try:
            self.send_response(status)
            self.send_header('Content-type', content_type)
            self.send_header('Content-Length', str(len(response_data)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type')
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
            self.send_header('Connection', 'close')
            self.end_headers()
            self.wfile.write(response_data)
        except Exception as e:
            print(f"❌ HTTP 服务向下行发送数据失败: {e}")

    def _send_text(self, response_data, status=200):
        self._send_bytes(status, 'text/plain; charset=utf-8', response_data)

    def _send_json(self, payload, status=200):
        self._send_bytes(status, 'application/json; charset=utf-8', _json_bytes(payload))

    def do_OPTIONS(self):
        self._send_bytes(204, 'text/plain; charset=utf-8', b'')

    def _handle_file_transfer_get(self, parsed_path):
        query = parse_qs(parsed_path.query)
        try:
            if parsed_path.path == "/file_transfer/health":
                usb_root = _scan_usb_root()
                desktop_root = os.path.abspath(_default_desktop_root())
                self._send_json({
                    "ok": True,
                    "version": FILE_TRANSFER_SERVER_VERSION,
                    "homeRoot": os.path.abspath(FILE_TRANSFER_HOME_ROOT),
                    "desktopRoot": desktop_root,
                    "usbRoot": usb_root,
                    "usbMounted": bool(usb_root),
                    "desktopStorage": _storage_usage(desktop_root),
                    "desktop_storage": _storage_usage(desktop_root),
                    "usbStorage": _storage_usage(usb_root) if usb_root else None,
                    "usb_storage": _storage_usage(usb_root) if usb_root else None,
                })
                return

            if parsed_path.path == "/file_transfer/list":
                side = query.get("side", ["desktop"])[0]
                client_path = query.get("path", ["/"])[0]
                self._send_json(_list_transfer_dir(side, client_path))
                return

            if parsed_path.path == "/file_transfer/job":
                job_id = query.get("id", [""])[0]
                job = _get_copy_transfer_job(job_id)
                if not job:
                    self._send_json({"ok": False, "error": "任务不存在"}, 404)
                    return
                job["ok"] = True
                self._send_json(job)
                return

            self._send_json({"ok": False, "error": "接口不存在"}, 404)
        except Exception as e:
            self._send_json({"ok": False, "error": str(e)}, 500)

    def _handle_stream_get(self, parsed_path):
        try:
            if parsed_path.path == "/stream/settings":
                payload = _stream_settings_public(_load_stream_settings())
                payload["ok"] = True
                self._send_json(payload)
                return
            self._send_json({"ok": False, "error": "接口不存在"}, 404)
        except Exception as e:
            self._send_json({"ok": False, "error": str(e)}, 500)

    def do_POST(self):
        parsed_path = urlparse(self.path)
        if parsed_path.path not in ("/file_transfer/copy", "/file_transfer/transfer", "/file_transfer/eject", "/file_transfer/cancel", "/file_transfer/zip", "/file_transfer/delete", "/stream/settings"):
            self._send_json({"ok": False, "error": "接口不存在"}, 404)
            return

        try:
            content_length = int(self.headers.get('Content-Length', '0'))
            raw_body = self.rfile.read(content_length).decode('utf-8') if content_length > 0 else "{}"
            body = json.loads(raw_body)
            if parsed_path.path == "/stream/settings":
                payload = _stream_settings_public(_save_stream_settings(body))
                payload["ok"] = True
                self._send_json(payload)
                return

            if parsed_path.path == "/file_transfer/eject":
                self._send_json(_eject_usb())
                return

            if parsed_path.path == "/file_transfer/cancel":
                job = _cancel_transfer_job(body.get("id") or body.get("job_id") or "")
                job["ok"] = True
                self._send_json(job)
                return

            if parsed_path.path == "/file_transfer/zip":
                job = _start_zip_job(body.get("sources"), body.get("destination", ""))
                job["ok"] = True
                self._send_json(job)
                return

            if parsed_path.path == "/file_transfer/delete":
                job = _start_delete_job(body.get("sources"))
                job["ok"] = True
                self._send_json(job)
                return

            operation = body.get("operation") or body.get("mode") or "copy"
            job = _start_copy_transfer_job(body.get("sources"), body.get("destination", "/"), operation)
            job["ok"] = True
            self._send_json(job)
        except Exception as e:
            self._send_json({"ok": False, "error": str(e)}, 400)

    def do_GET(self):
        global global_replay_process
        response_data = b""
        parsed_path = urlparse(self.path)

        if parsed_path.path == '/healthz':
            self._send_json(_health_metrics())
            return
        if parsed_path.path.startswith('/file_transfer/'):
            self._handle_file_transfer_get(parsed_path)
            return
        if parsed_path.path.startswith('/stream/'):
            self._handle_stream_get(parsed_path)
            return

        # 1. 动态路由解析：先统一计算出需要返回的二进制文本，不急着发网络头
        if self.path.startswith('/check_pose'):
            response_data = check_pose_condition().encode('utf-8')
            
        elif self.path.startswith('/check_file'):
            response_data = check_path_file().encode('utf-8')
            
        elif self.path.startswith('/check_replay_status'):
            if global_replay_process is None:
                response_data = b"IDLE"
            else:
                ret = global_replay_process.poll()
                if ret is None:
                    status = "RUNNING"
                    try:
                        with open("/tmp/replay_status.txt", "r") as f:
                            status = "RUNNING_" + f.read().strip()
                    except: pass
                    response_data = status.encode('utf-8')
                elif ret == 0: response_data = b"DONE_SUCCESS" 
                elif ret == 2: response_data = b"DONE_ABORTED" 
                else: response_data = b"ERROR"
                
        elif self.path.startswith('/check_camera'):
            try:
                # 阻塞等待最多 1.5 秒，如果收到一帧图像则证明存在
                rospy.wait_for_message(IMAGE_TOPIC, Image, timeout=1.5)
                response_data = b"OK"
            except Exception:
                response_data = b"NO_IMAGE"

        elif self.path.startswith('/radar/status'):
            # 雷达启动状态查询
            with _radar_process_groups_lock:
                tracked = len(_radar_process_groups)
            status_parts = [
                f"tracked_groups={tracked}",
                f"starting={int(global_radar_starting)}",
                f"result={global_radar_start_result}",
                f"message={global_radar_start_message}",
            ]
            response_data = "; ".join(status_parts).encode('utf-8')

        else:
            # 🌟 默认根路径（即查询剩余内存）
            try:
                response_data = f"{get_free_space():.1f}".encode('utf-8')
            except Exception:
                response_data = b"0.0"

        # 2. 核心修复：发送标准 HTTP 响应头，精准控制生命周期
        self._send_text(response_data)

    def log_message(self, format, *args): pass
    
def start_http_server():
    print(f"✅ NX 综合监测微服务已开启 (HTTP 6004 - 多线程支持, file-transfer {FILE_TRANSFER_SERVER_VERSION})")
    # 🌟 使用 ThreadingHTTPServer 并发处理 QGC 请求
    ThreadingHTTPServer(('0.0.0.0', 6004), SpaceHandler).serve_forever()

# ==========================================
# UDP 指令接收与执行服务
# ==========================================
def start_command_server():
    global global_replay_process
    # 🌟 服务启动时按配置刷新图像话题（环境变量/雷达 profile）
    _refresh_image_topic()
    try:
        rospy.init_node('nx_qgc_bridge_daemon', anonymous=True, disable_signals=True)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    except Exception as e: pass

    threading.Thread(target=start_http_server, daemon=True).start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 6003))
    print("✅ 小鲤鱼指令接收通道已开启 (UDP 6003)")
    
    bag_process = None
    path_record_proc = None
    video_record_proc = None

    while True:
        try:
            data, addr = sock.recvfrom(2048)
            cmd = data.decode('utf-8')
            print(f"\n[{time.strftime('%H:%M:%S')}] 收到 QGC 指令: {cmd}")
            
            my_env = os.environ.copy()
            my_env["DISPLAY"] = ":0"
            my_env["DISABLE_ROS1_EOL_WARNINGS"] = "1"
            
            # 🌟 统一雷达启动：根据 Radar_Launcher.yaml 配置分发
            # 支持命令 "Odin"（不录数据）和 "Odin-MindCloud"（录数据）
            if cmd in ("Odin", "Odin-MindCloud"):
                # 异步启动：主循环立即响应，后台完成 等上一轮收尾+清理+拉起新雷达
                if not any(t.name == "bg_start_radar" and t.is_alive() for t in threading.enumerate()):
                    threading.Thread(
                        target=_handle_radar_launch,
                        args=(cmd == "Odin-MindCloud", my_env),
                        name="bg_start_radar", daemon=True).start()
                    print("⏳ 启动指令已提交，后台准备中...", flush=True)
                else:
                    print("⏳ 上一轮启动仍在进行中，忽略重复指令", flush=True)
            
            elif cmd.startswith("RecordBag:"):
                topics = cmd.split(":")[1].split()
                bag_name = time.strftime("odin_data_%Y%m%d_%H%M%S.bag")
                bag_path = f"/home/fouruav/Desktop/Bags/{bag_name}" 
                bag_process = subprocess.Popen(["rosbag", "record", "-O", bag_path] + topics)

            elif cmd == "StopBag":
                if bag_process:
                    bag_process.send_signal(signal.SIGINT)
                    bag_process.wait()
                    bag_process = None

            elif cmd == "KillLiDAR":
                # 异步关闭：界面立即响应，后台完成 保存PCD+rename+清理
                if not any(t.name == "bg_kill_lidar" and t.is_alive() for t in threading.enumerate()):
                    threading.Thread(target=kill_lidar_processes, name="bg_kill_lidar", daemon=True).start()
                    print("⏳ 关闭指令已提交，后台保存 PCD 中...", flush=True)
                else:
                    print("⏳ 上一轮关闭仍在后台进行中，忽略重复指令", flush=True)

            # 🌟 --------------------------------------------------
            # 🌟 启动视频录制
            # 🌟 --------------------------------------------------
            elif cmd == "StartVideo":
                video_script = f"""
import rospy, cv2, os, time, sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

writer = None
def cb(msg):
    global writer
    try:
        img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        if writer is None:
            folder = "/home/fouruav/Desktop/Video"
            os.makedirs(folder, exist_ok=True)
            filename = os.path.join(folder, time.strftime("odin_video_%Y%m%d_%H%M%S.mp4"))
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            writer = cv2.VideoWriter(filename, fourcc, 20.0, (img.shape[1], img.shape[0]))
            print("🎥 开始写入视频:", filename)
        writer.write(img)
    except Exception as e:
        print("❌ 写入帧时出错:", e)

rospy.init_node('video_recorder_daemon', anonymous=True)
print("⏳ 等待订阅图像话题: {IMAGE_TOPIC}")
rospy.Subscriber("{IMAGE_TOPIC}", Image, cb)
rospy.spin()
if writer is not None:
    writer.release()
    print("🛑 录像已安全保存并关闭")
"""
                with open("/tmp/video_recorder_daemon.py", "w") as f: f.write(video_script)
                bash_cmd = "source /opt/ros/noetic/setup.bash && source /home/fouruav/catkin_ws/devel/setup.bash && exec python3 /tmp/video_recorder_daemon.py > /tmp/video_log.txt 2>&1"
                video_record_proc = subprocess.Popen(bash_cmd, shell=True, executable='/bin/bash', env=my_env)

            elif cmd == "StopVideo":
                if video_record_proc:
                    video_record_proc.send_signal(signal.SIGINT)
                    video_record_proc.wait()
                    video_record_proc = None

            # 🌟 --------------------------------------------------
            # 🌟 单张拍照
            # 🌟 --------------------------------------------------
            elif cmd == "TakePhoto":
                photo_script = f"""
import rospy, cv2, os, time, sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node('photo_taker_daemon', anonymous=True)
try:
    print("⏳ 准备捕获照片...")
    msg = rospy.wait_for_message("{IMAGE_TOPIC}", Image, timeout=5.0)
    img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    folder = "/home/fouruav/Desktop/Picture"
    os.makedirs(folder, exist_ok=True)
    filename = os.path.join(folder, time.strftime("odin_photo_%Y%m%d_%H%M%S.jpg"))
    cv2.imwrite(filename, img)
    print("✅ 照片保存成功:", filename)
except Exception as e:
    print("❌ 拍照发生错误:", e)
"""
                with open("/tmp/photo_taker_daemon.py", "w") as f: f.write(photo_script)
                bash_cmd = "source /opt/ros/noetic/setup.bash && source /home/fouruav/catkin_ws/devel/setup.bash && python3 /tmp/photo_taker_daemon.py > /tmp/photo_log.txt 2>&1"
                subprocess.Popen(bash_cmd, shell=True, executable='/bin/bash', env=my_env)

            elif cmd == "RecordPathStart":
                global PATH_FILE
                # 🌟 录制直接生成时间戳文件, 归档到 data_guidepoints/, 永不覆盖
                _guide_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "data_guidepoints")
                os.makedirs(_guide_dir, exist_ok=True)
                PATH_FILE = os.path.join(_guide_dir, "guidepoints_" + time.strftime("%Y%m%d_%H%M%S") + ".txt")
                print("[RecordPathStart] 新航迹文件: " + PATH_FILE)
                recorder_script = f"""
import rospy
from geometry_msgs.msg import PoseStamped
latest_pose = None
def pose_cb(msg): global latest_pose; latest_pose = msg
rospy.init_node("path_recorder_daemon", anonymous=True, disable_signals=True)
rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_cb)
with open("{PATH_FILE}", "w") as f: f.write("x,y,z,qx,qy,qz,qw\\n")
rate = rospy.Rate(40)
while not rospy.is_shutdown():
    if latest_pose:
        p, q = latest_pose.pose.position, latest_pose.pose.orientation
        with open("{PATH_FILE}", "a") as f: f.write(f"{{p.x}},{{p.y}},{{p.z}},{{q.x}},{{q.y}},{{q.z}},{{q.w}}\\n")
        latest_pose = None
    rate.sleep()
                                    """
                with open("/tmp/path_recorder_daemon.py", "w") as f: f.write(recorder_script)
                path_record_proc = subprocess.Popen(["python3", "/tmp/path_recorder_daemon.py"], env=my_env)

            elif cmd == "RecordPathStop":
                if path_record_proc:
                    path_record_proc.terminate()
                    path_record_proc.wait()
                    path_record_proc = None
                # 🌟 同步更新 guidepoints_new.txt (始终指向最新航迹)
                try:
                    _new_file = os.path.join(os.path.dirname(PATH_FILE), "guidepoints_new.txt")
                    with open(PATH_FILE, "rb") as _src, open(_new_file, "wb") as _dst:
                        _dst.write(_src.read())
                    print("[RecordPathStop] 已更新 guidepoints_new.txt -> " + _new_file)
                    # 🌟 自动提取终点 -> endpoint.txt (方案C: waypoint_mission 只发终点, CH10 触发)
                    try:
                        _end_file = os.path.join(os.path.dirname(PATH_FILE), "endpoint.txt")
                        _last_pt = None
                        with open(_new_file, "r") as _rf:
                            _lines = [l for l in _rf if l.strip()]
                            if _lines:
                                _last_line = _lines[-1].strip().split(",")
                                if len(_last_line) >= 3:
                                    _last_pt = (float(_last_line[0]), float(_last_line[1]), float(_last_line[2]))
                        if _last_pt:
                            with open(_end_file, "w") as _ef:
                                _ef.write(f"{_last_pt[0]:.6f} {_last_pt[1]:.6f} {_last_pt[2]:.6f} 0.1\n")
                            print(f"[RecordPathStop] 已更新 endpoint.txt -> {_last_pt}")
                        else:
                            print("[RecordPathStop] 提取终点失败(文件格式?)")
                    except Exception as _e2:
                        print("[RecordPathStop] 更新 endpoint.txt 失败: " + str(_e2))
                except Exception as _e:
                    print("[RecordPathStop] 更新 guidepoints_new.txt 失败: " + str(_e))

            elif cmd == "AbortReplay":
                if global_replay_process is not None:
                    global_replay_process.terminate()
                    global_replay_process.wait()
                    global_replay_process = None
                try:
                    subprocess.Popen(["rosservice", "call", "/mavros/set_mode", "0", "POSCTL"])
                except Exception:
                    pass

            elif cmd == "RecordPathReplay":
                replay_script = f"""
import rospy, time, sys
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode

current_state = State()
def state_cb(msg): global current_state; current_state = msg

current_pose = None
def pose_cb(msg): global current_pose; current_pose = msg

rospy.init_node("path_replay_daemon", anonymous=True, disable_signals=True)
rospy.Subscriber("/mavros/state", State, state_cb)
rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_cb)
pose_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

points = []
try:
    with open("{PATH_FILE}", "r") as f:
        for l in f.readlines()[1:]:
            parts = l.strip().split(',')
            if len(parts) >= 7: points.append([float(x) for x in parts])
except: sys.exit(1)

if not points: sys.exit(1)
Z0 = points[0][2]
rate = rospy.Rate(40)


with open("/tmp/replay_status.txt", "w") as f: f.write("WAITING")


while not rospy.is_shutdown():
    pose = PoseStamped()
    pose.pose.position.z = Z0
    pose.pose.orientation.w = 1.0
    pose_pub.publish(pose)
    
    if current_state.mode == "OFFBOARD" and current_state.armed:
        break
    rate.sleep()


with open("/tmp/replay_status.txt", "w") as f: f.write("FLYING")


wait_time = 0.0
while not rospy.is_shutdown():
    if current_state.mode != "OFFBOARD": sys.exit(2) 
    
    pose = PoseStamped()
    pose.pose.position.z = Z0
    pose.pose.orientation.w = 1.0
    pose_pub.publish(pose)
    
    if current_pose is not None:
        dz = abs(current_pose.pose.position.z - Z0)
        if dz <= 0.10:  
            break
            
    wait_time += 0.025
    if wait_time >= 15.0:
        break
    rate.sleep()


for p in points:
    if current_state.mode != "OFFBOARD": sys.exit(2) 
    pose = PoseStamped()
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p[0], p[1], p[2]
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = p[3], p[4], p[5], p[6]
    pose_pub.publish(pose)
    rate.sleep()


try:
    rospy.wait_for_service('/mavros/set_mode', timeout=2.0)
    set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    set_mode(custom_mode='POSCTL')
except: pass
sys.exit(0)
                            """
                with open("/tmp/path_replay_daemon.py", "w") as f: f.write(replay_script)
                global_replay_process = subprocess.Popen(["python3", "/tmp/path_replay_daemon.py"], env=my_env)
            
            # 🌟 --------------------------------------------------
            # 🌟 新增：处理用户自定义的三大按钮指令
            # 🌟 --------------------------------------------------
            elif cmd in ["UserCmd1", "UserCmd2", "UserCmd3"]:
                if os.path.exists(USER_DEFINE_YAML_PATH):
                    try:
                        with open(USER_DEFINE_YAML_PATH, 'r', encoding='utf-8') as f:
                            user_config = yaml.safe_load(f)
                        
                        if user_config and cmd in user_config and "command" in user_config[cmd]:
                            exec_cmd = user_config[cmd]["command"]
                            print(f"🚀 执行自定义指令 [{cmd}]: {exec_cmd}")
                            # 使用与启动雷达相同的环境变量去执行指令，确保 roslaunch 和 roscore 能正常找到
                            subprocess.Popen(exec_cmd, shell=True, executable='/bin/bash', env=my_env)
                        else:
                            print(f"⚠️ YAML 中未找到 {cmd} 或 command 字段未配置。")
                    except Exception as e:
                        print(f"❌ 读取或解析 YAML 失败: {e}")
                else:
                    print(f"⚠️ 找不到自定义配置文件: {USER_DEFINE_YAML_PATH}")
                    
        except Exception as e:
            print(f"❌ 指令执行出错: {e}")

if __name__ == '__main__':
    start_command_server()
