#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import socket
import struct
import cv2
import numpy as np
import mss
import psutil
import os
import time
import re
import json
from datetime import datetime
import sys
import subprocess
import traceback
import threading  
import signal    

def configure_display_environment():
    os.environ.setdefault("DISPLAY", ":0")

configure_display_environment()


HEARTBEAT_FILE = "/dev/shm/rviz_stream_heartbeat.txt"
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

class FullScreenStreamer:
    def __init__(self):
        rospy.init_node('full_screen_streamer', disable_signals=True) # 禁用信号，防止被 ROS 误杀
        self.port = 6000
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', self.port))
        self.server_socket.listen(1)
        self.client_conn = None

        # 🌟 集成的心跳服务：自动从推流客户端获取 QGC IP，无需手动配置
        self.qgc_ip = None
        self.qgc_udp_port = int(os.environ.get("LIYUMEN_QGC_UDP_PORT", "6004"))
        self.qgc_ip_file = os.environ.get("LIYUMEN_QGC_IP_FILE", "/dev/shm/qgc_ip.txt")

        # 用于双线程通信的"最新帧缓存池"和"线程锁"
        self.latest_frame = None
        self.frame_lock = threading.Lock()

        print(f"[{datetime.now().strftime('%H:%M:%S')}] 等待遥控器连接 (端口 {self.port})...")
        self.stats_lock = threading.Lock()
        self.system_stats = {
            "cpu_usage": None,
            "cpu_temp": None,
            "gpu_usage": None,
            "mem_percent": None,
            "updated_at": 0.0,
        }
        self.settings_lock = threading.Lock()
        self.stream_settings = dict(STREAM_DEFAULT_SETTINGS)
        self.settings_mtime = 0.0
        self.settings_last_check = 0.0

        print(f"[{datetime.now().strftime('%H:%M:%S')}] 系统状态缓存与截图线程就绪 (监听端口 {self.port})...")

    def _sanitize_stream_settings(self, candidate, base=None):
        settings = dict(STREAM_DEFAULT_SETTINGS)
        if isinstance(base, dict):
            settings.update(base)
        if not isinstance(candidate, dict):
            candidate = {}

        if "enabled" in candidate:
            settings["enabled"] = bool(candidate.get("enabled"))
        if "overlayEnabled" in candidate or "overlay_enabled" in candidate:
            settings["overlayEnabled"] = bool(candidate.get("overlayEnabled", candidate.get("overlay_enabled")))

        try:
            width = int(candidate.get("width", settings["width"]))
            height = int(candidate.get("height", settings["height"]))
        except Exception:
            width, height = int(settings["width"]), int(settings["height"])
        if (width, height) not in STREAM_ALLOWED_RESOLUTIONS:
            width, height = STREAM_DEFAULT_SETTINGS["width"], STREAM_DEFAULT_SETTINGS["height"]
        settings["width"] = width
        settings["height"] = height

        try:
            jpeg_quality = int(candidate.get("jpegQuality", candidate.get("jpeg_quality", settings["jpegQuality"])))
        except Exception:
            jpeg_quality = int(settings["jpegQuality"])
        settings["jpegQuality"] = max(35, min(85, jpeg_quality))

        try:
            fps = int(candidate.get("fps", settings["fps"]))
        except Exception:
            fps = int(settings["fps"])
        settings["fps"] = max(8, min(25, fps))
        return settings

    def load_stream_settings(self, force=False):
        now = time.time()
        if not force and now - self.settings_last_check < 0.5:
            with self.settings_lock:
                return dict(self.stream_settings)
        self.settings_last_check = now

        try:
            mtime = os.path.getmtime(STREAM_SETTINGS_PATH)
        except FileNotFoundError:
            with self.settings_lock:
                self.stream_settings = dict(STREAM_DEFAULT_SETTINGS)
                self.settings_mtime = 0.0
                return dict(self.stream_settings)
        except Exception:
            with self.settings_lock:
                return dict(self.stream_settings)

        if not force and mtime == self.settings_mtime:
            with self.settings_lock:
                return dict(self.stream_settings)

        try:
            with open(STREAM_SETTINGS_PATH, "r", encoding="utf-8") as settings_file:
                data = json.load(settings_file)
            with self.settings_lock:
                self.stream_settings = self._sanitize_stream_settings(data, self.stream_settings)
                self.settings_mtime = mtime
                return dict(self.stream_settings)
        except Exception:
            with self.settings_lock:
                return dict(self.stream_settings)

    def _pick_temperature(self, temperatures, sensor_names):
        # 1. 直接读取 Jetson sysfs 的 CPU 或核心温度
        try:
            import glob, os
            for tz in glob.glob("/sys/class/thermal/thermal_zone*"):
                with open(os.path.join(tz, "type"), "r") as f:
                    tz_type = f.read().strip()
                if tz_type in ["CPU-therm", "tj-therm", "soc-thermal", "cpu-thermal"]:
                    with open(os.path.join(tz, "temp"), "r") as f:
                        return int(f.read().strip()) / 1000.0
        except Exception:
            pass

        # 2. 从 psutil 回退 (仅限我们已知的 sensor)
        for sensor_name in sensor_names:
            entries = temperatures.get(sensor_name)
            if not entries:
                continue

            for entry in entries:
                if entry.current is not None:
                    return float(entry.current)

        return None

    def _format_temp(self, value):
        if value is None:
            return "N/A"
        return f"{value:.1f}C"

    def _read_gpu_usage(self):
        for path in (
            "/sys/devices/platform/17000000.ga10b/load",
            "/sys/devices/17000000.ga10b/load",
            "/sys/devices/gpu.0/load",
        ):
            try:
                with open(path, "r") as f:
                    return float(f.read().strip()) / 10.0
            except (OSError, ValueError):
                continue
        return None

    def stats_thread_worker(self):
        while not rospy.is_shutdown():
            stats = {"cpu_usage": None, "cpu_temp": None, "gpu_usage": None, "mem_percent": None, "updated_at": time.time()}
            try:
                stats["cpu_usage"] = float(psutil.cpu_percent(interval=0.1))
            except Exception:
                pass
            try:
                stats["mem_percent"] = float(psutil.virtual_memory().percent)
            except Exception:
                pass
            try:
                temperatures = psutil.sensors_temperatures()
            except Exception:
                temperatures = {}
            try:
                stats["cpu_temp"] = self._pick_temperature(temperatures, ["cpu-thermal", "soc-thermal", "tj-therm"])
            except Exception:
                pass
            try:
                stats["gpu_usage"] = self._read_gpu_usage()
            except Exception:
                pass
            with self.stats_lock:
                self.system_stats.update(stats)
            time.sleep(1.0)

    def draw_system_overlay(self, frame):
        with self.stats_lock:
            stats = self.system_stats.copy()

        cpu_usage_text = "N/A" if stats["cpu_usage"] is None else f"{stats['cpu_usage']:.0f}%"
        gpu_usage_text = "N/A" if stats["gpu_usage"] is None else f"{stats['gpu_usage']:.0f}%"
        mem_percent_text = "N/A" if stats["mem_percent"] is None else f"{stats['mem_percent']:.0f}%"
        lines = [
            f"CPU  {cpu_usage_text} ",
            f"GPU  {gpu_usage_text}",
            f"MEM  {mem_percent_text}",
            f"temp {self._format_temp(stats['cpu_temp'])}",
        ]

        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.55
        thickness = 1
        padding = 10
        line_height = 22

        widths = []
        for line in lines:
            (text_width, _text_height), _baseline = cv2.getTextSize(line, font, scale, thickness)
            widths.append(text_width)

        box_width = max(widths) + padding * 2
        box_height = line_height * len(lines) + padding * 2 - 2

        overlay = frame.copy()
      
        frame_h = frame.shape[0]
        x0 = 16
        y0 = max(8, frame_h - box_height - 16)
        x1 = x0 + box_width
        y1 = y0 + box_height
        cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)

        for index, line in enumerate(lines):
            y = y0 + padding + index * line_height + 14
            cv2.putText(frame, line, (x0 + padding, y), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)

    def update_heartbeat(self):
        try:
            with open(HEARTBEAT_FILE, "w") as f:
                f.write(str(time.time()))
        except:
            pass

    def wait_for_client(self):
        while not rospy.is_shutdown():
            self.update_heartbeat() 
            try:
                self.server_socket.settimeout(1.0)
                conn, addr = self.server_socket.accept()
                
                conn.settimeout(0.5) 
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                # 进一步极限压缩发送缓存，逼迫 TCP 放弃排队！
                conn.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
                
                self.client_conn = conn
                # 🌟 自动记录 QGC 客户端 IP，供集成的 ping 心跳服务使用（无需手动配置）
                self.qgc_ip = addr[0]
                self._update_qgc_ip_file()
                print(f"[{datetime.now().strftime('%H:%M:%S')}] ✅ QGC 遥控器已连接: {addr} (心跳目标: {self.qgc_ip}:{self.qgc_udp_port})")
                break
            except socket.timeout:
                continue
            except Exception as e:
                print(f"网络监听异常: {e}")
                time.sleep(1.0)

    def _update_qgc_ip_file(self):
        """把当前 QGC IP 写入共享文件（/dev/shm/qgc_ip.txt），供其他服务动态读取"""
        try:
            os.makedirs(os.path.dirname(self.qgc_ip_file), exist_ok=True)
            with open(self.qgc_ip_file, "w") as f:
                f.write(self.qgc_ip or "")
        except Exception:
            pass

    # 🌟 集成的心跳/链路监控线程：ping 已连接的 QGC 并把延迟上报
    #   目标 IP 自动取自推流客户端，无需每次手动配置
    def ping_thread_worker(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ping_re = re.compile(r'(?:time=|时间=)([\d\.]+)')
        while not rospy.is_shutdown():
            target_ip = self.qgc_ip
            if not target_ip:
                # 客户端尚未连接，静默等待
                time.sleep(0.5)
                continue

            ping_ms = 999
            try:
                output = subprocess.check_output(
                    f"ping -c 1 -W 1 {target_ip}",
                    shell=True, text=True, stderr=subprocess.STDOUT, timeout=3,
                )
                match = ping_re.search(output)
                if match:
                    ping_ms = int(float(match.group(1)))
                else:
                    print(f"⚠️ 未知 ping 输出格式:\n{output}")
            except subprocess.CalledProcessError:
                # 断网或超时，判定为 999 严重断连（静默，避免刷屏）
                ping_ms = 999
            except Exception as e:
                ping_ms = 999
                print(f"ping 异常: {e}")

            try:
                msg = f"PING:{ping_ms}"
                sock.sendto(msg.encode('utf-8'), (target_ip, self.qgc_udp_port))
            except Exception:
                pass

            try:
                with self.stats_lock:
                    stats = self.system_stats.copy()
                cpu = "N/A" if stats["cpu_usage"] is None else f"{stats['cpu_usage']:.0f}"
                gpu = "N/A" if stats["gpu_usage"] is None else f"{stats['gpu_usage']:.0f}"
                mem = "N/A" if stats["mem_percent"] is None else f"{stats['mem_percent']:.0f}"
                temp = "N/A" if stats["cpu_temp"] is None else f"{stats['cpu_temp']:.1f}"
                sock.sendto(f"STATS:{cpu},{gpu},{mem},{temp}".encode('utf-8'), (target_ip, self.qgc_udp_port))
            except Exception:
                pass

            time.sleep(0.4)  # 一秒 5 次

    # 专门负责在后台"疯狂截图并覆盖旧图"的独立线程
    def capture_thread_worker(self):
        with mss.mss() as sct:
            monitor_index = 1 if len(sct.monitors) > 1 else 0
            try:
                monitor_index = int(os.environ.get("LIYUMEN_MONITOR_INDEX", monitor_index))
            except Exception:
                monitor_index = 1 if len(sct.monitors) > 1 else 0
            if monitor_index < 0 or monitor_index >= len(sct.monitors):
                monitor_index = 1 if len(sct.monitors) > 1 else 0
            monitor = sct.monitors[monitor_index]
            print(
                f"[screen] 截图线程启动 monitors[{monitor_index}] "
                f"{monitor.get('width', 0)}x{monitor.get('height', 0)}"
                f"+{monitor.get('left', 0)}+{monitor.get('top', 0)}",
                flush=True,
            )
            while not rospy.is_shutdown():
                try:
                    # 🌟 无客户端连接时休眠，不截图不编码，避免空转占用 CPU
                    if self.client_conn is None:
                        with self.frame_lock:
                            self.latest_frame = None
                        time.sleep(0.5)
                        continue

                    settings = self.load_stream_settings()
                    if not settings.get("enabled", True):
                        with self.frame_lock:
                            self.latest_frame = None
                        time.sleep(0.2)
                        continue

                    sct_img = sct.grab(monitor)
                    img = np.array(sct_img)
                    img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR) 

                    width = int(settings.get("width", 960))
                    height = int(settings.get("height", 540))
                    resized_img = cv2.resize(img, (width, height), interpolation=cv2.INTER_LINEAR)
                    if settings.get("overlayEnabled", False):
                        self.draw_system_overlay(resized_img)

                    jpeg_quality = int(settings.get("jpegQuality", 50))
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
                    _, encoded_img = cv2.imencode('.jpg', resized_img, encode_param)
                    
                    jpeg_bytes = encoded_img.tobytes()
                    size = len(jpeg_bytes)
                    header = struct.pack('<I', size)
                    
                    # 核心：永远用最新的一帧无情覆盖掉旧数据
                    with self.frame_lock:
                        self.latest_frame = header + jpeg_bytes
                        
                    fps = max(1, int(settings.get("fps", 25)))
                    time.sleep(max(0.001, 1.0 / float(fps))) 
                except Exception as e:
                    time.sleep(0.1)

    def run(self):
        # 启动截图子线程
        capture_thread = threading.Thread(target=self.capture_thread_worker, daemon=True)
        capture_thread.start()

        stats_thread = threading.Thread(target=self.stats_thread_worker, daemon=True)
        stats_thread.start()

        # 🌟 启动集成的 ping 心跳线程（自动获取 QGC IP，无需手动配置）
        ping_thread = threading.Thread(target=self.ping_thread_worker, daemon=True)
        ping_thread.start()

        while not rospy.is_shutdown():
            try:
                if self.client_conn is None:
                    self.wait_for_client()
                
                while not rospy.is_shutdown():
                    self.update_heartbeat() 
                    
                    if self.client_conn is None:
                        time.sleep(0.1)
                        break # 跳出内层循环去重新等待连接

                    # 🌟 推流开关：enabled=false 时主动断开客户端连接，
                    #   让遥控器端真正停止接收（黑屏/断连），而不是停在最后一帧
                    stream_settings = self.load_stream_settings()
                    if not stream_settings.get("enabled", True):
                        print(f"🛑 推流已关闭 (enabled=false)，断开客户端连接...")
                        if self.client_conn:
                            try: self.client_conn.shutdown(socket.SHUT_RDWR)
                            except: pass
                            try: self.client_conn.close()
                            except: pass
                        self.client_conn = None
                        break  # 跳出内层循环，回到 wait_for_client 等待重新连接

                    # --- 阶段 2：只管发，不截图 ---
                    frame_to_send = None
                    with self.frame_lock:
                        if self.latest_frame is not None:
                            frame_to_send = self.latest_frame
                            # 拿走发送后，把池子清空，防止把同一张老图发两遍
                            self.latest_frame = None 
                            
                    if frame_to_send is None:
                        # 如果还没截好新图，微秒级等待，绝不阻塞网络
                        time.sleep(0.005)
                        continue

                    try:
                        self.client_conn.sendall(frame_to_send)
                        # 发送频率控制 (约 60 FPS)
                        time.sleep(0.015) 
                    except Exception as e:
                        print(f"⚠️ 网络波动，断开客户端... ({e})")
                        if self.client_conn:
                            try: self.client_conn.shutdown(socket.SHUT_RDWR)
                            except: pass
                            try: self.client_conn.close()
                            except: pass
                        self.client_conn = None
                        break # 跳出去重连

            except Exception as e:
                print(f"FATAL: 捕获到底层致命错误，准备自尽以求重生:\n{traceback.format_exc()}")
                sys.exit(1)

# =====================================================================
# 🛡️ 终极隔离守护神 (🌟 修复 4：找回看门狗 Watchdog 强杀逻辑)
# =====================================================================
def write_supervisor_log(msg):
    log_dir = os.environ.get(
        "LIYUMEN_LOG_DIR",
        os.path.join(os.path.expanduser("~"), ".cache", "liyumen"),
    )
    try:
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
    except Exception:
        log_dir = "/tmp"
    log_file = os.path.join(log_dir, "screen_full_rviz.txt")
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    full_msg = f"[{timestamp}] {msg}"
    print(full_msg)
    try:
        with open(log_file, "a") as f:
            f.write(full_msg + "\n")
    except:
        pass

def supervisor():
    # 护盾全开：拦截操作系统的常规误杀信号
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)

    write_supervisor_log("🛡️ Python 看门狗守护神已启动，开始监控双线程图传...")
    while True:
        try:
            # 启动前初始化心跳文件
            with open(HEARTBEAT_FILE, "w") as f: f.write(str(time.time()))
            
            # preexec_fn=os.setsid 是灵魂所在！它把子进程放进隔离进程组，彻底免疫 X11 的连坐击杀
            worker = subprocess.Popen(
                [sys.executable, sys.argv[0], '--worker'], 
                preexec_fn=os.setsid
            )
            
            # 核心：看门狗主动轮询监控
            while True:
                # 检查打工仔是不是自然死亡或崩溃了
                ret = worker.poll()
                if ret is not None:
                    write_supervisor_log(f"⚠️ 惊骇！图传进程崩溃 (退出码: {ret})，准备重启...")
                    break
                
                # 检查打工仔是不是"假死/冻结"了
                try:
                    if os.path.exists(HEARTBEAT_FILE):
                        with open(HEARTBEAT_FILE, "r") as f:
                            last_hb = float(f.read())
                        
                        # 如果心跳超过 4.0 秒没更新，判定为底层 X11 严重死锁！
                        if time.time() - last_hb > 4.0:
                            write_supervisor_log("🚨 严重异常：检测到图传进程死锁 (Heartbeat Timeout)！执行雷霆强杀 (kill -9)！")
                            worker.kill() # 无情斩首
                            worker.wait() 
                            break
                except Exception as hb_err:
                    pass
                
                time.sleep(1.0) # 每秒查一次岗
                
            time.sleep(1.0) # 死亡后给系统 1 秒钟的喘息时间释放端口，然后无限重生
            
        except Exception as e:
            write_supervisor_log(f"守护神自身异常: {e}")
            time.sleep(1.0)



def run_full_stream():
    """入口：核心分流逻辑（供编译为 .so 后由启动器调用）"""
    # 核心分流逻辑
    if len(sys.argv) > 1 and sys.argv[1] == '--worker':
        # 如果是子进程（打工仔），执行真正的图传逻辑
        try:
            streamer = FullScreenStreamer()
            streamer.run()
        except Exception as e:
            sys.exit(1)
    else:
        # 如果是直接运行该脚本，化身为父进程（守护神）
        supervisor()


if __name__ == '__main__':
    run_full_stream()
