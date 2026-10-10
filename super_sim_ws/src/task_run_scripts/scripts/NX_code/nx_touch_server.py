#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket
import os
import time
from pynput.mouse import Button, Controller
import mss

TOUCH_PORT = int(os.environ.get("LIYUMEN_TOUCH_PORT", "6002"))
DUPLICATE_TAP_SUPPRESS_SECONDS = float(os.environ.get("LIYUMEN_DUP_TAP_SUPPRESS_MS", "140")) / 1000.0
DUPLICATE_CLICK_SUPPRESS_SECONDS = float(os.environ.get("LIYUMEN_DUP_CLICK_SUPPRESS_MS", "1200")) / 1000.0
DUPLICATE_TAP_DISTANCE_PX = float(os.environ.get("LIYUMEN_DUP_TAP_DISTANCE_PX", "8"))
TOUCH_DEBUG = os.environ.get("LIYUMEN_TOUCH_DEBUG", "0") == "1"

def _is_near(point_a, point_b, threshold):
    if point_a is None or point_b is None:
        return False
    return abs(point_a[0] - point_b[0]) <= threshold and abs(point_a[1] - point_b[1]) <= threshold

def start_touch_server():
    mouse = Controller()
    
    # 获取真实屏幕分辨率，用于坐标映射
    with mss.mss() as sct:
        monitor = sct.monitors[1]
        screen_w = monitor["width"]
        screen_h = monitor["height"]
        offset_x = monitor["left"]
        offset_y = monitor["top"]

    # 开辟 6002 端口作为极速触控通道 (UDP)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', TOUCH_PORT))
    print(f"✅ 触控接收通道已开启 (UDP {TOUCH_PORT})，映射屏幕尺寸: {screen_w}x{screen_h}")

    left_down = False
    middle_down = False
    suppress_left_until_release = False
    left_press_time = 0.0
    left_press_pos = None
    last_tap_time = 0.0
    last_tap_pos = None
    last_left_click_time = 0.0
    last_left_click_pos = None

    while True:
            try:
                data, addr = sock.recvfrom(1024)
                # 数据格式: "类型,参数1,参数2"
                msg = data.decode('utf-8').split(',')
                event_type = int(msg[0])
                now = time.time()
                
                # ========= 滚轮缩放事件 (类型 4) =========
                if event_type == 4:
                    # msg[1] 传过来的是缩放变化量
                    scroll_dy = float(msg[1])
                    # mouse.scroll(dx, dy)，这里只用到垂直滚轮 dy
                    mouse.scroll(0, scroll_dy)
                    continue # 滚轮事件不需要处理坐标，直接跳过后续代码

                # ========= 触控拖拽事件 (类型 1, 2, 3) =========
                # 还原到 NX 屏幕的真实物理像素坐标
                nx_x = offset_x + float(msg[1]) * screen_w
                nx_y = offset_y + float(msg[2]) * screen_h
                current_pos = (nx_x, nx_y)
                if TOUCH_DEBUG:
                    print(f"[touch] type={event_type} x={nx_x:.1f} y={nx_y:.1f} from={addr}", flush=True)
                
                # 1. 移动鼠标到指定位置
                mouse.position = (nx_x, nx_y)
                
                # 2. 执行按键动作
                if event_type == 1:   # 手指按下
                    if left_down:
                        continue
                    if (now - last_tap_time <= DUPLICATE_TAP_SUPPRESS_SECONDS and
                            _is_near(current_pos, last_tap_pos, DUPLICATE_TAP_DISTANCE_PX)):
                        suppress_left_until_release = True
                        if TOUCH_DEBUG:
                            print("[touch] suppressed duplicate left press", flush=True)
                        continue
                    suppress_left_until_release = False
                    left_down = True
                    left_press_time = now
                    left_press_pos = current_pos
                    mouse.press(Button.left)
                elif event_type == 2: # 手指抬起
                    if suppress_left_until_release:
                        suppress_left_until_release = False
                        if TOUCH_DEBUG:
                            print("[touch] suppressed duplicate left release", flush=True)
                        continue
                    if left_down:
                        mouse.release(Button.left)
                        left_down = False
                        if now - left_press_time <= 0.35 and _is_near(current_pos, left_press_pos, DUPLICATE_TAP_DISTANCE_PX):
                            last_tap_time = now
                            last_tap_pos = current_pos
                elif event_type == 5: # 鼠标中键按下 (单指双击并按住)
                    if not middle_down:
                        middle_down = True
                        mouse.press(Button.middle)
                elif event_type == 6: # 鼠标中键抬起 (平移结束松手)
                    if middle_down:
                        mouse.release(Button.middle)
                        middle_down = False
                elif event_type == 7: # 鼠标右键单击 (长按触发)
                    mouse.click(Button.right, 1)
                elif event_type == 8: # 鼠标左键双击
                    mouse.click(Button.left, 2)
                    last_left_click_time = now
                    last_left_click_pos = current_pos
                elif event_type == 9: # 鼠标左键单击
                    if (now - last_left_click_time <= DUPLICATE_CLICK_SUPPRESS_SECONDS and
                            _is_near(current_pos, last_left_click_pos, DUPLICATE_TAP_DISTANCE_PX)):
                        if TOUCH_DEBUG:
                            print("[touch] suppressed delayed duplicate left click", flush=True)
                        continue
                    mouse.click(Button.left, 1)
                    last_left_click_time = now
                    last_left_click_pos = current_pos
                # event_type == 3 是手指滑动，自动移动 position 就行了
                
            except Exception as e:
                print(f"解析触控数据出错: {e}")

if __name__ == '__main__':
    start_touch_server()
