#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from nx_touch_server import start_touch_server

if __name__ == '__main__':
    try:
        start_touch_server()
    except KeyboardInterrupt:
        print("\n✅ 服务已退出。")
    except Exception as e:
        print(f"❌ 服务发生异常: {e}")