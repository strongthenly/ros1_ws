#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from nx_command_server import start_command_server

if __name__ == '__main__':
    try:
        start_command_server()
    except KeyboardInterrupt:
        print("\n 服务已安全退出。")
    except Exception as e:
        print(f" 运行发生严重异常: {e}")