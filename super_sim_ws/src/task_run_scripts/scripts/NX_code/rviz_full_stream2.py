#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import sys

from rviz_full_stream import FullScreenStreamer, supervisor

if __name__ == '__main__':

    if len(sys.argv) > 1 and sys.argv[1] == '--worker':

        try:
            streamer = FullScreenStreamer()
            streamer.run()
        except Exception as e:
            sys.exit(1)
    else:

        supervisor()