#!/bin/bash
# PX4 SITL 启动（连 AirSim 的 TCP 4560）
cd "$HOME/PX4-AirSim" || { echo "[X] 找不到 ~/PX4-AirSim"; exit 1; }
echo "[px4_sitl.sh] PX4 SITL 启动中 -> 连接 AirSim TCP 4560"
echo "[px4_sitl.sh] 日志同时写入 /tmp/px4_run.log"
make px4_sitl_default none_iris 2>&1 | tee /tmp/px4_run.log
