#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/liyan/twin_ws/devel/setup.bash
base="$(cd "$(dirname "$0")/.." && pwd)"
python3 "$base/tests/test_interfaces.py"
export ROS_MASTER_URI=http://127.0.0.1:11329
export ROS_HOSTNAME=localhost
mkdir -p /home/liyan/twin_ws/logs
roscore -p 11329 > /home/liyan/twin_ws/logs/test-roscore.log 2>&1 & master_pid=$!
python3 "$base/tests/mock_airsim.py" > /home/liyan/twin_ws/logs/test-mock.log 2>&1 & mock_pid=$!
bridge_pid=''
cleanup() { test -z "$bridge_pid" || kill "$bridge_pid" 2>/dev/null || true; kill "$master_pid" "$mock_pid" 2>/dev/null || true; }
trap cleanup EXIT
sleep 3
rosrun twin_bridge bridge.py _host:=127.0.0.1 _port:=42459 > /home/liyan/twin_ws/logs/test-bridge.log 2>&1 & bridge_pid=$!
sleep 2
rosrun twin_bridge check_streams.py _duration:=5 | tee /home/liyan/twin_ws/logs/stream-test.json
echo SYNTHETIC_PROTOCOL_TESTS_PASSED
