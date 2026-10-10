# ros1_ws

ros1-twin（WSL, ROS Noetic）中的工作区源码：
- `ros1_ws/` 控制与工具脚本
- `twin_ws/` AirSim 孪生桥接 + FAST-LIVO2
- `fuel_ws/` FUEL 探索 + fuel_bridge（含 PATCH-v1/v2 补丁）
- `super_sim_ws/` SUPER 规划
- `scripts/` 一键启动/停止脚本

编译：在各工作区目录执行 `catkin_make`。第三方来源见 THIRD_PARTY.md。
