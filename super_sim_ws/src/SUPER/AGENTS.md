# AGENTS.md — SUPER 无人机路径规划与任务执行

无人机路径规划与航点任务执行仓库（ROS 1 Noetic / Jetson）。修改前请阅读。

## 定位

为四旋翼提供**路径规划（FSM）、航点任务执行、RViz 交互**能力，订阅里程计/位姿并输出位置指令给飞控。

- 默认分支：`dev/4uav`（实机飞行分支），推送目标为 `origin/dev/4uav`

## 目录结构

```
SUPER/
├── super_planner/          # 主规划包
│   ├── config/             # real_flight.yaml（实机飞行配置，常用）
│   ├── launch/ rviz/ src/ include/ Apps/ ros/ scripts/
├── mission_planner/        # 任务/航点相关
├── mars_uav_sim/           # 仿真
├── rog_map/ misc/ scripts/
└── README.md
```

## 关键配置 `super_planner/config/real_flight.yaml`

```yaml
fsm:
  click_goal_en: true        # RViz 点击目标
  click_goal_topic: "/move_base_simple/goal"
  waypoint_goal_en: true     # 航点任务目标订阅（waypoint_mission 发布，不覆盖高度）
  waypoint_goal_topic: "/waypoint_goal"
  click_height: 1.0          # 点击目标固定高度(m)
  click_yaw_en: true
  cmd_topic: "/planning/pos_cmd"
```

> 修改配置注意 `waypoint_goal_en` 与 `click_goal_en` 的通道语义：
> 航点目标不覆盖高度（跟随录制坐标），点击目标用 `click_height` 固定高度。

## 常用命令

```bash
# 启动实机任务（通过 QGC UserCmd1 或手动）
roslaunch mission_planner real_flight.launch
```

## 注意

- 本仓库改动后推送 `origin dev/4uav`，并在顶层 `catkin_ws` 更新 `src/SUPER` 子模块指针
- 具体 FSM/航点逻辑改动前先读 `super_planner/src` 对应源码与 `README.md`
