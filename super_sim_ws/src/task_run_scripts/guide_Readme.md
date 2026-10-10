# SUPER 复飞功能使用指南（guide_Readme）

> 最后更新：2026-08-10
> 适用机器：fouruav（Orin NX + MID360S + D435 + FAST-LIVO2 + PX4）

---

## 1. 系统架构

### 1.1 复飞的两套机制

| 机制 | 入口 | 路径 | 特点 |
|---|---|---|---|
| **SUPER 航点任务（推荐）** | 按钮1/按钮3 | 航点文件 → waypoint_mission → fsm_node 规划 → bridge → PX4 | 有避障、有规划，走 `/planning/pos_cmd` |
| Path Replay（纯回放） | UDP 指令 RecordPathReplay | 直接回放路径文件 | 无避障，走 `/mavros/setpoint_position/local` |

### 1.2 数据流（SUPER 复飞）

```
航点文件 (data_guidepoints/guidepoints_new.txt)
    ↓ waypoint_mission 读取（抽稀 0.1m）
    ↓ CH10 拨杆触发（下降沿 >1500→<1500，且 pitch<1200）
/waypoint_goal
    ↓ fsm_node（SUPER 规划器，避障+轨迹优化）
/planning/pos_cmd
    ↓ super_to_px4_bridge（前馈控制）
/mavros/setpoint_raw/local
    ↓
PX4 飞控 → 无人机沿航迹飞行
```

### 1.3 桥接器（bridge）状态机

```
Phase1 预发布指令 → Phase2 解锁/切OFFBOARD(可选) → Phase3 等OFFBOARD
→ Phase4 自动起飞到 init_goal(z=1.2m)悬停 → Phase5 跟踪 SUPER 轨迹
```

---

## 2. 目录与文件

### 2.1 航点文件（核心）

| 路径 | 说明 |
|---|---|
| `task_run_scripts/data_guidepoints/guidepoints_new.txt` | **始终最新的航迹**（按钮3加载这个） |
| `task_run_scripts/data_guidepoints/guidepoints_YYYYMMDD_HHMMSS.txt` | 历史归档，每次录制一个，永不覆盖 |
| `task_run_scripts/data/real_mission.txt` | 按钮1的默认航点（空格格式 x y z 切换距离） |

### 2.2 关键配置

| 文件 | 关键参数 |
|---|---|
| `config/waypoint.yaml` | start_trigger_type=1(手动拨杆)、switch_dis=0.1、waypoint_downsample_dist=0.1 |
| `config/super_to_px4_bridge.yaml` | init_goal/z=1.2、use_velocity_ff/use_accel_ff、geofence_en=false、auto_arm/auto_offboard=false |
| `config/real_flight.yaml` | fsm 话题、rog_map(cloud_topic=/cloud_registered, odom_topic=/mavros/local_position/odom) |
| `rviz/top_down.rviz` | 已含 Waypoints(mkr) 显示 + Goal(黄色当前目标) |

### 2.3 自定义按钮（`scripts/NX_code/User_define_commands.yaml`）

| 按钮 | 功能 |
|---|---|
| UserCmd1 | 启动 SUPER（默认航点 real_mission.txt） |
| UserCmd2 | 关闭 SUPER（先切 POSCTL → SIGINT roslaunch → 兜底杀；不误杀推流/SLAM） |
| UserCmd3 | 启动 SUPER 复飞（加载 data_guidepoints/guidepoints_new.txt 最新航迹） |

---

## 3. 使用流程

### 3.1 录制航迹（时间戳归档，自动）

```bash
# 开始录制（UDP 6003）
python3 -c "import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.sendto(b'RecordPathStart', ('127.0.0.1', 6003))"

# 手持/遥控无人机沿目标路径走完（保持稳定高度！）
# 终点不要落地——保持高度直接停止

# 停止录制
python3 -c "import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.sendto(b'RecordPathStop', ('127.0.0.1', 6003))"
```

**录制纪律**：
- 起点 = 将来起飞点（复飞从当前位置规划到1号航点）
- 全程保持同一高度（如 0.75m，与 init_goal/z 一致最好）
- **终点保持高度再 Stop，不要降落到地面**（否则复飞会带地面俯冲段）

### 3.2 复飞操作（完整流程）

```
① 启动传感器+SLAM+飞控：bash start_fastlivo2.sh false
② 按按钮3（启动 SUPER，加载最新航迹）
③ 遥控器解锁 + 切 OFFBOARD → bridge 自动起飞到 1.2m 悬停
④ 悬停稳定后拨杆：CH10 先拨高(>1500)再拉低(<1500)，且 pitch<1200
   → 日志出现 "Mavros triggered." → 黄色目标点出现 → 开始沿航点飞行
⑤ 全程盯 RViz 黄色点（当前目标）移动；蓝色点=全部航点(静态)
⑥ 结束后按按钮2 关闭（自动切 POSCTL）
```

### 3.3 RViz 里怎么看

| 标记 | 颜色 | 含义 |
|---|---|---|
| Goal | 黄色 | **当前目标航点**（动态，到达后跳下一个） |
| Waypoints | 蓝色线+编号 | 全部航点（静态） |
| frontend_path | 路径线 | 规划轨迹 |

---

## 4. 关键参数说明

| 参数 | 当前值 | 含义 |
|---|---|---|
| waypoint_downsample_dist | 0.1 | 航点抽稀间距(m)，3D距离；0=不抽稀 |
| switch_dis | 0.1 | 距航点3D距离<此值=到达，切下一个(m) |
| init_goal/z | 1.2 | bridge 起飞悬停高度(m) |
| velocity_ff_gain | 0.7 | 速度前馈增益(0.5~1.0，过大抖动) |
| geofence_en | false | 地理围栏(建议真飞开启) |
| max_vel | 2.0 | 规划最大速度(m/s，建议首飞1.0) |

---

## 5. 故障排查

| 现象 | 原因 | 处理 |
|---|---|---|
| RViz 白屏/无地图 | fsm_node 没在跑，或重复启动同名顶替 | 确认仅一个 real_flight launch；ps 查 fsm_node |
| 看不到航点(蓝点) | RViz 在 waypoint_mission 发布后才订阅 | Waypoints 显示项取消勾选再勾上 |
| 拨杆不触发 | CH10 需"下降沿"(先高后低)；pitch<1200 | 确认拨杆顺序 |
| 复飞无轨迹 | 没切 OFFBOARD/没拨杆 | 按 3.2 流程走 |
| 按钮2 后推流黑屏 | 旧版按钮2 误杀 rviz_full_stream | 已修复为精确匹配 top_down.rviz |
| 录制文件带降落段 | 终点落到地面 | 重新录制，终点保持高度 |
| 航迹高度与起飞高度差 | 录制 0.75m vs init_goal 1.2m | 统一两者（改 init_goal 或重录） |

---

## 6. 备份与回滚

- 最新代码备份：`nx_command_server.py.bak_newfile`、`User_define_commands.yaml.bak_newfile`
- 回滚：`cp xxx.bak_newfile xxx`

## 7. 安全提醒

1. 首飞前：不装桨完整地面演练一遍（按钮3→切OFFBOARD→拨杆→按钮2）
2. 首次真飞：开阔场地、低速(max_vel=1.0)、低增益(velocity_ff_gain=0.3~0.5)
3. 建议开启 geofence_en=true（当前关闭）
4. 紧急接管：遥控器切 Position 优先于一切
