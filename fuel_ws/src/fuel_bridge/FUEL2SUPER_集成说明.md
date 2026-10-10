# FUEL → SUPER 集成（仿真）操作说明

目标：**FUEL 只回答"去哪"**（FIS frontier 决策），**SUPER 负责"怎么去"**（ROG-Map + A* + MINCO 规划与避障）。

---

## 一、架构与数据流

```
FAST-LIVO2 ──/cloud_registered──┬─> FUEL(exploration_node)
                                 │     ├─ 地图: SDFMap (自己的)
                                 │     ├─ 决策: FIS frontier → next_pos/next_yaw
                                 │     ├─ [新增] 发布 /exploration_goal (PoseStamped)
                                 │     └─ traj_server → /fuel/pos_cmd_sink  (改道:输出废弃, 防抢控制)
                                 │
                                 └─> SUPER(fsm_node + super_to_px4_bridge)
                                       ├─ 地图: rog_map (自己的)
                                       ├─ 目标: 订阅 /waypoint_goal
                                       └─ 输出: /planning/pos_cmd
                                                  ↓
                                       super_to_px4_bridge
                                                  ↓ /mavros/setpoint_raw/local
                                                PX4

                 /exploration_goal ──> goal_arbiter.py ──> /waypoint_goal
                                        (限频/到达才换/防死锁)
```

## 二、为什么必须做"防抢控制"和"目标仲裁"

| 问题 | 原因 | 处理 |
|---|---|---|
| **两条控制链抢飞机** | SUPER 的 bridge **无条件接受**任何节点发的 `/planning/pos_cmd`；FUEL 的 `traj_server` 也发这个话题 | FUEL 输出 **remap 到 `/fuel/pos_cmd_sink`**（内部逻辑照跑，输出无人订阅） |
| **目标被高频打断** | FUEL 每轮 replan（约 1Hz）都产出一个新目标；直接转发会让 SUPER 的 FSM 每秒重规划 → 轨迹抖动 | **goal_arbiter** 限频：冷启动 / 已到达 / 目标失效且过最小间隔 / 保超时 四种情况才放行 |

## 三、已完成的改动（均可回滚）

### 1. FUEL 发布探索目标点（`fuel_ws/src/FUEL/fuel_planner/exploration_manager/`）

- `include/.../fast_exploration_manager.h`：`private:` 段加 `ros::Publisher explore_goal_pub_;`
- `src/fast_exploration_manager.cpp`：
  - `initialize()` 里 `advertise<geometry_msgs::PoseStamped>("/exploration_goal", 10)`
  - `std::cout << "Next view: " ...` **之后**（所有 return 之前，覆盖全部分支）发布目标点，yaw 转四元数
- 备份：`*.bak_goalpub_<时间戳>`
- **验证**（运行时，权威）：`rostopic info /exploration_goal` → `Type: geometry_msgs/PoseStamped`，`Publishers: /exploration_node` ✓

### 2. FUEL 输出改道开关（`fuel_ws/src/fuel_bridge/launch/fuel_real.launch`）

```xml
<arg name="fuel_cmd_topic" default="planning/pos_cmd"/>
...
<remap from="/position_cmd" to="$(arg fuel_cmd_topic)"/>
```
- 默认 `planning/pos_cmd`：**FUEL 单独运行时行为不变**
- 与 SUPER 联调时传：`fuel_cmd_topic:=/fuel/pos_cmd_sink`
- 备份：`*.bak_sink_<时间戳>`

### 3. 目标仲裁器（`fuel_ws/src/fuel_bridge/scripts/goal_arbiter.py`）

参数（全部可用 `_param:=value` 覆盖）：

| 参数 | 默认 | 含义 |
|---|---|---|
| `~in_topic` | `/exploration_goal` | FUEL 候选目标 |
| `~out_topic` | `/waypoint_goal` | 转发给 SUPER |
| `~odom_topic` | `/odom_world` | 当前位置（用于到达判定） |
| `~arrive_radius` | 0.5 m | 距目标多近算"到达" |
| `~goal_shift` | 1.5 m | 目标变化多大算"失效" |
| `~min_interval` | 3.0 s | 最小放行间隔 |
| `~max_hold` | 20.0 s | 最长保持，超时强制放行（**防死锁**） |
| `~dry_run` | false | true = 只打印不发布（联调前预演） |

**离线单元测试 12/12 PASS**（假 ROS 桩，覆盖限频/到达/超时/无 odom 四类场景）。

## 四、启动（一键脚本）

**前提**：Windows 上 AirSim Blocks 已点 ▶ Play（唯一手动项）；建议 PX4 已在 tmux 里跑。

```bash
bash ~/super_explore_all.sh               # 正常联调（不保存 PCD）
bash ~/super_explore_all.sh 1             # ★ 联调 + 退出时保存建图 PCD（参数 1）
bash ~/super_explore_all.sh 0             # 明确不保存（默认）
bash ~/super_explore_all.sh 1 --dry-run   # 存 PCD + 预演（仲裁器只打印不发布）
bash ~/super_explore_all.sh --no-fuel     # 只起 SUPER（单独验证 SUPER 本身）
bash ~/super_explore_all.sh 1 --no-trigger  # 存 PCD，但不自动触发探索
bash ~/super_explore_all.sh 1 5.0         # 存 PCD + 仲裁器最小放行间隔 5 秒（裸数字）
bash ~/super_explore_all.sh 1 --print-params  # ★ 只打印参数解析结果并退出（不启动任何东西）
```

**参数顺序无关** —— `pcd` 与选项谁先谁后都行（`4.5 1` 也能正确解析）。
**联调前建议先 `--print-params` 核对一次**，输出形如：
```
PARAMS pcd=1 dry=no super=1 fuel=1 trig=no min=3.0
```

**参数 1 = PCD 开关**（`1` 保存 / `0` 不保存，默认 `0`；等价选项 `--pcd`）：

> ⚠️ 点云地图**只能由 FAST-LIVO2 产生**（SUPER/rog_map 没有任何写出 PCD 的代码）。
> `PCD_SAVE=1` 会让 `twin_livo_fast.sh` 执行 `rosparam set /pcd_save/pcd_save_en true` + `interval -1`。
> 输出：`~/twin_ws/src/FAST-LIVO2/Log/pcd/all_raw_points.pcd`（原始，可达 GB 级）
> 和 `all_downsampled_points.pcd`（抽稀）。
> **必须在关闭时发 SIGINT 才落盘** → **用 `bash ~/fuel_stop.sh` 关**（它 `pkill -INT`），
> **千万不要 `kill -9`**，否则 PCD 不会保存。

脚本按 7 步执行，**每步都有硬检查 + 明确诊断**（不干等）：

| 步骤 | 内容 | 关键检查 |
|---|---|---|
| 0/7 | AirSim RPC 41451 | 不可连 → 直接退出并提示点 Play |
| 1/7 | twin 链路（FAST-LIVO2） | `/clock` 是否步进、`/cloud_registered` 是否有数据 |
| 2/7 | PX4 专用 MAVLink 14610/14611 | PX4 未跑 → 提示，但不阻塞话题级自检 |
| 3/7 | MAVROS | SUPER 的 `odom_topic` 来自 `/mavros/local_position/odom` |
| 4/7 | SUPER（fsm_node + bridge，`sim_real_flight.yaml`） | fsm_node 存活；退出则报日志路径 |
| 5/7 | FUEL（`fuel_cmd_topic:=/fuel/pos_cmd_sink` 改道） | `/exploration_goal` 是否出现 |
| 6/7 | 目标仲裁器 | 存活；支持 `--dry-run` |
| 7/7 | **链路自检** + 触发探索 | **`/planning/pos_cmd` 必须只有 1 个发布者**（防抢控制生效） |

> ⚠️ 脚本里特意叠加了 `ROS_PACKAGE_PATH`（多工作区 source 会互相覆盖，见 skill 坑 G）。

## 四·五、关闭（一键脚本）

```bash
bash ~/super_explore_stop.sh                    # 常规关闭：保留 master，PCD 会保存
bash ~/super_explore_stop.sh --all               # 全停（连 ROS master 一起）
bash ~/super_explore_stop.sh --no-save           # 不等 PCD 保存，快速关（SIGKILL）
bash ~/super_explore_stop.sh --keep-px4 --keep-mavros   # 只停上层应用，留底层链路
bash ~/super_explore_stop.sh --print-params      # 只打印参数，不关任何东西
```

按 8 步执行，**结尾会自动核对每个进程是否真的停了**：

| 步骤 | 内容 |
|---|---|
| 1/8 | SUPER（`fsm_node` + `super_to_px4_bridge`）—— 先停控制输出端 |
| 2/8 | FUEL（`exploration_node` / `traj_server` / `waypoint_generator` / 两个桥） |
| 3/8 | 目标仲裁器 `goal_arbiter` |
| 4/8 | MAVROS（`--keep-mavros` 可保留） |
| 5/8 | **FAST-LIVO2 —— SIGINT 触发 savePCD，等最多 25s 并列出 PCD 文件** |
| 6/8 | 仿真桥 + RViz |
| 7/8 | PX4（`--keep-px4` 可保留） |
| 8/8 | ROS master（默认保留；`--all` 才停） |

### ✅ 该脚本已实测通过（隔离环境，不影响你的 master）

| 测试项 | 结果 |
|---|---|
| `--print-params` 输出正确且不关任何进程 | ✅ |
| 默认关闭：八个进程全部停掉 | ✅ |
| ⭐ FAST-LIVO2 收到 **SIGINT 并触发保存**（模拟 savePCD 的 marker 文件生成） | ✅ |
| master 默认保留 | ✅ |
| `--keep-px4` / `--keep-mavros` 正确保留 | ✅ |
| `--no-save` 走 SIGKILL 且**不**触发保存 | ✅ |
| ⭐ `--all` 真的停掉 master，且**不波及常驻的 11311 master** | ✅ |

> 测试方法：用 `ln -sf /usr/bin/sleep <名字>` 造出改名的假进程 + 一个带 `trap INT` 的假 `fastlivo_mapping`，
> 在**独立 master 端口 11321** 上跑，全程不碰用户常驻的 11311。

### 手动分步（脚本出问题时的备用）
```bash
source /opt/ros/noetic/setup.bash
source ~/fuel_ws/devel/setup.bash && source ~/super_sim_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$HOME/super_sim_ws/src:$HOME/fuel_ws/src:$ROS_PACKAGE_PATH

bash ~/twin_livo_fast.sh --full                     # 1) FAST-LIVO2
rosrun mavros ... / roslaunch mavros px4.launch ... # 2) MAVROS
~/super_sim_ws/devel/lib/super_planner/fsm_node \
    _config_path:=$(rospack find task_run_scripts)/config/sim_real_flight.yaml &   # 3) SUPER
roslaunch fuel_bridge fuel_real.launch enable_ctrl:=false \
    fuel_cmd_topic:=/fuel/pos_cmd_sink &            # 4) FUEL（改道）
rosrun fuel_bridge goal_arbiter.py &                # 5) 仲裁器
```


**判读要点**
- SUPER：日志出现 `Wait for goal.`（等目标）→ 收到目标后 `Get goal at [x,y,z]` → `GENERATE_TRAJ` → `FOLLOW_TRAJ`
- 仲裁器：`[arbiter] >>> 放行目标 (x,y,z) | 原因: ...` —— **原因**能直接告诉你为什么放行
- 关键话题：`/exploration_goal`(FUEL出) `/waypoint_goal`(仲裁→SUPER) `/planning/pos_cmd`(SUPER出)
- **`/planning/pos_cmd` 应只有一个发布者（fsm_node）** —— 若出现多个，说明 FUEL 没改道成功

## 五、注意事项

- 仿真配置 `sim_real_flight.yaml` 的 `fix_map_origin` 设为 `[0,0,0]`（实机是 `[0,0,1.5]`），**需实测核对**：
  SUPER 日志 `[ProbMap] virtual_ground_height: X` 与实际地图表现（skill 记录：yaml 值 ≠ 代码实际值）
- `frontend_in_known_free: true`（未知=障碍，防撞墙）→ 在探索场景里天然配合：前沿就在已知边界
- 探索目标高度由 FUEL 决定，SUPER 侧有 `goal_z_min/max`（当前 0.6/1.3）会钳制
