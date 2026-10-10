# SUPER 实机飞行调试方案

## 前提条件

- Odin1 已完成 FAST-LIO2 SLAM 标定，点云和里程计正常输出
- MAVROS 与 PX4 通信正常，可手动解锁和切换 OFFBOARD
- 遥控器已连接，可随时切回 Position/Manual 模式接管
- 螺旋桨在低速测试前不装

---

## 阶段 0：地面验证（不装螺旋桨）

**目的**：确认数据链路和控制信号正确，不涉及任何运动。

### 0.1 启动系统

```bash
# 终端1：启动实机 launch
roslaunch mission_planner real_flight.launch
```

### 0.2 逐项检查

```bash
# 检查1：MAVROS 连接状态
rostopic echo /mavros/state -n 1
# 期望：connected: true

# 检查2：Odin1 点云
rostopic echo /odin1/cloud_slam -n 1
# 期望：有 header 和 point 字段，frame_id 应为 "world"

# 检查3：里程计
rostopic echo /mavros/local_position/odom -n 1
# 期望：pose 位置合理（接近0,0,0或起飞点）

# 检查4：SUPER 规划器输出
rostopic echo /planning/pos_cmd -n 1
# 期望：position/velocity/acceleration 均有合理数值

# 检查5：Bridge 发给 PX4 的 setpoint
rostopic echo /mavros/setpoint_raw/local -n 1
# 期望：type_mask 应包含位置（不包含 IGNORE_PX/PY/PZ）
#        position 有值，velocity 有值（前馈开启），acceleration 有值

# 检查6：FSM 状态
rostopic echo /planning/state -n 1
# 或查看 ROS log，应显示 "WAIT_GOAL" 状态
```

### 0.3 手动发送目标测试（不飞行）

在 RViz 中发送一个 2D Nav Goal，观察：

```bash
# 监控 setpoint 变化
rostopic echo /mavros/setpoint_raw/local
# 期望：position 字段更新为目标位置附近
#        velocity 字段跟随轨迹方向变化
#        不应出现 NaN 或 inf
```

---

## 阶段 1：低空悬停测试（装螺旋桨，室内或无风环境）

**目的**：验证位置保持精度，不开速度前馈。

### 1.1 参数设置

```yaml
# 临时关闭前馈，纯位置控制
use_velocity_ff: false
use_accel_ff: false

# 低起飞高度
init_goal/z: 0.8

# 故障安全
failsafe_en: true
geofence_en: true
geofence:
  x_min: -2.0
  x_max: 2.0
  y_min: -2.0
  y_max: 2.0
  z_min: 0.0
  z_max: 1.5
```

### 1.2 测试步骤

1. 手动解锁，切 OFFBOARD
2. 无人机起飞到 0.8m
3. 悬停 30 秒，观察位置漂移

### 1.3 观察指标

```bash
# 记录实际位置 vs 目标位置
rostopic echo /mavros/local_position/pose
rostopic echo /mavros/setpoint_raw/local

# 用 rqt_plot 实时对比
rqt_plot /mavros/local_position/pose/pose/position/z /mavros/setpoint_raw/local/position/z
```

**通过标准**：
- 悬停位置漂移 < 0.3m
- 无明显振荡（z 方向上下抖动 < 0.1m）
- 遥控器接管正常（切回 Position 模式后立即响应）

### 1.4 如果悬停不稳

- 检查 SLAM 里程计是否跳变：`rostopic echo /mavros/local_position/odom`
- 检查点云是否正常：`rostopic echo /odin1/cloud_slam`
- 检查 PX4 EKF 是否报警：`rostopic echo /mavros/state`

---

## 阶段 2：低空悬停 + 速度前馈（室内或无风环境）

**目的**：验证前馈不会引入振荡。

### 2.1 参数设置

```yaml
use_velocity_ff: true
use_accel_ff: false
velocity_ff_gain: 0.3    # 从最低增益开始
```

### 2.2 测试步骤

1. 起飞到 0.8m 悬停
2. 观察是否出现振荡
3. 如果稳定，逐步增加增益：0.3 → 0.5 → 0.7

### 2.3 增益调整标准

| 现象 | 操作 |
|---|---|
| 悬停完全稳定，无抖动 | 增益 +0.2，继续测试 |
| 轻微抖动（肉眼可见高频振荡） | 增益 -0.1，这是当前上限 |
| 明确振荡 | 立即切回手动，增益降到 0.3 重新开始 |

### 2.4 通过标准

- `velocity_ff_gain = 0.7` 时悬停稳定，无振荡
- 如果 0.5 以上就振荡，停在稳定的最大增益值

---

## 阶段 3：前飞轨迹跟踪（室外开阔场地）

**目的**：测试直线飞行的轨迹跟踪能力。

### 3.1 参数设置

```yaml
# 使用阶段2确定的最大稳定增益
use_velocity_ff: true
use_accel_ff: false
velocity_ff_gain: <阶段2确定的值>

# 低速
max_vel: 1.0
max_acc: 3.0

# 小范围地理围栏
geofence_en: true
geofence:
  x_min: -5.0
  x_max: 5.0
  y_min: -5.0
  y_max: 5.0
  z_min: 0.0
  z_max: 2.0
```

### 3.2 测试步骤

1. 起飞到 1.2m 悬停
2. RViz 发送前方 3m 的目标点
3. 观察跟踪行为

### 3.3 观察指标

```bash
# 实时对比目标 vs 实际位置
rqt_plot /mavros/local_position/pose/pose/position/x /mavros/setpoint_raw/local/position/x

# 监控速度跟踪
rqt_plot /mavros/local_position/velocity/linear/x
```

**通过标准**：
- 跟踪误差 < 0.5m
- 无明显超调（到目标点后不会冲过头再回来）
- 停下来时能稳定悬停在目标点附近

### 3.4 如果跟踪误差大

- 考虑开启加速度前馈（进入阶段 3.5）
- 或者降低飞行速度（`max_vel: 0.8`）

### 3.5 测试加速度前馈

```yaml
use_accel_ff: true
```

重复阶段 3.2，对比开启前后的跟踪误差。如果跟踪明显改善且无振荡，保留开启。

---

## 阶段 4：转弯轨迹测试（室外）

**目的**：测试弯道跟踪和抗风能力。

### 4.1 参数设置

```yaml
# 使用前几个阶段确定的最佳参数
use_velocity_ff: true
use_accel_ff: true    # 如果阶段3验证了可以开
velocity_ff_gain: <确定的值>

max_vel: 1.5
max_acc: 4.0
```

### 4.2 测试步骤

1. 起飞到 1.2m
2. RViz 发送侧方 3m 的目标点（迫使 planner 生成转弯轨迹）
3. 观察转弯过程中的跟踪

### 4.3 测试航点飞行

```bash
# 准备测试航点（十字形路径）
# mission_planner/data/real_mission.txt:
# 4 3
# 0.0 0.0 1.2
# 3.0 0.0 1.2
# 3.0 3.0 1.2
# 0.0 3.0 1.2

# 启动时使用航点模式
roslaunch mission_planner real_flight.launch use_waypoint:=true
```

通过 RC 开关或 RViz 触发航点任务，观察自动切换。

**通过标准**：
- 能完成 4 个航点的完整飞行
- 每个航点切换距离 < 0.5m
- 全程无失控或大幅偏离

---

## 阶段 5：增益精细调优（室外有风环境）

**目的**：在真实风环境下确定最终参数。

### 5.1 测试矩阵

依次测试以下组合，每组悬停 30 秒 + 前飞 3m：

| 组号 | velocity_ff_gain | use_accel_ff | max_vel |
|---|---|---|---|
| A | 0.5 | false | 1.0 |
| B | 0.7 | false | 1.0 |
| C | 0.7 | true  | 1.0 |
| D | 0.7 | true  | 1.5 |
| E | 0.8 | true  | 1.5 |

每组记录：
- 悬停漂移距离
- 前飞跟踪误差
- 是否有振荡
- 风速（估算或用气象站）

### 5.2 最终参数确定

选择在目标风速下跟踪误差最小且无振荡的组合。

---

## 阶段 6：降落和故障安全测试

### 6.1 降落测试

```bash
# 飞行中触发降落
rosservice call /bridge/land

# 观察：
# 1. 是否开始平稳下降
# 2. 到达 land_height 后是否调用 MAVROS land
# 3. 触地后是否自动上锁
```

### 6.2 故障安全测试

```bash
# 测试1：RC信号丢失
# 飞行中关闭遥控器（或拔天线），观察：
# - 1秒后是否切换到位置保持
# - 重新打开遥控器后是否恢复

# 测试2：MAVROS断连（谨慎！）
# 飞行中 Ctrl+C 杀掉 MAVROS 节点，观察：
# - PX4 是否触发 failsafe（默认会 RTL）

# 测试3：地理围栏
# 飞到围栏边界，观察是否触发位置保持+降落
```

---

## 调试工具速查

```bash
# 实时画图对比目标 vs 实际
rqt_plot /mavros/local_position/pose/pose/position/x:y:z /mavros/setpoint_raw/local/position/x:y:z

# 查看 bridge 状态
rostopic echo /bridge/armed
rostopic echo /bridge/offboard

# 查看 PX4 健康状态
rostopic echo /mavros/state
rostopic echo /mavros/extended_state

# 录制数据用于离线分析
rosbag record -O debug_flight.bag \
  /mavros/local_position/pose \
  /mavros/setpoint_raw/local \
  /mavros/state \
  /mavros/battery \
  /planning/pos_cmd \
  /odin1/cloud_slam \
  /mavros/local_position/odom
```

---

## 炸机应急清单

1. **遥控器切 Position 模式** — 第一时间接管
2. **油门拉低** — PX4 会自动降落
3. **如果完全失控** — 切 Manual/Stabilized 模式，手动降落
4. **检查日志** — `rosbag` 录制的数据 + PX4 SD 卡 ulog
