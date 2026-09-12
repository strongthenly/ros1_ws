# ros1_ws — ROS1 无人机数字孪生仿真工作站

在 **Windows 上跑 AirSim 高保真仿真 + 真 PX4 飞控在环**，在 **WSL2（`ros1-twin`，Ubuntu 20.04 + ROS Noetic）** 里跑 **FAST-LIVO2** 定位建图与 **MAVROS offboard** 控制。用于无人机自主探索 / 规划算法的研究与验证。

---

## 一、系统架构

```
┌─ Windows (高保真仿真层) ────────────────────────────────┐
│  UE 4.27 + AirSim + CustomDrone(X8 实机外观)             │
│  PX4 v1.11.3 在环 (TCP 4560)  +  QGC v5.1.4              │
│  传感器: TwinLidarPreview(64线) / TwinImu / D435 相机     │
│  settings: C:\Users\<user>\Documents\AirSim\settings.json│
└────────────────────┬────────────────────────────────────┘
                     │ RPC 41451
┌────────────────────▼─ WSL2: ros1-twin (ROS Noetic) ──────┐
│  twin_bridge  →  /twin/lidar/points_preview   (~10 Hz)   │
│                  /twin/imu/data_raw           (~200 Hz)  │
│                  /twin/camera/color/image_raw (~3 Hz)    │
│  FAST-LIVO2   →  /aft_mapped_to_init  (定位 ~10 Hz)      │
│                  /cloud_registered    (地图 ~10 Hz)      │
│                  /path                (轨迹)             │
│  MAVROS       →  /mavros/*  (offboard 控制 PX4)          │
│  RViz         →  /cloud_registered + /path + Odometry    │
└───────────────────────────────────────────────────────────┘
```

**数据流**：AirSim 传感器 → `twin_bridge` → FAST-LIVO2（定位+建图）→ 规划器 → MAVROS → PX4 → 无人机

---

## 二、目录结构

| 目录 | 内容 |
|---|---|
| `twin_bridge/` | 仿真桥（AirSim → ROS）。`bridge.py` 已修改：点云补 `rgb` 字段以适配 FAST-LIVO2 |
| `fastlivo_config/` | FAST-LIVO2 配置。`TwinSim*` = 仿真数据流；`TwinReal*` = 实机 bag 回放 |
| `scripts/` | 一键启动 / 关闭脚本 |
| `control/` | MAVROS offboard 控制脚本（定点、前进、机动） |
| `tools/` | 点云分析工具 |
| `docs/` | 开发记录与总结 |

---

## 三、快速开始

### 前置（Windows 侧）
1. 启动 UE4 编辑器，打开 `Blocks.uproject`，点 **Play**
2. PX4 SITL 启动（在 `ros1-twin` 里，脚本会自动起）
3. 等待 AirSim 日志出现 `Simulator connected on TCP port 4560`

### 一键启动（WSL `ros1-twin`）
```bash
bash scripts/twin_livo_rviz.sh      # 全量启动: PX4 + 桥 + FAST-LIVO2 + RViz
bash scripts/twin_livo_fast.sh      # 快速补缺: 只启动"缺失"的组件(环境已就绪时 0-2 秒)
```

> **关于启动速度**：本机 `roscore` 包装脚本要 **140 秒**！但其内部的 `rosmaster`（2s）和
> `rosout`（0s）本体都极快 —— 所以启动脚本**绕开 `roscore`，直接起 `rosmaster + rosout`**
> → 全量启动 **约 6 秒**。只要 master/PX4/桥保持常驻，`twin_livo_fast.sh` 通常 **0-2 秒**。
> 所以别随手杀 master；用完只关 RViz 即可。

### 一键关闭
```bash
bash scripts/twin_livo_stop.sh          # 默认: 只停 桥/FAST-LIVO2/RViz（保留 master+PX4 常驻 → 下次秒开）
bash scripts/twin_livo_stop.sh --all    # 彻底清场: 连 master/PX4 一起停（下次需完整启动）
```

### 查看定位
```bash
rostopic echo /aft_mapped_to_init/pose/pose/position   # 实时 x y z
rostopic hz /cloud_registered                          # 地图频率
```
> RViz 的 Fixed Frame 须为 **`camera_init`**

---

## 四、FAST-LIVO2 配置说明

| 文件 | 用途 | 关键参数 |
|---|---|---|
| `TwinSim.yaml` | 仿真数据流（`/twin/*`） | `lidar_type: 4`（PointCloud2），`img_en: 0` |
| `TwinSim_camera.yaml` | 仿真相机内参 | 加载到 `/laserMapping` 节点 |
| `TwinReal.yaml` | 实机 bag 回放（`/livox/*`） | `lidar_type: 1`（Livox Avia/CustomMsg） |
| `TwinReal_camera.yaml` | 相机内参（实机 `camera_self.yaml`） | — |

**加载方式**
```bash
rosparam load fastlivo_config/TwinSim.yaml
rosparam load fastlivo_config/TwinSim_camera.yaml /laserMapping
rosrun fast_livo fastlivo_mapping __name:=laserMapping
```

---

## 五、控制脚本（MAVROS offboard）

> 前置：MAVROS 已连上 PX4（见「六、关键配置」）
> 运行：`python3 control/xxx.py`

| 脚本 | 功能 |
|---|---|
| `hover_verify.py` | 定点验证：起飞到 (0,0,5) 悬停 20s，输出稳态漂移 |
| `move_forward.py` | 沿机头方向前进 2 m |
| `move_forward_cont.py` | 持续前进（速度/距离可调） |
| `maneuver.py` | 多段机动（前进10m → 右转90° → 前进20m） |

**实测结果**：定点悬停稳态漂移 < 5 cm；连续直线飞行 40 m 高度/航向稳定。

> ⚠️ 脚本结束停止发 setpoint 后，PX4 的 offboard 超时会触发 **AUTO.LAND**（安全行为）

---

## 六、关键配置与踩过的坑

### 1. MAVROS 端口（必须用专用 MAVLink）
默认 onboard 端口 14540 被占，MAVROS 启动即 FATAL。解法是在 PX4 控制台另开一条专用链路：
```
pxh> mavlink start -u 14610 -o 14611 -r 2000000 -t 127.0.0.1 -m onboard
```
然后 MAVROS 用：
```bash
roslaunch mavros px4.launch fcu_url:=udp://:14611@127.0.0.1:14610
```

### 2. GeographicLib geoid 数据（MAVROS 启动必需）
缺 `egm96-5.pgm` 会报 `GeographicLib exception` 并退出：
```bash
# 文件须放在 /usr/share/GeographicLib/geoids/
ls /usr/share/GeographicLib/geoids/egm96-5.pgm
```

### 3. 仿真点云须有 rgb 字段
FAST-LIVO2 的 `l515_handler`（`lidar_type: 4`）反序列化为 `pcl::PointXYZRGB`，
**XYZ-only 点云会失败**，故 `bridge.py` 里补零值 rgb 字段。

### 4. Livox 消息类型跨版本兼容
实机 bag 用 `livox_ros_driver2/CustomMsg`，与 `livox_ros_driver/CustomMsg` **md5 相同**（`e4d6829b...`），
补一个最小 `livox_ros_driver2` 消息包即可回放。

### 5. LiDAR 垂直 FOV 造成的盲区（非故障）
MID360 的真实 FOV 为 **-7° ~ +52°**（向下仅 7°）。飞行高度 h 时，
**地面/低处障碍的盲区半径 ≈ h / tan(7°)**：
- 3 m 高 → 盲区 ≈ 24 m（实测最近点 23 m，吻合）
- 0.4 m 高 → 盲区 ≈ 3 m

### 6. shell 脚本里 `pkill -f` 的自匹配陷阱
若脚本文件名包含模式串（如 `twin_livo_rviz.sh` 含 `rviz`），`pkill -f rviz` 会**杀掉脚本自己**。
改用 `pkill -x rviz`（精确进程名）或用更具体的模式。

### 7. roslaunch 报 run_id 不匹配
```
RLException: run_id on parameter server does not match declared run_id: xxx vs yyy
```
= 有**残留的 roslaunch** 占着 master 的 `/run_id`（例如之前跑过 mavros 的 roslaunch 没退干净）。
启动前必须清理**所有** roslaunch，而不是只清理目标那一个：
```bash
pkill -f roslaunch; pkill -f rosmaster; pkill -f roscore
# 等它们真正退出后再起干净的 roscore
```

### 8. 启动时序要求（LockStep 模式）
AirSim 配的是 `LockStep`，**必须先有 PX4 连上 TCP 4560，仿真才跑**；链路一断 AirSim 就回到
`Waiting for TCP connection on port 4560`，此时**传感器零数据**（桥自然也没数据）。所以：
- 启动前 **Blocks 必须处于 Play**
- PX4 启动后要**等 TCP4560 真正建立**再启桥（判据 `ss -tn | grep ':4560'`）
- 桥从启动到出数据约需 **30 秒**（等 LiDAR 初始化 + 仿真步进）
- 不要在每次启动时无条件杀掉正在跑的 PX4（重启一次要多等几十秒）

---

## 七、实机标定参数（来自 fouruav 机载）

从实机 `~/catkin_ws/src/FAST-LIVO2/config/` 读取，已写入 `fastlivo_config/`：

| 项 | 值 |
|---|---|
| 雷达—IMU 平移 | `[-0.011, -0.02329, 0.04412]` |
| 雷达—IMU 旋转 | 单位阵 |
| 相机—雷达 旋转 Rcl | `[-0.000601,-0.999899,0.014194, -0.006397,-0.014190,-0.999879, 0.999979,-0.000691,-0.006388]` |
| 相机—雷达 平移 Pcl | `[0.011720, -0.078665, -0.048631]` |
| 相机内参 | fx=893.68, fy=892.146, cx=382.965, cy=301.193, 800×600 |
| IMU 噪声 | acc=0.5, gyr=0.3, b_acc=1e-4, b_gyr=1e-4 |
| 实机话题 | `/livox/lidar`, `/livox/imu_tf`, `/usb_cam/image_raw` |

---

## 八、环境版本

| 组件 | 版本 |
|---|---|
| Unreal Engine | 4.27.2 |
| AirSim | 1.8.1（Blocks） |
| PX4 | v1.11.3（AirSim 集成须 v1.11.x） |
| QGC | v5.1.4 |
| WSL 发行版 | `ros1-twin`（Ubuntu 20.04 + ROS Noetic） |
| FAST-LIVO2 | `~/twin_ws/src/FAST-LIVO2` |
| MAVROS | ROS Noetic 官方包 |

---

## 九、下一步

- [ ] 接规划器（EGO-Planner / FAST-Planner）：`/aft_mapped_to_init` → 规划器 → `/mavros/setpoint_position/local`
- [ ] 接入探索算法（frontier / FUEL）
- [ ] LiDAR FOV 方案：保持实机 FOV（高保真）或放宽垂直 FOV（便于仿真探索）
