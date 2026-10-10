# AGENTS.md — task_run_scripts 开发说明

任务运行脚本与 launch 编排仓库（ROS 1 Noetic / Jetson）。修改前请阅读。

## 定位

为无人机机载系统提供 **launch 编排**（雷达/相机/PX4/任务）与**辅助脚本**，
并作为 **NX_code 机载服务套件**的父仓库（`scripts/NX_code` 为 git submodule）。

## 目录结构

```
task_run_scripts/
├── launch/                 # 主要 launch 编排（Odin / fastlivo2 各自分文件夹解耦）
│   ├── odin/                    # Odin 方案
│   │   ├── odin1_ros1.launch            # Odin 驱动 + rviz
│   │   └── odin_px4_and_others.launch   # px4+mavros+realman+tf + Odin 里程计转发
│   ├── fastlivo2/               # Mid360/Fast-LIVO2 方案
│   │   ├── mapping_MID360.launch    # Fast-LIVO2 建图（pcd_save/rviz/config_name 参数）
│   │   ├── msg_MID360s.launch       # Livox MID360 驱动
│   │   ├── mid360_imu_tf.launch     # IMU TF
│   │   ├── rs_d435_camera.launch    # Realsense D435
│   │   └── fastlivo2_px4_and_others.launch  # px4+mavros+realman+tf（无 Odin 转发）
│   ├── px4.launch / node.launch / realman.launch
│   └── real_flight.launch / tf_to_mavros.launch / apm.launch / start_all_odin_apm.launch
├── config/                 # MID360s_config.json、realman.yaml 等
├── scripts/                # NX_code(submodule)、check_pcd.py、colorize_pcd.py、record.py 等
├── rviz/                   # slam_rviz.rviz
└── shipment_privacy_cleaner.sh
```

## 关键要点

- **建图 launch** `fastlivo2/mapping_MID360.launch`：`pcd_save` 参数决定是否输出 PCD（由 `start_fastlivo2.sh` 传入 `true/false`），`config_name` 选 `MID360_Fly_Lidar.yaml`（建图）或 `MID360_Fly.yaml`（飞行）
- **Odin 里程计转发**：`odin/odin_px4_and_others.launch` 含 `odom_throttle_node` + `odom_to_mavros_node`（订阅 `/odin1/odometry_highfreq` → `/mavros/vision_pose/pose`），**fastlivo2 版不含**
- **PX4 串口**：`px4.launch` 使用 `/dev/ttyTHS0:921600`（独占资源，重启时注意冲突）
- **NX_code 子模块**：改 `scripts/NX_code` 内容后，需在此仓库更新子模块指针并推送（见顶层 `docs/DEVELOPMENT.md`）

## 常用命令

```bash
bash scripts/NX_code/start_fastlivo2.sh [true|false]   # Mid360 雷达启停（常用入口）
bash scripts/NX_code/start_odin.sh [true|false]        # Odin 雷达启停
```

## 更多

- 子模块说明：`scripts/NX_code/AGENTS.md`
- 雷达配置：`scripts/NX_code/Radar_Launcher.yaml`
- X2 PCD 异步写盘：见顶层 `docs/RADAR_PCD_X2.md`
