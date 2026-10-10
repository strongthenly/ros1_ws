# ROS1 无人机数字孪生工作区

目标：Windows UE4.27/AirSim + WSL `ros1-twin` Ubuntu20.04/ROS Noetic + 官方 FAST-LIVO2。

```mermaid
flowchart LR
  UE[UE4/AirSim] -->|RPC| Bridge[twin_bridge]
  Bridge --> Preview[/twin 预览话题]
  Preview --> Gate{标定与逐点时间验证}
  Gate -->|通过| LIVO[FAST-LIVO2]
  LIVO --> SafePose[/twin/fast_livo/vision_pose_preview]
  LIVO -->|控制配置通过| MAVROS[MAVROS]
  MAVROS <--> PX4[PX4]
```

本阶段已完成 ROS1 基础环境安装、数据接口代码、AirSim 传感器启用和 Windows 侧传感器烟测。AirSim 通用雷达用于接口调试，不是 MID360 扫描物理模型。它没有逐点采样时间、真实线号及反射强度，不能伪造这些字段交给 FAST-LIVO2。普通 D435 无内置 IMU；当前桥接的 IMU 是 AirSim 机体 IMU，不能当作已标定的 MID360 IMU。

## 使用

1. 在普通 Windows PowerShell 中运行 `continue-setup.ps1`，完成 Sophus、FAST-LIVO2 和工作区编译。
2. 运行 `verify-platform.ps1`；通过标志为 `PLATFORM_VERIFY_COMPLETE`。
3. 关闭并重新启动 Blocks，使当前 `Documents/AirSim/settings.json` 中的传感器配置生效。
4. 运行 `start-sensor-preview.ps1`。脚本默认自动解析 Windows 主机地址，也可用 `-AirSimHost 172.x.x.x` 指定。
5. 另开 WSL 终端，运行 `rosrun twin_bridge check_streams.py`。

ROS 工作区：`~/twin_ws`。`source ~/twin_ws/devel/setup.bash`。

- `roslaunch twin_bridge bridge.launch host:=WINDOWS_HOST_IP`：启动接口预览桥；默认不向飞控写控制命令。
- `rosrun twin_bridge check_streams.py`：检测消息频率、时间戳和尺寸。
- AirSim 的活动配置已加入 `TwinImu`、`TwinLidarPreview` 和 640×480 RGB/深度输出；启用前备份位于 `C:\Users\liyan\Documents\AirSim\settings.before-twin-sensors.20260910-170629.json`。
- `sensor-smoke-test.json` 记录了独立离屏实例的传感器通过结果。
- FAST-LIVO2 接入必须先完成 `config/calibration.pending.yaml` 与带逐点时间的雷达数据源，不能用通用 XYZ 点云冒充 Livox 数据。
- 标定完成后，`roslaunch twin_bridge validated_sim_inputs.launch` 才会把 IMU、图像和严格校验后的 Livox 消息接入算法标准话题。
- MID360/D435 参数模板是 `config/fast_livo.pending.yaml` 和 `config/camera_d435.pending.yaml`。启动算法：`rosrun twin_bridge start_fastlivo.py --calibration ... --algorithm-config ... --camera-config ...`。任一关键参数未验证时脚本会拒绝启动；默认把上游的 MAVROS 位姿输出隔离到 `/twin/fast_livo/vision_pose_preview`。
- PX4/MAVROS 参数填写到 `config/control.pending.yaml`；验证端口无冲突后将 `status` 改为 `verified`，再运行 `rosrun twin_bridge start_mavros.py --config ...`。
- 需要把 FAST-LIVO2 位姿反馈给 PX4 时，启动算法额外传入 `--control-config ~/twin_ws/config/control.pending.yaml`；只有控制配置和失效保护均已验证才会解除话题隔离。
- 首次使用 MAVROS 前运行一次 `install-mavros-data.ps1`，安装 GeographicLib 地理数据。

## 数据接口

| 话题 | 类型 | 来源与限制 |
|---|---|---|
| /clock | rosgraph_msgs/Clock | AirSim 传感器时间，禁止混用墙钟 |
| /twin/imu/data_raw | sensor_msgs/Imu | AirSim 机体 FRD 转 FLU，无姿态真值 |
| /twin/lidar/points_preview | sensor_msgs/PointCloud2 | 通用雷达 XYZ，局部 FLU；不是 Livox CustomMsg |
| /twin/camera/color/image_raw | sensor_msgs/Image | AirSim RGB8；模拟理想针孔 |
| /twin/camera/color/camera_info | sensor_msgs/CameraInfo | 来自模拟分辨率/FOV；非实机内参 |
| /twin/camera/depth/image_raw | sensor_msgs/Image | 理想平面深度，32FC1 米；非 D435 噪声模型 |

预览话题单独命名，不覆盖实机话题。真实传感器 TF 暂不发布，避免假外参污染算法。
外部算法的控制闭环、MAVROS 端口和里程计回灌需在确认 PX4 当前 MAVLink 拓扑后配置，防止与已有控制链路争用。

## 尚需实机信息

MID360 型号/固件、点频和帧率；算法 IMU 来源与频率；雷达-IMU、雷达-相机、机体-传感器外参及变换方向；D435 内参/畸变/分辨率/曝光/帧率；时间同步与偏移；质量、惯量、轴距、推力曲线；PX4 版本和估计器设置。

当前 Codex 执行沙箱无法再次访问 WSL 服务（`Wsl/Service/E_ACCESSDENIED`），所以 Linux 工作区的最后编译与 ROS 话题测试由上述两个 PowerShell 脚本承接；ROS Noetic 软件包安装日志已出现 `ROS1_BASE_SETUP_COMPLETE`。

FAST-LIVO2 上游：https://github.com/hku-mars/FAST-LIVO2
Ubuntu 镜像：https://cloud-images.ubuntu.com/releases/focal/release/
ROS Noetic 已结束常规维护，此环境用于兼容 ROS1 算法开发。
