# AGENTS.md — NX_code 机载服务套件

运行于 Jetson Orin NX 的机载服务集合（QGC 遥控/地面站交互）。修改前请阅读。

## 定位

提供 **视频推流、触控交互、指令控制、文件传输** 能力，并支持 **Odin / Mid360** 雷达启停。
本仓库为 `jihaofly/NX_code`（fork 自 `Liu404mob/NX_code`），作为 `task_run_scripts` 的子模块位于 `scripts/NX_code`。

## 核心文件

| 文件 | 说明 |
|------|------|
| `nx_command_server.py` | 核心：UDP 指令(6003) + HTTP 监测/文件传输(6004) + 雷达控制 |
| `start_fastlivo2.sh` | Mid360 Fast-LIVO2 启停脚本（各节点并行启动，wait 常驻） |
| `Radar_Launcher.yaml` | 雷达启动/关闭统一配置（active_profile: mid360，可 env 覆盖） |
| `nx_touch_server.py` | 触控转鼠标（UDP 6002） |
| `rviz_full_stream.py` | 全屏推流（TCP 6000，含 ping 心跳） |
| `odom_to_mavros.py` | 里程计转发 MAVROS |

## 端口

| 端口 | 协议 | 服务 |
|------|------|------|
| 6000 | TCP | rviz_full_stream.py |
| 6002 | UDP | nx_touch_server.py |
| 6003 | UDP | nx_command_server.py 指令 |
| 6004 | HTTP | nx_command_server.py 监测/文件传输/流设置 |

## 指令（UDP 6003）

`Odin` / `Odin-MindCloud`（启雷达，见 Radar_Launcher.yaml）、`KillLiDAR`（关雷达）、
`RecordBag:`、`StopBag`、`StartVideo`/`StopVideo`、`TakePhoto`、
`RecordPathStart/Stop/Replay`、`UserCmd1/2/3`（User_define_commands.yaml）。

## 雷达启动 / 关闭

- 启动：`_handle_radar_launch(with_cloud)` 读 `Radar_Launcher.yaml` → `bash start_fastlivo2.sh [true|false]`（setsid 独立进程组）
- 关闭：`kill_lidar_processes()`（互斥锁保护；进程组 SIGINT + pattern 回退；启动前会 join 上一轮 bg_kill_lidar ≤120s）
- **X2 PCD**：建图主进程收到 SIGINT 秒退，PCD 由 fork 出的 `pcd_saver` 子进程后台写（setsid 脱离 + 改名）
- **匹配主进程用 `-x fastlivo_mappi`**（comm 15 字符截断，勿用 `-f fastlivo_mapping`，会误伤 pcd_saver）

## 关键坑

- 建图配置 `MID360_Fly_Lidar.yaml` `img_en=0` → 只写 intensity 单 PCD 文件
- 推流服务用 `pgrep -f "lib/rviz"` 之类精确 pattern，勿用裸 `rviz`（避免误杀 rviz_full_stream）
- `scripts/NX_code` 内改动后须在父仓库 `task_run_scripts` 更新子模块指针并推送

## 更多

- 端口/指令/配置详见本仓库 `README.md`
- X2 完整设计见顶层 `docs/RADAR_PCD_X2.md`
