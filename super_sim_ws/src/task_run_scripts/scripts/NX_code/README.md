# NX_code — NX 机载服务套件（QGC 远程操控）

本仓库是运行在 **Jetson Orin NX** 上的机载服务集合，为 QGroundControl（QGC）遥控器/地面站提供**视频推流、触控交互、指令控制、文件传输**等能力，并支持 **Odin / Mid360** 两种激光雷达的启动与关闭。

---

## 📁 目录结构

```
NX_code/
├── .github/workflows/build-deploy.yml  # ⭐ CI：push master 自动编译并发布 deploy 分支
├── nx_command_server.py     # 核心服务：UDP 指令通道(6003) + HTTP 综合监测(6004)
├── nx_touch_server.py       # 触控转鼠标服务（UDP 6002）
├── rviz_full_stream.py      # 全屏截图推流服务（TCP 6000，带资源覆盖）
├── odom_to_mavros.py        # 里程计转发到 MAVROS
├── start_qgc_servers.sh     # 自启脚本：启动 roscore + 三大微服务
├── start_fastlivo2.sh       # Mid360 Fast-LIVO2 雷达启动脚本（各节点同时启动）
├── build_secure.sh          # ⭐ 安全打包脚本：Cython 编译防源码泄露
├── DEPLOY_CI.md             # ⭐ CI 自动发布 + 双 Clone 部署详细文档
├── Radar_Launcher.yaml      # ⭐ 雷达启动/关闭统一配置（Odin / mid360）
├── User_define_commands.yaml# 用户自定义按钮指令（UserCmd1/2/3）
├── setup.py                 # Cython 编译脚本（源码打包）
├── odin_stream.desktop      # 桌面自启：全屏推流
├── qgc_servers.desktop      # 桌面自启：QGC 三大服务
├── My_Flight_Path.txt       # 航迹数据文件（运行时生成，不入库）
├── 依赖.txt                 # Python 依赖安装说明
└── deploy/                  # 安全打包输出目录（编译产物，不入库）
```

> 注：`nx_command_server2.py`、`nx_touch_server2.py`、`rviz_full_stream2.py` 为各服务的备份/包装版本。

---

## 🌐 网络端口一览

| 端口 | 协议 | 服务 | 说明 |
|------|------|------|------|
| 6000 | TCP | `rviz_full_stream.py` | 全屏截图 JPEG 推流（生产者/消费者分离，帧覆盖策略）+ **集成 ping 心跳** |
| 6002 | UDP | `nx_touch_server.py` | 触控坐标转鼠标控制（归一化坐标 → 物理像素） |
| 6003 | UDP | `nx_command_server.py` | QGC 指令接收通道 |
| 6004 | HTTP | `nx_command_server.py` | 综合监测 + 文件传输 + 流设置服务 |

> 💡 **ping 心跳已集成进推流服务**：`rviz_full_stream.py` 会在 QGC 推流客户端连接时自动获取其 IP，
> 并向该 IP 的 6004 端口周期上报延迟，**无需手动配置 IP**（环境变量 `LIYUMEN_QGC_UDP_PORT` 可改上报端口）。

---

## 🚀 启动方式

### 方式一：开机自启（推荐）
将 `qgc_servers.desktop` 与 `odin_stream.desktop` 放入 `~/.config/autostart/`，
系统登录后会自动执行 `start_qgc_servers.sh`（等待桌面就绪 → 启动 roscore → 启动三大微服务）。

### 方式二：手动启动
```bash
# 1. 启动 QGC 三大服务（推流 / 触控 / 指令）
bash start_qgc_servers.sh

# 2. 如需 Mid360 雷达（Fast-LIVO2）
bash start_fastlivo2.sh false   # 仅建图，不录数据
bash start_fastlivo2.sh true    # 带 MindCloud，录制数据并输出 pcd
```

---

## 📡 UDP 指令协议（6003）

向 NX 的 6003 端口发送 UTF-8 文本指令即可触发动作：

| 指令 | 功能 |
|------|------|
| `Odin` | 启动雷达（不带 MindCloud，不录数据） |
| `Odin-MindCloud` | 启动雷达（带 MindCloud，录数据） |
| `KillLiDAR` | 关闭雷达（进程组精准关闭 + pattern 回退清理） |
| `RecordBag:topic1 topic2 ...` | 录制 ROS bag |
| `StopBag` | 停止录制 bag |
| `StartVideo` / `StopVideo` | 启动 / 停止视频录制 |
| `TakePhoto` | 单张拍照 |
| `RecordPathStart` / `RecordPathStop` | 开始 / 停止航迹录制 |
| `RecordPathReplay` | 航迹复飞 |
| `AbortReplay` | 中止复飞并切回定点模式 |
| `UserCmd1` / `UserCmd2` / `UserCmd3` | 执行 `User_define_commands.yaml` 中定义的自定义命令 |

> ⭐ **雷达启动/关闭已统一配置化**：具体启动命令、录数据开关、清理规则均在
> `Radar_Launcher.yaml` 中定义，修改 `active_profile` 即可在 `odin` / `mid360` 间切换。

---

## 🌐 HTTP 接口（6004）

| 路径 | 说明 |
|------|------|
| `GET /check_pose` | 检查飞行器位姿状态（高度 / 原点偏离） |
| `GET /check_file` | 检查航迹文件是否存在 |
| `GET /check_replay_status` | 查询复飞状态 |
| `GET /check_camera` | 检查相机话题是否有图像 |
| `GET /radar/status` | 查询雷达启动状态与追踪的进程组数量 |
| `GET /file_transfer/health` | 文件传输服务健康检查 |
| `GET /file_transfer/list` | 列出 Home / U 盘目录 |
| `POST /file_transfer/copy` | 复制文件到 U 盘 |
| `POST /file_transfer/zip` | 压缩文件 |
| `POST /file_transfer/delete` | 删除文件 |
| `POST /file_transfer/eject` | 安全弹出 U 盘 |
| `GET/POST /stream/settings` | 读取 / 修改推流设置 |

---

## 🛠️ 雷达配置（Radar_Launcher.yaml）

```yaml
active_profile: "mid360"   # 可选 "odin" / "mid360"

profiles:
  odin:    # Odin 激光雷达（默认 ROS 驱动方案）
    image_topic: "/odin1/image/undistorted"   # 图像话题
    launch_with_cloud:      # 带 MindCloud 的启动命令
    launch_without_cloud:   # 不带 MindCloud 的启动命令
    cleanup:                # 关闭时的清理规则
  mid360:  # Mid360 激光雷达（Fast-LIVO2 方案）
    image_topic: "/camera/color/image_raw"    # 图像话题(通常 D435)
    ...
```

**切换雷达类型三种方式：**
1. 直接修改 `active_profile`
2. 设置环境变量 `LIYUMEN_RADAR_PROFILE=mid360`（优先级最高）
3. 运行时直接改 YAML，下次 `Odin` / `Odin-MindCloud` 指令自动生效

**图像话题配置（`/check_camera`、录像、拍照使用）：**
- 优先级：环境变量 `LIYUMEN_IMAGE_TOPIC` > 当前 profile 的 `image_topic` > 默认 `/odin1/image/undistorted`
- 切换雷达类型后，图像话题会自动跟随（Odin 用自带相机、Mid360 用 D435）

关闭雷达采用**混合策略**：先通过进程组（PGID）精准关闭，再用 `cleanup.pkill_patterns`
与 `cleanup.rosnode_patterns` 回退清理残留进程，确保不遗留。

### 💾 PCD 保存机制（X2 异步写盘）

Mid360 建图模式（`start_fastlivo2.sh true`）下，Fast-LIVO2 的 PCD 采用 **X2 异步写盘**：

- **保存时机**：关闭雷达（`KillLiDAR` / `Ctrl+C`）时，`fastlivo_mapping` 收到 SIGINT
  → `savePCD()` fork 出独立子进程 `pcd_saver` 后台写盘，**主进程立即退出**。
- **效果**：PCD 写盘不再阻塞关闭/启动流程 —— 即使大点云写盘需要 1~2 分钟，
  下一次启动雷达也几乎不受影响；短时间内反复开关雷达不会产生长时间等待。
- **输出文件**：`src/FAST-LIVO2/Log/pcd/<启动时刻>.pcd`（如 `20260807_150000.pcd`），
  文件名即建图会话时间戳，天然不覆盖历史数据。
- **配套清理**：外部清理用进程名 `fastlivo_mappi`（Linux 15 字符截断）精确匹配建图主进程，
  不会误杀后台写盘子进程 `pcd_saver`（已脱离进程组，不占串口 / 端口 / USB）。
- **兜底**：若 `fork()` 失败，自动退化为同步写盘，PCD 仍完整保存（关闭会等待其写完）。

> ⚠️ 关闭后 `pcd_saver` 会在后台继续写盘（孤儿进程）：可用 `pgrep pcd_saver` 确认进度，
> 或检查 `Log/pcd/` 下是否生成对应时间戳的 `.pcd` 文件。

---

## 🔧 用户自定义按钮（User_define_commands.yaml）

```yaml
UserCmd1:
  name: "SUPER航点任务"
  command: "source /opt/ros/noetic/setup.bash && roslaunch mission_planner real_flight.launch"
```

在 QGC 上触发 `UserCmd1` 后，NX 端会以与启动雷达相同的 ROS 环境执行该命令。

---

## 📦 依赖安装

```bash
pip3 install pynput mss pyyaml
```

- **pynput** — 触控转鼠标（`nx_touch_server.py`）
- **mss** — 全屏截图（`rviz_full_stream.py`）
- **pyyaml** — 解析 YAML 配置（`nx_command_server.py`）

ROS 依赖：`rospy`、`geometry_msgs`、`sensor_msgs`、`cv_bridge`、`mavros_msgs`。
若使用 Cython 打包源码，需安装 `cython` 并运行 `python3 setup.py build_ext --inplace`。

---

## 📝 注意事项

- 推流/触控依赖本地图形桌面：需 `export DISPLAY=:0` 且 `xhost +local:`（纯 SSH 无桌面环境时 rviz 无法弹窗）。
- `My_Flight_Path.txt` 为运行时生成数据，已被 `.gitignore` 忽略，不入库。
- 各服务的 `*2.py` 为备份/包装版本，请以主版本为准。

---

## 🔐 代码保护打包（防部署泄露）

部署到外部机器时，可先将核心 `.py` 编译为 Cython 二进制扩展 `.so`，
源码不随部署包分发，防止代码泄露。

### 一键打包
```bash
bash build_secure.sh [输出目录]   # 默认输出到 deploy/
```

打包过程（在临时目录进行，**原工程目录保持完整源码不受影响**）：
1. Cython 将核心 `.py` 编译为 `.so`
2. 删除源码 `.py` 与中间产物 `.c`
3. 为每个服务生成薄启动器 `*_launcher.py`
4. 生成与 `start_qgc_servers.sh` 兼容的 `*2.py` 包装器
5. 拷贝资源文件（`yaml`/`sh`/`desktop`/`README` 等）并自动修正入口调用

### 部署
- 将整个 `deploy/` 目录拷贝到目标 NX 即可（详见其中的 `DEPLOY_README.txt`）
- 启动方式与源码版一致，无需改动 `start_qgc_servers.sh`

### 启动器对照
| 服务 | 启动命令 |
|------|----------|
| 指令+监测 (6003/6004) | `python3 nx_command_server_launcher.py` |
| 触控转鼠标 (6002) | `python3 nx_touch_server_launcher.py` |
| 全屏推流+心跳 (6000) | `python3 rviz_full_stream_launcher.py --worker` |
| 里程计转发 | `python3 odom_to_mavros_launcher.py` |

> 前提：目标机需安装 `cython` 编译产物的运行依赖，且 `.so` 与目标机 Python 版本/架构匹配。
> 安全说明：`.so` 为编译产物，可显著提高保护强度，但无法做到绝对不可逆，请妥善保管源码与仓库权限。

---

## 🤖 CI 自动发布（开发拉 master / 部署拉 deploy）

推送代码后由 **GitHub Actions 自动编译**并把产物发布到 `deploy` 分支，实现
「**开发机拉 master（源码）、部署机拉 deploy（编译产物，无源码）**」的双 Clone 工作流。

```
开发机 push master ──▶ GitHub Actions ──▶ 编译 deploy/ ──▶ 发布 deploy 分支
                              │                    │
                        self-hosted runner         │
                        (NX 本机, aarch64+Py3.8)    ▼
                                           部署机 clone -b deploy
```

| 角色 | 分支 | 内容 | 用法 |
|------|------|------|------|
| **开发机** | `master` | 完整 `.py` 源码 | 改代码 → `git push`，CI 自动发布 |
| **部署机** | `deploy` | 仅 `.so` 编译产物 + 配置 | `git clone -b deploy ...`，开箱即用 |

### 部署机拉取
```bash
# 全新部署
git clone -b deploy git@github.com:jihaofly/NX_code.git nx_deploy
# 更新部署包
cd nx_deploy && git pull origin deploy
```

> ⚠️ `deploy` 分支由 CI 强制覆盖，仅含编译产物；源码只在 `master` 分支。
> 完整配置步骤（注册 self-hosted runner、手动重跑等）见 [`DEPLOY_CI.md`](DEPLOY_CI.md)。
