# 排障手册（troubleshooting）

> 最后更新：2026-08-10（feat/obstacle-to-px4 分支验证期间）
> 适用：fouruav（Orin NX + MID360S + D435 + FAST-LIVO2 + PX4）

---

## 1. 编译

### 1.1 编译失败：livox_ros_driver2 报 ament_cmake_auto 找不到

**现象**：`catkin_make` 报 `Could not find a package configuration file provided by "ament_cmake_auto"`

**原因**：livox_ros_driver2 支持 ROS1/ROS2 双版本，由 `ROS_EDITION` 变量切换。直接 `catkin_make` 默认走 **ROS2 分支**，需要 ament_cmake_auto（ROS2 包，Noetic 没有）。

**解决**：必须带参数编译
```bash
cd ~/catkin_ws && catkin_make -j4 -DROS_EDITION=ROS1
```

**验证**：`~/catkin_ws/devel/lib/task_run_scripts/obstacle_to_px4_node` 存在

---

## 2. 避障链路（obstacle_to_px4）

### 2.1 链路总览

```
点云 → obstacle_to_px4 (36扇区) → /mavros/obstacle/send (LaserScan) 
     → MAVROS distance_sensor 插件 → MAVLink OBSTACLE_DISTANCE → PX4
```

### 2.2 现象：`no cloud received yet, stop publishing`

**原因**：TF 里没有 `livox_frame` 帧。原始 `/livox/lidar` 点云在 livox_frame 系，但 FAST-LIVO2 不发布这个帧；launch 里的静态变换 `livox_frame→aft_mapped` 因父帧不存在而失效。

**解决**：改 `config/mid360.yaml` 的 cloud_topic 用校正后点云
```yaml
cloud_topic: /cloud_registered   # camera_init 系，有 camera_init→aft_mapped TF
target_frame: aft_mapped
```
备份：`mid360.yaml.bak_livox`

**判读**：日志出现 `cloud pts=3905 processed=229 first=(2.65,-0.26,-0.16)` + `pub bin0=0.45 ...` = 工作正常

### 2.3 现象：数据正常但 PX4 不避障

**原因**：PX4 避障参数未开启（出厂默认关闭）

**检查**：
```bash
rosservice call /mavros/param/get "param_id: 'COM_OB_ACT'"    # 0=关闭
rosservice call /mavros/param/get "param_id: 'COM_OB_RADIUS'" # 生效半径
```

**开启（QGC 或命令）**：
```bash
rosservice call /mavros/param/set "param_id: 'COM_OB_ACT' value: {integer: 1}"
rosservice call /mavros/param/set "param_id: 'COM_OB_RADIUS' value: {integer: 5}"
```

⚠️ 开启后首次真机：小半径(2m)+低速，确认行为再加大。

### 2.4 现象：min_dist 持续很小（如 0.36m）

**可能**：机架/雷达自身遮挡、或测试环境物体太近。RViz 看 `sector_marker`（扇形可视化）定位是哪个扇区。

---

## 3. 复飞（SUPER 航点任务）

### 3.1 现象：起飞悬停成功，但复飞中断、飞机落地贴地转桨

**日志特征**：
```
-- [Fsm] PlanFromRest failed, try replan.       (无限重试)
-- [MINCO] TrajOpt failed, The function value became NaN or Inf.
[WARN] -- [SUPER] in [PlanFromRest] GenerateExpTrajectory failed with FAILED.
[WARN] SUPER command timeout. Switching to position hold.
```

**根因链**：
```
航点位置在地图里被判为"占用"(障碍)
  → FSM setGoalPosiAndYaw 调 getNearestInfCellNot 把目标吸附到最近空闲格
  → 目标高度突变(如 0.55→0.825m)
  → MINCO 轨迹优化 NaN（起点/终点状态约束冲突）
  → 规划失败 → 无指令 → 悬停不住 → 落地
```

**排查**：
1. 对比日志：`Pub goal to`（waypoint_mission 发的原始坐标）vs `Get goal at`（FSM 吸附后的坐标），差太多就是被吸附了
2. 起飞前看 RViz：蓝色航点是否落在灰色障碍格子里

**对策**：
- 起飞前雷达转一圈，让 ROG-Map 清掉人走过留下的障碍块（射线 p_miss 机制）
- 统一高度：航点高度 ≈ 起飞高度(init_goal/z)
- 必要时调 `p_free`（当前 0.30）让空闲判定更积极

### 3.2 现象：拨杆后无反应，`Mavros triggered.` 不出现

**原因**：CH10 触发条件是**下降沿**——`last_ch_10 > 1500 && ch_10 < 1500`。一直停在低位(1050)永远不会触发。代码只查 CH10，pitch(CH2) 不参与判断。

**正确动作**：先拨 CH10 到高位(>1500) → 再拉回低位(<1500)

### 3.3 现象：RViz 看不到航点

**原因**：waypoint_mission 的 mkr 只发布一次（订阅者出现时）。RViz 后启动就错过了。

**解决**：top_down.rviz 的 Waypoints 显示项取消勾选再勾上（触发重新发布）

---

## 4. 录制航迹

### 4.1 纪律

- **终点保持高度再 Stop**，不要降落到地面（复飞会带地面俯冲段）
- 起点 = 将来起飞点
- 全程保持高度稳定（与 init_goal/z 一致最好）

### 4.2 文件机制

| 文件 | 作用 |
|---|---|
| `data_guidepoints/guidepoints_new.txt` | 始终最新（按钮3 加载） |
| `data_guidepoints/guidepoints_时间戳.txt` | 历史归档，永不覆盖 |
| `scripts/NX_code/My_Flight_Path.txt` | 旧版录制文件（已不更新） |

---

## 5. 按钮

| 按钮 | 功能 | 备注 |
|---|---|---|
| UserCmd1 | 启动 SUPER（默认航点） | |
| UserCmd2 | 关闭 SUPER | 先 POSCTL，pkill 'top_down.rviz' 精确匹配（勿用 'rviz' 会误杀推流） |
| UserCmd3 | 启动 SUPER 复飞（加载 guidepoints_new.txt） | |

---

## 6. Git 操作

### 6.1 编译前记得
- 本地修改未提交时，切分支前先 `git stash`（task_run_scripts 和 scripts/NX_code 各一份）
- 完整快照：`tar czf ~/task_run_scripts_backup_日期.tar.gz --exclude='task_run_scripts/log' --exclude='task_run_scripts/.git' -C ~/catkin_ws/src task_run_scripts`

### 6.2 当前状态（2026-08-10）
- 分支：feat/obstacle-to-px4（拉取后）
- 本地修改：stash 中
- 快照：~/task_run_scripts_backup_20260810.tar.gz

---

## 7. 常用检查命令速查

```bash
# 复飞日志
tail -50 ~/catkin_ws/src/task_run_scripts/log/real_flight.log
grep -a "triggered\|Pub goal\|Get goal" real_flight.log

# TF 检查
rosrun tf tf_echo camera_init aft_mapped

# 避障数据
rostopic hz /mavros/obstacle/send
rostopic echo /obstacle_to_px4/status -n 1

# PX4 避障参数
rosservice call /mavros/param/get "param_id: 'COM_OB_ACT'"
```
