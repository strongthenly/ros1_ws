# obstacle_warning.md — 避障参数与数据源不匹配导致 POSCTL 锁死（踩坑记录）

> **日期**: 2026-08-11  
> **机型**: fouruav（NxtPX4v2 飞控, PX4 v1.17.0 微空固件, MID-360S + FAST-LIVO2）  
> **现象**: 切 POSCTL（Position）模式后，过一段时间自动回退 HOLD/原模式，报 "No valid global position estimate"

---

## 一、最终结论（一句话）

**PX4 碰撞预防（Collision Prevention）参数 `CP_DIST` 被设为 1（开启），但避障数据源 `obstacle_to_px4` 没在运行 → `CP_GO_NO_DATA=0` 时 PX4 认为"四周 1m 内全是障碍" → 拒绝位置控制 → POSCTL 切不进去/保持不住，自动回退。**

**不是定位问题**（EKF 视觉融合正常），**是避障系统在"有参数、无数据"的矛盾配置下把位置控制锁死了。**

---

## 二、完整排查链（避免下次重走弯路）

### 症状
- `rosservice call /mavros/set_mode "{custom_mode: 'POSCTL'}"` 能切，但几秒后自动回退
- MAVROS 报 `navigation error: No valid global position estimate`
- QGC 显示 Position 但飞控实际保持原模式

### 排查顺序（已走过的弯路，别重复）
1. ❌ 查 EKF 定位（`pos_horiz_abs=False`）→ 误导。PX4 1.17 里 `/mavros/estimator_status` 的 flag 布局与 `.ulg` 不同，**要看 .ulg 里 `estimator_status_flags` 的 `cs_ev_pos`**——实测 `cs_ev_pos=1`（视觉融合正常！）
2. ❌ 查 `EKF2_AID_MASK` → 该参数在 PX4 1.17 已被 `EKF2_EV_CTRL` 等替代（`EKF2_AID_MASK` 读出来是 0 但不代表禁用视觉）
3. ❌ 查视觉话题/坐标系 → 全部正常（vision_pose 平滑跟随、MAVROS 订阅、EV_CTRL=11）
4. ✅ **查避障参数** → `CP_DIST=1, CP_GO_NO_DATA=0, obstacle_to_px4 未运行` → 锁死根源

### 决定性证据
- `.ulg` 日志：`estimator_aid_src_ev_pos: fused=1`（视觉位置全程融合）
- 参数文件：`CP_DIST=1.0`、`CP_GO_NO_DATA=0`、`COM_RC_LOSS_T=0.5`
- 进程：`obstacle_to_px4` 未运行（无 `/mavros/obstacle/send` 数据）

---

## 三、PX4 避障参数速查（重要）

| 参数 | 值 | 含义 | 注意 |
|---|---|---|---|
| `CP_DIST` | **-1 = 关闭** / 1~2 = 开启 | 碰撞预防距离(m) | **负值=关闭**；开启时必须有障碍数据源 |
| `CP_GO_NO_DATA` | 0 / 1 | 无障碍数据时能否飞 | **0=不可飞(保守,危险)**；1=无数据方向可飞 |
| `CP_DELAY` | 0~0.2 | 避障响应延迟 | 震荡就调大 |
| `CP_GUIDE_ANG` | 0 | 引导角 | 阶段5再开 |

### ⚠️ 致命组合（本次踩的坑）
```
CP_DIST=1（开了避障） + CP_GO_NO_DATA=0（无数据不可飞） + 数据源没跑
= PX4 认为全向 1m 内都是障碍 = 位置控制永久锁死
```

---

## 四、解决方案

### 方案 A：不用避障（恢复纯手动/纯 OFFBOARD 飞行）
```bash
# QGC 或命令行把 CP_DIST 改为 -1（负值=关闭）
rosservice call /mavros/param/set "{param_id: 'CP_DIST', value: {integer: -1, real: 0.0}}"
```

### 方案 B：要用避障（必须配套数据源）
```bash
# 1. 先启动避障数据源（否则无数据=全向障碍）
roslaunch task_run_scripts obstacle_to_px4.launch drone:=mid360
# 2. 建议 CP_GO_NO_DATA=1（无数据方向可飞，避免锁死）
# 3. 确认 /mavros/obstacle/send 有数据（rostopic hz）
```

### 推荐
- **日常飞行/测试**：方案 A（CP_DIST=-1），避障系统不成熟前别开
- **避障专项测试**：方案 B，先确认数据源稳定再开

---

## 五、快速检查清单（下次遇到"POSCTL 切不进去"）

```bash
# 1. 避障参数
rosparam get /mavros/param 2>/dev/null | grep CP_DIST || \
  rosservice call /mavros/param/get "{param_id: 'CP_DIST'}"
#   如果 CP_DIST > 0 → 避障开了，查数据源↓

# 2. 避障数据源是否在跑
ps aux | grep obstacle_to_px4
rostopic hz /mavros/obstacle/send --window 3

# 3. 视觉定位是否正常（.ulg 为准）
#    estimator_status_flags: cs_ev_pos=1 → 视觉融合正常（别被 /mavros/estimator_status 误导）

# 4. 模式实际状态（.ulg 里 nav_state 是否真的变了）
```

---

## 六、相关链接

- 避障节点: `task_run_scripts/src/obstacle_to_px4.cpp` + `launch/obstacle_to_px4.launch`
- 配置: `task_run_scripts/config/mid360.yaml`（cloud_topic=/cloud_registered）
- 飞控: NxtPX4v2, PX4 v1.17.0（参数文件 `NxtPX4v2_Copter_PX4v1.17.0.param`）
