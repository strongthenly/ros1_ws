#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
goal_arbiter.py — FUEL 探索目标 → SUPER 航点的仲裁器（[FUEL2SUPER]）

背景
----
FUEL 每轮 replan（约 1 Hz）都会产出一个新的探索目标点（发布在 /exploration_goal）。
若把它直接转发给 SUPER 的 /waypoint_goal，SUPER 的 FSM 每秒都会收到新目标 →
轨迹被反复打断 → 飞机抖动/乱转。所以必须**限频**。

限频条件（满足任一即放行）
--------------------------
  1. 尚无当前目标            （冷启动，立即放行）
  2. 已到达当前目标          （距目标 < arrive_radius）
  3. 当前目标已失效          （FUEL 新候选与当前目标相差 > goal_shift，且距上次放行 > min_interval）
  4. 保持超时                （距上次放行 > max_hold，强制放行，防死锁）

另外两道保护（2026-09-22 新增，借鉴实机 task_node_nav 的超时放弃机制）
--------------------------------------------------------------------
  A. 目标高度夹取：FUEL 的视点高度会超出 SUPER 的地图范围与 FSM 允许区间
     （实测 FUEL 给过 z=2.42，而 rog_map 垂直范围 ±2.25、FSM 的 goal_z 区间 0.6~1.3），
     导致的后果是 SUPER 反复 "[EXPTrajOpt] Ill corridor!" / 规划失败。
     → 放行前把 z 夹到 [z_min, min(z_max, ceiling)]。

     ★2026-10-08 更新默认值：实机 map_size 已改为 [30,30,10]（fix_map_origin z=1.5）
      → rog_map 实际覆盖 z ∈ [-3.5, 6.5]，原先的 ceiling=2.2 已过期；
      → 配合 FSM 的 goal_z 0.6/5.0，本节点默认改为 z_min 0.6 / z_max 5.0 / ceiling 6.0。
      ⚠️ 改限高必须【两处同改】(sim/real 的 fsm goal_z 与本节点)，否则只改一处等于没改。

  B. 卡死检测 + 目标收缩：
     实机 SUPER 的 AVOID_WAIT 是**每 0.3s 无限重试、永不放弃**，一旦目标不可达
     （典型：FUEL 探到 8m 外、超出已建图范围 → SFC 退化成 "Ill corridor"）就会
     永久悬停；而飞机不动又让 FUEL 依据 traj_server 的执行反馈判定"未到达"，
     于是反复发同一个目标 → 死锁（仲裁器原来的 max_hold "防死锁"放行的还是同一个目标）。
     实机在任务层用 "超时 → 放弃该航点 → 试下一个" 规避，本节点做等价的事：
       · 目标放行后 stuck_timeout 秒内，飞机几乎没动（< stuck_move_eps）
         且离目标仍 > arrive_radius  → 判定该目标不可达
       · 把目标沿"飞机→目标"方向收缩到 shrink_radius 米内再放行 → 飞机先动起来
         → FUEL 随之推进并给出新目标 → 自然脱困
       · 连续收缩 max_shrinks 次仍无进展 → 停止放行并告警（诚实失败，不静默死锁）；
         FUEL 给出**明显不同的新目标**时计数清零，重新尝试

输入 / 输出
-----------
  in : /exploration_goal   geometry_msgs/PoseStamped   (FUEL 发布)
  in : /odom_world         nav_msgs/Odometry           (当前位置，可配置)
  in : /waypoint_goal_task geometry_msgs/PoseStamped   (可选：watch 触发信号，见下)
  out: /waypoint_goal      geometry_msgs/PoseStamped   (SUPER 的 fsm 订阅)

实机启动触发（watch 模式，2026-10-08 加）
----------------------------------------
  实机上 CH10 拨动会触发 waypoint_mission 发出一条 /waypoint_goal。
  探索模式下把 waypoint_mission 的输出改道到 /waypoint_goal_task（用 waypoint_explore.yaml），
  本节点订阅它：一旦收到 → 判定"操作员已拨 CH10，开始探索" →
     ① 自己发一条 /move_base_simple/goal 触发 FUEL 离开 WAIT_TRIGGER
     ② 此后接管 /waypoint_goal（本节点独占，fsm 不再收到 waypoint_mission 的点）
  未配置 watch_topic（默认空）时行为与以前完全一致，立即工作。

不参与控制
----------
本节点**只做目标转发**，不发任何 setpoint / pos_cmd，不影响飞行安全链路。

用法
----
  rosrun fuel_bridge goal_arbiter.py
  rosrun fuel_bridge goal_arbiter.py _arrive_radius:=0.5 _min_interval:=3.0 _dry_run:=true
  rosrun fuel_bridge goal_arbiter.py _z_min:=0.6 _z_max:=5.0 _ceiling:=6.0 _stuck_timeout:=15.0
  # 实机探索模式（等 CH10）：
  rosrun fuel_bridge goal_arbiter.py _watch_topic:=/waypoint_goal_task \
      _z_min:=0.6 _z_max:=5.0 _ceiling:=6.0
"""
import math
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class GoalArbiter(object):
    def __init__(self):
        # ---- 基础 ----
        self.in_topic = rospy.get_param("~in_topic", "/exploration_goal")
        self.out_topic = rospy.get_param("~out_topic", "/waypoint_goal")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom_world")
        self.arrive_radius = float(rospy.get_param("~arrive_radius", 0.5))   # m
        self.min_interval = float(rospy.get_param("~min_interval", 3.0))     # s
        self.goal_shift = float(rospy.get_param("~goal_shift", 1.5))         # m
        self.max_hold = float(rospy.get_param("~max_hold", 20.0))            # s
        self.dry_run = bool(rospy.get_param("~dry_run", False))              # true=只打印不发

        # ---- A. 目标高度夹取 ----
        self.z_min = float(rospy.get_param("~z_min", 0.6))       # m，与 FSM goal_z_min 一致
        self.z_max = float(rospy.get_param("~z_max", 5.0))       # m，与 FSM goal_z_max 一致
        self.ceiling = float(rospy.get_param("~ceiling", 6.0))   # m，必须 < rog_map 顶棚(6.5)

        # ---- B. 卡死检测 + 收缩 ----
        self.stuck_timeout = float(rospy.get_param("~stuck_timeout", 15.0))   # s
        self.stuck_move_eps = float(rospy.get_param("~stuck_move_eps", 0.5))  # m
        self.shrink_radius = float(rospy.get_param("~shrink_radius", 3.0))    # m
        self.max_shrinks = int(rospy.get_param("~max_shrinks", 3))

        # ---- C. watch 触发（CH10 → 开始探索），2026-10-08 加 ----
        self.watch_topic = str(rospy.get_param("~watch_topic", ""))   # 空 = 不启用，立即工作
        self.trigger_topic = str(rospy.get_param("~trigger_topic", "/move_base_simple/goal"))
        self.armed = not bool(self.watch_topic)
        self.trigger_sent = False

        self.cur_goal = None          # 已放行的目标 (x,y,z)（可能是收缩后的点）
        self.cur_goal_raw = None      # 放行时 FUEL 的原始候选（用于判断"目标是否真的变了"）
        self.last_emit_t = None
        self.release_pos = None       # 放行时的飞机位置（判卡死用）
        self.shrink_count = 0         # 连续收缩次数（目标真变了就清零）
        self.gave_up = False          # 连续收缩超限后置位，停止放行
        self.candidate = None         # FUEL 最新候选
        self.pos = None               # 当前位置

        self.pub = rospy.Publisher(self.out_topic, PoseStamped, queue_size=1, latch=True)
        self.trigger_pub = rospy.Publisher(self.trigger_topic, PoseStamped, queue_size=1)
        rospy.Subscriber(self.in_topic, PoseStamped, self.on_goal, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.on_odom, queue_size=10)
        if self.watch_topic:
            rospy.Subscriber(self.watch_topic, PoseStamped, self.on_watch, queue_size=1)
        rospy.Timer(rospy.Duration(0.2), self.tick)

        rospy.loginfo(
            "[arbiter] in=%s out=%s odom=%s | arrive=%.2fm shift=%.2fm min_interval=%.1fs max_hold=%.1fs dry_run=%s",
            self.in_topic, self.out_topic, self.odom_topic,
            self.arrive_radius, self.goal_shift, self.min_interval, self.max_hold, self.dry_run)
        rospy.loginfo(
            "[arbiter] z 夹取=[%.2f, %.2f] 上限=%.2f | 卡死判定: %.1fs 内位移<%.2fm | 收缩到 %.1fm 内，最多 %d 次",
            self.z_min, self.z_max, self.ceiling,
            self.stuck_timeout, self.stuck_move_eps, self.shrink_radius, self.max_shrinks)
        if self.watch_topic:
            rospy.loginfo(
                "[arbiter] ★watch 模式: 等 %s 上的信号（= 拨 CH10）→ 触发 %s 让 FUEL 启动",
                self.watch_topic, self.trigger_topic)
        else:
            rospy.loginfo("[arbiter] 无 watch 配置 → 立即工作（与旧版行为一致）")

    # ---------- 回调 ----------
    def on_watch(self, msg):
        """CH10 信号到达 = 操作员要开始探索：触发 FUEL + 解锁放行。"""
        if self.armed:
            return
        p = msg.pose.position
        z = max(self.z_min, min(min(self.z_max, self.ceiling), p.z))
        rospy.loginfo("[arbiter] ★收到 watch 信号 (%.2f, %.2f, %.2f) → 开始探索", p.x, p.y, p.z)
        t = PoseStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.pose.position.x = p.x
        t.pose.position.y = p.y
        t.pose.position.z = z
        t.pose.orientation.w = 1.0
        self.trigger_pub.publish(t)
        self.trigger_sent = True
        self.armed = True
        rospy.loginfo("[arbiter] 已发触发到 %s（让 FUEL 离开 WAIT_TRIGGER）", self.trigger_topic)

    def on_goal(self, msg):
        p = msg.pose.position
        self.candidate = (p.x, p.y, p.z)
        rospy.loginfo_throttle(5.0, "[arbiter] FUEL 候选目标: (%.2f, %.2f, %.2f)", p.x, p.y, p.z)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pos = (p.x, p.y, p.z)

    # ---------- 工具 ----------
    @staticmethod
    def dist(a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

    def elapsed(self):
        return None if self.last_emit_t is None else (rospy.Time.now() - self.last_emit_t).to_sec()

    def clamp_z(self, g):
        """A. 高度夹取：既要落在 FSM 允许区间，也不能超过地图顶棚。"""
        hi = min(self.z_max, self.ceiling)
        return (g[0], g[1], max(self.z_min, min(hi, g[2])))

    def shrink_toward(self, pos, goal, radius):
        """把 goal 沿 pos→goal 方向收回，使距离不超过 radius（只收 x/y/z 三轴等比）。"""
        d = self.dist(pos, goal)
        if d <= radius or d < 1e-6:
            return goal
        k = radius / d
        return (pos[0] + (goal[0] - pos[0]) * k,
                pos[1] + (goal[1] - pos[1]) * k,
                pos[2] + (goal[2] - pos[2]) * k)

    def is_stuck(self, dt):
        """B. 卡死判定：放行后 dt 秒内飞机几乎没动，且离目标仍远。"""
        if self.pos is None or self.release_pos is None or self.cur_goal is None:
            return False
        if dt is None or dt < self.stuck_timeout:
            return False
        moved = self.dist(self.pos, self.release_pos)
        remaining = self.dist(self.pos, self.cur_goal)
        return moved < self.stuck_move_eps and remaining > self.arrive_radius

    # ---------- 主循环 ----------
    def tick(self, _evt):
        if not self.armed:
            return          # watch 模式下：等 CH10 信号，此前一概不放行
        if self.candidate is None:
            return
        now = rospy.Time.now()
        reason = None
        publish_goal = None      # 实际要发的点（None = 用 candidate 夹取后）

        if self.cur_goal is None:
            reason = "冷启动（无当前目标）"
        else:
            dt = self.elapsed()
            # 用【原始候选】判断目标有没有真的变，避免把"收缩后的点"误判成目标变化
            shifted = (self.cur_goal_raw is not None
                       and self.dist(self.candidate, self.cur_goal_raw) > self.goal_shift)
            arrived = (self.pos is not None
                       and self.dist(self.pos, self.cur_goal) < self.arrive_radius)

            if arrived:
                reason = "已到达当前目标（距 %.2fm）" % self.dist(self.pos, self.cur_goal)
            elif shifted and dt is not None and dt >= self.min_interval:
                reason = "目标已失效（变化 %.2fm > %.2fm，且已过 %.1fs）" % (
                    self.dist(self.candidate, self.cur_goal_raw), self.goal_shift, dt)
            elif self.is_stuck(dt):
                # B. 不可达 → 收缩重试（等价于实机"超时放弃该航点、换个能到的点"）
                if self.shrink_count >= self.max_shrinks:
                    if not self.gave_up:
                        self.gave_up = True
                        rospy.logwarn(
                            "[arbiter] 目标 (%.2f, %.2f, %.2f) 连续 %d 次收缩仍无进展，"
                            "判定不可达 → 停止放行，等待 FUEL 给出新目标",
                            self.cur_goal_raw[0], self.cur_goal_raw[1], self.cur_goal_raw[2],
                            self.shrink_count)
                    return
                if self.pos is None:
                    return
                publish_goal = self.shrink_toward(self.pos, self.candidate, self.shrink_radius)
                reason = ("目标不可达（%.1fs 内位移 %.2fm、仍差 %.2fm）→ 收缩到 %.1fm 内重试"
                          "（第 %d/%d 次）" % (
                              dt, self.dist(self.pos, self.release_pos),
                              self.dist(self.pos, self.cur_goal), self.shrink_radius,
                              self.shrink_count + 1, self.max_shrinks))
                self.shrink_count += 1
            elif dt is not None and dt >= self.max_hold:
                reason = "保持超时 %.1fs（强制放行，防死锁）" % dt

        if reason is None:
            return

        # 放行（先夹取高度；收缩分支已自带点位）
        prev = self.cur_goal
        g = self.clamp_z(publish_goal if publish_goal is not None else self.candidate)
        self.cur_goal_raw = self.candidate
        self.cur_goal = g
        self.last_emit_t = now
        self.release_pos = self.pos
        # 目标真的变了 → 复位放弃状态，重新给它机会
        if publish_goal is None:
            self.shrink_count = 0
            self.gave_up = False

        m = PoseStamped()
        m.header.stamp = now
        m.header.frame_id = "world"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = g
        m.pose.orientation.w = 1.0
        if not self.dry_run:
            self.pub.publish(m)

        rospy.loginfo("[arbiter] >>> 放行目标 (%.2f, %.2f, %.2f) | 原因: %s | 上一目标: %s | %s",
                      g[0], g[1], g[2], reason,
                      "无" if prev is None else "(%.2f, %.2f, %.2f)" % prev,
                      "[DRY-RUN 未实际发布]" if self.dry_run else "已发布到 " + self.out_topic)


if __name__ == "__main__":
    rospy.init_node("goal_arbiter")
    GoalArbiter()
    rospy.spin()
