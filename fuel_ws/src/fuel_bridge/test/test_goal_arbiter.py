#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
goal_arbiter 单元测试（canonical 的一部分，由 test/smoke_test.sh 调用）

用极简 ROS 桩替换 rospy/geometry_msgs/nav_msgs，直接驱动 GoalArbiter 的
on_goal/on_odom/tick，验证放行条件、限频、高度夹取、卡死收缩与放弃行为。
**不需要 master / 仿真。**

用法: python3 test_goal_arbiter.py     # exit 0 = ALL PASS
"""
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
TARGET = os.path.join(HERE, os.pardir, "scripts", "goal_arbiter.py")


# ---------- 极简 ROS 桩 ----------
class Any(object):
    """自适应 Mock：任意属性访问都自动创建子对象（支持 m.header.stamp = x）。"""
    def __getattr__(self, n):
        v = Any()
        setattr(self, n, v)
        return v

    def __setattr__(self, n, v):
        object.__setattr__(self, n, v)


class SimTime(object):
    """可手动推进的假 ros::Time：类变量 v = 当前时刻，测试里直接改 SimTime.v。"""
    v = 1000.0

    def __init__(self, v=None):
        self.v = SimTime.v if v is None else v

    @staticmethod
    def now():
        return SimTime()

    @staticmethod
    def Duration(x):
        return x

    def to_sec(self):
        return self.v

    def __sub__(self, other):
        return SimTime(self.v - other.v)     # 差值（不是"当前时刻"！）


def _install_stubs():
    rospy = types.ModuleType("rospy")
    rospy.Time = SimTime
    rospy.Duration = lambda x: x
    rospy.get_param = lambda k, d=None: d
    rospy.loginfo = lambda *a, **k: None
    rospy.loginfo_throttle = lambda *a, **k: None
    rospy.logwarn = lambda *a, **k: None
    rospy.init_node = lambda *a, **k: None
    rospy.spin = lambda: None

    class Pub(object):
        def __init__(self, *a, **k):
            self.sent = []

        def publish(self, m):
            p = m.pose.position
            self.sent.append((p.x, p.y, p.z))

    rospy.Publisher = Pub
    rospy.Subscriber = lambda *a, **k: None
    rospy.Timer = lambda *a, **k: None
    sys.modules["rospy"] = rospy

    gm = types.ModuleType("geometry_msgs.msg")
    gm.PoseStamped = Any
    nm = types.ModuleType("nav_msgs.msg")
    nm.Odometry = Any
    for name, mod in (("geometry_msgs", gm), ("nav_msgs", nm)):
        pkg = types.ModuleType(name)
        pkg.msg = mod
        sys.modules[name] = pkg
        sys.modules[name + ".msg"] = mod


_install_stubs()

ns = {"__name__": "arb_under_test"}
with open(TARGET) as f:
    exec(compile(f.read(), TARGET, "exec"), ns)
GoalArbiter = ns["GoalArbiter"]

RESULTS = []


def check(name, cond):
    print("  %s %s" % ("PASS" if cond else "FAIL", name))
    RESULTS.append(bool(cond))


def mk(pos=None):
    """造一个跳过 __init__（避开 rospy 依赖）的仲裁器实例。"""
    a = GoalArbiter.__new__(GoalArbiter)
    a.in_topic, a.out_topic, a.odom_topic = "/exploration_goal", "/waypoint_goal", "/odom_world"
    a.arrive_radius, a.min_interval, a.goal_shift, a.max_hold = 0.5, 3.0, 1.5, 20.0
    a.dry_run = False
    # A. 高度夹取
    a.z_min, a.z_max, a.ceiling = 0.6, 1.3, 2.2
    # B. 卡死检测 + 收缩
    a.stuck_timeout, a.stuck_move_eps = 15.0, 0.5
    a.shrink_radius, a.max_shrinks = 3.0, 3
    a.cur_goal = a.last_emit_t = a.candidate = None
    a.cur_goal_raw = a.release_pos = None
    a.shrink_count = 0
    a.gave_up = False
    a.pos = pos
    a.pub = sys.modules["rospy"].Publisher()
    return a


def feed(a, goal):
    """模拟收到一帧 FUEL 候选目标。"""
    m = Any()
    m.pose.position.x, m.pose.position.y, m.pose.position.z = goal
    a.on_goal(m)


print("===== goal_arbiter 单测 =====")
print(" A) 基本限频")
a = mk(pos=(0, 0, 1))
feed(a, (1, 0, 1)); a.tick(None)
check("A1 冷启动立即放行", len(a.pub.sent) == 1 and a.pub.sent[0] == (1, 0, 1))
feed(a, (1, 0, 1)); a.tick(None)
check("A2 相同目标不重复放行", len(a.pub.sent) == 1)
feed(a, (1.5, 0, 1)); a.tick(None)
check("A3 小变化(<shift)+未到间隔 => 不放行", len(a.pub.sent) == 1)
feed(a, (5, 0, 1)); a.tick(None)
check("A4 大变化(>shift)但未到间隔 => 不放行", len(a.pub.sent) == 1)
SimTime.v += 4.0; a.tick(None)
check("A5 过最小间隔 => 放行", len(a.pub.sent) == 2 and a.pub.sent[1] == (5, 0, 1))

print(" B) 到达判定")
b = mk(pos=(0, 0, 1.0))          # 起点远离目标
feed(b, (5, 0, 1)); b.tick(None)
check("B1 冷启动放行", len(b.pub.sent) == 1)
SimTime.v += 0.5
feed(b, (9, 0, 1)); b.tick(None)
check("B2 未到达+未到间隔 => 不放行", len(b.pub.sent) == 1)
b.pos = (5.1, 0, 1.0)            # 距目标 0.1m < arrive_radius
b.tick(None)
check("B3 到达当前目标 => 立即放行", len(b.pub.sent) == 2 and b.pub.sent[1] == (9, 0, 1))

print(" C) 防死锁超时")
c = mk(pos=(0, 0, 1))
feed(c, (1, 0, 1)); c.tick(None)
SimTime.v += 25.0
feed(c, (1.2, 0, 1)); c.tick(None)   # 小变化、未到达，但已超 max_hold
check("C1 超 max_hold => 强制放行", len(c.pub.sent) == 2 and c.pub.sent[1] == (1.2, 0, 1))

print(" D) 无 odom 时仅靠时间限频")
d = mk(pos=None)
feed(d, (1, 0, 1)); d.tick(None)
check("D1 冷启动放行", len(d.pub.sent) == 1)
d.tick(None)
check("D2 无位置+同目标 => 不放行", len(d.pub.sent) == 1)
SimTime.v += 25.0; d.tick(None)
check("D3 无位置+超时 => 强制放行", len(d.pub.sent) == 2)

print(" E) 高度夹取（FUEL 视点会超出 FSM 区间与地图顶棚）")
e1 = mk(pos=(0, 0, 1))
feed(e1, (1, 0, 2.42)); e1.tick(None)      # 实测 FUEL 给过 z=2.42（> 地图顶棚 2.25）
check("E1 高于 z_max/顶棚 => 夹到 1.3", e1.pub.sent[0] == (1, 0, 1.3))
check("E1b 原始候选仍记全高度", e1.cur_goal_raw == (1, 0, 2.42))
e2 = mk(pos=(0, 0, 1))
feed(e2, (1, 0, 0.1)); e2.tick(None)
check("E2 低于 z_min => 夹到 0.6", e2.pub.sent[0] == (1, 0, 0.6))
e3 = mk(pos=(0, 0, 1))
feed(e3, (1, 0, 1.0)); e3.tick(None)
check("E3 区间内不动", e3.pub.sent[0] == (1, 0, 1.0))

print(" F) ⭐ 卡死检测 + 目标收缩（借鉴实机 task_node_nav 的超时放弃）")
f = mk(pos=(0, 0, 1.0))
feed(f, (10, 0, 1.0)); f.tick(None)
check("F1 冷启动放行原目标", f.pub.sent[-1] == (10, 0, 1.0))
check("F1b 未到 stuck_timeout 不判卡死", f.shrink_count == 0)
SimTime.v += 16.0; f.tick(None)          # 飞机 16s 没动、仍差 10m
check("F2 卡死 => 收缩到 shrink_radius 内", f.pub.sent[-1] == (3.0, 0, 1.0))
check("F3 收缩计数 +1", f.shrink_count == 1)
check("F4 收缩后 cur_goal 是收缩点", f.cur_goal == (3.0, 0, 1.0))
check("F5 cur_goal_raw 仍是 FUEL 原目标（防误判'目标已失效'）", f.cur_goal_raw == (10, 0, 1.0))
check("F6 尚未放弃", f.gave_up is False)

print(" G) ⭐ 连续收缩达上限 => 诚实放弃（不再静默死锁）")
SimTime.v += 16.0; f.tick(None)          # 第 2 次
SimTime.v += 16.0; f.tick(None)          # 第 3 次
n_after_3 = len(f.pub.sent)
check("G1 前 3 次都还会放行", n_after_3 == 4 and f.shrink_count == 3)
SimTime.v += 16.0; f.tick(None)          # 第 4 次：超上限
check("G2 超上限后停止放行", len(f.pub.sent) == n_after_3)
check("G3 gave_up 置位", f.gave_up is True)
SimTime.v += 16.0; f.tick(None)
check("G4 继续不发（保持放弃）", len(f.pub.sent) == n_after_3)

print(" H) ⭐ FUEL 给出明显不同的新目标 => 放弃状态复位")
feed(f, (0.5, 0, 1.0))                   # 与 cur_goal_raw(10,0,1) 差 9.5m > shift
SimTime.v += 4.0; f.tick(None)
check("H1 新目标重新放行", len(f.pub.sent) == n_after_3 + 1)
check("H2 放弃状态已复位", f.gave_up is False and f.shrink_count == 0)
check("H3 新目标就是 FUEL 给的", f.pub.sent[-1] == (0.5, 0, 1.0))

print(" I) 收缩方向正确（沿 飞机→目标 方向等比收回）")
i1 = mk(pos=(1.0, 2.0, 1.0))
got = i1.shrink_toward((1.0, 2.0, 1.0), (1.0, 12.0, 1.0), 4.0)
check("I1 收回后距离 = radius", abs(GoalArbiter.dist(got, (1.0, 2.0, 1.0)) - 4.0) < 1e-6)
check("I2 方向不变（x 保持 1.0）", abs(got[0] - 1.0) < 1e-6)
check("I3 目标已在半径内则不动", i1.shrink_toward((1.0, 2.0, 1.0), (1.0, 3.0, 1.0), 4.0) == (1.0, 3.0, 1.0))

n_ok = sum(RESULTS)
print("  -> goal_arbiter: %s (%d/%d)" % ("PASS" if n_ok == len(RESULTS) else "FAIL", n_ok, len(RESULTS)))
sys.exit(0 if n_ok == len(RESULTS) else 1)
