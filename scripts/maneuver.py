#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 机动序列: 前进10m -> 右转90度 -> 前进20m  (OFFBOARD 位置+yaw setpoint)
import rospy, math
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

D1 = 10.0
D2 = 20.0
SPEED = 2.0
TURN_SEC = 4.0

state = State(); cur = PoseStamped()
def cb_s(m):
    global state; state = m
def cb_p(m):
    global cur; cur = m

rospy.init_node('maneuver', anonymous=True)
rospy.Subscriber('/mavros/state', State, cb_s)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, cb_p)
pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
rate = rospy.Rate(20)

sp = PoseStamped()

def set_yaw(y):
    sp.pose.orientation.x = 0.0
    sp.pose.orientation.y = 0.0
    sp.pose.orientation.z = math.sin(y / 2.0)
    sp.pose.orientation.w = math.cos(y / 2.0)

def set_pos(p):
    sp.pose.position.x = p.x
    sp.pose.position.y = p.y
    sp.pose.position.z = p.z

def hold(p, y, sec):
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < sec:
        set_pos(p); set_yaw(y); pub.publish(sp); rate.sleep()

def fly(p0, yaw, dist, label):
    rospy.loginfo('--- %s (yaw=%.1f°) ---', label, math.degrees(yaw))
    t0 = rospy.Time.now(); d = 0.0; last = t0; lastlog = -99.0
    while not rospy.is_shutdown():
        now = rospy.Time.now(); dt = (now - last).to_sec(); last = now
        d = min(d + SPEED * dt, dist)
        pt = Point(p0.x + math.cos(yaw) * d, p0.y + math.sin(yaw) * d, p0.z)
        set_pos(pt); set_yaw(yaw); pub.publish(sp); rate.sleep()
        p = cur.pose.position
        moved = math.sqrt((p.x - p0.x) ** 2 + (p.y - p0.y) ** 2)
        if d - lastlog >= 5.0:
            lastlog = d
            rospy.loginfo('  指令 d=%.1fm | pos x=%.2f y=%.2f z=%.2f | 实际移动 %.1fm',
                          d, p.x, p.y, p.z, moved)
        if d >= dist:
            break
    hold(Point(p0.x + math.cos(yaw) * dist, p0.y + math.sin(yaw) * dist, p0.z), yaw, 1.5)

def turn(p_hold, y0, y1):
    rospy.loginfo('--- 转向 %.1f° -> %.1f° (右转 %.0f°) ---',
                  math.degrees(y0), math.degrees(y1), math.degrees(y0 - y1))
    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        f = min((rospy.Time.now() - t0).to_sec() / TURN_SEC, 1.0)
        set_pos(p_hold); set_yaw(y0 + (y1 - y0) * f); pub.publish(sp); rate.sleep()
        if f >= 1.0:
            break
    hold(p_hold, y1, 1.5)

# ---- 主流程 ----
rospy.loginfo('等待 FCU ...')
while not rospy.is_shutdown() and not state.connected:
    rate.sleep()
rospy.sleep(1.0)

q = cur.pose.orientation
yaw0 = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
p0 = Point(cur.pose.position.x, cur.pose.position.y, cur.pose.position.z)
z0 = p0.z
rospy.loginfo('起点 x=%.2f y=%.2f z=%.2f yaw=%.1f°', p0.x, p0.y, p0.z, math.degrees(yaw0))

set_pos(p0); set_yaw(yaw0)
for _ in range(40):
    pub.publish(sp); rate.sleep()

rospy.wait_for_service('/mavros/set_mode')
rospy.wait_for_service('/mavros/cmd/arming')
sm = rospy.ServiceProxy('/mavros/set_mode', SetMode)
ar = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
if state.mode != 'OFFBOARD':
    sm(custom_mode='OFFBOARD')
if not state.armed:
    ar(True)
rospy.sleep(0.5)

# ① 前进 D1
fly(p0, yaw0, D1, '第①段 前进 %.0fm' % D1)
p1 = Point(cur.pose.position.x, cur.pose.position.y, z0)
yaw1 = yaw0 - math.radians(90.0)     # 右转 90°
# ② 转向
turn(p1, yaw0, yaw1)
# ③ 前进 D2
fly(p1, yaw1, D2, '第③段 前进 %.0fm' % D2)

pf = cur.pose.position
rospy.loginfo('=' * 55)
rospy.loginfo('✅ 机动完成: 终点 x=%.2f y=%.2f z=%.2f', pf.x, pf.y, pf.z)
rospy.loginfo('   起终点直线距离 %.2f m (起点 %.2f,%.2f -> 终点 %.2f,%.2f)',
              math.sqrt((pf.x - p0.x) ** 2 + (pf.y - p0.y) ** 2), p0.x, p0.y, pf.x, pf.y)
rospy.loginfo('保持悬停 5s ...')
hold(Point(pf.x, pf.y, z0), yaw1, 5.0)
