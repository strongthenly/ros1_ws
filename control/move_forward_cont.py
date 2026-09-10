#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 控制无人机持续向前飞 (OFFBOARD 位置 setpoint 连续递增)
# 安全上限: 最多 MAX_DIST 米 / MAX_SEC 秒, 之后停住悬停
import rospy, math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

SPEED    = 2.0    # 前进速度 m/s
MAX_DIST = 40.0   # 最远飞 40m
MAX_SEC  = 40.0   # 最长 40s

state = State(); cur = PoseStamped()
def cb_s(m):
    global state; state = m
def cb_p(m):
    global cur; cur = m

rospy.init_node('move_forward_cont', anonymous=True)
rospy.Subscriber('/mavros/state', State, cb_s)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, cb_p)
pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
rate = rospy.Rate(20)

rospy.loginfo('等待 FCU ...')
while not rospy.is_shutdown() and not state.connected:
    rate.sleep()
rospy.sleep(1.0)

q = cur.pose.orientation
yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
p0 = cur.pose.position
z0 = p0.z
fx, fy = math.cos(yaw), math.sin(yaw)   # 机头方向单位向量
rospy.loginfo('起点 x=%.3f y=%.3f z=%.3f yaw=%.1f°  前进方向=(%.2f, %.2f)',
              p0.x, p0.y, p0.z, math.degrees(yaw), fx, fy)

sp = PoseStamped()
sp.pose.position.x = p0.x; sp.pose.position.y = p0.y; sp.pose.position.z = z0
sp.pose.orientation.w = 1.0

# 预发 2s
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

rospy.loginfo('开始持续前进 (%.1f m/s, 上限 %.0fm/%.0fs) ...', SPEED, MAX_DIST, MAX_SEC)
t0 = rospy.Time.now()
d = 0.0
last_t = t0
last_log = -99.0
while not rospy.is_shutdown():
    now = rospy.Time.now()
    dt = (now - last_t).to_sec(); last_t = now
    d += SPEED * dt
    sp.pose.position.x = p0.x + fx * d
    sp.pose.position.y = p0.y + fy * d
    sp.pose.position.z = z0
    pub.publish(sp)
    rate.sleep()
    p = cur.pose.position
    moved = math.sqrt((p.x-p0.x)**2 + (p.y-p0.y)**2)
    if d - last_log >= 2.0:   # 约每 2m 打一次
        last_log = d
        rospy.loginfo('指令 d=%.1fm | 实际位置 x=%.2f y=%.2f z=%.2f | 已飞 %.1fm',
                      d, p.x, p.y, p.z, moved)
    if d >= MAX_DIST or (now - t0).to_sec() >= MAX_SEC:
        rospy.loginfo('到达上限 (d=%.1fm, t=%.1fs), 停止前进', d, (now-t0).to_sec())
        break

rospy.loginfo('保持悬停 5s ...')
t1 = rospy.Time.now()
while not rospy.is_shutdown() and (rospy.Time.now()-t1).to_sec() < 5:
    pub.publish(sp); rate.sleep()
p = cur.pose.position
moved = math.sqrt((p.x-p0.x)**2 + (p.y-p0.y)**2)
rospy.loginfo('=' * 50)
rospy.loginfo('✅ 结束: 实际飞了 %.2f m, 终点 x=%.2f y=%.2f z=%.2f', moved, p.x, p.y, p.z)
