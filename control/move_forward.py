#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 控制无人机沿机头方向前进 DIST 米 (OFFBOARD), 保持当前高度
import rospy, math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

DIST = 2.0

state = State(); cur = PoseStamped()
def cb_s(m):
    global state; state = m
def cb_p(m):
    global cur; cur = m

rospy.init_node('move_forward', anonymous=True)
rospy.Subscriber('/mavros/state', State, cb_s)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, cb_p)
pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
rate = rospy.Rate(20)

rospy.loginfo('等待 FCU ...')
while not rospy.is_shutdown() and not state.connected:
    rate.sleep()
rospy.sleep(1.5)

# 当前位姿 + 机头方向
q = cur.pose.orientation
yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
p0 = cur.pose.position
rospy.loginfo('当前位置: x=%.3f y=%.3f z=%.3f  yaw=%.1f°', p0.x, p0.y, p0.z, math.degrees(yaw))

tx = p0.x + DIST*math.cos(yaw)
ty = p0.y + DIST*math.sin(yaw)
tz = p0.z
rospy.loginfo('目标(前进 %.1fm): x=%.3f y=%.3f z=%.3f', DIST, tx, ty, tz)

sp = PoseStamped()
sp.pose.position.x = tx; sp.pose.position.y = ty; sp.pose.position.z = tz
sp.pose.orientation.w = 1.0

rospy.loginfo('预发 setpoint 3s ...')
for _ in range(60):
    pub.publish(sp); rate.sleep()

rospy.wait_for_service('/mavros/set_mode')
rospy.wait_for_service('/mavros/cmd/arming')
sm = rospy.ServiceProxy('/mavros/set_mode', SetMode)
ar = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

if state.mode != 'OFFBOARD':
    rospy.loginfo('切 OFFBOARD (当前 %s)', state.mode); sm(custom_mode='OFFBOARD')
if not state.armed:
    rospy.loginfo('Arm'); ar(True)
rospy.sleep(0.5)

rospy.loginfo('开始前进 ...')
t0 = rospy.Time.now()
while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 25:
    pub.publish(sp); rate.sleep()
    p = cur.pose.position
    err = math.sqrt((p.x-tx)**2 + (p.y-ty)**2 + (p.z-tz)**2)
    moved = math.sqrt((p.x-p0.x)**2 + (p.y-p0.y)**2)
    rospy.loginfo('pos x=%.3f y=%.3f z=%.3f | 已移动 %.3fm | 距目标 %.3fm', p.x, p.y, p.z, moved, err)
    if err < 0.20:
        rospy.loginfo('=' * 50)
        rospy.loginfo('✅ 已到达目标! 实际移动 %.3f m, 误差 %.3f m', moved, err)
        break

rospy.loginfo('悬停保持 5s ...')
t1 = rospy.Time.now()
while not rospy.is_shutdown() and (rospy.Time.now() - t1).to_sec() < 5:
    pub.publish(sp); rate.sleep()
rospy.loginfo('完成 (保持 OFFBOARD 悬停)')
