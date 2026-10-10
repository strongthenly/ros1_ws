#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
#  offboard 定点悬停验证 (ROS1 + MAVROS + PX4)
#  流程: 预发 setpoint -> 切 OFFBOARD -> Arm -> 飞到 (0,0,5) -> 悬停 20s
#  输出: 实时位置 + 悬停期间最大漂移
#  用法: python3 ~/hover_verify.py     (需先跑 MAVROS)
# ============================================================
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

TARGET = (0.0, 0.0, 1.5)   # 目标点 x y z (m)
HOVER_SEC = 20.0           # 悬停观察时长
RATE_HZ = 20.0

state = State()
cur = PoseStamped()

def cb_state(msg):
    global state
    state = msg

def cb_pose(msg):
    global cur
    cur = msg

def main():
    rospy.init_node('hover_verify', anonymous=True)
    rospy.Subscriber('/mavros/state', State, cb_state)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, cb_pose)
    pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rate = rospy.Rate(RATE_HZ)

    rospy.loginfo('等待 FCU 连接 ...')
    while not rospy.is_shutdown() and not state.connected:
        rate.sleep()
    rospy.loginfo('FCU 已连接, mode=%s', state.mode)

    sp = PoseStamped()
    sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = TARGET
    sp.pose.orientation.w = 1.0

    # 预发 setpoint(>2s) 才允许切 OFFBOARD
    rospy.loginfo('预发 setpoint 2s ...')
    for _ in range(int(RATE_HZ * 3)):
        pub.publish(sp)
        rate.sleep()

    rospy.wait_for_service('/mavros/set_mode')
    rospy.wait_for_service('/mavros/cmd/arming')
    set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

    rospy.loginfo('切 OFFBOARD ...')
    set_mode(custom_mode='OFFBOARD')
    rospy.loginfo('Arm ...')
    arming(True)
    rospy.sleep(1.0)

    rospy.loginfo('悬停 %.0fs, 目标 %s ...', HOVER_SEC, str(TARGET))
    t0 = rospy.Time.now()
    samples = []
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < HOVER_SEC:
        pub.publish(sp)
        rate.sleep()
        p = cur.pose.position
        samples.append((p.x, p.y, p.z))
        rospy.loginfo('pos x=%.3f y=%.3f z=%.3f  mode=%s', p.x, p.y, p.z, state.mode)

    if samples:
        # 只统计"到达目标后"的稳态段（排除起飞爬升过程, 否则会把 0->5m 算成漂移）
        stable = [s for s in samples if s[2] > TARGET[2] - 0.5]
        stable = stable if len(stable) > 10 else samples
        xs = [s[0] for s in stable]; ys = [s[1] for s in stable]; zs = [s[2] for s in stable]
        drift = max(max(xs)-min(xs), max(ys)-min(ys), max(zs)-min(zs))
        rospy.loginfo('=' * 50)
        rospy.loginfo('x 范围: %.3f ~ %.3f', min(xs), max(xs))
        rospy.loginfo('y 范围: %.3f ~ %.3f', min(ys), max(ys))
        rospy.loginfo('z 范围: %.3f ~ %.3f', min(zs), max(zs))
        rospy.loginfo('最大漂移: %.3f m', drift)
        rospy.loginfo('判定: %s', '✅ 定点正常(<0.5m)' if drift < 0.5 else '⚠️ 漂移偏大, 检查定位')
    rospy.loginfo('落地 (LAND) ...')
    set_mode(custom_mode='AUTO.LAND')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
