#!/bin/bash
# ============================================================
#  坐标系对齐诊断：对比 FAST-LIVO2(camera_init) 与 PX4 EKF(ENU)
#  只读，不发任何控制指令 —— 安全
#  用法: bash ~/fuel_frame_check.sh
# ============================================================
export ROS_MASTER_URI=http://127.0.0.1:11311
source /opt/ros/noetic/setup.bash 2>/dev/null
source ~/twin_ws/devel/setup.bash 2>/dev/null

python3 - <<'PY'
import math, sys
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

rospy.init_node('fuel_frame_check', anonymous=True, disable_signals=True)

def yaw_of(q):
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

def wrap(d):
    while d > 180: d -= 360
    while d < -180: d += 360
    return d

fuel = px4 = None

try:
    o = rospy.wait_for_message('/aft_mapped_to_init', Odometry, timeout=10)
    p, q = o.pose.pose.position, o.pose.pose.orientation
    fuel = (p.x, p.y, p.z, math.degrees(yaw_of(q)))
    print("=== FAST-LIVO2  (/aft_mapped_to_init, frame=camera_init) ===")
    print("   pos = (%.3f, %.3f, %.3f)" % fuel[:3])
    print("   yaw = %.2f deg" % fuel[3])
except Exception as e:
    print("=== FAST-LIVO2 无数据: %s" % e)
    print("    (先跑 bash ~/twin_livo_fast.sh --full)")

try:
    m = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped, timeout=10)
    p, q = m.pose.position, m.pose.orientation
    px4 = (p.x, p.y, p.z, math.degrees(yaw_of(q)))
    print("")
    print("=== PX4 / MAVROS  (/mavros/local_position/pose, frame=ENU) ===")
    print("   pos = (%.3f, %.3f, %.3f)" % px4[:3])
    print("   yaw = %.2f deg" % px4[3])
except Exception as e:
    print("")
    print("=== PX4 无数据: %s" % e)
    print("    (MAVROS 没起 / 没连上 PX4)")

print("")
print("================== 结论 ==================")
if fuel and px4:
    dx, dy, dz = (fuel[0]-px4[0]), (fuel[1]-px4[1]), (fuel[2]-px4[2])
    dyaw = wrap(fuel[3] - px4[3])
    print("位置偏差  : dx=%.3f  dy=%.3f  dz=%.3f  (模长 %.3f m)" % (dx, dy, dz, math.sqrt(dx*dx+dy*dy+dz*dz)))
    print("YAW 偏差  : %.2f deg   <-- 关键！" % dyaw)
    if abs(dyaw) > 10:
        print(">>> 结论: 两个坐标系朝向差 %.0f 度，FUEL 的目标点会被 PX4 朝错误方向执行" % dyaw)
        print(">>> 这正是'朝墙飞'的原因 —— 需要给控制桥加 yaw 对齐")
    elif math.sqrt(dx*dx+dy*dy+dz*dz) > 0.5:
        print(">>> 结论: 朝向基本一致，但原点差较大 → 需要加平移对齐")
    else:
        print(">>> 结论: 两个坐标系基本对齐（<10°/0.5m），撞墙另有原因（见 max_ray_length / 未知区域）")
else:
    print("数据不全，无法判断 —— 请先确保 twin 链路 + MAVROS 都跑着")
PY
