#!/bin/bash
# ============================================================
#  FUEL 建图质量检查（只读，不发控制指令）
#  判断 FUEL 的 ESDF/占据图是否真的"看见"了障碍
#  用法: bash ~/fuel_map_check.sh
# ============================================================
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://127.0.0.1:11311}
source /opt/ros/noetic/setup.bash 2>/dev/null
source ~/twin_ws/devel/setup.bash 2>/dev/null
source ~/fuel_ws/devel/setup.bash 2>/dev/null

python3 - <<'PY'
import struct, math
import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand

rospy.init_node('fuel_map_check', anonymous=True, disable_signals=True)

def pts(topic, timeout=15):
    try:
        m = rospy.wait_for_message(topic, PointCloud2, timeout=timeout)
    except Exception:
        return None
    n = m.width * m.height
    step = m.point_step
    out = []
    for i in range(n):
        b = i * step
        try:
            x, y, z = struct.unpack_from('<fff', m.data, b)
        except Exception:
            break
        if -1e6 < x < 1e6:
            out.append((x, y, z))
    return out

def stat(name, topic, ref=None):
    p = pts(topic)
    if p is None:
        print("  %-26s : 无数据" % name); return None
    xs = [a[0] for a in p]; ys = [a[1] for a in p]; zs = [a[2] for a in p]
    print("  %-26s : %6d 点   x[%7.2f,%7.2f] y[%7.2f,%7.2f] z[%6.2f,%6.2f]"
          % (name, len(p), min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)))
    if ref and p:
        near = [a for a in p if math.hypot(a[0]-ref[0], a[1]-ref[1]) < 3.0 and abs(a[2]-ref[2]) < 3.0]
        print("       └ 距参考点 3m 内的障碍点数: %d" % len(near))
    return p

print("=== 1) 无人机当前位姿 ===")
try:
    s = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped, timeout=8)
    ref = (s.pose.position.x, s.pose.position.y, s.pose.position.z)
    print("  PX4 位置  = (%.2f, %.2f, %.2f)" % ref)
except Exception:
    ref = None; print("  PX4 位置  = 无数据（MAVROS 没跑？）")

try:
    s = rospy.wait_for_message('/odom_world', __import__('nav_msgs.msg', fromlist=['Odometry']).Odometry, timeout=8)
    p = s.pose.pose.position
    print("  FUEL 位置 = (%.2f, %.2f, %.2f)" % (p.x, p.y, p.z))
except Exception:
    print("  FUEL 位置 = 无数据（输入桥没跑？）")

try:
    c = rospy.wait_for_message('/planning/pos_cmd', PositionCommand, timeout=8)
    print("  FUEL 目标 = (%.2f, %.2f, %.2f)  yaw=%.1f°" % (c.position.x, c.position.y, c.position.z, math.degrees(c.yaw)))
except Exception:
    print("  FUEL 目标 = 无数据（探索未触发？）")

print("")
print("=== 2) FUEL 建图（FUEL 眼里的世界）===")
stat("occupancy_all(全部障碍)", '/sdf_map/occupancy_all', ref)
stat("occupancy_local(局部)", '/sdf_map/occupancy_local', ref)
stat("occupancy_local_inflate", '/sdf_map/occupancy_local_inflate', ref)
stat("esdf(距离场)", '/sdf_map/esdf')
stat("unknown(未知区)", '/sdf_map/unknown')

print("")
print("=== 3) 判断 ===")
print("  若 occupancy_all 点数少(<300) 或范围小 → FUEL 没看见多少障碍，撞墙是'没看到'")
print("  若点数正常但目标点附近 3m 无点 → 目标落在未观测区（信息驱动探索的固有风险）")
print("  若 esdf 无数据 → ESDF 没在更新，规划器拿不到距离场（严重）")
PY
