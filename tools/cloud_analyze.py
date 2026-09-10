#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 对比 /twin/lidar/points_preview (原始桥输出) 和 /cloud_registered (FAST-LIVO2 输出)
import rospy, numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def analyze(topic, name):
    try:
        msg = rospy.wait_for_message(topic, PointCloud2, timeout=12)
    except Exception as e:
        print("[%s] 超时: %s" % (name, e)); return
    pts = np.array(list(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True)))
    print("=" * 55)
    print("[%s]  frame_id=%s" % (name, msg.header.frame_id))
    print("  点数: %d" % len(pts))
    if len(pts) == 0:
        return
    for i, ax in enumerate('xyz'):
        v = pts[:, i]
        print("  %s: min=%.3f max=%.3f 范围=%.3f 中位=%.3f" % (ax, v.min(), v.max(), v.max()-v.min(), np.median(v)))
    # 垂直角分布(相对雷达水平面)
    r = np.linalg.norm(pts[:, :2], axis=1)
    ang = np.degrees(np.arctan2(pts[:, 2], r))
    print("  俯仰角: min=%.1f° max=%.1f°  中位=%.1f°" % (ang.min(), ang.max(), np.median(ang)))
    print("  俯仰角 >0(上方)占比: %.1f%%" % (100.0 * np.sum(ang > 0) / len(ang)))
    print("  俯仰角 <-7°(超出FOV下界)占比: %.1f%%" % (100.0 * np.sum(ang < -7) / len(ang)))
    print("  水平距离: min=%.2f max=%.2f" % (r.min(), r.max() if len(r) else 0))

if __name__ == '__main__':
    rospy.init_node('cloud_analyze', anonymous=True)
    analyze('/twin/lidar/points_preview', '桥原始点云 /twin/lidar')
    analyze('/cloud_registered', 'FAST-LIVO2 /cloud_registered')
