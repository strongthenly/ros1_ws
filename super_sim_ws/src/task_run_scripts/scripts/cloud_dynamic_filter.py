#!/usr/bin/env python3
"""
cloud_dynamic_filter.py — FAPP 阶段1: 动态点云分割 (v3: 时间一致性滑动窗口)
原理: 体素计数表(每帧所有点 count++, 2s 遗忘)
  - count >= CONFIRM_N(5帧≈0.5s持续出现) → static(进地图)
  - count < CONFIRM_N(新出现) → dynamic(人/噪声, 剔除)
效果:
  静态物(墙/地面): 一直有 → 快速转static ✅
  快速穿行的人(<0.5s): 全程dynamic → 不进地图 → 无残留 ✅
  站着的人(>0.5s): 转static(等同静态障碍, 合理) ✅
  噪声: count低 + 密度聚类剔除 ✅
输出:
  /odin1/cloud_static   (静态, 供 SUPER 地图)
  /odin1/cloud_dynamic  (动态, RViz 可视化)
"""
import rospy
import tf2_ros
import tf2_sensor_msgs
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
import numpy as np
import threading
import time
from collections import defaultdict

VOX = 0.4           # 体素大小(m) (0.2→0.4: 容忍点云边界抖动, 静态物体素稳定才能count到60)
CONFIRM_N = 60      # 静态确认帧数(≈6s 连续出现: 穿行的人<6s全程dynamic不进地图; 站6s+的静止物转static)
FORGET_T = 8.0      # 遗忘时间(s): 超过未命中则清除(需>CONFIRM时长6s)
GROUND_Z = 0.10     # 地面过滤
DENSITY_R = 0.35    # 密度聚类半径(m)
DENSITY_K = 3       # 密度阈值

class DynamicFilter:
    def __init__(self):
        rospy.init_node('cloud_dynamic_filter', anonymous=True)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub_static = rospy.Publisher('/odin1/cloud_static', PointCloud2, queue_size=1, latch=True)
        self.pub_dynamic = rospy.Publisher('/odin1/cloud_dynamic', PointCloud2, queue_size=1, latch=True)
        rospy.Subscriber('/odin1/cloud_raw', PointCloud2, self.cb, queue_size=1, buff_size=2**24)
        self.voxel_cnt = {}   # coord(tuple) -> (count, last_time)
        self.lock = threading.Lock()
        self.seq = 0
        rospy.loginfo("cloud_dynamic_filter v3 started (confirm=%d, forget=%.1fs)" % (CONFIRM_N, FORGET_T))

    def cb(self, msg):
        try:
            trans = self.tf_buffer.lookup_transform('odom', msg.header.frame_id,
                                                    rospy.Time(0), rospy.Duration(0.2))
            out = tf2_sensor_msgs.do_transform_cloud(msg, trans)
            out.header.frame_id = 'odom'
            out.header.stamp = rospy.Time.now()

            gen = pc2.read_points(out, field_names=('x','y','z'), skip_nans=True)
            arr = np.array(list(gen), dtype=np.float32)
            if arr.shape[0] == 0:
                return
            arr = arr[arr[:, 2] >= GROUND_Z]
            if arr.shape[0] == 0:
                return

            now = time.time()
            self.seq += 1

            # 体素化
            key = np.floor(arr[:, :3] / VOX).astype(np.int32)
            keys = [tuple(k) for k in key]

            # 更新计数表(count++, 遗忘)
            with self.lock:
                # 当前帧体素去重计数
                from collections import Counter
                key_cnt = Counter(keys)
                for k, c in key_cnt.items():
                    if k in self.voxel_cnt:
                        cnt, _ = self.voxel_cnt[k]
                        self.voxel_cnt[k] = (min(cnt + c, 20), now)
                    else:
                        self.voxel_cnt[k] = (c, now)
                # 遗忘: 超时清除(每 20 帧做一次, 减开销)
                if self.seq % 20 == 0:
                    stale = [k for k, (c, t) in self.voxel_cnt.items() if now - t > FORGET_T]
                    for k in stale:
                        del self.voxel_cnt[k]

            # 判定 static/dynamic
            static_mask = np.array([self.voxel_cnt[k][0] >= CONFIRM_N for k in keys], dtype=bool)
            static_pts = arr[static_mask]
            dyn_cand = arr[~static_mask]

            # 密度聚类(动态候选: 成块保留, 散点剔除)
            dyn_pts = self.density_filter(dyn_cand) if len(dyn_cand) > 0 else dyn_cand

            if len(static_pts) > 0:
                self.pub_static.publish(pc2.create_cloud_xyz32(out.header, static_pts.tolist()))
            if len(dyn_pts) > 0:
                self.pub_dynamic.publish(pc2.create_cloud_xyz32(out.header, dyn_pts.tolist()))
        except Exception as e:
            rospy.logwarn_throttle(2, "filter fail: %s" % e)

    def density_filter(self, pts):
        if len(pts) <= DENSITY_K:
            return np.zeros((0,3), dtype=np.float32)
        from scipy.spatial import cKDTree
        tree = cKDTree(pts[:, :3])
        cnt = tree.query_ball_point(pts[:, :3], DENSITY_R, return_length=True)
        return pts[np.array(cnt) >= DENSITY_K]

if __name__ == '__main__':
    try:
        DynamicFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
