#!/usr/bin/env python3
"""
cloud_raw_lidar_to_odom.py
将 /odin1/cloud_raw (frame=lidar) 变换到 odom 系 + 体素降采样 + 地面过滤 → /odin1/cloud_raw_odom
- 地面过滤: z < 0.1 的点剔除(地面不再被当障碍, 修复"飞机被障碍包围")
- 动态障碍: 当前帧点云, 人走即消失
"""
import rospy
import tf2_ros
import tf2_sensor_msgs
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
import numpy as np

GROUND_Z = 0.10   # 地面过滤阈值(m): 低于此高度的点(地面/噪声)剔除

class CloudLidarToOdom:
    def __init__(self):
        rospy.init_node('cloud_raw_lidar_to_odom', anonymous=True)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher('/odin1/cloud_raw_odom', PointCloud2, queue_size=1, latch=True)
        rospy.Subscriber('/odin1/cloud_raw', PointCloud2, self.cb, queue_size=1, buff_size=2**24)
        rospy.loginfo("cloud_raw_lidar_to_odom started (ground_filt=%.2f, voxel 0.05)" % GROUND_Z)

    def cb(self, msg):
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom', msg.header.frame_id, rospy.Time(0),
                rospy.Duration(0.2))
            out = tf2_sensor_msgs.do_transform_cloud(msg, trans)
            out.header.frame_id = 'odom'
            out.header.stamp = rospy.Time.now()
            try:
                gen = pc2.read_points(out, field_names=("x", "y", "z"), skip_nans=True)
                arr = np.array(list(gen), dtype=np.float32)
                if arr.shape[0] > 0:
                    # 1) 地面过滤: 剔除 z < GROUND_Z 的点
                    mask = arr[:, 2] >= GROUND_Z
                    arr = arr[mask]
                    # 2) 体素降采样 5cm
                    if arr.shape[0] > 2000:
                        key = np.floor(arr / 0.05).astype(np.int32)
                        _, idx = np.unique(key, axis=0, return_index=True)
                        arr = arr[idx]
                    out = pc2.create_cloud_xyz32(out.header, arr.tolist())
            except Exception as e:
                rospy.logwarn_throttle(5, "filter fail: %s" % e)
            self.pub.publish(out)
        except Exception as e:
            rospy.logwarn_throttle(2, "TF fail: %s" % e)

if __name__ == '__main__':
    try:
        CloudLidarToOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
