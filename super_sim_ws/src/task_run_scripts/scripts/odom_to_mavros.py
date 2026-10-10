#!/usr/bin/env python3
import os
import sys
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs

# 单实例锁: pid 文件方式(防止多个 odom_to_mavros 抢发 vision_pose)
_LOCK = '/tmp/odom_to_mavros.lock'
try:
    if os.path.exists(_LOCK):
        with open(_LOCK) as _lf:
            _old_pid = int(_lf.read().strip() or 0)
        try:
            os.kill(_old_pid, 0)  # 进程存在?
            print(f"[odom_to_mavros] 已有实例 PID={_old_pid} 在运行, 本实例退出", flush=True)
            sys.exit(0)
        except (ProcessLookupError, ValueError):
            pass  # 旧进程已死, 继续
    with open(_LOCK, 'w') as _lf:
        _lf.write(str(os.getpid()))
except Exception as _e:
    print(f"[odom_to_mavros] 锁检查异常({_e}), 继续运行", flush=True)

class OdomToVisionPoseTF:
    def __init__(self):
        rospy.init_node('odom_to_vision_pose', anonymous=True)

        # 发布到 MAVROS vision pose 话题
        self.vision_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

        # TF buffer & listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 订阅 odom（高频）
        rospy.Subscriber('/odin1/odometry_highfreq', Odometry, self.odom_callback)
        self._last_pub = rospy.Time(0)

        rospy.loginfo("OdomToVisionPoseTF started")

    def odom_callback(self, msg: Odometry):
        # 构造来自 odom 的 PoseStamped（frame_id 应为 odom）
        pose_odom = PoseStamped()
        pose_odom.header = msg.header
        pose_odom.pose = msg.pose.pose

        # 目标：把 pose 从 odom -> map
        try:
            target_frame = 'map'
            source_frame = pose_odom.header.frame_id or 'odom'
            # 高速回调里不要阻塞等待 TF；直接取最新可用变换，避免把 400Hz 输入拖成低频输出。
            trans = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(0.0)
            )
            pose_map = tf2_geometry_msgs.do_transform_pose(pose_odom, trans)

            # 发布给 PX4/MAVROS：使用新鲜时间戳并设置 frame_id 为 map
            pose_map.header.stamp = rospy.Time.now()
            pose_map.header.frame_id = target_frame
            # 限速 50Hz(20ms): PX4 EKF 对 vision_pose 有频率上限, 400Hz 全转发会触发异常
            now = rospy.Time.now()
            if (now - self._last_pub).to_sec() >= 0.02:
                self._last_pub = now
                self.vision_pub.publish(pose_map)

            rospy.loginfo_throttle(10, "Published vision_pose in map frame")

        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException, tf2_ros.TransformException) as e:
            rospy.logwarn_throttle(5, "TF map->odom not available yet: %s" % str(e))

if __name__ == '__main__':
    try:
        converter = OdomToVisionPoseTF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass