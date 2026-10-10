#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FUEL 输入桥: FAST-LIVO2  →  FUEL
  订阅:  /aft_mapped_to_init (nav_msgs/Odometry)    ← FAST-LIVO2 位姿
         /cloud_registered   (sensor_msgs/PointCloud2) ← FAST-LIVO2 配准点云
  发布:  /odom_world    (nav_msgs/Odometry)     → FUEL 里程计
         /map_ros/pose  (geometry_msgs/PoseStamped) → FUEL 传感器位姿(用于 raycast)
         /map_ros/cloud (sensor_msgs/PointCloud2)    → FUEL 点云输入
  处理:  把 frame_id 从 camera_init 统一改写成 world（FUEL 内部约定）
"""
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2

WORLD = "world"


class FuelInputBridge:
    def __init__(self):
        self.odom_pub = rospy.Publisher("/odom_world", Odometry, queue_size=20)
        self.pose_pub = rospy.Publisher("/map_ros/pose", PoseStamped, queue_size=20)
        self.cloud_pub = rospy.Publisher("/map_ros/cloud", PointCloud2, queue_size=20)

        self.n_odom = 0
        self.n_cloud = 0
        rospy.Subscriber("/aft_mapped_to_init", Odometry, self.odom_cb, queue_size=20)
        rospy.Subscriber("/cloud_registered", PointCloud2, self.cloud_cb, queue_size=5)

        rospy.Timer(rospy.Duration(5.0), self.report)
        rospy.loginfo("[fuel_bridge] FAST-LIVO2 -> FUEL 输入桥已启动")

    def odom_cb(self, msg):
        msg.header.frame_id = WORLD
        msg.child_frame_id = "body"
        self.odom_pub.publish(msg)

        p = PoseStamped()
        p.header.stamp = msg.header.stamp
        p.header.frame_id = WORLD
        p.pose = msg.pose.pose
        self.pose_pub.publish(p)

        self.n_odom += 1

    def cloud_cb(self, msg):
        msg.header.frame_id = WORLD
        self.cloud_pub.publish(msg)
        self.n_cloud += 1

    def report(self, _):
        rospy.loginfo("[fuel_bridge] odom=%d  cloud=%d (累计)", self.n_odom, self.n_cloud)


if __name__ == "__main__":
    rospy.init_node("fuel_input_bridge")
    FuelInputBridge()
    rospy.spin()
