#!/usr/bin/env python3
# TF ?????? Odin ??????? TF?odom->imu->lidar ??? ROS ???????
# ? RViz ? wall-clock ???? Marker/LaserScan ??????
# ???????????? map->odom?????????????????
import rospy
from tf2_msgs.msg import TFMessage

STALE_SKIP_SECS = 10.0

class TfClockProxy:
    def __init__(self):
        rospy.init_node('tf_clock_proxy', anonymous=True)
        self.pub = rospy.Publisher('/tf', TFMessage, queue_size=100)
        rospy.Subscriber('/tf', TFMessage, self.cb)
        rospy.loginfo("tf_clock_proxy started")

    def cb(self, msg):
        now = rospy.Time.now()
        out = TFMessage()
        for t in msg.transforms:
            dt = abs((now - t.header.stamp).to_sec())
            if dt < STALE_SKIP_SECS:
                continue  # ??????? TF ???????
            t.header.stamp = now
            out.transforms.append(t)
        if out.transforms:
            self.pub.publish(out)

if __name__ == '__main__':
    TfClockProxy()
    rospy.spin()
