# #!/usr/bin/env python3
# import rospy
# from nav_msgs.msg import Odometry

# class OdomToPX4:
#     def __init__(self):
#         rospy.init_node('odom_to_px4', anonymous=True)
        
#         # 修改发布者，直接发布 Odometry 消息到 MAVROS 的里程计接口
#         self.odom_pub = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size=10)
        
#         # 订阅高频 odom
#         rospy.Subscriber('/odin1/odometry', Odometry, self.odom_callback)
        
#         rospy.loginfo("Full Odom to PX4 converter started")

#     def odom_callback(self, msg):
#         try:
#             # 创建一个新的 Odometry 对象以避免污染原始数据
#             px4_odom = Odometry()
#             px4_odom.header = msg.header
#             px4_odom.child_frame_id = msg.child_frame_id
#             px4_odom.pose = msg.pose
#             px4_odom.twist = msg.twist

#             # 关键修正1：时间戳更新为当前时间
#             # px4_odom.header.stamp = rospy.Time.now()
#             px4_odom.header.stamp = msg.header.stamp
            
#             # 关键修正2：设置标准的 frame_id
#             px4_odom.header.frame_id = "odom" # 视你的 TF 树而定，通常使用 "odom" 或 "map"
#             px4_odom.child_frame_id = "base_link"
            
#             # 关键修正3：保留你的高度补偿逻辑
#             px4_odom.pose.pose.position.z = px4_odom.pose.pose.position.z 
            
#             # 直接发布完整的消息，MAVROS 插件会自动处理 ENU 到 NED 的转换并映射给 PX4
#             self.odom_pub.publish(px4_odom)
            
#             rospy.loginfo_throttle(10, "Successfully converted full odom to PX4 odometry")
            
#         except Exception as e:
#             rospy.logerr("Error in odom callback: %s", str(e))

# if __name__ == '__main__':
#     try:
#         converter = OdomToPX4()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass


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
# class OdomToVisionPose:
#     def __init__(self):
#         rospy.init_node('odom_to_vision_pose', anonymous=True)
        
#         # 创建发布者，发布到 /mavros/vision_pose/pose
#         self.vision_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        
#         # 修正订阅话题名称为实际使用的 /odin1/odometry_highfreq
#         rospy.Subscriber('/odin1/odometry_highfreq', Odometry, self.odom_callback)
        
#         rospy.loginfo("Odom to Vision Pose converter started")

#     def odom_callback(self, msg):
#         """
#         回调函数：将Odometry消息转换为PoseStamped消息
#         """
#         try:
#             # 创建新的PoseStamped消息
#             vision_msg = PoseStamped()
            
#             # 关键修正1：时间戳必须使用当前时间，PX4要求新鲜时间戳 msg.header.stamp rospy.Time.now()
#             vision_msg.header.stamp = rospy.Time.now()
#             # 关键修正2：frame_id应设置为"map"，表示全局固定坐标系
#             vision_msg.header.frame_id = "map"
            
#             # 关键修正3：直接复制位姿数据，无需手动转换坐标系
#             # MAVROS的vision_pose插件会自动进行ENU(ROS)到NED(PX4)的转换
#             vision_msg.pose = msg.pose.pose
#             vision_msg.pose.position.z = vision_msg.pose.position.z  
#             # 发布转换后的消息
#             self.vision_pub.publish(vision_msg)
            
#             # 限速日志，避免刷屏
#             rospy.loginfo_throttle(10, "Successfully converted odom to vision_pose")
            
#         except Exception as e:
#             rospy.logerr("Error in odom callback: %s", str(e))
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
            # 限速 20Hz(50ms): PX4 EKF 对 vision_pose 有频率上限, 400Hz全转发会触发异常
            now = rospy.Time.now()
            if (now - self._last_pub).to_sec() >= 0.05:
                self._last_pub = now
                self.vision_pub.publish(pose_map)

        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException, tf2_ros.TransformException) as e:
            rospy.logwarn_throttle(5, "TF map->odom not available yet: %s" % str(e))

def run_odom_to_mavros():
    """入口：启动里程计->MAVROS 转发节点（供编译为 .so 后由启动器调用）"""
    try:
        # converter = OdomToVisionPose()
        converter = OdomToVisionPoseTF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    run_odom_to_mavros()