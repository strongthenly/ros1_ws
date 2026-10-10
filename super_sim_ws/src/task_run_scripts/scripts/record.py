#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import os
import sys
import termios
import tty

class WaypointRecorder:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('waypoint_recorder', anonymous=True)
        
        # 订阅无人机位姿话题
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb)
        
        # 当前位姿变量
        self.current_pose = None
        self.pose_received = False
        
        # 保存文件路径
        self.filename = "/home/fouruav/odin_ws/src/task_run_scripts/waypoints.txt"
        
        # 检查文件是否存在
        if os.path.exists(self.filename):
            choice = input("File exists! Overwrite? (y/n): ")
            if choice.lower() == 'y':
                open(self.filename, 'w').close()
            else:
                sys.exit(0)
                
        # 设置键盘监听
        self.setup_keyboard()

    def pose_cb(self, msg):
        """位姿回调函数"""
        self.current_pose = msg.pose
        self.pose_received = True

    def setup_keyboard(self):
        """配置非阻塞键盘输入"""
        self.old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def restore_keyboard(self):
        """恢复键盘设置"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)

    def record_waypoint(self):
        """记录当前航点（直接保存四元数）"""
        if not self.pose_received:
            print("Waiting for first pose data...")
            return

        # 提取位置
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        z = self.current_pose.position.z
        
        # 直接提取四元数
        qx = self.current_pose.orientation.x
        qy = self.current_pose.orientation.y
        qz = self.current_pose.orientation.z
        qw = self.current_pose.orientation.w
        
        # 写入文件（位置+原始四元数）
        with open(self.filename, 'a') as f:
            line = f"{x:.6f}, {y:.6f}, {z:.6f}, {qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f}\n"
            f.write(line)
            print(f"Recorded: {line.strip()}")

    def run(self):
        print("Press [Enter] to record waypoint, [Esc] to exit...")
        try:
            while not rospy.is_shutdown():
                key = sys.stdin.read(1)
                if key == '\x1b':  # ESC键
                    break
                elif key == '\n' or key == '\r':  # 回车键
                    self.record_waypoint()
                rospy.sleep(0.3)
        finally:
            self.restore_keyboard()
            print("\nWaypoints saved to:", os.path.abspath(self.filename))

if __name__ == '__main__':
    recorder = WaypointRecorder()
    recorder.run()