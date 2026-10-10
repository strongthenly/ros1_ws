#!/usr/bin/env python3
# coding=utf-8
import rospy
import math
from nav_msgs.msg import Odometry
from mavros_msgs.srv import SetMode, SetModeRequest
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int32
import os

from mavros_msgs.msg import ExtendedState
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import BatteryState

# --- 全局变量 ---
pos_x, pos_y, pos_z = 0, 0, 0
cur_yaw = 0.0
battery_voltage = 25.2  # 默认满电电压 6S
all_number = Int32() 

yaw_tolerance_rad = math.radians(10.0)


# --- 回调函数 ---
def battery_cb(msg):
    global battery_voltage
    battery_voltage = msg.voltage

def pose_cb(m):
    """无人机自身位置与偏航角回调"""
    global pos_x, pos_y, pos_z, cur_yaw
    pos_x = m.pose.pose.position.x
    pos_y = m.pose.pose.position.y
    pos_z = m.pose.pose.position.z
    qx = m.pose.pose.orientation.x
    qy = m.pose.pose.orientation.y
    qz = m.pose.pose.orientation.z
    qw = m.pose.pose.orientation.w
    _, _, cur_yaw = euler_from_quaternion([qx, qy, qz, qw])


# --- 工具函数 ---
def angle_diff(a, b):
    """计算角度差（最短角距），返回[-pi, pi]"""
    return math.atan2(math.sin(b - a), math.cos(b - a))

def publish_pose(pose_stamped):
    """发布位姿控制指令"""
    goal_pos_pub.publish(pose_stamped)


# --- 核心类与控制函数 ---
class WaypointLoader:
    def __init__(self, filename):
        self.filename = filename
        self.waypoints = []
        self.load_waypoints()
    
    def load_waypoints(self):
        """从文件加载航点"""
        global all_number
        if not os.path.exists(self.filename):
            rospy.logerr(f"航点文件 {self.filename} 不存在!")
            return False
        
        with open(self.filename, 'r') as f:
            for line in f:
                parts = line.strip().split(',')
                if len(parts) == 7:  # 位置+四元数格式
                    try:
                        wp = {
                            'position': (float(parts[0]), float(parts[1]), float(parts[2])),
                            'orientation': {
                                'x': float(parts[3]),
                                'y': float(parts[4]),
                                'z': float(parts[5]),
                                'w': float(parts[6])
                            }
                        }
                        self.waypoints.append(wp)
                    except ValueError:
                        rospy.logwarn(f"无法解析航点: {line.strip()}")
        rospy.loginfo(f"成功加载 {len(self.waypoints)} 个航点")
        all_number.data = len(self.waypoints)
        return len(self.waypoints) > 0 
    
    def get_waypoints(self):
        return self.waypoints

def navigate_to_waypoint(wp, timeout=20):
    """导航到特定航点，监控是否到达"""
    global pos_x, pos_y, pos_z, cur_yaw, yaw_tolerance_rad

    Tolerant_distance = 0.25
    
    # 创建PoseStamped消息
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = wp['position'][0]
    goal.pose.position.y = wp['position'][1]
    goal.pose.position.z = wp['position'][2]
    goal.pose.orientation.x = wp['orientation']['x']
    goal.pose.orientation.y = wp['orientation']['y']
    goal.pose.orientation.z = wp['orientation']['z']
    goal.pose.orientation.w = wp['orientation']['w']

    publish_pose(goal)
    rospy.loginfo(f"导航到航点: X={wp['position'][0]:.2f}, Y={wp['position'][1]:.2f}, Z={wp['position'][2]:.2f}")

    # 目标航向
    _, _, des_yaw = euler_from_quaternion([
        wp['orientation']['x'], wp['orientation']['y'], 
        wp['orientation']['z'], wp['orientation']['w']
    ])

    start_time = rospy.Time.now().to_sec()
    distance = float('inf')
    yaw_error = float('inf')

    while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - start_time) < timeout:
        distance = math.sqrt(
            (wp['position'][0] - pos_x)**2 +
            (wp['position'][1] - pos_y)**2 +
            (wp['position'][2] - pos_z)**2
        )
            
        yaw_error = abs(angle_diff(cur_yaw, des_yaw))
        
        
        # 判断是否到达
        if distance < Tolerant_distance and yaw_error < yaw_tolerance_rad:
            rospy.loginfo(f"到达目标位置! 位置误差: {distance:.2f}米, 航向误差: {math.degrees(yaw_error):.1f}°")
            return True


        rospy.sleep(0.1)

    rospy.logwarn(f"导航到航点超时! 最终距离: {distance:.2f}米, 最终航向误差: {math.degrees(yaw_error):.1f}°")
    return False

def land_action():
    while setModeServer.call(Land_set_mode).mode_sent == False:
        rospy.loginfo("请求降落模式...")
        rospy.sleep(0.5)
    rospy.loginfo("已切换至降落模式 (AUTO.LAND)")

def POSCTL_action():
    while setModeServer(custom_mode='POSCTL').mode_sent == False:
        rospy.loginfo("请求定点模式 (POSCTL)...")
        rospy.sleep(0.5)
    rospy.loginfo("已切换至定点模式 (POSCTL)")


# --- 主函数 ---
if __name__ == '__main__':
    rospy.init_node("Goal_Navigation_task_node")

    # 获取参数
    waypoint_file = rospy.get_param('~waypoint_file', '/home/fouruav/odin_ws/src/task_run_scripts/waypoints.txt')
    yaw_tolerance_deg = rospy.get_param('~yaw_tolerance_deg', 10.0)
    yaw_tolerance_rad = math.radians(yaw_tolerance_deg)
    
    # 话题订阅与发布
    rospy.Subscriber('/mavros/local_position/odom', Odometry, pose_cb)
    rospy.Subscriber('/mavros/battery', BatteryState, battery_cb)
    
    goal_pos_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)


    # 服务客户端
    setModeServer = rospy.ServiceProxy('mavros/set_mode', SetMode)
    Land_set_mode = SetModeRequest()
    Land_set_mode.custom_mode = 'AUTO.LAND'

    rospy.sleep(1.0)
    rospy.loginfo("极简版导航与电量监控系统已启动")

    # 加载航点
    waypoint_loader = WaypointLoader(waypoint_file)
    waypoints = waypoint_loader.get_waypoints()
    if not waypoints:
        rospy.logerr("没有可用的航点，系统将退出")
        exit(1)


    VOLTAGE_THRESHOLD = 21.6      # 6S返航电压阈值 (3.6V * 6)
    LAND_VOLTAGE_THRESHOLD = 20.4 # 6S迫降电压阈值 (3.4V * 6)
    POS_EX_FLAG = 0

    # 导航核心循环
    for i, wp in enumerate(waypoints):
        # 1. 检查电池电量
        if battery_voltage < VOLTAGE_THRESHOLD:
            rospy.logwarn(f"电池电压过低: {battery_voltage:.2f}V < {VOLTAGE_THRESHOLD}V, 中断任务并返航!")
            
            # 发布返航点 (0,0,1)
            return_home = PoseStamped()
            return_home.header.frame_id = 'map'
            return_home.header.stamp = rospy.Time.now()
            return_home.pose.position.x = 0
            return_home.pose.position.y = 0
            return_home.pose.position.z = 1
            return_home.pose.orientation.w = 1.0 # 保持默认朝向
            publish_pose(return_home)
            rospy.loginfo("已发布返航点(0,0,1)")

            HOME_DISTANCE_THRESHOLD = 0.3
            rospy.loginfo("开始返航监控...")
            
            while not rospy.is_shutdown():
                distance_to_home = math.sqrt(pos_x**2 + pos_y**2 + (pos_z - 1)**2) # 注意这里Z轴算的是到高度1的距离

                # 到达返航点上方，执行降落
                if distance_to_home < HOME_DISTANCE_THRESHOLD:
                    rospy.loginfo("已到达返航点上方(0,0,1)，执行降落")
                    POSCTL_action()
                    land_action()
                    POS_EX_FLAG = 1
                    break

                # 返航途中电压严重不足，直接迫降
                if battery_voltage < LAND_VOLTAGE_THRESHOLD:
                    rospy.logwarn(f"返航途中电压危急: {battery_voltage:.2f}V < {LAND_VOLTAGE_THRESHOLD}V, 就地迫降!")
                    POSCTL_action()
                    land_action()
                    POS_EX_FLAG = 1
                    break

                rospy.sleep(0.5)
            break # 触发返航后，跳出航点遍历循环

        # 2. 正常导航到下一个航点
        rospy.loginfo(f"正在导航到航点 {i+1}/{len(waypoints)}")
        if not navigate_to_waypoint(wp):
            rospy.logwarn(f"导航到航点 {i+1} 失败，尝试下一个航点")

    # 所有航点执行完毕或触发返航后的最终降落处理
    if POS_EX_FLAG == 0:
        rospy.loginfo("所有航点任务完成，准备降落...")
        POSCTL_action()
        land_action()

    rospy.loginfo("节点运行结束!")