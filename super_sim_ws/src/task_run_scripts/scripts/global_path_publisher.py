#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
global_path_publisher.py — 路径A: 录制航迹 → /global_path + /global_path_marker + CH9 返航触发
============================================================================================================
用途: 给 SUPER 规划器注入外部全局路径引导(guide_path)。读 data_guidepoints/guidepoints_new.txt (CSV 7列),
     发布 /global_path 给 FSM, 同时发 /global_path_marker (LINE_STRIP 绿线) 供 RViz 显示。

新增功能 (2026-08-24, CH9 返航):
  ~reverse       启动时反向发布 (默认 false) —— 未用时可忽略
  CH9 下降沿触发返航 (与 CH10 触发语义完全一致: last>1500 && now<1500, 先拨高位再拉低):
    → ① pkill 精确杀 waypoint_mission (停发去程终点 goal, 防新旧 goal 打架)
    → ② 切换为反向发布 (航迹起点变终点, FSM 收到新 guide_path 自动重规划返航)
    → ③ 发布返航 goal = 新终点(原航迹起点) 到 /waypoint_goal (latch+周期重发)
  注意: 平时(未触发 CH9)绝不发布 /waypoint_goal (坑33: 发布器发 goal 会绕过 CH10 门禁,
        这里 CH9 是用户主动拨杆=与 CH10 同级触发, 语义安全)。

用法:
  rosrun task_run_scripts global_path_publisher.py _file:=/path/to/guidepoints_new.txt
参数:
  ~file        航迹文件路径
  ~min_pt_dist 抽稀间距, 0=不抽稀 (默认 0.0)
  ~frame_id    坐标系 (默认 map)
  ~rate        重发频率 Hz (默认 1.0)
  ~reverse     启动反向 (默认 false)
"""
import csv
import math
import subprocess

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from mavros_msgs.msg import RCIn

# 全局状态 (函数式保持改动最小)
_wps = []            # 原始航迹 (加载顺序)
_cur_wps = []        # 当前发布方向 (正向或反向)
_reverse = False     # 启动反向标志
_returned = False    # 是否已触发返航 (防重复)
_last_ch9 = 1500.0   # CH9 上次值 (下降沿检测)
_pub = None
_pub_marker = None
_pub_goal = None
_frame_id = "map"


def load_waypoints(path, min_dist=0.0):
    """读 CSV(x,y,z,qx,qy,qz,qw), min_dist<=0 不抽稀全发, 返回 [(x,y,z), ...]"""
    pts = []
    with open(path) as f:
        reader = csv.reader(f)
        next(reader, None)  # 跳过表头
        for row in reader:
            if len(row) >= 3:
                try:
                    pts.append((float(row[0]), float(row[1]), float(row[2])))
                except ValueError:
                    continue
    if not pts:
        rospy.logwarn("[global_path] 文件为空: %s", path)
        return []
    if min_dist <= 0:
        kept = pts
        rospy.loginfo("[global_path] 不抽稀, 全发 %d 点", len(kept))
    else:
        kept = [pts[0]]
        for p in pts[1:]:
            dx = p[0] - kept[-1][0]
            dy = p[1] - kept[-1][1]
            dz = p[2] - kept[-1][2]
            if math.sqrt(dx * dx + dy * dy + dz * dz) >= min_dist:
                kept.append(p)
        rospy.loginfo("[global_path] 加载 %d 点 (抽稀 %d->%d, min_dist=%.2fm)",
                      len(pts), len(pts), len(kept), min_dist)
    return kept


def build_msgs():
    """按 _cur_wps 构建 path_msg + marker_array (每次切换方向后重建)"""
    global _pub, _pub_marker, _frame_id
    path_msg = Path()
    path_msg.header.frame_id = _frame_id
    for (x, y, z) in _cur_wps:
        ps = PoseStamped()
        ps.header.frame_id = _frame_id
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.w = 1.0
        path_msg.poses.append(ps)

    marker = Marker()
    marker.header.frame_id = _frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "global_path"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.08
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    for (x, y, z) in _cur_wps:
        pt = Point()
        pt.x, pt.y, pt.z = x, y, z
        marker.points.append(pt)
    marker_array = MarkerArray()
    marker_array.markers.append(marker)
    return path_msg, marker_array


def publish_return_goal():
    """发布返航 goal = 当前发布方向的终点 (原航迹起点) 到 /waypoint_goal"""
    global _pub_goal
    if not _cur_wps:
        return
    gx, gy, gz = _cur_wps[-1]
    goal = PoseStamped()
    goal.header.frame_id = _frame_id
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = gx
    goal.pose.position.y = gy
    goal.pose.position.z = gz
    goal.pose.orientation.w = 1.0
    _pub_goal.publish(goal)
    rospy.loginfo("[global_path] 返航 goal -> /waypoint_goal = (%.3f, %.3f, %.3f)", gx, gy, gz)


def trigger_return():
    """CH9 下降沿: 杀 waypoint_mission + 切反向 + 发返航 goal"""
    global _cur_wps, _returned, _wps
    if _returned:
        return
    _returned = True
    rospy.loginfo("[global_path] ⚡ CH9 下降沿触发返航!")

    # ① 精确杀 waypoint_mission (停发去程 goal; 字符类防 pkill 自误杀)
    try:
        subprocess.Popen(["pkill", "-f", "waypoint_missio[n]"])
        rospy.loginfo("[global_path] 已杀 waypoint_mission (去程 goal 停止)")
    except Exception as e:
        rospy.logwarn("[global_path] pkill waypoint_mission 失败: %s", e)

    # ② 切换方向: 当前方向反转 (正向->反向)
    _cur_wps = list(reversed(_cur_wps))
    rospy.loginfo("[global_path] 已切换为反向发布, 新终点 = 原航迹起点 (%.3f, %.3f, %.3f)",
                  _cur_wps[-1][0], _cur_wps[-1][1], _cur_wps[-1][2])


def rc_callback(msg):
    """/mavros/rc/in 回调: 检测 CH9 (channels[8]) 下降沿 (last>1500 && now<1500)"""
    global _last_ch9, _returned
    if len(msg.channels) < 9:
        return
    ch9 = float(msg.channels[8])
    if (not _returned) and (_last_ch9 > 1500.0 and ch9 < 1500.0):
        trigger_return()
    _last_ch9 = ch9


def main():
    global _wps, _cur_wps, _reverse, _pub, _pub_marker, _pub_goal, _frame_id
    rospy.init_node("global_path_publisher")
    _frame_id = rospy.get_param("~frame_id", "map")
    _reverse = rospy.get_param("~reverse", False)
    rate = rospy.Rate(rospy.get_param("~rate", 1.0))

    _pub = rospy.Publisher("/global_path", Path, queue_size=1, latch=True)
    _pub_marker = rospy.Publisher("/global_path_marker", MarkerArray, queue_size=1, latch=True)
    _pub_goal = rospy.Publisher("/waypoint_goal", PoseStamped, queue_size=1, latch=True)
    rospy.Subscriber("/mavros/rc/in", RCIn, rc_callback, queue_size=1)

    _wps = load_waypoints(rospy.get_param("~file",
        "/home/fouruav/catkin_ws/src/task_run_scripts/data_guidepoints/guidepoints_new.txt"),
        rospy.get_param("~min_pt_dist", 0.0))
    if not _wps:
        rospy.logerr("[global_path] 无有效航点, 退出")
        return
    _cur_wps = list(reversed(_wps)) if _reverse else list(_wps)
    path_msg, marker_array = build_msgs()
    rospy.loginfo("[global_path] 发布 %d 个航点 -> /global_path (frame=%s, reverse=%s)",
                  len(path_msg.poses), _frame_id, _reverse)

    while not rospy.is_shutdown():
        path_msg.header.stamp = rospy.Time.now()
        marker_array.markers[0].header.stamp = rospy.Time.now()
        _pub.publish(path_msg)
        _pub_marker.publish(marker_array)
        if _returned:
            publish_return_goal()   # 返航后周期重发返航 goal (latch 双保险)
        rate.sleep()


if __name__ == "__main__":
    main()
