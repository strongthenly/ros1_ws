#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FUEL 输出控制桥:  FUEL  →  MAVROS/PX4
  订阅:  /planning/pos_cmd   (quadrotor_msgs/PositionCommand)
  发布:  /mavros/setpoint_position/local (geometry_msgs/PoseStamped, ENU)
         /mavros/setpoint_raw/local      (mavros_msgs/PositionTarget) [可选 raw 模式]

坐标系: FUEL 的 position 在 camera_init(ENU)。MAVROS 的 setpoint_position/local
        在 ROS 侧同样是 ENU，MAVROS 内部自动转 PX4 的 NED —— 所以我们不做手工转换。

模式:
  ~mode=position (默认)  位置 + yaw（最稳，PX4 位置控制）
  ~mode=raw              位置 + 速度 + 加速度 + yaw（轨迹跟踪更准，需 PX4 加速度限制已配好）

用法:
  rosrun fuel_bridge fuel_ctrl_bridge.py
"""
import math
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from quadrotor_msgs.msg import PositionCommand

# MAVROS PositionTarget type_mask 的"忽略位"常量
IGN = dict(px=1, py=2, pz=4, vx=8, vy=16, vz=32,
           afx=64, afy=128, afz=256, force=512, yaw=1024, yaw_rate=2048)


def yaw_of(q):
    """从四元数取 yaw(rad)"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_pi(a):
    """把角度归一到 (-pi, pi]"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def rot_z(v, psi):
    """把向量 v 绕 z 轴旋转 psi(rad)"""
    c, s = math.cos(psi), math.sin(psi)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1], v[2])


def yaw_to_quat(yaw):
    """把 yaw(rad) 转成只绕 z 轴的四元数 (x,y,z,w)"""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class FuelCtrlBridge:
    def __init__(self):
        self.mode = rospy.get_param("~mode", "position")          # position | raw
        self.auto_offboard = rospy.get_param("~auto_offboard", True)
        self.rate_hz = rospy.get_param("~rate", 50.0)
        self.hold_last = rospy.get_param("~hold_last", True)      # 无新指令时保持最后 setpoint
        self.offboard_wait = rospy.get_param("~offboard_wait", 3.0)  # 预发 setpoint 秒数
        # seed 悬停: >0 时，在收到 FUEL 指令前先把无人机托到该高度悬停（解决"FUEL未触发→无setpoint→无法起飞"的时序矛盾）
        self.seed_z = rospy.get_param("~seed_z", 1.5)
        # 坐标系朝向对齐：FAST-LIVO2 的 camera_init 与 PX4 EKF 的 yaw 定义不同（实测差 ~82°）
        # 不对齐的话 FUEL 的目标会被 PX4 朝错误方向执行 → 撞墙
        self.auto_align = rospy.get_param("~auto_align", True)
        self.align_samples = int(rospy.get_param("~align_samples", 15))   # 用来平均的样本数
        self.psi_offset_deg = rospy.get_param("~psi_offset_deg", None)    # 手动指定则跳过自动对齐
        self.psi_offset = None      # rad；= yaw_px4 - yaw_fuel
        self._dyaw_buf = []
        if self.psi_offset_deg is not None:
            self.psi_offset = math.radians(float(self.psi_offset_deg))

        self.state = State()
        self.last_cmd = None
        self.cur_pos = None     # 当前本地位置（ENU，来自 /mavros/local_position/pose）
        self.px4_yaw = None     # PX4/ENU 系 yaw
        self.fuel_yaw = None    # FUEL(camera_init) 系 yaw
        self.seeded = False     # 是否已开始 seed 悬停
        self.t0 = None          # 首次发出 setpoint 的时间
        self.switched = False   # 是否已切过 OFFBOARD
        self._next_action = None  # 下次允许执行 切模式/arm 的时间（节流用）

        rospy.Subscriber("/mavros/state", State, self.state_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pos_cb)
        rospy.Subscriber("/odom_world", Odometry, self.odom_cb)
        rospy.Subscriber("/planning/pos_cmd", PositionCommand, self.cmd_cb)
        if self.psi_offset_deg is not None:
            rospy.loginfo("[fuel_ctrl] 手动指定朝向对齐 psi_offset = %.2f deg", self.psi_offset_deg)

        self.pub_pos = rospy.Publisher("/mavros/setpoint_position/local",
                                       PoseStamped, queue_size=10)
        self.pub_raw = rospy.Publisher("/mavros/setpoint_raw/local",
                                       PositionTarget, queue_size=10)

        self.sm = None
        self.ar = None
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.tick)
        rospy.loginfo("[fuel_ctrl] 控制桥启动 mode=%s rate=%.0fHz auto_offboard=%s",
                      self.mode, self.rate_hz, self.auto_offboard)

    # ---------------- callbacks ----------------
    def state_cb(self, msg):
        self.state = msg

    def pos_cb(self, msg):
        self.cur_pos = msg.pose.position
        self.px4_yaw = yaw_of(msg.pose.orientation)
        self._try_align()

    def odom_cb(self, msg):
        self.fuel_yaw = yaw_of(msg.pose.pose.orientation)
        self._try_align()

    def cmd_cb(self, msg):
        self.last_cmd = msg

    # ---------------- 朝向对齐估计 ----------------
    def _try_align(self):
        """用 (yaw_px4 - yaw_fuel) 的平均值估计两坐标系朝向差；取够样本后锁定"""
        if self.psi_offset is not None or not self.auto_align:
            return
        if self.px4_yaw is None or self.fuel_yaw is None:
            return
        self._dyaw_buf.append(wrap_pi(self.px4_yaw - self.fuel_yaw))
        if len(self._dyaw_buf) >= self.align_samples:
            # 用 sin/cos 平均，避免 ±180° 附近的绕圈问题
            s = sum(math.sin(d) for d in self._dyaw_buf)
            c = sum(math.cos(d) for d in self._dyaw_buf)
            self.psi_offset = math.atan2(s, c)
            rospy.loginfo("[fuel_ctrl] 坐标系朝向对齐估计完成: psi_offset = %.2f deg (%d 样本)",
                          math.degrees(self.psi_offset), len(self._dyaw_buf))

    def aligned(self):
        """是否已具备可用的对齐参数"""
        return (self.psi_offset is not None) if self.auto_align else True

    def to_px4(self, p, yaw):
        """把 FUEL(camera_init) 系的目标位姿转到 PX4(ENU) 系"""
        psi = self.psi_offset if self.psi_offset is not None else 0.0
        x, y, z = rot_z((p.x, p.y, p.z), psi)
        return (x, y, z, wrap_pi(yaw + psi))

    # ---------------- seed 悬停（FUEL 接管前先把无人机托到 seed_z） ----------------
    def publish_seed(self):
        if self.seed_z <= 0 or self.cur_pos is None:
            return
        sp = PoseStamped()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = "world"
        sp.pose.position.x = self.cur_pos.x
        sp.pose.position.y = self.cur_pos.y
        sp.pose.position.z = self.seed_z
        sp.pose.orientation.w = 1.0
        self.pub_pos.publish(sp)
        if not self.seeded:
            self.seeded = True
            rospy.loginfo("[fuel_ctrl] seed 悬停开始: 抬到 %.2f m 等 FUEL 轨迹接管", self.seed_z)
        if self.t0 is None:
            self.t0 = rospy.Time.now()

    # ---------------- 切 OFFBOARD + Arm（seed 与 FUEL 两个阶段共用） ----------------
    def try_offboard(self):
        if not self.auto_offboard or not self.state.connected or self.t0 is None:
            return
        now = rospy.Time.now()
        if (now - self.t0).to_sec() < self.offboard_wait:
            return  # 预发阶段：先只发 setpoint（PX4 硬要求）
        if self.sm is None:
            # 不能做阻塞式服务等待（会卡住 tick 回调）；服务不存在时调用抛异常并被捕获
            self.sm = rospy.ServiceProxy("/mavros/set_mode", SetMode)
            self.ar = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
            rospy.loginfo("[fuel_ctrl] set_mode/arming 代理已创建")
        if self.state.mode != "OFFBOARD":
            # 节流：模式/arm 动作每 2 秒最多一次（否则 50Hz 刷屏）
            if self._next_action is None or now >= self._next_action:
                self._next_action = now + rospy.Duration(2.0)
                try:
                    self.sm(custom_mode="OFFBOARD")
                    rospy.loginfo("[fuel_ctrl] → 切 OFFBOARD")
                except Exception as e:
                    rospy.logwarn("[fuel_ctrl] set_mode 失败: %s", e)
        elif not self.state.armed:
            if self._next_action is None or now >= self._next_action:
                self._next_action = now + rospy.Duration(2.0)
                try:
                    self.ar(True)
                    rospy.loginfo("[fuel_ctrl] → Arm")
                except Exception as e:
                    rospy.logwarn("[fuel_ctrl] arming 失败: %s", e)

    # ---------------- main loop ----------------
    def tick(self, _):
        cmd = self.last_cmd
        if cmd is None:
            # 还没收到 FUEL 指令 → seed 悬停（先托起来，才能切 OFFBOARD）
            self.publish_seed()
            self.try_offboard()
            return

        p = cmd.position
        yaw = cmd.yaw
        if self.t0 is None:
            self.t0 = rospy.Time.now()
            rospy.loginfo("[fuel_ctrl] 收到首条 FUEL 指令")

        # 安全：朝向对齐未完成前不转发 FUEL 目标（否则会把无人机朝错误方向送）
        if not self.aligned():
            rospy.logwarn_throttle(2.0, "[fuel_ctrl] 朝向对齐未完成，暂不转发 FUEL 目标（防撞）")
            return

        # FUEL(camera_init) -> PX4(ENU) 坐标系转换
        x, y, z, yaw_p = self.to_px4(p, yaw)
        psi = self.psi_offset if self.psi_offset is not None else 0.0

        if self.mode == "raw":
            vx, vy, vz = rot_z((cmd.velocity.x, cmd.velocity.y, cmd.velocity.z), psi)
            ax, ay, az = rot_z((cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z), psi)
            tgt = PositionTarget()
            tgt.header.stamp = rospy.Time.now()
            tgt.header.frame_id = "world"
            tgt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            # 位掩码：使用 pos+vel+acc+yaw（忽略 force / yaw_rate）
            tgt.type_mask = IGN['force'] | IGN['yaw_rate']
            tgt.position.x, tgt.position.y, tgt.position.z = x, y, z
            tgt.velocity.x, tgt.velocity.y, tgt.velocity.z = vx, vy, vz
            tgt.acceleration_or_force.x = ax
            tgt.acceleration_or_force.y = ay
            tgt.acceleration_or_force.z = az
            tgt.yaw = yaw_p
            tgt.yaw_rate = 0.0
            self.pub_raw.publish(tgt)
        else:
            sp = PoseStamped()
            sp.header.stamp = rospy.Time.now()
            sp.header.frame_id = "world"
            sp.pose.position.x = x
            sp.pose.position.y = y
            sp.pose.position.z = z
            qx, qy, qz, qw = yaw_to_quat(yaw_p)
            sp.pose.orientation.x = qx
            sp.pose.orientation.y = qy
            sp.pose.orientation.z = qz
            sp.pose.orientation.w = qw
            self.pub_pos.publish(sp)

        # 切 OFFBOARD + Arm（与 seed 阶段共用同一套逻辑）
        self.try_offboard()


if __name__ == "__main__":
    rospy.init_node("fuel_ctrl_bridge")
    FuelCtrlBridge()
    rospy.spin()
