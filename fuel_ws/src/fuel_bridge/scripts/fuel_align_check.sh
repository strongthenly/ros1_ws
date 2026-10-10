#!/bin/bash
# ============================================================
#  朝向对齐「实时」验证：FUEL 位姿/目标 vs PX4 位姿 是否满足旋转关系
#  原理: 若对齐正确，应有  p_px4 ≈ Rz(psi) · p_fuel
#  只读，不发任何控制指令 —— 安全
#  用法: bash ~/fuel_align_check.sh [采样秒数]
# ============================================================
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://127.0.0.1:11311}
source /opt/ros/noetic/setup.bash 2>/dev/null
source ~/twin_ws/devel/setup.bash 2>/dev/null
source ~/fuel_ws/devel/setup.bash 2>/dev/null

DUR=${1:-12}
python3 - "$DUR" <<'PY'
import math, sys
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand

dur = float(sys.argv[1])
rospy.init_node('fuel_align_check', anonymous=True, disable_signals=True)

def yaw_of(q): return math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))
def wrap(a):
    while a > math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a
def rotz(v, p):
    c, s = math.cos(p), math.sin(p)
    return (c*v[0]-s*v[1], s*v[0]+c*v[1])

S = {'fuel': None, 'px4': None, 'cmd': None}
def cb_fuel(m):
    p = m.pose.pose.position
    S['fuel'] = (p.x, p.y, p.z, yaw_of(m.pose.pose.orientation))
def cb_px4(m):
    p = m.pose.position
    S['px4'] = (p.x, p.y, p.z, yaw_of(m.pose.orientation))
def cb_cmd(m):
    p = m.position
    S['cmd'] = (p.x, p.y, p.z, m.yaw)

rospy.Subscriber('/odom_world', Odometry, cb_fuel, queue_size=1)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, cb_px4, queue_size=1)
rospy.Subscriber('/planning/pos_cmd', PositionCommand, cb_cmd, queue_size=1)
rospy.sleep(2)

print("采样 %.0f 秒 ...（Ctrl+C 可中断）\n" % dur)
t0 = rospy.Time.now()
errs, psis = [], []
while (rospy.Time.now()-t0).to_sec() < dur:
    f, x = S['fuel'], S['px4']
    if f and x:
        psi = wrap(x[3]-f[3])                      # 实测朝向差
        exp = rotz((f[0], f[1]), psi)              # Rz(psi)·p_fuel
        err = math.hypot(x[0]-exp[0], x[1]-exp[1])
        psis.append(psi); errs.append(err)
        print("FUEL pos=(%6.2f,%6.2f) yaw=%6.1f | PX4 pos=(%6.2f,%6.2f) yaw=%6.1f | psi=%6.1f | 预测PX4=(%6.2f,%6.2f) 残差=%.3f m"
              % (f[0],f[1],math.degrees(f[3]), x[0],x[1],math.degrees(x[3]),
                 math.degrees(psi), exp[0], exp[1], err))
    else:
        miss = [k for k in ('fuel','px4') if not S[k]]
        print("等待数据: 缺 %s（fuel=/odom_world, px4=/mavros/local_position/pose）" % ",".join(miss))
    rospy.sleep(1.0)

print("\n================== 结论 ==================")
if not errs:
    print("数据不足 —— 确保 FAST-LIVO2 + 输入桥 + MAVROS 都在跑")
    sys.exit(1)
avg_psi = math.degrees(sum(psis)/len(psis))
avg_err = sum(errs)/len(errs)
mx = max(errs)
print("平均 psi_offset = %.2f deg" % avg_psi)
print("平均残差        = %.3f m   (最大 %.3f m)" % (avg_err, mx))
if avg_err < 0.3:
    print(">>> 对齐正确：PX4 位置与 Rz(psi)·FUEL位置 吻合（残差 <0.3m）")
    print(">>> 说明 FUEL 的目标已被正确变换，方向问题已解决")
elif avg_err < 1.0:
    print(">>> 基本对齐（残差 0.3~1m）—— 可能有轻微漂移/滞后，可接受")
else:
    print(">>> 对齐异常（残差 >1m）—— 朝向关系不成立，需要检查（可能 psi 符号反了）")
    print(">>> 补救：给控制桥加 psi_offset_deg:=%.2f 手动指定并观察" % (-avg_psi))
PY
