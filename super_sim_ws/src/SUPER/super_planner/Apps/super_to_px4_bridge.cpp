/**
 * super_to_px4_bridge.cpp
 *
 * 功能说明：
 *   SUPER路径规划器与PX4飞控之间的桥接节点。
 *   将SUPER输出的PositionCommand（位置+速度+加速度+偏航）转换为
 *   MAVROS的setpoint_raw指令发送给PX4，实现精确的轨迹跟踪控制。
 *
 * 主要特性：
 *   1. 位置+速度前馈+加速度前馈的复合控制模式（抗风/高动态跟踪）
 *   2. 偏航角/偏航角速率两种控制模式
 *   3. 超时检测：SUPER指令丢失后自动切换到位置保持
 *   4. 自动解锁、自动切OFFBOARD
 *   5. 自动降落流程（服务触发/低电量触发/地理围栏触发）
 *   6. 失效保护：遥控器信号丢失检测、低电量检测、地理围栏、MAVROS连接检测
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/BatteryState.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <Eigen/Eigen>
#include <tf/tf.h>
#include <cmath>
#include <vector>

class BridgeController {
private:
    ros::NodeHandle nh;          // 全局节点句柄
    ros::NodeHandle private_nh;  // 私有节点句柄，用于读取参数

    // ======================== 订阅者 ========================
    ros::Subscriber super_cmd_sub;        // SUPER路径规划器输出的指令
    ros::Subscriber goal_sub;             // RViz 2D Nav Goal 发送的目标点
    ros::Subscriber current_pose_sub;     // MAVROS反馈的当前位姿
    ros::Subscriber state_sub;            // MAVROS飞控状态（模式/解锁状态）
    ros::Subscriber extended_state_sub;   // MAVROS扩展状态（着陆检测等）
    ros::Subscriber rc_sub;               // 遥控器输入（用于信号丢失检测）
    ros::Subscriber battery_sub;          // 电池状态（用于低电量保护）

    // ======================== 发布者 ========================
    ros::Publisher setpoint_raw_pub;       // 发送给PX4的原始指令（位置+速度+加速度）
    ros::Publisher setpoint_position_pub;  // 纯位置指令（用于起飞/保持阶段）
    ros::Publisher armed_status_pub;       // 发布解锁状态给其他节点
    ros::Publisher offboard_status_pub;    // 发布OFFBOARD模式状态给其他节点

    // ======================== 服务 ========================
    ros::ServiceServer land_service;       // 对外提供的降落服务接口
    ros::ServiceClient arming_client;      // MAVROS解锁/上锁服务
    ros::ServiceClient set_mode_client;    // MAVROS飞行模式切换服务
    ros::ServiceClient land_client;        // MAVROS降落命令服务

    // ======================== 状态变量 ========================
    quadrotor_msgs::PositionCommand last_super_cmd;  // 最近一次收到的SUPER指令
    geometry_msgs::PoseStamped current_goal;          // 当前目标点（来自RViz）
    geometry_msgs::PoseStamped current_pose;          // 当前位姿（来自MAVROS反馈）
    geometry_msgs::PoseStamped init_goal;             // 起飞目标高度
    geometry_msgs::PoseStamped hold_position;         // 指令超时后的位置保持点

    mavros_msgs::State current_state;          // 飞控当前状态
    mavros_msgs::ExtendedState extended_state; // 飞控扩展状态（着陆检测）

    bool super_cmd_active;        // SUPER指令是否处于活跃状态
    bool use_current_yaw;         // 收到新目标点时是否沿用当前偏航角
    bool offboard_mode_engaged;   // 是否已切入OFFBOARD模式
    bool use_yaw_angle_control;   // true=偏航角控制, false=偏航角速率控制
    // ======================== 起飞后扫描参数 (2026-08-19 新增) ========================
    // 起飞悬停后先原地转 ±scan_angle_deg 扫描建图, 把障碍物周围未知区变已知, 再进入主循环
    bool scan_after_takeoff_en;   // 是否在起飞后执行扫描阶段
    double scan_angle_deg;        // 扫描角度(度), 默认90 = 转 ±90° 覆盖左右两侧
    double scan_hold_time;        // 每个扫描方向保持时间(秒), 默认1.0
    double scan_yaw_rate;         // 扫描偏航角速度(rad/s), 默认1.0 (比正常0.5快, 扫描不磨叽)
    // ==============================================================================
    bool landing_active;          // 是否正在执行降落流程
    bool armed;                   // 飞控是否已解锁
    ros::Time last_super_cmd_time;  // 上次收到SUPER指令的时间戳
    ros::Time last_print_time;      // 上次打印状态的时间戳（控制日志频率）
    ros::Time last_rc_time;         // 上次收到遥控器信号的时间戳

    // ======================== 基本控制参数 ========================
    double timeout_threshold;    // SUPER指令超时阈值（秒），超时后切换到位置保持
    double max_velocity_xy;      // XY方向最大速度限制（m/s）
    double max_velocity_z;       // Z方向最大速度限制（m/s）
    double max_yaw_rate;         // 最大偏航角速率（rad/s）
    double max_yaw_angle_rate;   // 偏航角控制模式下的最大变化速率（rad/s）
    std::string super_cmd_topic; // SUPER指令的ROS话题名

    // ======================== 位置控制参数 ========================
    double position_control_gain;  // 位置控制增益
    double max_position_error;     // 最大允许位置误差

    // ======================== 速度/加速度前馈参数 ========================
    // 前馈控制用于抗风和提高轨迹跟踪精度
    // 纯位置控制在室外环境下会因跟踪滞后导致炸机
    bool use_velocity_ff;       // 是否使用速度前馈
    bool use_accel_ff;          // 是否使用加速度前馈
    double velocity_ff_gain;    // 速度前馈增益（0~1），用于调节前馈强度
    // 振荡修复: 加速度前馈一阶低通(绕行/恢复切换时加速度突变 → 平滑)
    Eigen::Vector3d prev_acc_ff{Eigen::Vector3d::Zero()};
    double acc_ff_smooth{0.5};  // 低通系数: 0.5=平衡(50%新+50%旧), 平滑与响应折中

    // ======================== 自动解锁/切模式参数 ========================
    bool auto_arm;       // 是否自动解锁
    bool auto_offboard;  // 是否自动切入OFFBOARD模式

    // ======================== 降落参数 ========================
    double land_speed;   // 降落下降速度（m/s）
    double land_height;  // 触发MAVROS降落服务的高度阈值（m）
    bool auto_disarm;    // 着陆后是否自动上锁

    // ======================== 失效保护参数 ========================
    bool failsafe_en;            // 是否启用失效保护
    double battery_threshold;    // 低电量阈值（百分比）
    double rc_loss_timeout;      // 遥控器信号丢失超时（秒）
    bool geofence_en;            // 是否启用地理围栏
    double geofence_x_min, geofence_x_max;  // X方向围栏范围
    double geofence_y_min, geofence_y_max;  // Y方向围栏范围
    double geofence_z_min, geofence_z_max;  // Z方向围栏范围

public:
    /**
     * 构造函数：初始化所有参数、订阅者、发布者和服务
     */
    BridgeController() :
        private_nh("~"),
        super_cmd_active(false),
        use_current_yaw(true),
        offboard_mode_engaged(false),
        use_yaw_angle_control(true),
        landing_active(false),
        armed(false),
        last_print_time(ros::Time::now()),
        last_rc_time(ros::Time::now()) {

        // 从参数服务器读取基本控制参数
        private_nh.param<double>("timeout_threshold", timeout_threshold, 0.5);
        private_nh.param<double>("max_velocity_xy", max_velocity_xy, 1.2);
        private_nh.param<double>("max_velocity_z", max_velocity_z, 1.0);
        private_nh.param<double>("max_yaw_rate", max_yaw_rate, 1.5);
        private_nh.param<double>("max_yaw_angle_rate", max_yaw_angle_rate, 0.5);
        private_nh.param<std::string>("super_cmd_topic", super_cmd_topic, "/planning/pos_cmd");
        private_nh.param<bool>("use_current_yaw", use_current_yaw, true);
        private_nh.param<bool>("use_yaw_angle_control", use_yaw_angle_control, true);

        // 起飞后扫描参数 (2026-08-19 新增)
        private_nh.param<bool>("scan_after_takeoff_en", scan_after_takeoff_en, false);
        private_nh.param<double>("scan_angle_deg", scan_angle_deg, 90.0);
        private_nh.param<double>("scan_hold_time", scan_hold_time, 1.0);
        private_nh.param<double>("scan_yaw_rate", scan_yaw_rate, 1.0);

        // 从参数服务器读取位置控制参数
        private_nh.param<double>("position_control_gain", position_control_gain, 1.0);
        private_nh.param<double>("max_position_error", max_position_error, 0.5);

        // 从参数服务器读取速度/加速度前馈参数
        // 默认开启，纯位置控制在室外会因跟踪滞后导致炸机
        private_nh.param<bool>("use_velocity_ff", use_velocity_ff, true);
        private_nh.param<bool>("use_accel_ff", use_accel_ff, true);
        private_nh.param<double>("velocity_ff_gain", velocity_ff_gain, 0.7);

        // 从参数服务器读取自动解锁/切模式参数
        private_nh.param<bool>("auto_arm", auto_arm, false);
        private_nh.param<bool>("auto_offboard", auto_offboard, false);

        // 从参数服务器读取降落参数
        private_nh.param<double>("land_speed", land_speed, 0.3);
        private_nh.param<double>("land_height", land_height, 0.15);
        private_nh.param<bool>("auto_disarm", auto_disarm, true);

        // 从参数服务器读取失效保护参数
        private_nh.param<bool>("failsafe_en", failsafe_en, true);
        private_nh.param<double>("battery_threshold", battery_threshold, 20.0);
        private_nh.param<double>("rc_loss_timeout", rc_loss_timeout, 1.0);
        private_nh.param<bool>("geofence_en", geofence_en, false);
        private_nh.param<double>("geofence/x_min", geofence_x_min, -10.0);
        private_nh.param<double>("geofence/x_max", geofence_x_max, 10.0);
        private_nh.param<double>("geofence/y_min", geofence_y_min, -10.0);
        private_nh.param<double>("geofence/y_max", geofence_y_max, 10.0);
        private_nh.param<double>("geofence/z_min", geofence_z_min, 0.0);
        private_nh.param<double>("geofence/z_max", geofence_z_max, 5.0);

        // 从参数服务器读取初始起飞目标点
        private_nh.param<double>("init_goal/x", init_goal.pose.position.x, 0.0);
        private_nh.param<double>("init_goal/y", init_goal.pose.position.y, 0.0);
        private_nh.param<double>("init_goal/z", init_goal.pose.position.z, 1.0);
        init_goal.pose.orientation.w = 1.0;  // 四元数初始化为无旋转

        // 将当前目标和保持位置初始化为起飞目标点
        current_goal = init_goal;
        hold_position = init_goal;

        // ========== 注册订阅者 ==========
        super_cmd_sub = nh.subscribe(super_cmd_topic, 10, &BridgeController::superCmdCallback, this);
        goal_sub = nh.subscribe("/move_base_simple/goal", 10, &BridgeController::goalCallback, this);
        current_pose_sub = nh.subscribe("/mavros/local_position/pose", 10, &BridgeController::poseCallback, this);
        state_sub = nh.subscribe("/mavros/state", 10, &BridgeController::stateCallback, this);
        extended_state_sub = nh.subscribe("/mavros/extended_state", 10, &BridgeController::extendedStateCallback, this);
        rc_sub = nh.subscribe("/mavros/rc/in", 10, &BridgeController::rcCallback, this);
        battery_sub = nh.subscribe("/mavros/battery", 10, &BridgeController::batteryCallback, this);

        // ========== 注册发布者 ==========
        setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        setpoint_position_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        armed_status_pub = nh.advertise<std_msgs::Bool>("/bridge/armed", 1);
        offboard_status_pub = nh.advertise<std_msgs::Bool>("/bridge/offboard", 1);

        // ========== 注册服务 ==========
        land_service = nh.advertiseService("/bridge/land", &BridgeController::landCallback, this);
        arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

        // 打印初始化参数信息，方便调试
        ROS_INFO("=== super_to_px4_bridge initialized ===");
        ROS_INFO("  Timeout threshold: %.2f s", timeout_threshold);
        ROS_INFO("  Max velocity XY: %.2f m/s", max_velocity_xy);
        ROS_INFO("  Max velocity Z: %.2f m/s", max_velocity_z);
        ROS_INFO("  Max yaw rate: %.2f rad/s", max_yaw_rate);
        ROS_INFO("  Max yaw angle rate: %.2f rad/s", max_yaw_angle_rate);
        ROS_INFO("  SUPER command topic: %s", super_cmd_topic.c_str());
        ROS_INFO("  Use current yaw: %s", use_current_yaw ? "true" : "false");
        ROS_INFO("  Yaw control mode: %s", use_yaw_angle_control ? "ANGLE" : "RATE");
        ROS_INFO("  Initial goal: (%.2f, %.2f, %.2f)",
                 init_goal.pose.position.x,
                 init_goal.pose.position.y,
                 init_goal.pose.position.z);
        ROS_INFO("  Auto arm: %s", auto_arm ? "true" : "false");
        ROS_INFO("  Auto OFFBOARD: %s", auto_offboard ? "true" : "false");
        ROS_INFO("  Land speed: %.2f m/s", land_speed);
        ROS_INFO("  Land height: %.2f m", land_height);
        ROS_INFO("  Auto disarm: %s", auto_disarm ? "true" : "false");
        ROS_INFO("  Failsafe enabled: %s", failsafe_en ? "true" : "false");
        ROS_INFO("  Battery threshold: %.1f%%", battery_threshold);
        ROS_INFO("  RC loss timeout: %.1f s", rc_loss_timeout);
        ROS_INFO("  Geofence enabled: %s", geofence_en ? "true" : "false");
        ROS_INFO("  Velocity feedforward: %s", use_velocity_ff ? "true" : "false");
        ROS_INFO("  Acceleration feedforward: %s", use_accel_ff ? "true" : "false");
        ROS_INFO("  Velocity FF gain: %.2f", velocity_ff_gain);
    }

    // ======================== 回调函数 ========================

    /**
     * SUPER路径规划器指令回调
     * 接收PositionCommand消息（包含位置、速度、加速度、偏航角/角速率）
     * 每次收到新指令时更新时间戳，用于后续超时检测
     */
    void superCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
        last_super_cmd = *msg;
        last_super_cmd_time = ros::Time::now();
        super_cmd_active = true;
    }

    /**
     * RViz目标点回调
     * 收到2D Nav Goal后更新当前目标点
     * 如果use_current_yaw=true，则忽略目标点中的偏航角，沿用当前偏航
     */
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_goal = *msg;
        if (use_current_yaw) {
            current_goal.pose.orientation = current_pose.pose.orientation;
        }
        ROS_INFO("New goal received: x=%.2f, y=%.2f, z=%.2f",
                 current_goal.pose.position.x,
                 current_goal.pose.position.y,
                 current_goal.pose.position.z);
    }

    /** 当前位姿回调，缓存MAVROS反馈的机体位置和姿态 */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose = *msg;
    }

    /**
     * 飞控状态回调
     * 更新OFFBOARD模式标志和解锁状态
     */
    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_state = *msg;
        offboard_mode_engaged = (msg->mode == "OFFBOARD");
        armed = msg->armed;
    }

    /** 扩展状态回调，用于检测无人机是否已着陆 */
    void extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& msg) {
        extended_state = *msg;
    }

    /** 遥控器输入回调，仅更新时间戳用于信号丢失检测 */
    void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg) {
        last_rc_time = ros::Time::now();
    }

    /**
     * 电池状态回调
     * 当电量低于阈值时触发自动降落
     */
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
        if (failsafe_en && msg->percentage < battery_threshold / 100.0) {
            ROS_WARN("Battery low: %.1f%% < %.1f%%. Triggering landing.",
                     msg->percentage * 100.0, battery_threshold);
            triggerLanding();
        }
    }

    // ======================== 服务回调 ========================

    /**
     * 降落服务回调
     * 外部节点可通过调用 /bridge/land 服务触发降落
     */
    bool landCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
        ROS_INFO("Land service called.");
        triggerLanding();
        res.success = true;
        res.message = "Landing sequence initiated.";
        return true;
    }

    // ======================== 辅助函数 ========================

    /**
     * 触发降落流程
     * 标记降落状态，停止接收SUPER指令，锁定当前位置作为降落基准点
     */
    void triggerLanding() {
        if (!landing_active) {
            landing_active = true;
            super_cmd_active = false;      // 停止跟踪SUPER指令
            hold_position = current_pose;  // 锁定当前位姿作为降落起始点
            ROS_INFO("Landing triggered. Descending to %.2f m at %.2f m/s.", land_height, land_speed);
        }
    }

    /** 将值限制在 [-max_value, max_value] 范围内 */
    double limitValue(double value, double max_value) {
        if (value > max_value) return max_value;
        if (value < -max_value) return -max_value;
        return value;
    }

    /**
     * 从四元数中提取偏航角（yaw），返回值为度数
     * 包含四元数归一化检查，防止异常值
     */
    double getYawFromQuaternion(const geometry_msgs::Quaternion& quat) {
        const double norm = std::sqrt(
            quat.x * quat.x +
            quat.y * quat.y +
            quat.z * quat.z +
            quat.w * quat.w);
        if (!std::isfinite(norm) || norm < 1e-6) {
            return 0.0;
        }
        tf::Quaternion tf_quat;
        tf::quaternionMsgToTF(quat, tf_quat);
        double roll, pitch, yaw_rad;
        tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw_rad);
        return yaw_rad * 180.0 / M_PI;
    }

    /**
     * 计算偏航角指令
     * 将目标偏航角与当前偏航角的误差归一化到 [-π, π]
     * 并按最大变化率进行限幅，实现平滑的偏航过渡
     *
     * @param target_yaw  目标偏航角（rad）
     * @param current_yaw 当前偏航角（rad）
     * @param dt          时间步长（s）
     * @return 限幅后的偏航角指令（rad）
     */
    double calculateYawCommand(double target_yaw, double current_yaw, double dt) {
        double error = target_yaw - current_yaw;
        // 将误差归一化到 [-π, π]，避免走远路
        while (error > M_PI) error -= 2 * M_PI;
        while (error < -M_PI) error += 2 * M_PI;
        // 按最大变化率限幅
        double max_change = max_yaw_angle_rate * dt;
        if (error > max_change) return current_yaw + max_change;
        if (error < -max_change) return current_yaw - max_change;
        return target_yaw;
    }

    /**
     * 地理围栏检查
     * 检查当前位姿是否超出预设的安全边界
     * @return true=在围栏内, false=越界
     */
    bool checkGeofence() {
        if (!geofence_en) return true;
        const auto& p = current_pose.pose.position;
        if (p.x < geofence_x_min || p.x > geofence_x_max ||
            p.y < geofence_y_min || p.y > geofence_y_max ||
            p.z < geofence_z_min || p.z > geofence_z_max) {
            ROS_WARN("Geofence violation! Position: (%.2f, %.2f, %.2f)", p.x, p.y, p.z);
            return false;
        }
        return true;
    }

    /**
     * 发布飞控状态信息
     * 将解锁状态和OFFBOARD模式状态发布给其他节点
     */
    void publishStatus() {
        std_msgs::Bool armed_msg;
        armed_msg.data = armed;
        armed_status_pub.publish(armed_msg);

        std_msgs::Bool offboard_msg;
        offboard_msg.data = offboard_mode_engaged;
        offboard_status_pub.publish(offboard_msg);
    }

    // ======================== 主循环 ========================

    /**
     * 主运行函数，包含完整的飞行生命周期：
     *   Phase 1: 预发布位置指令（PX4要求切OFFBOARD前已有指令流）
     *   Phase 2: 自动解锁和切OFFBOARD（如果启用）
     *   Phase 3: 等待OFFBOARD模式生效
     *   Phase 4: 等待无人机达到起飞高度
     *   Phase 5: 主控制循环（指令跟踪/位置保持/降落）
     */
    void run() {
        ros::Rate rate(70.0);  // 控制频率70Hz

        // ========== Phase 1: 预发布起飞目标点 ==========
        // PX4要求在切入OFFBOARD模式之前已经有稳定的指令流
        // 否则切入OFFBOARD后会立即退出
        ROS_INFO("Publishing initial setpoints for takeoff to %.2f meters...", init_goal.pose.position.z);
        for (int i = 0; i < 50 && ros::ok(); ++i) {
            setpoint_position_pub.publish(init_goal);
            ros::spinOnce();
            rate.sleep();
        }

        // ========== Phase 2: 自动解锁和切OFFBOARD模式 ==========
        if (auto_arm) {
            // 等待MAVROS与飞控建立连接
            ROS_INFO("Waiting for MAVROS connection...");
            while (ros::ok() && !current_state.connected) {
                ros::spinOnce();
                rate.sleep();
            }
            ROS_INFO("MAVROS connected. Arming...");

            // 发送解锁指令
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed successfully.");
            } else {
                ROS_WARN("Arming failed! Continuing anyway...");
            }
            ros::Duration(1.0).sleep();  // 等待解锁生效

            // 切入OFFBOARD模式
            if (auto_offboard) {
                mavros_msgs::SetMode offb_mode;
                offb_mode.request.custom_mode = "OFFBOARD";
                if (set_mode_client.call(offb_mode) && offb_mode.response.mode_sent) {
                    ROS_INFO("OFFBOARD mode set successfully.");
                } else {
                    ROS_WARN("Setting OFFBOARD mode failed!");
                }
            }
        }

        // ========== Phase 3: 等待OFFBOARD模式生效 ==========
        // 如果未启用自动解锁，则需要操作手通过遥控器手动切换
        if (!auto_arm) {
            ROS_INFO("Waiting for OFFBOARD mode engagement (manual switch required)...");
        }
        while (ros::ok() && !offboard_mode_engaged) {
            // 持续发布目标点，确保PX4不会因指令流中断而拒绝OFFBOARD
            setpoint_position_pub.publish(init_goal);
            publishStatus();
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("OFFBOARD mode engaged.");

        // ========== Phase 4: 等待无人机达到目标起飞高度 ==========
        ROS_INFO("Waiting for drone to reach target altitude...");
        ros::Time takeoff_start_time = ros::Time::now();
        bool altitude_reached = false;

        while (ros::ok() && !altitude_reached) {
            // 检查是否到达目标高度（误差<0.1m即认为到达）
            if (std::abs(current_pose.pose.position.z - init_goal.pose.position.z) < 0.1) {
                altitude_reached = true;
                ROS_INFO("Drone reached target altitude of %.2f meters", init_goal.pose.position.z);
                break;
            }
            // 起飞超时保护（30秒内未达到目标高度则继续执行）
            if ((ros::Time::now() - takeoff_start_time).toSec() > 30.0) {
                ROS_WARN("Takeoff timeout! Current altitude: %.2f m, Target: %.2f m",
                         current_pose.pose.position.z, init_goal.pose.position.z);
                break;
            }
            // 持续发布起飞目标点
            setpoint_position_pub.publish(init_goal);
            publishStatus();
            // 每秒打印一次当前高度，方便监控起飞过程
            if ((ros::Time::now() - last_print_time).toSec() > 1.0) {
                ROS_INFO("Current altitude: %.2f m, Target: %.2f m",
                         current_pose.pose.position.z, init_goal.pose.position.z);
                last_print_time = ros::Time::now();
            }
            ros::spinOnce();
            rate.sleep();
        }

        // ========== Phase 4.5: 起飞后扫描阶段 (2026-08-19 新增) ==========
        // 起飞悬停后, 原地转 ±scan_angle_deg 让雷达扫过左右两侧,
        // 把障碍物周围的"未知"区域变"已知", 使 frontend_in_known_free=true 时
        // A* 能看到绕行通道(解决"明明右边有路却贴着障碍走")
        if (scan_after_takeoff_en) {
            ROS_INFO("Scan phase: rotating +/- %.1f deg for map coverage...", scan_angle_deg);
            double start_yaw_scan = tf::getYaw(current_pose.pose.orientation);
            double scan_angle_rad = scan_angle_deg * M_PI / 180.0;
            // 扫描序列: +angle -> -angle -> 回中 (覆盖左右两侧, 最后回到原始朝向)
            std::vector<double> scan_offsets = {scan_angle_rad, -scan_angle_rad, 0.0};
            // 渐进旋转: 当前偏航(从 MAVROS 反馈取, 不依赖内部变量)
            double cur_yaw_scan = start_yaw_scan;
            ros::Time scan_last_t = ros::Time::now();
            for (size_t si = 0; si < scan_offsets.size(); si++) {
                double target_yaw_scan = start_yaw_scan + scan_offsets[si];
                // ---- 渐进旋转段: 每帧只转 scan_yaw_rate*dt, 直到到位 (防甩头) ----
                while (ros::ok()) {
                    ros::Time now_t = ros::Time::now();
                    double dt_s = (now_t - scan_last_t).toSec();
                    scan_last_t = now_t;
                    if (dt_s <= 0.0) dt_s = 0.02;
                    if (dt_s > 0.1) dt_s = 0.1;
                    // 角度误差归一化到 [-pi, pi]
                    double yaw_err = target_yaw_scan - cur_yaw_scan;
                    while (yaw_err > M_PI) yaw_err -= 2 * M_PI;
                    while (yaw_err < -M_PI) yaw_err += 2 * M_PI;
                    // 到位判定 (0.02 rad ~ 1.1°)
                    if (std::abs(yaw_err) < 0.02) break;
                    // 限速步进
                    double max_step = scan_yaw_rate * dt_s;
                    double step = std::max(-max_step, std::min(max_step, yaw_err));
                    cur_yaw_scan += step;
                    // 发布: 位置保持 init_goal, 偏航渐进逼近
                    geometry_msgs::PoseStamped scan_pose = init_goal;
                    tf::Quaternion q_scan;
                    q_scan.setRPY(0, 0, cur_yaw_scan);
                    scan_pose.pose.orientation.x = q_scan.x();
                    scan_pose.pose.orientation.y = q_scan.y();
                    scan_pose.pose.orientation.z = q_scan.z();
                    scan_pose.pose.orientation.w = q_scan.w();
                    setpoint_position_pub.publish(scan_pose);
                    publishStatus();
                    ros::spinOnce();
                    rate.sleep();
                }
                // ---- 到位后保持 scan_hold_time, 让雷达充分扫过该方向 ----
                ros::Time scan_hold_start = ros::Time::now();
                while (ros::ok() && (ros::Time::now() - scan_hold_start).toSec() < scan_hold_time) {
                    geometry_msgs::PoseStamped scan_pose = init_goal;
                    tf::Quaternion q_scan;
                    q_scan.setRPY(0, 0, cur_yaw_scan);
                    scan_pose.pose.orientation.x = q_scan.x();
                    scan_pose.pose.orientation.y = q_scan.y();
                    scan_pose.pose.orientation.z = q_scan.z();
                    scan_pose.pose.orientation.w = q_scan.w();
                    setpoint_position_pub.publish(scan_pose);
                    publishStatus();
                    ros::spinOnce();
                    rate.sleep();
                }
                ROS_INFO("Scan phase: step %lu/%lu, yaw offset %.1f deg", si+1, scan_offsets.size(),
                         scan_offsets[si] * 180.0 / M_PI);
            }
            ROS_INFO("Scan phase done. (back to original heading)");
        }

        // ========== Phase 5: 主控制循环 ==========
        ROS_INFO("Starting main control loop.");

        double current_yaw = tf::getYaw(current_pose.pose.orientation);
        ros::Time last_control_time = ros::Time::now();

        while (ros::ok()) {
            ros::Time now = ros::Time::now();
            // 计算控制周期dt，并进行异常值保护
            double dt = (now - last_control_time).toSec();
            if (dt <= 0.0) {
                dt = 1.0 / 70.0;  // 时间戳异常时使用默认周期
            } else if (dt > 0.1) {
                dt = 0.1;  // 最大dt限制，防止首次循环或长时间暂停后积分过大
            }
            last_control_time = now;

            // 发布飞控状态
            publishStatus();

            // ========== 降落流程处理 ==========
            // 降落时优先级最高，会跳过后续所有逻辑
            if (landing_active) {
                // 按设定速度匀速下降
                double target_z = current_pose.pose.position.z - land_speed * dt;
                if (target_z <= land_height) {
                    // 到达降落高度阈值，调用MAVROS降落服务完成最终着陆
                    ROS_INFO("Reached landing height. Calling MAVROS land...");
                    mavros_msgs::CommandTOL land_srv;
                    land_srv.request.altitude = 0;
                    land_srv.request.latitude = 0;
                    land_srv.request.longitude = 0;
                    land_srv.request.min_pitch = 0;
                    land_srv.request.yaw = 0;
                    if (land_client.call(land_srv) && land_srv.response.success) {
                        ROS_INFO("Land command sent successfully.");
                    } else {
                        ROS_WARN("Land command failed! Publishing zero setpoint.");
                    }

                    // 等待飞控确认已着陆（最多等待10秒）
                    ros::Time land_wait_start = ros::Time::now();
                    while (ros::ok() && (ros::Time::now() - land_wait_start).toSec() < 10.0) {
                        if (extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
                            ROS_INFO("Drone detected on ground.");
                            if (auto_disarm) {
                                // 着陆后等待1秒再上锁，确保电机完全停转
                                ros::Duration(1.0).sleep();
                                mavros_msgs::CommandBool disarm_cmd;
                                disarm_cmd.request.value = false;
                                if (arming_client.call(disarm_cmd) && disarm_cmd.response.success) {
                                    ROS_INFO("Vehicle disarmed successfully.");
                                } else {
                                    ROS_WARN("Disarming failed!");
                                }
                            }
                            landing_active = false;
                            break;
                        }
                        // 等待期间继续发布位置指令，防止飞控退出OFFBOARD
                        setpoint_position_pub.publish(hold_position);
                        ros::spinOnce();
                        rate.sleep();
                    }
                    // 超时未检测到着陆，强制上锁
                    if (landing_active) {
                        ROS_WARN("Landing timeout. Forcing disarm.");
                        if (auto_disarm) {
                            mavros_msgs::CommandBool disarm_cmd;
                            disarm_cmd.request.value = false;
                            arming_client.call(disarm_cmd);
                        }
                        landing_active = false;
                    }
                } else {
                    // 未到达降落高度，继续匀速下降
                    hold_position.pose.position.z = target_z;
                    setpoint_position_pub.publish(hold_position);
                }
                ros::spinOnce();
                rate.sleep();
                continue;  // 降落期间跳过后续所有控制逻辑
            }

            // ========== 失效保护检查 ==========
            if (failsafe_en) {
                // 遥控器信号丢失检测
                // 检测到信号丢失后停止跟踪SUPER指令，切换到位置保持
                if ((now - last_rc_time).toSec() > rc_loss_timeout) {
                    ROS_WARN_THROTTLE(2.0, "RC signal lost for %.1f s! Position hold.",
                                      (now - last_rc_time).toSec());
                    if (super_cmd_active) {
                        super_cmd_active = false;
                        hold_position = current_pose;
                        current_yaw = tf::getYaw(current_pose.pose.orientation);
                    }
                }

                // 地理围栏检查
                // 越界后触发降落
                if (!checkGeofence()) {
                    ROS_WARN_THROTTLE(2.0, "Geofence violation! Position hold + landing.");
                    triggerLanding();
                    ros::spinOnce();
                    rate.sleep();
                    continue;
                }

                // MAVROS连接检查
                // 连接断开后停止跟踪SUPER指令，切换到位置保持
                if (!current_state.connected) {
                    ROS_WARN_THROTTLE(2.0, "MAVROS disconnected! Position hold.");
                    if (super_cmd_active) {
                        super_cmd_active = false;
                        hold_position = current_pose;
                        current_yaw = tf::getYaw(current_pose.pose.orientation);
                    }
                }
            }

            // ========== SUPER指令超时检测 ==========
            // 当SUPER指令活跃且未超时时，才跟踪规划器输出
            bool super_active = super_cmd_active &&
                              (now - last_super_cmd_time).toSec() < timeout_threshold;

            // ========== 状态日志（每秒打印一次） ==========
            if ((now - last_print_time).toSec() > 1.0) {
                if (super_active) {
                    const char* mode_str = use_velocity_ff ? "POS+VEL" : "POS";
                    if (use_yaw_angle_control) {
                        ROS_INFO("[%s] x=%.2f, y=%.2f, z=%.2f, vx=%.2f, vy=%.2f, vz=%.2f, yaw=%.2f deg",
                                 mode_str,
                                 last_super_cmd.position.x,
                                 last_super_cmd.position.y,
                                 last_super_cmd.position.z,
                                 last_super_cmd.velocity.x,
                                 last_super_cmd.velocity.y,
                                 last_super_cmd.velocity.z,
                                 last_super_cmd.yaw * 180.0 / M_PI);
                    } else {
                        ROS_INFO("[%s] x=%.2f, y=%.2f, z=%.2f, vx=%.2f, vy=%.2f, vz=%.2f, yaw_rate=%.2f deg/s",
                                 mode_str,
                                 last_super_cmd.position.x,
                                 last_super_cmd.position.y,
                                 last_super_cmd.position.z,
                                 last_super_cmd.velocity.x,
                                 last_super_cmd.velocity.y,
                                 last_super_cmd.velocity.z,
                                 last_super_cmd.yaw_dot * 180.0 / M_PI);
                    }
                } else {
                    // 位置保持模式
                    double yaw = getYawFromQuaternion(hold_position.pose.orientation);
                    ROS_INFO("[POS_HOLD] x=%.2f, y=%.2f, z=%.2f, yaw=%.2f deg",
                             hold_position.pose.position.x,
                             hold_position.pose.position.y,
                             hold_position.pose.position.z,
                             yaw);
                }
                last_print_time = now;
            }

            // ========== 控制指令输出 ==========
            if (super_active) {
                // --- SUPER指令跟踪模式 ---
                // 使用setpoint_raw发送位置+速度前馈+加速度前馈的复合指令
                mavros_msgs::PositionTarget setpoint_raw;
                setpoint_raw.header.stamp = now;
                setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

                // 构建type_mask：通过置位来告诉PX4忽略哪些字段
                // 位置始终发送，速度和加速度根据前馈开关决定
                // 注意：不可设置 FORCE 标志，否则 PX4 会把 acceleration_or_force
                //      解释为"力"，而 PX4 offboard 位置控制不支持 force 字段，
                //      会导致 FCU 报 "SET_POSITION_TARGET_LOCAL_NED force not supported"
                setpoint_raw.type_mask = 0;

                // 如果不使用速度前馈，则告诉PX4忽略速度字段
                if (!use_velocity_ff) {
                    setpoint_raw.type_mask |=
                        mavros_msgs::PositionTarget::IGNORE_VX |
                        mavros_msgs::PositionTarget::IGNORE_VY |
                        mavros_msgs::PositionTarget::IGNORE_VZ;
                }

                // 如果不使用加速度前馈，则告诉PX4忽略加速度字段
                if (!use_accel_ff) {
                    setpoint_raw.type_mask |=
                        mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ;
                }

                // 设置位置指令（来自SUPER规划器）
                setpoint_raw.position.x = last_super_cmd.position.x;
                setpoint_raw.position.y = last_super_cmd.position.y;
                setpoint_raw.position.z = last_super_cmd.position.z;

                // 速度前馈（用于抗风和提高跟踪精度）
                // 乘以增益系数防止前馈过强导致振荡
                if (use_velocity_ff) {
                    setpoint_raw.velocity.x = last_super_cmd.velocity.x * velocity_ff_gain;
                    setpoint_raw.velocity.y = last_super_cmd.velocity.y * velocity_ff_gain;
                    setpoint_raw.velocity.z = last_super_cmd.velocity.z * velocity_ff_gain;
                }

                // 加速度前馈（用于高动态机动场景）
                if (use_accel_ff) {
                    // 振荡修复: 一阶低通平滑, 抑制绕行/恢复切换时加速度突变(俯冲)
                    Eigen::Vector3d acc_raw(last_super_cmd.acceleration.x,
                                            last_super_cmd.acceleration.y,
                                            last_super_cmd.acceleration.z);
                    Eigen::Vector3d acc_sm = acc_ff_smooth * acc_raw + (1.0 - acc_ff_smooth) * prev_acc_ff;
                    prev_acc_ff = acc_sm;
                    setpoint_raw.acceleration_or_force.x = acc_sm.x();
                    setpoint_raw.acceleration_or_force.y = acc_sm.y();
                    setpoint_raw.acceleration_or_force.z = acc_sm.z();
                } else {
                    prev_acc_ff.setZero();
                }

                // 偏航控制
                if (use_yaw_angle_control) {
                    // 偏航角控制模式：忽略偏航角速率，发送目标偏航角
                    setpoint_raw.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
                    current_yaw = calculateYawCommand(last_super_cmd.yaw, current_yaw, dt);
                    setpoint_raw.yaw = current_yaw;
                } else {
                    // 偏航角速率控制模式：忽略偏航角，发送偏航角速率
                    setpoint_raw.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW;
                    setpoint_raw.yaw_rate = limitValue(last_super_cmd.yaw_dot, max_yaw_rate);
                }

                setpoint_raw_pub.publish(setpoint_raw);
            } else {
                // --- 位置保持模式 ---
                // SUPER指令超时或未激活时，锁定当前位置
                if (super_cmd_active) {
                    // 刚超时，打印警告并切换到保持模式
                    ROS_WARN("SUPER command timeout. Switching to position hold.");
                    super_cmd_active = false;
                    hold_position = current_pose;
                    current_yaw = tf::getYaw(current_pose.pose.orientation);
                }
                setpoint_position_pub.publish(hold_position);
            }

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "super_to_px4_bridge");
    BridgeController controller;
    controller.run();
    return 0;
}
