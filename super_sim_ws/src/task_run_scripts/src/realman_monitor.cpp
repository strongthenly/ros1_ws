#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <deque>
#include <cmath>
#include <chrono>
#include <functional>
#include <mutex>
#include <fstream>
#include <iomanip>
#include <ctime>

class DisplacementMonitor {
public:
    DisplacementMonitor() : nh_("~"), trigger_count_(0), last_trigger_time_(0), 
                            first_msg_received_(false), is_monitoring_active_(false) {
        // 加载参数
        nh_.param("displacement_threshold", displacement_threshold_, 1.0);
        nh_.param("max_queue_size", max_queue_size_, 50);  // 基于100Hz位置更新
        nh_.param("start_delay_sec", start_delay_sec_, 5.0); // 读取延时启动参数，默认5秒
        
        ROS_INFO("RealMan Config -> Threshold: %.2f m | Delay Start: %.1f s", 
                 displacement_threshold_, start_delay_sec_);
        
        // 高效订阅位置数据
        pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 
                                  10, 
                                  &DisplacementMonitor::poseCallback, 
                                  this,
                                  ros::TransportHints().tcpNoDelay());
        
        // 延迟初始化服务客户端
        set_mode_client_.reset();
        
        // 优化位移计算周期
        displacement_timer_ = nh_.createTimer(ros::Duration(0.025),  // 40Hz高频检查
                                            &DisplacementMonitor::checkDisplacement, 
                                            this);
        
        last_pos_time_ = ros::Time::now();
        last_displacement_ = 0.0;
        
        ROS_INFO("RealMan Node Initialized. Waiting for first pose message...");
    }

private:
    void writeLogToFile(const std::string& event_msg) {
        std::string log_file_path = "/home/fouruav/odin_ws/src/task_run_scripts/log/accident_log.txt";
        std::ofstream log_file(log_file_path, std::ios::app);
        if (log_file.is_open()) {
            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);
            log_file << "[" << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "] " 
                     << event_msg << std::endl;
            log_file.close();
        } else {
            ROS_ERROR("Failed to open log file! Path: %s", log_file_path.c_str());
        }
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        ros::Time now = ros::Time::now();
        
        // ================= 新增：延时启动逻辑 =================
        if (!first_msg_received_) {
            first_msg_received_ = true;
            first_msg_time_ = now;
            ROS_INFO("First pose received! Waiting %.1f seconds before starting monitor...", start_delay_sec_);
        }

        if (!is_monitoring_active_) {
            if ((now - first_msg_time_).toSec() < start_delay_sec_) {
                return; // 还在延时保护期内，不记录数据，也不进行检测
            } else {
                is_monitoring_active_ = true;
                ROS_INFO("Delay finished. RealMan-Start Monitoring!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            }
        }
        // =======================================================

        geometry_msgs::Point current_position = msg->pose.position;
        
        // 添加时间戳位置到队列
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            position_history_.push_back({now, current_position});
            
            while (position_history_.size() > max_queue_size_) {
                position_history_.pop_front();
            }
        }
    }

    void checkDisplacement(const ros::TimerEvent& event) {
        // 如果监控未激活（还在延时保护期内），直接返回
        if (!is_monitoring_active_) return;

        TimedPosition old_position;
        TimedPosition current_position;
        
        // 保护队列访问
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (position_history_.size() < 2) return;
            
            current_position = position_history_.back();
            
            auto it = position_history_.cbegin();
            const ros::Time half_sec_ago = current_position.stamp - ros::Duration(0.5);
            
            while (it != position_history_.cend() && 
                   it->stamp < half_sec_ago && 
                   it != position_history_.cend() - 1) {
                it++;
            }
            
            old_position = *it;
        }
        
        const double time_diff = (current_position.stamp - old_position.stamp).toSec();
        if (time_diff < 0.01) return;
        
        const double dx = current_position.position.x - old_position.position.x;
        const double dy = current_position.position.y - old_position.position.y;
        const double dz = current_position.position.z - old_position.position.z;
        const double displacement = std::sqrt(dx*dx + dy*dy + dz*dz);
        last_displacement_ = displacement;
        
        if (displacement > displacement_threshold_) {
            const ros::Time now = ros::Time::now();
            
            if (trigger_count_ == 0 || 
                (now.toSec() - last_trigger_time_) > anti_bounce_window_) {
                
                ROS_WARN("!!!!!!!!Odom Disorder (%.2f > %.2fm)! Trig STABILIZED Mode!!!!!!!!!!!!!!!!!!", 
                         displacement, displacement_threshold_);
                
                char log_msg[256];
                snprintf(log_msg, sizeof(log_msg), 
                         "Odom Disorder Detected! Displacement: %.2fm (Threshold: %.2fm). Triggering STABILIZED Mode. (Trigger Count: %d)", 
                         displacement, displacement_threshold_, trigger_count_ + 1);
                writeLogToFile(log_msg);

                triggerStabilizeMode(); 
                
                last_trigger_time_ = now.toSec();
                trigger_count_++;
                
                if (trigger_count_ > max_trigger_count_) {
                    ROS_WARN("REALMAN-trigger_count=3->ros::shutdown()");
                    writeLogToFile("CRITICAL: Trigger count exceeded maximum (3). Shutting down node.");
                    ros::shutdown();
                }
            }
        }
    }

    void triggerStabilizeMode() {
        if (!set_mode_client_) {
            set_mode_client_ = std::make_shared<ros::ServiceClient>(
                nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode")
            );
        }
        
        mavros_msgs::SetMode set_mode_req;
        set_mode_req.request.custom_mode = "STABILIZED";
        
        auto result = set_mode_client_->call(set_mode_req);
        if (result && set_mode_req.response.mode_sent) {
            ROS_WARN("Switch to STABILIZED SUCCESS!");
            writeLogToFile("-> Switch to STABILIZED mode successful.");
        } else {
            ROS_ERROR("Retry STABILIZED..........");
            
            ros::Duration(0.1).sleep();
            auto retry_result = set_mode_client_->call(set_mode_req);
            if (retry_result && set_mode_req.response.mode_sent) {
                ROS_WARN("Retry STABILIZED..SUCCESS........!");
                writeLogToFile("-> Retry switch to STABILIZED mode successful.");
            } else {
                ROS_ERROR("Retry STABILIZED..FAIL........!");
                writeLogToFile("-> ERROR: Failed to switch to STABILIZED mode even after retry!");
            }
        }
    }

    struct TimedPosition {
        ros::Time stamp;
        geometry_msgs::Point position;
    };
    
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    std::shared_ptr<ros::ServiceClient> set_mode_client_;
    ros::Timer displacement_timer_;
    
    std::deque<TimedPosition> position_history_;
    std::mutex queue_mutex_;
    
    double displacement_threshold_ = 1.0;
    double start_delay_sec_ = 5.0; // 延时启动时间
    double last_displacement_ = 0.0;
    int max_queue_size_ = 50;  
    
    bool first_msg_received_;      // 是否收到第一条消息
    ros::Time first_msg_time_;     // 收到第一条消息的时间
    bool is_monitoring_active_;    // 监控是否激活
    
    ros::Time last_pos_time_;
    double anti_bounce_window_ = 1.0;  
    int trigger_count_ = 0;
    double last_trigger_time_ = 0.0;
    const int max_trigger_count_ = 3;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "realman_displacement_monitor");
    ros::NodeHandle nh;
    DisplacementMonitor monitor;
    ros::MultiThreadedSpinner spinner(2); 
    spinner.spin();
    return 0;
}