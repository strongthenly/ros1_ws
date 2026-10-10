//
// Created by yunfan on 2022/3/26.
//

#ifndef MISSION_PLANNER_CONFIG
#define MISSION_PLANNER_CONFIG

#include "vector"
#include "string"
#include "iostream"
#include <iomanip>
#include <fstream>
#include <sstream>
#include <utils/eigen_alias.hpp>
#include <utils/yaml_loader.hpp>
#include <utils/color_text.hpp>
namespace mission_planner {
    using namespace super_utils;
    using namespace color_text;
    using namespace std;
    enum TriggerType {
        RVIZ_CLICK = 0,
        MAVROS_RC = 1,
        TARGET_ODOM = 2
    };

    enum CmdType {
        PATH = 0,
        WAYPOINT = 1
    };

    class MissionConfig {
    public:
        // Bool Params

        int start_trigger_type;
        int cmd_type;
        double start_program_delay;
        string path_pub_topic, goal_pub_topic, odom_topic, waypoints_file_name;
        vec_E<Vec3f> waypoints;
        vec_E<Vec4f> waypoint_quats;  // 四元数 (qx, qy, qz, qw)，为空时表示无方向约束
        vector<double> switch_dis_vec;
        double switch_dis;
        double odom_timeout;
        double publish_dt;
        double waypoint_downsample_dist;  // 航点距离降采样阈值(m)，与上一个保留航点距离小于此值则跳过，0=不降采样

        double str2double(string s) {
            double d;
            stringstream ss;
            ss << s;
            ss >> setprecision(16) >> d;
            ss.clear();
            return d;
        }

        void LoadWaypoint(string file_name) {
            ifstream theFile(file_name);
            std::string line;
            Vec3f log;
            bool has_quat = false;
            bool first_line = true;
            Vec3f last_kept_wp = Vec3f::Zero();  // 上一个保留的航点位置
            bool has_last_kept = false;
            while (std::getline(theFile, line)) {
                // 跳过空行
                if (line.empty()) continue;

                // 检测 CSV 格式（包含逗号）
                if (line.find(',') != std::string::npos) {
                    // CSV 格式: x,y,z,qx,qy,qz,qw
                    // 跳过表头行（包含字母的行）
                    if (first_line && (line.find('x') != std::string::npos ||
                                       line.find('X') != std::string::npos)) {
                        first_line = false;
                        continue;
                    }
                    first_line = false;
                    has_quat = true;

                    std::vector<std::string> result;
                    std::stringstream ss(line);
                    std::string token;
                    while (std::getline(ss, token, ',')) {
                        // 去除首尾空格
                        token.erase(0, token.find_first_not_of(" \t"));
                        token.erase(token.find_last_not_of(" \t") + 1);
                        result.push_back(token);
                    }

                    if (result.size() >= 7) {
                        Vec3f wp;
                        wp.x() = str2double(result[0]);
                        wp.y() = str2double(result[1]);
                        wp.z() = str2double(result[2]);

                        Vec4f quat;
                        quat.x() = str2double(result[3]);  // qx
                        quat.y() = str2double(result[4]);  // qy
                        quat.z() = str2double(result[5]);  // qz
                        quat.w() = str2double(result[6]);  // qw

                        // 距离降采样：与上一个保留航点距离 < 阈值则跳过
                        if (has_last_kept && waypoint_downsample_dist > 0 &&
                            (wp - last_kept_wp).norm() < waypoint_downsample_dist) {
                            continue;
                        }

                        waypoints.push_back(wp);
                        waypoint_quats.push_back(quat);
                        switch_dis_vec.push_back(switch_dis);
                        last_kept_wp = wp;
                        has_last_kept = true;
                    }
                } else {
                    // 原始格式: x y z switch_distance（空格分隔）
                    first_line = false;
                    std::vector<std::string> result;
                    std::istringstream iss(line);
                    for (std::string s; iss >> s;) {
                        result.push_back(s);
                    }
                    if (result.size() >= 4) {
                        for (size_t i = 0; i < result.size() - 1; i++) {
                            log(i) = str2double(result[i]);
                        }
                        switch_dis_vec.push_back(str2double(result[result.size() - 1]));
                        waypoints.push_back(log);
                    }
                }
            }

            cout << GREEN << " -- [MISSION] Load " << waypoints.size() << " waypoints." << RESET << endl;
            if (has_quat) {
                cout << GREEN << " -- [MISSION] Format: CSV with quaternion (x,y,z,qx,qy,qz,qw)" << RESET << endl;
            }
            for (int i = 0; i < waypoints.size(); i++) {
                cout << BLUE << " -- [MISSION] Waypoint " << i << " at (" << waypoints[i].x() << ", "
                     << waypoints[i].y() << ", " << waypoints[i].z() << ")" << " Switch dis = " << switch_dis_vec[i];
                if (has_quat && i < waypoint_quats.size()) {
                    cout << " Quat=(" << waypoint_quats[i].x() << ", " << waypoint_quats[i].y() << ", "
                         << waypoint_quats[i].z() << ", " << waypoint_quats[i].w() << ")";
                }
                cout << RESET << endl;
            }
        }
        
        MissionConfig() {};

        MissionConfig(const std::string & cfg_path) {
            yaml_loader::YamlLoader loader(cfg_path);

            loader.LoadParam("start_trigger_type", start_trigger_type, 1);
            loader.LoadParam("start_program_delay", start_program_delay, 1.0);
            loader.LoadParam("cmd_type", cmd_type, 1);
            loader.LoadParam("switch_dis", switch_dis, 1.0);
            loader.LoadParam("odom_timeout", odom_timeout, 0.1);
            loader.LoadParam("publish_dt", publish_dt, 1.0);
            loader.LoadParam("waypoint_downsample_dist", waypoint_downsample_dist, 0.0);
            loader.LoadParam("goal_pub_topic", goal_pub_topic, string("/planner/goal"));
            loader.LoadParam("odom_topic", odom_topic, string("/lidar_slam/odom"));
//            loader.LoadParam("path_pub_topic", path_pub_topic, string("/planner/path_cmd"));
//            loader.LoadParam("waypoints_file_name", waypoints_file_name, string("a_working_waypoints.txt"));

        }

    };
}
#endif //PLANNER_CONFIG_HPP
