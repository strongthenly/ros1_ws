#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "waypoint_mission/ros2_waypoint_planner.hpp"

/* DW_DISABLED_FOR_SIM: 仿真无 libdw-dev，改用 backtrace(上游默认) */
#define BACKWARD_HAS_DW 0
#include "utils/backward.hpp"
namespace backward{
    backward::SignalHandling sh;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("waypoint_mission");

    /* Publisher and subscriber */
    mission_planner::WaypointPlanner wpl(node);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}