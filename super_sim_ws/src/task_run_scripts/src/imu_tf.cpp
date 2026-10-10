#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include "../include/Mahony_AHRS.h"
#include "../include/digital_filter.h"
#include <string.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

ros::Publisher imu_tf_publisher;
ros::Subscriber imu_subscriber;

void imu_subscriberCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
        // 获取原始加速度和角速度
        tf::Vector3 acceleration(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        tf::Vector3 angular_velocity(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

        // 创建旋转矩阵
        tf::Matrix3x3 rotation;
        // rotation.setRPY(0, -M_PI / 6, M_PI); // 绕pitch旋转30度，绕yaw旋转180度
        rotation.setRPY(0, M_PI / 9, 0); // 绕pitch旋转20度，绕yaw旋转0度

        // 应用旋转
        acceleration = rotation * acceleration;
        angular_velocity = rotation * angular_velocity;
        
        

    // 创建新的IMU消息
    sensor_msgs::Imu new_imu;
    new_imu.header = msg->header;
    new_imu.header.stamp = msg->header.stamp;
    new_imu.header.frame_id = "livox_frame";
    new_imu.orientation.x = 0;
    new_imu.orientation.y = 0;
    new_imu.orientation.z = 0;
    new_imu.orientation.w = 0;
    new_imu.linear_acceleration.x = acceleration.x();
    new_imu.linear_acceleration.y = acceleration.y();
    new_imu.linear_acceleration.z = acceleration.z();
    new_imu.angular_velocity.x = angular_velocity.x();
    new_imu.angular_velocity.y = angular_velocity.y();
    new_imu.angular_velocity.z = angular_velocity.z();

    // 发布新的IMU消息
    imu_tf_publisher.publish(new_imu);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_tf");

  ros::NodeHandle node;

  int ret;

  imu_subscriber = node.subscribe("imu_topic", 1, &imu_subscriberCallback);

  imu_tf_publisher = node.advertise<sensor_msgs::Imu>("/livox/imu_tf", 10);

  // ros::spin() 阻塞直到节点关闭；spin 返回后直接退出
  ros::spin();

  return 0;
}
