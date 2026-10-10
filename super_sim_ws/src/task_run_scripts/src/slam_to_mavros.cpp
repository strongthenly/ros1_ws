#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
// #include <mavros_msgs/LandingTarget.h>

#include <string.h>
#include <mutex>
#include <iostream>
#include <cstring>
#include <fstream>

std::mutex mtx;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_to_mavros");

  ros::NodeHandle node;

  std::string target_frame_id = "/camera_init";

  std::string source_frame_id = "/body" ;

  if(node.getParam("target_frame_id", target_frame_id))
  {
    ROS_INFO("Get target_frame_id parameter: %s", target_frame_id.c_str());
  }
  else
  {
    ROS_WARN("Using default target_frame_id: %s", target_frame_id.c_str());
  }

  // The frame for which we find the tranform to target_frame_id, the original "camera" frame
  if(node.getParam("source_frame_id", source_frame_id))
  {
    ROS_INFO("Get source_frame_id parameter: %s", source_frame_id.c_str());
  }
  else
  {
    ROS_WARN("Using default source_frame_id: %s", source_frame_id.c_str());
  }


  ros::Publisher slam_pose_publisher = node.advertise<geometry_msgs::PoseStamped>("slam_pose", 20);

  tf::TransformListener tf_listener;

  tf::StampedTransform transform;

  geometry_msgs::PoseStamped msg_slam_pose;

  tf_listener.waitForTransform(target_frame_id, source_frame_id, ros::Time::now(), ros::Duration(3.0));
  ros::Time now = ros::Time(0);
  tf_listener.lookupTransform(target_frame_id, source_frame_id, now, transform);
  // ros::Time last_tf_time = ros::Time::now();
  ros::Time last_tf_time = transform.stamp_;
  ros::Time last_precland_tf_time = ros::Time::now();
  
  // Limit the rate of publishing data, otherwise the other telemetry port might be flooded
  ros::Rate rate(20);
  ros::Time last_cout_time = ros::Time::now();
  ROS_WARN("wait.ok()");
  while (node.ok())
  { 
    // For tf, Time(0) means "the latest available" transform in the buffer.

    ros::Time current_cout_time = ros::Time::now();
   
    try
    {
      tf_listener.lookupTransform(target_frame_id, source_frame_id, now, transform);

      // Only publish pose messages when we have new transform data.
      if (last_tf_time < transform.stamp_)
      {
        last_tf_time = transform.stamp_;

        static tf::Vector3 position_slam;

        static tf::Quaternion quat_slam;

        position_slam = transform.getOrigin();
        quat_slam = transform.getRotation();

        // // Create PoseStamped message to be sent
        msg_slam_pose.header.stamp = transform.stamp_;
        msg_slam_pose.header.frame_id = transform.frame_id_;
        msg_slam_pose.pose.position.x = position_slam.getX();
        msg_slam_pose.pose.position.y = position_slam.getY();
        msg_slam_pose.pose.position.z = position_slam.getZ();
        msg_slam_pose.pose.orientation.x = quat_slam.getX();
        msg_slam_pose.pose.orientation.y = quat_slam.getY();
        msg_slam_pose.pose.orientation.z = quat_slam.getZ();
        msg_slam_pose.pose.orientation.w = quat_slam.getW();

        slam_pose_publisher.publish(msg_slam_pose);
        if(current_cout_time - last_cout_time > ros::Duration(1.0))
        {std::cout<<"X:"<<std::setprecision(1)<<msg_slam_pose.pose.position.x  <<"\n"<<"Y:"
        <<msg_slam_pose.pose.position.y <<"\n"
        <<"Z:"<<msg_slam_pose.pose.position.z <<";"<<std::endl;
        last_cout_time = current_cout_time;
        }
        std::lock_guard<std::mutex> lock(mtx);

        std::ofstream outFile("/home/fouruav/share/drone_pose.txt");
        if (!outFile) {
            std::cerr << "Error opening file for writing." << std::endl;
        }
        outFile <<std::fixed <<std::setprecision(2)<< position_slam.getX() << std::endl;
        outFile <<std::fixed <<std::setprecision(2)<< position_slam.getY() << std::endl;
        outFile <<std::fixed <<std::setprecision(2)<< position_slam.getZ() << std::endl;
        outFile.close();
        
        // Publish pose of body frame in world frame
        // camera_pose_publisher.publish(msg_body_pose);

        // // Publish trajectory path for visualization
        // body_path.header.stamp = msg_body_pose.header.stamp;
        // body_path.header.frame_id = msg_body_pose.header.frame_id;
        // body_path.poses.push_back(msg_body_pose);
        // body_path_pubisher.publish(body_path);
      }
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }


    rate.sleep();
  }
  return 0;
}
