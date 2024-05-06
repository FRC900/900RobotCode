// Write a ROS node to advertise these services:
/*
ros::ServiceServer service = nh_.advertiseService("tagslam_pub_map_to_odom", service_cb);
ros::ServiceServer relocalize_point_srv = nh_.advertiseService("relocalize_point", relocalize_to_point_cb);
ros::ServiceServer toggle_relocalize = nh_.advertiseService("toggle_map_to_odom", toggle_service_cb);
ros::ServiceServer toggle_cmd_vel_limit_server = nh_.advertiseService("toggle_cmd_vel_limit", toggle_cmd_vel_limit_cb);
*/
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include "behavior_actions/RelocalizePoint.h"

bool service_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  ROS_INFO("Service called");
  return true;
}

bool relocalize_to_point_cb(behavior_actions::RelocalizePoint::Request &req, behavior_actions::RelocalizePoint::Response &res) {
  ROS_INFO("Service called");
  return true;
}

bool toggle_service_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  ROS_INFO("Service called");
  return true;
}

bool toggle_cmd_vel_limit_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  ROS_INFO("Service called");
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fake_map_to_odom");
  ros::NodeHandle nh_;

  ros::ServiceServer service = nh_.advertiseService("tagslam_pub_map_to_odom", service_cb);
  ros::ServiceServer relocalize_point_srv = nh_.advertiseService("relocalize_point", relocalize_to_point_cb);
  ros::ServiceServer toggle_relocalize = nh_.advertiseService("toggle_map_to_odom", toggle_service_cb);
  ros::ServiceServer toggle_cmd_vel_limit_server = nh_.advertiseService("toggle_cmd_vel_limit", toggle_cmd_vel_limit_cb);

  ros::spin();
  return 0;
}