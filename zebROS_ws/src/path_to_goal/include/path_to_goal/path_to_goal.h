#include <ros/ros.h>
#include <cube_detection/CubeDetection.h>
#include <swerve_point_generator/FullGenCoefs.h>
#include <talon_swerve_drive_controller/MotionProfilePoints.h>
#include <base_trajectory/GenerateSpline.h>
#include <talon_state_controller/TalonState.h>
#include <robot_visualizer/ProfileFollower.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <behaviors/PathAction.h>
#include <actionlib/server/simple_action_server.h>

ros::Subscriber cube_sub;
ros::ServiceServer cmd_service;
ros::ServiceClient point_gen;
ros::ServiceClient swerve_controller;
ros::ServiceClient spline_gen;
ros::ServiceClient VisualizeService;
cube_detection::CubeDetection cube_location;
cube_detection::CubeDetection qr_location;
bool outOfPoints;
ros::Subscriber talon_sub;
bool command;

void cubeCallback(cube_detection::CubeDetection sub_location);

void talonStateCallback(const talon_state_controller::TalonState &talon_state);

void QRCallback(cube_detection::CubeDetection sub_location);
