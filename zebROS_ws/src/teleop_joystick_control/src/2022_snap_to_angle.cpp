#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <angles/angles.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <algorithm>
#include <cmath>

ros::Publisher closest_angle_pub;
std::vector<double> fender_angles;
std_msgs::Float64 min_angle;

void callBack(const sensor_msgs::Imu &imuState) {
  const tf2::Quaternion imuQuat(imuState.orientation.x, imuState.orientation.y, imuState.orientation.z, imuState.orientation.w);
  double roll;
  double pitch;
  double yaw;
  tf2::Matrix3x3(imuQuat).getRPY(roll, pitch, yaw);
  std::vector<double> nearest_angles;
  nearest_angles.push_back(std::fabs(angles::shortest_angular_distance(yaw, fender_angles[0])));
  nearest_angles.push_back(std::fabs(angles::shortest_angular_distance(yaw, fender_angles[1])));
  nearest_angles.push_back(std::fabs(angles::shortest_angular_distance(yaw, fender_angles[2])));
  nearest_angles.push_back(std::fabs(angles::shortest_angular_distance(yaw, fender_angles[3])));
  int min_angle_index = std::min_element(nearest_angles.begin(), nearest_angles.end()) - nearest_angles.begin();
  //min_angle.data = 0.1;
  //ROS_INFO_STREAM("Angle 1: " << fender_angles[0] << " Angle 2: " << fender_angles[1] << " Angle 3: " << fender_angles[2] << " Angle 4: " << fender_angles[3]);
  min_angle.data = fender_angles[min_angle_index];
  closest_angle_pub.publish(min_angle);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "closest_angle");
  ros::NodeHandle nh;

  ros::Subscriber imu = nh.subscribe("/imu/zeroed_imu", 1, callBack);
  closest_angle_pub = nh.advertise<std_msgs::Float64>("/imu/nearest_angle", 10);

  double fender_angle;
  if (!nh.getParam("fender_angle", fender_angle)) {
    ROS_ERROR("Could not read fender_angle in teleop joystick snap to goal");
  }
  fender_angles.push_back(fender_angle);
  fender_angles.push_back(fender_angle + M_PI/2);
  fender_angles.push_back(fender_angle + M_PI);
  fender_angles.push_back(fender_angle + (3*M_PI)/2);

	ros::spin();

  return 0;
}
