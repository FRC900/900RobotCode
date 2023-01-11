#ifndef INC_PF_MSGS__
#define INC_PF_MSGS__

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include "pf_localization/particle.hpp"

inline geometry_msgs::Pose toPose(const Particle& particle) {
  geometry_msgs::Pose pose;
  pose.position.x = particle.x_;
  pose.position.y = particle.y_;
  pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0, 0, particle.rot_);
  pose.orientation.x = q.getX();
  pose.orientation.y = q.getY();
  pose.orientation.z = q.getZ();
  pose.orientation.w = q.getW();
  return pose;
}

#endif