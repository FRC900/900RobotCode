
#include "base_trajectory_msgs/GenerateSpline.h"

void writeMatlabSplines(const base_trajectory_msgs::GenerateSpline::Response &msg);
void writeMatlabPath(const std::vector<geometry_msgs::PoseStamped> &poses);

