
#include "base_trajectory_msgs/GenerateSpline.h"

void writeMatlabSplines(const base_trajectory_msgs::GenerateSpline::Response &msg, int figureNum, const std::string &label);
void writeMatlabPath(const std::vector<geometry_msgs::PoseStamped> &poses, int figureNum, const std::string &label);

