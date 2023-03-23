#ifndef MATLAB_PRINTER_INC_
#define MATLAB_PRINTER_INC_


#include "base_trajectory_msgs/GenerateSpline.h"
#include "spline_util/spline_util.h"

template <class T, size_t O>
void writeMatlabSplines(const Trajectory<T, O> &trajectory, int figureNum, const std::string &label);
void writeMatlabPath(const std::vector<geometry_msgs::PoseStamped> &poses, int figureNum, const std::string &label);

void writeMatlabPathNew(const std::vector<geometry_msgs::PoseStamped> &poses,
						const std::vector<geometry_msgs::PoseStamped> &poses_velocity,
						const std::vector<geometry_msgs::PoseStamped> &poses_acceleration,
						int figureNum, const std::string &label);
void writeMatlabMovieScript(const size_t frameCount, const std::string &label);
#endif
