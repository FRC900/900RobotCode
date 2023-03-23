#include <fstream>
#include <angles/angles.h>
#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "base_trajectory/matlab_printer.h"

// For printing out matlab code for testing
template <class T, size_t O>
static void printCoefs(std::stringstream &s, const std::string &name, const TrajectoryPerJoint<T, O> &tpj, int figureNum)
{
	for (size_t i = 0; i < tpj.size(); i++)
	{
		const auto &coefs = tpj[i].getCoefs();
		s << "p" << name << i << "_" << figureNum << " = [";
		for (int j = coefs.size() - 1; j >= 0; j--)
		{
			s << coefs[j];
			if (j != 0)
				s << ", ";
		}
		s << "];" << std::endl;
	}
}

// For printing out matlab code for testing
template <class T, size_t O>
static void printPolyval(std::stringstream &s, const std::string &name, const TrajectoryPerJoint<T, O> &tpj, int figureNum)
{
	s << "p" << name << "_y_" << figureNum << " = [";
	for (size_t i = 0; i < tpj.size(); i++)
	{
		double x_offset;
		if (i == 0)
		{
			x_offset = 0;
		}
		else
		{
			x_offset = tpj[i - 1].endTime();
		}
		s << "polyval(p" << name << i << "_" << figureNum << ", x" << i << "_" << figureNum << " - " << x_offset << ")";
		if (i < (tpj.size() - 1))
		{
			s << ", ";
		}
	}
	s << "];" << std::endl;
}

void writeMatlabSplinesBoilerplate(std::stringstream &s, int figureNum, const std::string &label)
{
	s << "figure(" << figureNum << ")" << std::endl;
	s << "subplot(1,1,1)" << std::endl;
	s << "subplot(3,3,1)" << std::endl;
	s << "title('" << label << "')" << std::endl;
	s << "plot(px_y_" << figureNum << ", py_y_" << figureNum << ")" << std::endl;
	s << "xlabel('X position')" << std::endl;
	s << "ylabel('Y position')" << std::endl;
	s << "subplot(3,3,2)" << std::endl;
	s << "plot3(px_y_" << figureNum << ", py_y_" << figureNum << ", porient_y_" << figureNum << ")" << std::endl;
	s << "xlabel('X position')" << std::endl;
	s << "ylabel('Y position')" << std::endl;
	s << "zlabel('Orientation')" << std::endl;
	s << "subplot(3,3,3)" << std::endl;
	s << "plot(x_" << figureNum << ", porient_y_" << figureNum << ", x_" << figureNum << ", pdorient_y_" << figureNum << ")" << std::endl;
	s << "xlabel('arbT')" << std::endl;
	s << "ylabel('Orient / dOrient')" << std::endl;
	s << "subplot(3,3,4)" << std::endl;
	s << "plot (x_" << figureNum << ",px_y_" << figureNum << ", x_" << figureNum << ", py_y_" << figureNum << ")" << std::endl;
	s << "xlabel('arbT')" << std::endl;
	s << "ylabel('X / Y Position')" << std::endl;
	s << "subplot(3,3,5)" << std::endl;
	s << "plot (x_" << figureNum << ",pdx_y_" << figureNum << ", x_" << figureNum << ", pdy_y_" << figureNum << ")" << std::endl;
	s << "xlabel('arbT')" << std::endl;
	s << "ylabel('X / Y Velocity')" << std::endl;
	s << "subplot(3,3,6)" << std::endl;
	s << "plot (x_" << figureNum << ",pddx_y_" << figureNum << ", x_" << figureNum << ", pddy_y_" << figureNum << ")" << std::endl;
	s << "xlabel('arbT')" << std::endl;
	s << "ylabel('X / Y Acceleration')" << std::endl;
	s << "subplot(3,3,7)" << std::endl;
	s << "plot (x_" << figureNum << ",pdddx_y_" << figureNum << ", x_" << figureNum << ", pdddy_y_" << figureNum << ")" << std::endl;
	s << "xlabel('arbT')" << std::endl;
	s << "ylabel('X / Y Jerk')" << std::endl;
}

// Generate matlab / octave code for displaying generated splines from the input trajectory
template <class T, size_t O>
void writeMatlabSplines(const Trajectory<T, O> &trajectory, int figureNum, const std::string &label)
{
	std::stringstream s;
	s << std::endl;
	double prev_x = 0;
	for (size_t i = 0; i < trajectory[0].size(); i++)
	{
		const auto this_x = trajectory[0][i].endTime();
		const auto range = this_x - prev_x;
		s << "x" << i << "_" << figureNum << " = " << prev_x << ":" << range / 100.
		  << ":" << this_x << ";" << std::endl;
		prev_x = this_x;
	}
	s << "x_" << figureNum << " = [";
	for (size_t i = 0; i < trajectory[0].size(); i++)
	{
		s << "x" << i << "_" << figureNum;
		if (i < trajectory[0].size() - 1)
			s << ", ";
	}
	s << "];" << std::endl;
	s << std::endl;
	printCoefs(s, "x", trajectory[0], figureNum);
	printCoefs(s, "y", trajectory[1], figureNum);
	printCoefs(s, "orient", trajectory[2], figureNum);
	for (size_t i = 0; i < trajectory[0].size(); i++)
	{
		s << "pdx" << i << "_" << figureNum << " = polyder(px" << i << "_" << figureNum << ");" << std::endl;
		s << "pddx" << i << "_" << figureNum << " = polyder(pdx" << i << "_" << figureNum << ");" << std::endl;
		s << "pdddx" << i << "_" << figureNum << " = polyder(pddx" << i << "_" << figureNum << ");" << std::endl;
		s << "pdy" << i << "_" << figureNum << " = polyder(py" << i << "_" << figureNum << ");" << std::endl;
		s << "pddy" << i << "_" << figureNum << " = polyder(pdy" << i << "_" << figureNum << ");" << std::endl;
		s << "pdddy" << i << "_" << figureNum << " = polyder(pddy" << i << "_" << figureNum << ");" << std::endl;
		s << "pdorient" << i << "_" << figureNum << " = polyder(porient" << i << "_" << figureNum << ");" << std::endl;
		s << "pddorient" << i << "_" << figureNum << " = polyder(pdorient" << i << "_" << figureNum << ");" << std::endl;
		s << "pdddorient" << i << "_" << figureNum << " = polyder(pddorient" << i << "_" << figureNum << ");" << std::endl;
	}
	printPolyval(s, "x", trajectory[0], figureNum);
	printPolyval(s, "dx", trajectory[0], figureNum);
	printPolyval(s, "ddx", trajectory[0], figureNum);
	printPolyval(s, "dddx", trajectory[0], figureNum);
	printPolyval(s, "y", trajectory[1], figureNum);
	printPolyval(s, "dy", trajectory[1], figureNum);
	printPolyval(s, "ddy", trajectory[1], figureNum);
	printPolyval(s, "dddy", trajectory[1], figureNum);
	printPolyval(s, "orient", trajectory[2], figureNum);
	printPolyval(s, "dorient", trajectory[2], figureNum);
	printPolyval(s, "ddorient", trajectory[2], figureNum);
	printPolyval(s, "dddorient", trajectory[2], figureNum);
	writeMatlabSplinesBoilerplate(s, figureNum, label);

	const std::string filename(label + ".m");
	std::ofstream of(filename);
	of << s.str();

	//ROS_INFO_STREAM_FILTER(&messageFilter, "Matlab_splines : " << s.str());
}

template void writeMatlabSplines(const XYTTrajectory<double> &trajectory, int figureNum, const std::string &label);
template void writeMatlabSplines(const ArcLengthTrajectory<double> &trajectory, int figureNum, const std::string &label);

static void writeMatlabDoubleArray(std::stringstream &s, const std::string &name, const std::vector<double> &values, int figureNum)
{
	s << name << "_" << figureNum << " = [";
	for (size_t i = 0; i < values.size(); i++)
	{
		s << values[i];
		if (i < (values.size() - 1))
			s << ", ";
	}
	s << "];" << std::endl;
}

void writeMatlabPath(const std::vector<geometry_msgs::PoseStamped> &poses, int figureNum, const std::string &label)
{
	if (poses.size() == 0)
	{
		ROS_WARN_STREAM("base_trajectory : writeMatlabPath called with empty path");
		return;
	}
	ros::Time first_pose_time = poses[0].header.stamp;

	std::array<std::vector<double>, 4> positions;
	for (const auto &pose : poses)
	{
		positions[0].push_back((pose.header.stamp - first_pose_time).toSec());
		positions[1].push_back(pose.pose.position.x);
		positions[2].push_back(pose.pose.position.y);
		double roll;
		double pitch;
		double yaw;
		const tf2::Quaternion rotQuat(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

		tf2::Matrix3x3(rotQuat).getRPY(roll, pitch, yaw);
		positions[3].push_back(yaw);
	}
	std::array<std::vector<double>, 3> velocities;
	for (size_t i = 0; i < velocities.size(); i++)
		velocities[i].push_back(0);
	for (size_t i = 1; i < positions[0].size(); i++)
	{
		const auto dt = positions[0][i] - positions[0][i-1];

		velocities[0].push_back((positions[1][i] - positions[1][i-1]) / dt);
		velocities[1].push_back((positions[2][i] - positions[2][i-1]) / dt);
		velocities[2].push_back(angles::shortest_angular_distance(positions[3][i-1], positions[3][i]) / dt);
	}
	std::array<std::vector<double>, 3> accelerations;
	for (size_t i = 0; i < accelerations.size(); i++)
	{
		accelerations[i].push_back(0);
	}
	for (size_t i = 1; i < positions[0].size(); i++)
	{
		const auto dt = positions[0][i] - positions[0][i-1];

		accelerations[0].push_back((velocities[0][i] - velocities[0][i-1]) / dt);
		accelerations[1].push_back((velocities[1][i] - velocities[1][i-1]) / dt);
		accelerations[2].push_back(angles::shortest_angular_distance(velocities[2][i-1], velocities[2][i]) / dt);
	}

	std::stringstream str;
	writeMatlabDoubleArray(str, "path_t", positions[0], figureNum);
	writeMatlabDoubleArray(str, "path_x", positions[1], figureNum);
	writeMatlabDoubleArray(str, "path_y", positions[2], figureNum);
	writeMatlabDoubleArray(str, "path_r", positions[3], figureNum);
	writeMatlabDoubleArray(str, "path_dx", velocities[0], figureNum);
	writeMatlabDoubleArray(str, "path_dy", velocities[1], figureNum);
	writeMatlabDoubleArray(str, "path_dr", velocities[2], figureNum);
	writeMatlabDoubleArray(str, "path_ddx", accelerations[0], figureNum);
	writeMatlabDoubleArray(str, "path_ddy", accelerations[1], figureNum);
	writeMatlabDoubleArray(str, "path_ddr", accelerations[2], figureNum);
	str << "figure(" << figureNum << ")" << std::endl;
	str << "subplot(1,1,1)" << std::endl;
	str << "subplot(2,3,1)" << std::endl;
	str << "title('" << label << "')" << std::endl;
	str << "plot(path_t_" << figureNum << ", path_x_" << figureNum << ", path_t_" << figureNum << ", path_y_" << figureNum << ")" << std::endl;
	str << "xlabel('T(seconds)')" << std::endl;
	str << "subplot(2,3,2)" << std::endl;
	str << "plot(path_t_" << figureNum << ", path_r_" << figureNum << ", path_t_" << figureNum << ", path_dr_" << figureNum << ")" << std::endl;
	str << "xlabel('T(seconds)')" << std::endl;
	str << "ylabel('Orientation Position/Velocity')" << std::endl;
	str << "subplot(2,3,3)" << std::endl;
	str << "plot3(path_x_" << figureNum << ", path_y_" << figureNum << ", path_r_" << figureNum << ")" << std::endl;
	str << "xlabel('X position')" << std::endl;
	str << "ylabel('Y position')" << std::endl;
	str << "zlabel('Orientation')" << std::endl;
	str << "subplot(2,3,4)" << std::endl;
	str << "plot(path_t_" << figureNum << ", path_dx_" << figureNum << ", path_t_" << figureNum << ", path_dy_" << figureNum << ")" << std::endl;
	str << "xlabel('T(seconds)')" << std::endl;
	str << "ylabel('X / Y Velocity')" << std::endl;
	str << "subplot(2,3,5)" << std::endl;
	str << "plot(path_t_" << figureNum << ", path_ddx_" << figureNum << ", path_t_" << figureNum << ", path_ddy_" << figureNum << ")" << std::endl;
	str << "xlabel('T(seconds)')" << std::endl;
	str << "ylabel('X / Y Acceleration')" << std::endl;
	str << "subplot(2,3,6)" << std::endl;
	str << "plot(path_x_" << figureNum << ", path_y_" << figureNum << ")" << std::endl;
	str << "xlabel('X position')" << std::endl;
	str << "ylabel('Y position')" << std::endl;
	const std::string filename(label + ".m");
	std::ofstream of(filename);
	of << str.str();

	//ROS_INFO_STREAM_FILTER(&messageFilter, "Matlab_paths: " << std::endl << str.str());
}

void writeMatlabPathNew(const std::vector<geometry_msgs::PoseStamped> &poses,
						const std::vector<geometry_msgs::PoseStamped> &poses_velocity,
						const std::vector<geometry_msgs::PoseStamped> &poses_acceleration,
						int figureNum, const std::string &label)
{
	if (poses.size() == 0)
	{
		ROS_WARN_STREAM("base_trajectory : writeMatlabPath called with empty path");
		return;
	}
	ros::Time first_pose_time = poses[0].header.stamp;

	std::array<std::vector<double>, 4> positions;
	for (const auto &pose : poses)
	{
		positions[0].push_back((pose.header.stamp - first_pose_time).toSec());
		positions[1].push_back(pose.pose.position.x);
		positions[2].push_back(pose.pose.position.y);
		double roll;
		double pitch;
		double yaw;
		const tf2::Quaternion rotQuat(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

		tf2::Matrix3x3(rotQuat).getRPY(roll, pitch, yaw);
		positions[3].push_back(yaw);
	}
	std::array<std::vector<double>, 3> velocities;
	for (const auto &pose : poses_velocity)
	{
		velocities[0].push_back(pose.pose.position.x);
		velocities[1].push_back(pose.pose.position.y);
		double roll;
		double pitch;
		double yaw;
		const tf2::Quaternion rotQuat(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

		tf2::Matrix3x3(rotQuat).getRPY(roll, pitch, yaw);
		velocities[2].push_back(yaw);
	}
	std::array<std::vector<double>, 3> accelerations;
	for (const auto &pose : poses_acceleration)
	{
		accelerations[0].push_back(pose.pose.position.x);
		accelerations[1].push_back(pose.pose.position.y);
		double roll;
		double pitch;
		double yaw;
		const tf2::Quaternion rotQuat(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

		tf2::Matrix3x3(rotQuat).getRPY(roll, pitch, yaw);
		accelerations[2].push_back(yaw);
	}

	std::stringstream str;
	writeMatlabDoubleArray(str, "path_t", positions[0], figureNum);
	writeMatlabDoubleArray(str, "path_x", positions[1], figureNum);
	writeMatlabDoubleArray(str, "path_y", positions[2], figureNum);
	writeMatlabDoubleArray(str, "path_r", positions[3], figureNum);
	writeMatlabDoubleArray(str, "path_dx", velocities[0], figureNum);
	writeMatlabDoubleArray(str, "path_dy", velocities[1], figureNum);
	writeMatlabDoubleArray(str, "path_dr", velocities[2], figureNum);
	writeMatlabDoubleArray(str, "path_ddx", accelerations[0], figureNum);
	writeMatlabDoubleArray(str, "path_ddy", accelerations[1], figureNum);
	writeMatlabDoubleArray(str, "path_ddr", accelerations[2], figureNum);
	str << "figure(" << figureNum << ")" << std::endl;
	str << "subplot(1,1,1)" << std::endl;
	str << "subplot(2,3,1)" << std::endl;
	str << "title('" << label << "')" << std::endl;
	str << "plot(path_t_" << figureNum << ", path_x_" << figureNum << ", path_t_" << figureNum << ", path_y_" << figureNum << ")" << std::endl;
	str << "xlabel('T(seconds)')" << std::endl;
	str << "subplot(2,3,2)" << std::endl;
	str << "plot(path_t_" << figureNum << ", path_r_" << figureNum << ", path_t_" << figureNum << ", path_dr_" << figureNum << ")" << std::endl;
	str << "xlabel('T(seconds)')" << std::endl;
	str << "ylabel('Orientation Position/Velocity')" << std::endl;
	str << "subplot(2,3,3)" << std::endl;
	str << "plot3(path_x_" << figureNum << ", path_y_" << figureNum << ", path_r_" << figureNum << ")" << std::endl;
	str << "xlabel('X position')" << std::endl;
	str << "ylabel('Y position')" << std::endl;
	str << "zlabel('Orientation')" << std::endl;
	str << "subplot(2,3,4)" << std::endl;
	str << "plot(path_t_" << figureNum << ", path_dx_" << figureNum << ", path_t_" << figureNum << ", path_dy_" << figureNum << ")" << std::endl;
	str << "xlabel('T(seconds)')" << std::endl;
	str << "ylabel('X / Y Velocity')" << std::endl;
	str << "subplot(2,3,5)" << std::endl;
	str << "plot(path_t_" << figureNum << ", path_ddx_" << figureNum << ", path_t_" << figureNum << ", path_ddy_" << figureNum << ")" << std::endl;
	str << "xlabel('T(seconds)')" << std::endl;
	str << "ylabel('X / Y Acceleration')" << std::endl;
	str << "subplot(2,3,6)" << std::endl;
	str << "plot(path_x_" << figureNum << ", path_y_" << figureNum << ")" << std::endl;
	str << "xlabel('X position')" << std::endl;
	str << "ylabel('Y position')" << std::endl;
	const std::string filename(label + ".m");
	std::ofstream of(filename);
	of << str.str();

	//ROS_INFO_STREAM_FILTER(&messageFilter, "Matlab_paths: " << std::endl << str.str());
}

// Write matlab code to generate a series of png images from plots output by
// writeMatlabSplines code
void writeMatlabMovieScript(const size_t frameCount, const std::string &label)
{
	std::stringstream s;

	s << label << 1 << std::endl;
	s << "figure(1);" << std::endl;
	s << "subplot(1,1,1);" << std::endl;
	s << "plot(px_y_" << 1 << ", py_y_" << 1 << ");" << std::endl;
	s << "xlimits = xlim; ylimits = ylim;" << std::endl;

	for (size_t figureNum = 2; figureNum < frameCount; figureNum++)
	{
		s << label << figureNum << std::endl;
		s << "figure(1);" << std::endl;
		s << "subplot(1,1,1);" << std::endl;
		s << "plot(px_y_" << figureNum << ", py_y_" << figureNum << ");" << std::endl;
		s << "xl = xlim; yl = ylim;" << std::endl;
		s << "xlimits(1) = min(xlimits(1), xl(1));" << std::endl;
		s << "xlimits(2) = max(xlimits(2), xl(2));" << std::endl;
		s << "ylimits(1) = min(ylimits(1), yl(1));" << std::endl;
		s << "ylimits(2) = max(ylimits(2), yl(2));" << std::endl;
		s << "pause(0.05);" << std::endl;
		s << "close all" << std::endl;
	}

	for (size_t figureNum = 1; figureNum < frameCount; figureNum++)
	{
		s << label << figureNum << std::endl;
		s << "figure(1);" << std::endl;
		s << "subplot(1,1,1);" << std::endl;
		s << "title('" << label << "');" << std::endl;
		s << "plot(px_y_" << figureNum << ", py_y_" << figureNum << ");" << std::endl;
		s << "xlabel('X position');" << std::endl;
		s << "ylabel('Y position');" << std::endl;
		s << "xlim(xlimits); ylim(ylimits);" << std::endl;
		s << "saveas(1, \"frame" << figureNum << ".png\");" << std::endl;
		s << "pause(0.05);" << std::endl;
		s << "close all;" << std::endl;
	}
	const std::string filename(label + "Script.m");
	std::ofstream of(filename);
	of << s.str();
}
