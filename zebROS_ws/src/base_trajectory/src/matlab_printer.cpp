#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "base_trajectory/matlab_printer.h"
#include "base_trajectory/message_filter.h"

// For printing out matlab code for testing
static void printCoefs(std::stringstream &s, const std::string &name, const std::vector<base_trajectory_msgs::Coefs> &coefs, int figureNum)
{
	for (size_t i = 0; i < coefs.size(); i++)
	{
		s << "p" << name << i << "_" << figureNum << " = [";
		for (size_t j = 0; j < coefs[i].spline.size(); j++)
		{
			s << coefs[i].spline[j];
			if (j < coefs[i].spline.size() - 1)
				s << ", ";
		}
		s << "];" << std::endl;
	}
}

// For printing out matlab code for testing
static void printPolyval(std::stringstream &s, const std::string &name, size_t size, const std::vector<double> &end_points, int figureNum)
{
	s << "p" << name << "_y_" << figureNum << " = [";
	for (size_t i = 0; i < size; i++)
	{
		double x_offset;
		if (i == 0)
		{
			x_offset = 0;
		}
		else
		{
			x_offset = end_points[i - 1];
		}
		s << "polyval(p" << name << i << "_" << figureNum << ", x" << i << "_" << figureNum << " - " << x_offset << ")";
		if (i < size - 1)
			s << ", ";
	}
	s << "];" << std::endl;
}

// Generate matlab / octave code for displaying generated splines
void writeMatlabSplines(const base_trajectory_msgs::GenerateSpline::Response &msg, int figureNum, const std::string &label)
{
	std::stringstream s;
	s << std::endl;
	for (size_t i = 0; i < msg.end_points.size(); i++)
	{
		double range;
		double prev_x;
		if (i == 0)
		{
			range = msg.end_points[0];
			prev_x = 0;
		}
		else
		{
			range = msg.end_points[i] - msg.end_points[i-1];
			prev_x = msg.end_points[i-1];
		}
		s << "x" << i << "_" << figureNum << " = " << prev_x << ":" << range / 100.
		  << ":" << msg.end_points[i] << ";" << std::endl;
	}
	s << "x_" << figureNum << " = [";
	for (size_t i = 0; i < msg.end_points.size(); i++)
	{
		s << "x" << i << "_" << figureNum;
		if (i < msg.end_points.size() - 1)
			s << ", ";
	}
	s << "];" << std::endl;
	s << std::endl;
	printCoefs(s, "x", msg.x_coefs, figureNum);
	printCoefs(s, "y", msg.y_coefs, figureNum);
	printCoefs(s, "orient", msg.orient_coefs, figureNum);
	for (size_t i = 0; i < msg.x_coefs.size(); i++)
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
	printPolyval(s, "x", msg.x_coefs.size(), msg.end_points, figureNum);
	printPolyval(s, "dx", msg.x_coefs.size(), msg.end_points, figureNum);
	printPolyval(s, "ddx", msg.x_coefs.size(), msg.end_points, figureNum);
	printPolyval(s, "dddx", msg.x_coefs.size(), msg.end_points, figureNum);
	printPolyval(s, "y", msg.y_coefs.size(), msg.end_points, figureNum);
	printPolyval(s, "dy", msg.y_coefs.size(), msg.end_points, figureNum);
	printPolyval(s, "ddy", msg.y_coefs.size(), msg.end_points, figureNum);
	printPolyval(s, "dddy", msg.y_coefs.size(), msg.end_points, figureNum);
	printPolyval(s, "orient", msg.orient_coefs.size(), msg.end_points, figureNum);
	printPolyval(s, "dorient", msg.orient_coefs.size(), msg.end_points, figureNum);
	printPolyval(s, "ddorient", msg.orient_coefs.size(), msg.end_points, figureNum);
	printPolyval(s, "dddorient", msg.orient_coefs.size(), msg.end_points, figureNum);
	s << "figure(" << figureNum << ")" << std::endl;
	s << "subplot(1,1,1)" << std::endl;
	s << "subplot(3,2,1)" << std::endl;
	s << "title('" << label << "')" << std::endl;
	s << "plot(px_y_" << figureNum << ", py_y_" << figureNum << ")" << std::endl;
	s << "xlabel('X position')" << std::endl;
	s << "ylabel('Y position')" << std::endl;
	s << "subplot(3,2,2)" << std::endl;
	s << "plot(x_" << figureNum << ", porient_y_" << figureNum << ", x_" << figureNum << ", pdorient_y_" << figureNum << ")" << std::endl;
	s << "xlabel('arbT')" << std::endl;
	s << "ylabel('Orient / dOrient')" << std::endl;
	s << "subplot(3,2,3)" << std::endl;
	s << "plot (x_" << figureNum << ",px_y_" << figureNum << ", x_" << figureNum << ", py_y_" << figureNum << ")" << std::endl;
	s << "xlabel('arbT')" << std::endl;
	s << "ylabel('X / Y Position')" << std::endl;
	s << "subplot(3,2,4)" << std::endl;
	s << "plot (x_" << figureNum << ",pdx_y_" << figureNum << ", x_" << figureNum << ", pdy_y_" << figureNum << ")" << std::endl;
	s << "xlabel('arbT')" << std::endl;
	s << "ylabel('X / Y Velocity')" << std::endl;
	s << "subplot(3,2,5)" << std::endl;
	s << "plot (x_" << figureNum << ",pddx_y_" << figureNum << ", x_" << figureNum << ", pddy_y_" << figureNum << ")" << std::endl;
	s << "xlabel('arbT')" << std::endl;
	s << "ylabel('X / Y Acceleration')" << std::endl;
	s << "subplot(3,2,6)" << std::endl;
	s << "plot (x_" << figureNum << ",pdddx_y_" << figureNum << ", x_" << figureNum << ", pdddy_y_" << figureNum << ")" << std::endl;
	s << "xlabel('arbT')" << std::endl;
	s << "ylabel('X / Y Jerk')" << std::endl;
	ROS_INFO_STREAM_FILTER(&messageFilter, "Matlab_splines : " << s.str());

}

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
		velocities[2].push_back((positions[3][i] - positions[3][i-1]) / dt);
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
		accelerations[2].push_back((velocities[2][i] - velocities[2][i-1]) / dt);
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
	str << "subplot(2,2,1)" << std::endl;
	str << "title('" << label << "')" << std::endl;
	str << "plot(path_t_" << figureNum << ", path_x_" << figureNum << ", path_t_" << figureNum << ", path_y_" << figureNum << ")" << std::endl;
	str << "xlabel('T(seconds)')" << std::endl;
	str << "ylabel('X / Y Position')" << std::endl;
	str << "subplot(2,2,2)" << std::endl;
	str << "plot(path_t_" << figureNum << ", path_dx_" << figureNum << ", path_t_" << figureNum << ", path_dy_" << figureNum << ")" << std::endl;
	str << "xlabel('T(seconds)')" << std::endl;
	str << "ylabel('X / Y Velocity')" << std::endl;
	str << "subplot(2,2,3)" << std::endl;
	str << "plot(path_t_" << figureNum << ", path_ddx_" << figureNum << ", path_t_" << figureNum << ", path_ddy_" << figureNum << ")" << std::endl;
	str << "xlabel('T(seconds)')" << std::endl;
	str << "ylabel('X / Y Acceleration')" << std::endl;
	str << "subplot(2,2,4)" << std::endl;

	ROS_INFO_STREAM_FILTER(&messageFilter, "Matlab_paths: " << std::endl << str.str());
}


