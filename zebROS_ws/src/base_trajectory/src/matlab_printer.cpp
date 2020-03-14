#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "base_trajectory/matlab_printer.h"
#include "base_trajectory/message_filter.h"

// For printing out matlab code for testing
static void printCoefs(std::stringstream &s, const std::string &name, const std::vector<base_trajectory_msgs::Coefs> &coefs)
{
	for (size_t i = 0; i < coefs.size(); i++)
	{
		s << "p" << name << i << " = [";
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
static void printPolyval(std::stringstream &s, const std::string &name, size_t size, const std::vector<double> &end_points)
{
	s << "p" << name << "_y = [";
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
		s << "polyval(p" << name << i << ", x" << i << " - " << x_offset << ")";
		if (i < size - 1)
			s << ", ";
	}
	s << "];" << std::endl;
}

// Generate matlab / octave code for displaying generated splines
void writeMatlabSplines(const base_trajectory_msgs::GenerateSpline::Response &msg)
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
		s << "x" << i << " = " << prev_x << ":" << range / 100.
		  << ":" << msg.end_points[i] << ";" << std::endl;
	}
	s << "x = [";
	for (size_t i = 0; i < msg.end_points.size(); i++)
	{
		s << "x" << i;
		if (i < msg.end_points.size() - 1)
			s << ", ";
	}
	s << "];" << std::endl;
	s << std::endl;
	printCoefs(s, "x", msg.x_coefs);
	printCoefs(s, "y", msg.y_coefs);
	printCoefs(s, "orient", msg.orient_coefs);
	for (size_t i = 0; i < msg.x_coefs.size(); i++)
	{
		s << "pdx" << i << " = polyder(px" << i << ");" << std::endl;
		s << "pddx" << i << " = polyder(pdx" << i << ");" << std::endl;
		s << "pdddx" << i << " = polyder(pddx" << i << ");" << std::endl;
		s << "pdy" << i << " = polyder(py" << i << ");" << std::endl;
		s << "pddy" << i << " = polyder(pdy" << i << ");" << std::endl;
		s << "pdddy" << i << " = polyder(pddy" << i << ");" << std::endl;
		s << "pdorient" << i << " = polyder(porient" << i << ");" << std::endl;
		s << "pddorient" << i << " = polyder(pdorient" << i << ");" << std::endl;
		s << "pdddorient" << i << " = polyder(pddorient" << i << ");" << std::endl;
	}
	printPolyval(s, "x", msg.x_coefs.size(), msg.end_points);
	printPolyval(s, "dx", msg.x_coefs.size(), msg.end_points);
	printPolyval(s, "ddx", msg.x_coefs.size(), msg.end_points);
	printPolyval(s, "dddx", msg.x_coefs.size(), msg.end_points);
	printPolyval(s, "y", msg.y_coefs.size(), msg.end_points);
	printPolyval(s, "dy", msg.y_coefs.size(), msg.end_points);
	printPolyval(s, "ddy", msg.y_coefs.size(), msg.end_points);
	printPolyval(s, "dddy", msg.y_coefs.size(), msg.end_points);
	printPolyval(s, "orient", msg.orient_coefs.size(), msg.end_points);
	printPolyval(s, "dorient", msg.orient_coefs.size(), msg.end_points);
	printPolyval(s, "ddorient", msg.orient_coefs.size(), msg.end_points);
	printPolyval(s, "dddorient", msg.orient_coefs.size(), msg.end_points);
	s << "figure(1)" << std::endl;
	s << "subplot(1,1,1)" << std::endl;
	s << "subplot(3,2,1)" << std::endl;
	s << "plot(px_y, py_y)" << std::endl;
	s << "subplot(3,2,2)" << std::endl;
	s << "plot(x, porient_y, x, pdorient_y)" << std::endl;
	s << "subplot(3,2,3)" << std::endl;
	s << "plot (x,px_y, x, py_y)" << std::endl;
	s << "subplot(3,2,4)" << std::endl;
	s << "plot (x,pdx_y, x, pdy_y)" << std::endl;
	s << "subplot(3,2,5)" << std::endl;
	s << "plot (x,pddx_y, x, pddy_y)" << std::endl;
	s << "subplot(3,2,6)" << std::endl;
	s << "plot (x,pdddx_y, x, pdddy_y)" << std::endl;
	ROS_INFO_STREAM_FILTER(&messageFilter, "Matlab_splines : " << s.str());

}

static void writeMatlabDoubleArray(std::stringstream &s, const std::string &name, const std::vector<double> &values)
{
	s << name << " = [";
	for (size_t i = 0; i < values.size(); i++)
	{
		s << values[i];
		if (i < (values.size() - 1))
			s << ", ";
	}
	s << "];" << std::endl;
}

void writeMatlabPath(const std::vector<geometry_msgs::PoseStamped> &poses)
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
	writeMatlabDoubleArray(str, "path_t", positions[0]);
	writeMatlabDoubleArray(str, "path_x", positions[1]);
	writeMatlabDoubleArray(str, "path_y", positions[2]);
	writeMatlabDoubleArray(str, "path_r", positions[3]);
	writeMatlabDoubleArray(str, "path_dx", velocities[0]);
	writeMatlabDoubleArray(str, "path_dy", velocities[1]);
	writeMatlabDoubleArray(str, "path_dr", velocities[2]);
	writeMatlabDoubleArray(str, "path_ddx", accelerations[0]);
	writeMatlabDoubleArray(str, "path_ddy", accelerations[1]);
	writeMatlabDoubleArray(str, "path_ddr", accelerations[2]);
	str << "figure(2)" << std::endl;
	str << "subplot(1,1,1)" << std::endl;
	str << "subplot(3,2,1)" << std::endl;
	str << "plot(path_t, path_x, path_t, path_y)" << std::endl;
	str << "subplot(3,2,2)" << std::endl;
	str << "plot(path_t, path_dx, path_t, path_dy)" << std::endl;
	str << "subplot(3,2,3)" << std::endl;
	str << "plot(path_t, path_ddx, path_t, path_ddy)" << std::endl;
	str << "subplot(3,2,4)" << std::endl;
	str << "subplot(3,2,5)" << std::endl;
	str << "subplot(3,2,6)" << std::endl;

	ROS_INFO_STREAM_FILTER(&messageFilter, "Matlab_paths: " << std::endl << str.str());
}


