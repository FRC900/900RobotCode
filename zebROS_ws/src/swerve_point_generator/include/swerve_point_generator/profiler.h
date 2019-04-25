#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include <swerve_point_generator/GenerateSwerveProfile.h> //ROS Service data type
#include <swerve_point_generator/spline.h> //tk::spline library

#include <ros/console.h>

namespace swerve_profile
{
const double max_path_radius = 1.0e10;

//Data type for characterizing a point on the path
struct path_point
{
	double radius;
	double pos_x;
	double pos_y;
	double path_angle;
	//double path_angle_deriv;
	double orientation;
	double angular_velocity;
	double angular_accel;
	path_point(void):
		radius(max_path_radius),
		pos_x(0),
		pos_y(0),
		path_angle(0),
		orientation(0),
		angular_velocity(0),
		angular_accel(0)
	{
	}
};

//Spline storage data type
struct spline_coefs
{
	double a;
	double b;
	double c;
	double d;
	double e;
	double f;
	spline_coefs(void):
		a(0),
		b(0),
		c(0),
		d(0),
		e(0),
		f(0)
	{
	}
	spline_coefs(const double &aa, const double &bb, const double &cc, const double &dd, const double &ee, const double &ff) :
		a(aa),
		b(bb),
		c(cc),
		d(dd),
		e(ee),
		f(ff)
	{
	}

	const spline_coefs first_derivative(void) const
	{
		return spline_coefs(0, 5 * a, 4 * b, 3 * c, 2 * d, 1 * e);
	}

	void print(std::ostream &os) const
	{
		os << "a:" << a << " b:" << b << " c:" << c << " d:" << d << " e:" << e << " f:" << f;
	}
};

// Add operator << which calls print() for all classes which
// have that method defined
template<class T>
auto operator<<(std::ostream &os, const T &t) -> decltype(t.print(os), os)
{
	t.print(os);
	return os;
}

class SwerveMessageFilter : public ros::console::FilterBase
{
	public:
		SwerveMessageFilter(bool enabled)
		{
			enabled_ = enabled;
		}
		bool isEnabled(void) override
		{
			return enabled_;
		}
		bool isEnabled(ros::console::FilterParams &) override
		{
			return isEnabled();
		}
	private:
		bool enabled_;
};

class swerve_profiler
{
	public:
		//Constructor saves swerve characteristics and dt
		swerve_profiler(double max_wheel_dist, double max_wheel_mid_accel, double max_wheel_vel,
						double dt, double ang_accel_conv,
						double max_wheel_brake_accel, bool debug = true);

		//Generates full profile. Has some spline manipulation options
		bool generate_profile(std::vector<spline_coefs> x_splines,
							  std::vector<spline_coefs> y_splines,
							  std::vector<spline_coefs> orient_splines, const double initial_v, const double final_v,
							  swerve_point_generator::GenerateSwerveProfile::Response &out_msg,
							  const std::vector<double> &end_points, double t_shift, bool flip_dirc);
		//swerve_point_generator::GenerateSwerveProfile::Response is part of ROS custom service data type
		double getDT(void) const { return dt_; }

	private:
		//Gets all the information for the path_point struct
		void comp_point_characteristics(const std::vector<spline_coefs> &x_splines,
										const std::vector<spline_coefs> &y_splines,
										const std::vector<spline_coefs> &x_splines_first_deriv,
										const std::vector<spline_coefs> &y_splines_first_deriv,
										const std::vector<spline_coefs> &x_splines_second_deriv,
										const std::vector<spline_coefs> &y_splines_second_deriv,
										const std::vector<spline_coefs> &orient_splines,
										const std::vector<spline_coefs> &orient_splines_first_deriv,
										const std::vector<spline_coefs> &orient_splines_second_deriv, path_point &holder_point,
										const std::vector<double> &end_points, const std::vector<double> &dtds_by_spline,
										const std::vector<double> &arc_length_by_spline, const double t,
										const double arc_length);
		//TODO: consider putting some of these things together in structs

		//Creates cubic spline interpolation
		tk::spline parametrize_spline(const std::vector<spline_coefs> &x_spline,
									  const std::vector<spline_coefs> &y_spline, const std::vector<double> &end_points,
									  double &total_arc_length, std::vector<double> &dtds_by_spline,
									  std::vector<double> &arc_length_by_spline);

		//Calculates a point on some spline
		void calc_point(const spline_coefs &spline, const double t, double &returner);

		//Applies constraints to solve for next velocity
		bool solve_for_next_V(const path_point &path, const double path_length, double &current_v,
							  const double current_pos, const double accel_defined, std::vector<double> &accelerations);

		// Generates next pos/vel/acc from current pos/vel plus a point on the spline
		void calc_angular_terms(const double arb_t, const std::vector<spline_coefs> &orient_splines, const std::vector<double> &end_points,
				double curr_pos, double curr_vel, bool back_pass,
				double &next_pos, double &next_vel, double &next_acc);

		bool coerce(double &val, const double min, const double max);

		//Solves quadratic equation for greater root
		bool poly_solve(const double a, const double b, const double c, double &x);

		//Saved information from constructor:
		double max_wheel_dist_; //From center of rotation
		double max_wheel_vel_; //v_max
		double max_wheel_mid_accel_; //a_max for speeding up
		double max_wheel_brake_accel_; //a_max for slowing down
		double ang_accel_conv_; //c_a
		double dt_;
		double t_shift_; //Spline parameter shift
		bool flip_dirc_; //Spline reverse
		double t_total_; //Total time

		SwerveMessageFilter message_filter_;
};
}
