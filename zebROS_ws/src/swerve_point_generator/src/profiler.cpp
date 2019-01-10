#include <swerve_point_generator/profiler.h>
#include <ros/console.h>
#include <iostream>

namespace swerve_profile
{
//Constructor
swerve_profiler::swerve_profiler(double max_wheel_dist, double max_wheel_mid_accel,
								 double max_wheel_vel, double max_steering_accel,
								 double max_steering_vel, double dt,
								 double ang_accel_conv, double max_wheel_brake_accel)
	:
	max_wheel_dist_(max_wheel_dist),
	max_wheel_mid_accel_(max_wheel_mid_accel),
	max_wheel_vel_(max_wheel_vel),
	max_steering_accel_(max_steering_accel),
	max_steering_vel_(max_steering_vel),
	dt_(dt),
	ang_accel_conv_(ang_accel_conv),
	max_wheel_brake_accel_(max_wheel_brake_accel)
{
}

//Generation function
bool swerve_profiler::generate_profile(std::vector<spline_coefs> x_splines,
									   std::vector<spline_coefs> y_splines,
									   std::vector<spline_coefs> orient_splines,
									   const double initial_v, const double final_v,
									   swerve_point_generator::GenerateSwerveProfile::Response &out_msg,
									   const std::vector<double> &end_points, double t_shift, bool flip_dirc)
{
	t_shift_ = t_shift;
	flip_dirc_ = flip_dirc;
	// Bounds checking. Not safe to proceed if end_points is empty.
	if (end_points.empty())
	{
		ROS_ERROR("Endpoints should never be empty!");
		return false;
	}
	if (x_splines.empty())
	{
		ROS_ERROR("x-splines should also not be empty");
		return false;
	}
	if (y_splines.empty())
	{
		ROS_ERROR("y-splines should also not be empty");
		return false;
	}
	if (orient_splines.empty())
	{
		ROS_ERROR("orient-splines should also not be empty");
		return false;
	}
	if (x_splines.size() != y_splines.size())
	{
		ROS_ERROR("x-splines and y-splines not same size");
		return false;
	}
	if (x_splines.size() != orient_splines.size())
	{
		ROS_ERROR("x-splines and orient-splines not same size");
		return false;
	}
	if (x_splines.size() != end_points.size())
	{
		ROS_ERROR("x-splines and end points not same size");
		return false;
	}

	t_total_ = end_points[end_points.size() - 1]; //assumes t starts at 0
	tk::spline spline;

	ROS_WARN("generate_profile called");

	std::vector<double> velocities;
	velocities.reserve(155 / dt_); //For full auto :)
	std::vector<double> positions;
	positions.reserve(155 / dt_); //For full auto :)

	std::vector<spline_coefs> x_splines_first_deriv;
	std::vector<spline_coefs> y_splines_first_deriv;
	std::vector<spline_coefs> orient_splines_first_deriv;
	std::vector<spline_coefs> x_splines_second_deriv;
	std::vector<spline_coefs> y_splines_second_deriv;
	std::vector<spline_coefs> orient_splines_second_deriv;
	//Take dervitives of splines
	for (size_t i = 0; i < x_splines.size(); i++)
	{
		ROS_INFO_STREAM("x splines[" << i << "] : " << x_splines[i]);
		x_splines_first_deriv.push_back(x_splines[i].first_derivative());
		ROS_INFO_STREAM("x splines[" << i << "] first deriv: " << x_splines_first_deriv[i]);

		x_splines_second_deriv.push_back(x_splines_first_deriv[i].first_derivative());
		ROS_INFO_STREAM("x splines[" << i << "] second deriv: " << x_splines_second_deriv[i]);

		ROS_INFO_STREAM("y splines[" << i << "] : " << y_splines[i]);
		y_splines_first_deriv.push_back(y_splines[i].first_derivative());
		ROS_INFO_STREAM("y splines[" << i << "] first deriv: " << y_splines_first_deriv[i]);

		y_splines_second_deriv.push_back(y_splines_first_deriv[i].first_derivative());
		ROS_INFO_STREAM("y splines[" << i << "] second deriv: " << y_splines_second_deriv[i]);

		orient_splines_first_deriv.push_back(orient_splines[i].first_derivative());
		ROS_INFO_STREAM("orient splines[" << i << "] first deriv: " << orient_splines_first_deriv[i]);

		ROS_INFO_STREAM("orient splines[" << i << "] : " << orient_splines[i]);
		orient_splines_second_deriv.push_back(orient_splines_first_deriv[i].first_derivative());
		ROS_INFO_STREAM("orient splines[" << i << "] second deriv: " << orient_splines_second_deriv[i]);
	}
	//Flip if needed
	if (flip_dirc_)
	{
		//ROS_ERROR_STREAM("flipping");
		std::reverse(x_splines.begin(), x_splines.end());
		std::reverse(y_splines.begin(), y_splines.end());
		std::reverse(orient_splines.begin(), orient_splines.end());
		std::reverse(x_splines_first_deriv.begin(), x_splines_first_deriv.end());
		std::reverse(y_splines_first_deriv.begin(), y_splines_first_deriv.end());
		std::reverse(orient_splines_first_deriv.begin(), orient_splines_first_deriv.end());
		std::reverse(x_splines_second_deriv.begin(), x_splines_second_deriv.end());
		std::reverse(y_splines_second_deriv.begin(), y_splines_second_deriv.end());
		std::reverse(orient_splines_second_deriv.begin(), orient_splines_second_deriv.end());
	}
	//ROS_WARN("called2");
	std::vector<double> dtds_for_spline;
	std::vector<double> arc_length_for_spline;
	double total_arc;
	double period_t;
	//Run spline parametrizing code - also gets dtds and arc lengths
	spline = parametrize_spline(x_splines_first_deriv, y_splines_first_deriv, end_points,
								total_arc, dtds_for_spline, arc_length_for_spline, period_t);
	int point_count = 0;
	std::vector<double> accelerations;
	path_point holder_point;
	//back pass
	//ROS_INFO_STREAM("total arc: " <<total_arc);
	//i is the arc length we are at in the loop
	double curr_v = final_v;
	for (double i = total_arc /*- .1*/ ; i > 0;)
	{
		i -= curr_v * dt_;
		if (i < 0)
		{
			continue;
		}

		ROS_INFO_STREAM("back pass V: " << curr_v);
		velocities.push_back(curr_v); //For limiting the velocity on the back pass
		positions.push_back(i);

		point_count++;
		//if (point_count % 100 == 0)
		//ROS_INFO_STREAM("num points: " << point_count );

		const double t_raw2 = spline(i); //Get t value from the cubic spline interpolation of t vs arc length
		//ROS_INFO_STREAM("curr_v: " << curr_v);
		//ROS_WARN("even_now");

		//Compute all the path info
		comp_point_characteristics(x_splines, y_splines, x_splines_first_deriv, y_splines_first_deriv,
								   x_splines_second_deriv, y_splines_second_deriv, orient_splines, orient_splines_first_deriv,
								   orient_splines_second_deriv, holder_point, end_points, dtds_for_spline, arc_length_for_spline,
								   t_raw2, i);

		ROS_INFO_STREAM("in back pass after comp_point_characteristics orientation_velocities = " << holder_point.angular_velocity);

		//Solve for the next V using constraints
		if (!solve_for_next_V(holder_point, total_arc, curr_v, i, max_wheel_brake_accel_, accelerations))
		{
			return false;
		}
		ROS_INFO_STREAM("in back pass after solve_for_next_V orientation_velocities = " << holder_point.angular_velocity);
	}
	//ROS_WARN("called3");
	//ROS_INFO_STREAM("passed loop 1");
	velocities.erase(velocities.end() - 1); //End must be erased
	positions.erase(positions.end() - 1);

	// TODO : check this - if point_count is initialized to
	// non-0, it could run out of space below?
	point_count = out_msg.points.size();
	if (out_msg.points.size() == 0)
		out_msg.points.resize(155 / dt_); //For full auto :)  TODO: optimize
	curr_v = initial_v;
	double starting_point = positions.size() - 1;
	ros::Duration now(0);
	ros::Duration period(dt_);
	//Same as back pass, but now forward

	for (double i = 0; i < total_arc /* - .1*/;)
	{
		i += curr_v * dt_;

		if (i > total_arc)
		{
			continue;
		}

		const double t_raw3 = spline(i);
		//ROS_INFO_STREAM("i val: " << i << " t val: " << t_raw3 << " curr v: " << curr_v);

		comp_point_characteristics(x_splines, y_splines, x_splines_first_deriv, y_splines_first_deriv,
								   x_splines_second_deriv, y_splines_second_deriv, orient_splines, orient_splines_first_deriv,
								   orient_splines_second_deriv, holder_point, end_points, dtds_for_spline, arc_length_for_spline,
								   t_raw3, i);
		//ROS_INFO_STREAM("holder_point.pos_x = " << holder_point.pos_x << " holder_point.pos_y = " << holder_point.pos_y);

		//save output values
		out_msg.points[point_count].positions.push_back(holder_point.pos_x);
		out_msg.points[point_count].positions.push_back(holder_point.pos_y);
		out_msg.points[point_count].positions.push_back(holder_point.orientation);
		out_msg.points[point_count].velocities.push_back(cos(holder_point.path_angle) * curr_v );
		out_msg.points[point_count].velocities.push_back(sin(holder_point.path_angle) * curr_v );
		out_msg.points[point_count].velocities.push_back(
			holder_point.angular_velocity * curr_v / max_wheel_dist_);
		ROS_INFO_STREAM("fpass: orient_v = " << holder_point.angular_velocity << ", curr_v*1000 = " << curr_v*1000 << " o_v * curr_v / wheel_dist " << holder_point.angular_velocity * curr_v / max_wheel_dist_);
		//out_msg.points[point_count].velocities.push_back(holder_point.path_angle_deriv * (current_v));
		out_msg.points[point_count].time_from_start = now;
		//ROS_INFO_STREAM(now);
		now += period;
		point_count++;
		//ROS_ERROR_STREAM("1: " << curr_v);
		if (!solve_for_next_V(holder_point, total_arc, curr_v, i, max_wheel_mid_accel_, accelerations)) //originally not the right number of arguments
		{
			return false;
		}
		//ROS_ERROR_STREAM("2: " << curr_v);
		for (size_t k = 0; k < positions.size(); k++)
		{
			if (starting_point - k < 0 || positions[starting_point - k] > i)
			{
				starting_point -= k;
				break;
			}
			//Find point
		}
		//Make sure starting point doesn't go less than 1
		coerce(starting_point, 1, 1000000000000);
		//Linear interpolation to get vel cap

		const double v_sp1 = velocities[starting_point + 1];
		const double v_s   = velocities[starting_point];
		const double p_sp1 = positions[starting_point + 1];
		const double p_s   = positions[starting_point];
		const double vel_cap = i * (v_s - v_sp1) / (p_s - p_sp1) -
							   p_s * (v_s - v_sp1) / (p_s - p_sp1) + v_s;
		//Keep below back pass
		//ROS_INFO_STREAM("pre cut max: " << curr_v);
		//if(curr_v > vel_cap)
		//{
		//    ROS_INFO_STREAM("cut by previous vel max: " << vel_cap << " curr_v: " << curr_v);
		//}
		if (!coerce(curr_v, -100000000000, vel_cap))
		{
			if (!solve_for_next_V(holder_point, total_arc, curr_v, i, max_wheel_mid_accel_, accelerations))
			{
				return false;
			}
			if (coerce(curr_v, -100000000000, vel_cap))
			{
				accelerations.clear();
			}
		}
		else
		{
			accelerations.clear();
		}

		//ROS_INFO_STREAM("post cut max: " << curr_v);
	}
	//ROS_WARN("called3");
	//ROS_ERROR("finished raw generation");
	ROS_INFO_STREAM("time: " << point_count * dt_);
	ROS_INFO_STREAM("total_arc: " << total_arc);
	out_msg.points.erase(out_msg.points.begin() + point_count, out_msg.points.end());
	ROS_ERROR_STREAM("p: " << out_msg.points.size());
	ROS_INFO_STREAM("point_count in profiler: " << point_count);
	for(int i = 0; i < out_msg.points.size(); i++)
	{
		ROS_INFO_STREAM("orientation velocities in profiler= " << out_msg.points[i].velocities[2]);
	}
	return true;
}

bool swerve_profiler::coerce(double &val, const double min, const double max)
{
	if (val > max)
	{
		val = max;
		return true;
	}
	else if (val < min)
	{
		val = min;
		return true;
	}
	else
	{
		return false;
	}
}

// path.angular_velocity = v-sub-a * d in paper
// path.angular_acceleration = a-sub-a * c-sub-a * d
bool swerve_profiler::solve_for_next_V(const path_point &path, const double path_length, double &current_v,
									   const double current_pos, const double accel_defined, std::vector<double> &accelerations)
{
	//ROS_INFO_STREAM(__LINE__ << ": " << path.angular_velocity);
	//This if statement is here so we only try to solve for points on the path
	if (current_pos >= 0 && current_pos <= path_length)
	{
		const double theta = fmod(fabs(path.path_angle - path.orientation), M_PI / 4.); // Paper says PI/2?
		const double cos_t = cos(theta);
		const double sin_t = sin(theta);

		ROS_INFO_STREAM("accel_defined:" << accel_defined);

		//Maximum V based on maximum at wheel V
		const double path_angular_velocity_squared = path.angular_velocity * path.angular_velocity;
		const double v_general_max = max_wheel_vel_ * sqrt(1.0 /
				(1.0 + sqrt(2) * fabs(path.angular_velocity) * (cos_t + sin_t) + path_angular_velocity_squared));

		//if(current_v > v_general_max)
		//{
		//	ROS_INFO_STREAM("cut by general max:" << v_general_max);
		//
		//}

		//Maximum V based on maximum at wheel a
		const double path_radius_squared = path.radius * path.radius;
		const double path_angular_accel_squared = path.angular_accel * path.angular_accel;
		const double path_a_over_r = path.angular_accel / path.radius;
		double v_curve_max;
		if (!poly_solve(sqrt(1.0 / path_radius_squared + path_angular_accel_squared + sqrt(2) * path_a_over_r * (cos_t + sin_t)),
						2.0 * accel_defined * sqrt(1.0 + path_angular_velocity_squared + sqrt(2) * fabs(path.angular_velocity) * (sin_t + cos_t)) / max_wheel_vel_,
						-2.0 * accel_defined,
						v_curve_max))
		{

			ROS_ERROR("poly_solve - solve for next V v_curve_max - failed");
			return false;
		}

		ROS_INFO_STREAM("solve_for_next_V :theta:" << theta << " cos_t:" << cos_t << " sin_t:" << sin_t
				<< " path_radius_squared:" << path_radius_squared
				<< " path_angular_accel_squared:" << path_angular_accel_squared
				<< " path_a_over_r:" << path_a_over_r);
		const double v_curve_max_2 = sqrt(accel_defined /
										  sqrt(1.0 / path_radius_squared + path_angular_accel_squared + sqrt(2) * path_a_over_r * (cos_t + sin_t)));

		//if(current_v > v_curve_max)
		//{
		//	ROS_INFO_STREAM("cut by curve max:" << v_curve_max << " radius:" << path.radius << " eff_max_a:" << eff_max_a);
		//
		//}

		ROS_INFO_STREAM("solve_for_next_V, current_v:" << current_v << " v_general_max:" << v_general_max << " v_curve_max:" << v_curve_max << " v_curve_max_2:" << v_curve_max_2);

		const bool b1 = !coerce(current_v, -v_curve_max, v_curve_max);
		const bool b2 = !coerce(current_v, -v_curve_max_2, v_curve_max_2);
		const bool b3 = !coerce(current_v, -v_general_max, v_general_max);
		if (b1 && b2 && b3) //If we need to threshhold, we don't need to iterate using accel
		{
			//this is where it all breaks
			const double current_v_squared = current_v * current_v;
			const double max_wheel_orientation_accel = fabs(path.angular_accel * current_v_squared);
			//const double max_wheel_orientation_vel = fabs(path.angular_velocity * current_v);
			const double path_induced_a = current_v_squared / path.radius;

			const double a_sub_t_term = sqrt(2.0) * max_wheel_orientation_accel;
			const double ones_term = a_sub_t_term * path_induced_a;

			const double accel_defined_squared = accel_defined * accel_defined;
			const double max_wheel_orientation_accel_squared = max_wheel_orientation_accel * max_wheel_orientation_accel;
			const double path_induced_a_squared = path_induced_a * path_induced_a;
			const double non_trig_ones_term = - accel_defined_squared + max_wheel_orientation_accel_squared + path_induced_a_squared;

			double accel1;
			if (!poly_solve(1,
						(cos_t + sin_t) * a_sub_t_term,
						(cos_t - sin_t) * ones_term + non_trig_ones_term,
						accel1))
			{

				ROS_ERROR("poly_solve - solve for next V accel1 - failed");
				return false;
			}

			double accel2;
			if (!poly_solve(1,
						(-cos_t + sin_t) * a_sub_t_term,
						( cos_t + sin_t) * ones_term + non_trig_ones_term,
						accel2))
			{

				ROS_ERROR("poly_solve - solve for next V accel2 - failed");
				return false;
			}

			//choosing smaller accel
			const double accel_final = std::min(accel1, accel2);

			ROS_INFO_STREAM("accel1:" << accel1 << " accel2:" << accel2 << " accel_final:" << accel_final);

			//Implementation of adams-bashforth:
			const size_t s = accelerations.size();
			ROS_INFO_STREAM("accelerations.size = " << accelerations.size());
			if (s == 0)
			{
				current_v += accel_final * dt_;
			}
			else if (s == 1)
			{
				current_v += dt_ / 2 * (3 * accel_final - accelerations[0]);
			}
			else if (s == 2)
			{
				current_v += dt_ / 12 * (23 * accel_final - 16 * accelerations[1] + 5 * accelerations[0]);
			}
			else if (s == 3)
			{
				current_v += dt_ / 24 * (55 * accel_final - 59 * accelerations[2] + 37 * accelerations[1] - 9 * accelerations[0]);
			}
			else
			{
				current_v += dt_ / (1901. / 720. * accel_final - 1387. / 360. * accelerations[3] + 109. / 30. * accelerations[2] - 637. / 360. * accelerations[1] + 251. / 720. * accelerations[0]);
			}

			//Threshold again
			ROS_INFO_STREAM("solve_for_next_V, after adding accel current_v:" << current_v << " v_general_max:" << v_general_max << " v_curve_max:" << v_curve_max << " v_curve_max_2:" << v_curve_max_2);
			const bool b1 = coerce(current_v, -v_curve_max, v_curve_max);
			const bool b2 = coerce(current_v, -v_curve_max_2, v_curve_max_2);
			const bool b3 = coerce(current_v, -v_general_max, v_general_max);
			if (b1 || b2 || b3)
			{
				accelerations.clear();
			}
			else
			{
				// Maintain up to 4 of the most recent acceleration values
				// Should never have more than 4 entries, but use a while
				// loop just to be safe
				while (accelerations.size() > 3)
					accelerations.erase(accelerations.begin());
				accelerations.push_back(accel_final);
			}
			//ROS_INFO_STREAM("curve max:" << current_v);
		}
		else
		{
			accelerations.clear();
		}
	}
	else
	{
		//If we are off the path we assume that maximum acceleration can be applied
		accelerations.clear();
		current_v += accel_defined * dt_;
		ROS_INFO_STREAM("off path +" << current_v << " accel_defined: " << accel_defined);
		coerce(current_v, -max_wheel_vel_, max_wheel_vel_);
		ROS_INFO_STREAM("off path coerce " << current_v);
	}
	return true;
	//ROS_INFO_STREAM(__LINE__ << ": " << path.angular_velocity);
}

const double spline_points = 100.;
tk::spline swerve_profiler::parametrize_spline(const std::vector<spline_coefs> &x_splines_first_deriv,
		const std::vector<spline_coefs> &y_splines_first_deriv,
		const std::vector<double> &end_points, double &total_arc_length,
		std::vector<double> &dtds_by_spline,
		std::vector<double> &arc_length_by_spline, double &period_t)
{
	//
	total_arc_length = 0;
	period_t = (end_points[0] - 0.0) / spline_points;
	double start = 0;
	double arc_before = 0;
	double b_val = 0;
	std::vector<double> t_vals;
	std::vector<double> s_vals;
	t_vals.reserve(x_splines_first_deriv.size() * (static_cast<size_t>(spline_points) + 1));
	s_vals.reserve(x_splines_first_deriv.size() * (static_cast<size_t>(spline_points) + 1));
	//ROS_INFO_STREAM("Running parametrize");

	//ROS_WARN_STREAM(x_splines_first_deriv.size());

	ROS_INFO_STREAM("THE SIZE HERE IS: " << x_splines_first_deriv.size());
	for (size_t i = 0; i < x_splines_first_deriv.size(); i++)
	{
		ROS_INFO_STREAM("endpoints: " << end_points[i]);

		if (i != 0)
		{
			period_t = (end_points[i] - end_points[i - 1]) / spline_points; //100 is super arbitrary
			start = end_points[i - 1];
		}
		if (i > 1)
		{
			dtds_by_spline.push_back((end_points[i - 1] - end_points[i - 2]) /  (total_arc_length
									 - arc_before));
		}
		else if (i == 1)
		{
			dtds_by_spline.push_back((end_points[0] - 0) /  (total_arc_length - arc_before));
		}
		arc_before = total_arc_length;
		ROS_INFO_STREAM("arc_before: " << arc_before);
		for (size_t k = 0; k < static_cast<size_t>(spline_points); k++)
		{
			const double a_val = k * period_t + start;
			b_val = (k + 1) * period_t + start;
			t_vals.push_back(a_val);
			s_vals.push_back(total_arc_length);
			//TODO: improve efficiency here
			double x_at_a;
			double x_at_b;
			double y_at_a;
			double y_at_b;
			double x_at_avg;
			double y_at_avg;
			calc_point(x_splines_first_deriv[i], a_val, x_at_a);
			calc_point(x_splines_first_deriv[i], b_val, x_at_b);
			calc_point(y_splines_first_deriv[i], a_val, y_at_a);
			calc_point(y_splines_first_deriv[i], b_val, y_at_b);
			calc_point(x_splines_first_deriv[i], (a_val + b_val) / 2, x_at_avg);
			calc_point(y_splines_first_deriv[i], (a_val + b_val) / 2, y_at_avg);

			//f(t) = sqrt((dx/dt)^2 + (dy/dt)^2)

			//ROS_INFO_STREAM("period_t: " << period_t);
			//ROS_INFO_STREAM("idek: " << hypot(x_at_a, y_at_a) + 4 * hypot(x_at_avg, y_at_avg) + hypot(x_at_b, y_at_b));
			total_arc_length += period_t / 6. * (hypot(x_at_a, y_at_a) + 4. * hypot(x_at_avg, y_at_avg) + hypot(x_at_b, y_at_b));
			//ROS_INFO_STREAM("arc_now: " << total_arc_length);
			//Simpsons rule
			//ROS_INFO_STREAM("Spline: " << i << " t_val: " << a_val <<"  arc_length: " << total_arc_length);
		}
		arc_length_by_spline.push_back(total_arc_length);
	}
	if (x_splines_first_deriv.size() == 1)
	{
		dtds_by_spline.push_back((end_points[x_splines_first_deriv.size() - 1] - 0)
								 /  (total_arc_length - arc_before));
	}
	else
	{
		dtds_by_spline.push_back((end_points[x_splines_first_deriv.size() - 1]
								  - end_points[x_splines_first_deriv.size() - 2]) /  (total_arc_length - arc_before));
	}

	//Put in the last values
	t_vals.push_back(b_val);
	s_vals.push_back(total_arc_length);

	//Spline fit of t interms of s (we input a t -> s)
	tk::spline s;

	s.set_points(s_vals, t_vals);
	for (size_t i = 0; i < t_vals.size(); i++)
	{
		ROS_INFO_STREAM("t_val = " << t_vals[i] << " s vals = " << s_vals[i]);
		//ROS_INFO_STREAM("s_vale = " << s_vals[i] << " s vals = " << s(s_vals[i]));
	}
	ROS_INFO_STREAM("successful parametrize spline");
	return s;
}
bool swerve_profiler::poly_solve(const double a, const double b, const double c, double &x)
{
	const double det = b * b - 4 * a * c;
	if (det < 0)
	{
		x = 0;
		return false;
	}
	else
	{
		x = (-b + sqrt(det)) / (2 * a); //This is just one of the roots, but it should be fine for all
										//cases it is used here
		return true;
	}
}
void swerve_profiler::calc_point(const spline_coefs &spline, double t, double &returner)
{
	if (flip_dirc_)t = t_total_ - t;
	t += t_shift_;
	const double t_squared = t * t;
	const double t_cubed   = t_squared * t;
	const double t_fourth  = t_squared * t_squared;
	const double t_fifth   = t_cubed * t_squared;
	returner = spline.a * t_fifth + spline.b * t_fourth + spline.c * t_cubed + spline.d * t_squared + spline.e * t + spline.f;
	//if (t)
	//ROS_INFO_STREAM("calc_point a:" << spline << " t:" << t << " f(t):" << returner);
}

void swerve_profiler::comp_point_characteristics(const std::vector<spline_coefs> &x_splines,
		const std::vector<spline_coefs> &y_splines, const std::vector<spline_coefs> &x_splines_first_deriv,
		const std::vector<spline_coefs> &y_splines_first_deriv, const std::vector<spline_coefs> &x_splines_second_deriv,
		const std::vector<spline_coefs> &y_splines_second_deriv, const std::vector<spline_coefs> &orient_splines,
		const std::vector<spline_coefs> &orient_splines_first_deriv,
		const std::vector<spline_coefs> &orient_splines_second_deriv, path_point &holder_point,
		const std::vector<double> &end_points, const std::vector<double> &dtds_by_spline,
		const std::vector<double> &arc_length_by_spline, const double t, const double arc_length)
{
	size_t which_spline;
	which_spline = 0;
	//Find the spline based on t
	for (; which_spline < x_splines.size() - 1; which_spline++)
	{
		if (t < end_points[which_spline])
		{
			break;
		}
	}

	//Find t_o based on which spline and arc length
	double t_o;
	if (which_spline != 0)
	{
		t_o = (arc_length - arc_length_by_spline[which_spline - 1]) * dtds_by_spline[which_spline]
			  + end_points[which_spline - 1];
	}
	else
	{
		t_o = arc_length * dtds_by_spline[which_spline]; //assumes t starts at 0
	}
	double first_deriv_orient;
	double second_deriv_orient;
	double first_deriv_x;
	double first_deriv_y;
	double second_deriv_x;
	double second_deriv_y;

	//Calculate all the points
	calc_point(x_splines[which_spline], t, holder_point.pos_x);
	calc_point(y_splines[which_spline], t, holder_point.pos_y);
	calc_point(orient_splines[which_spline], t_o, holder_point.orientation); // TODO

	calc_point(x_splines_first_deriv[which_spline], t, first_deriv_x);
	calc_point(y_splines_first_deriv[which_spline], t, first_deriv_y);
	calc_point(x_splines_second_deriv[which_spline], t, second_deriv_x);
	calc_point(y_splines_second_deriv[which_spline], t, second_deriv_y);
	calc_point(orient_splines_first_deriv[which_spline], t_o, first_deriv_orient);
	calc_point(orient_splines_second_deriv[which_spline], t_o, second_deriv_orient);

	//ROS_INFO_STREAM("which spline: " << which_spline << " t_raw: "<< t << " x: "
	//<< holder_point.pos_x << " y: " << holder_point.pos_y << " " <<
	//x_splines[which_spline]);

	//Radius = (x'^2 + y'^2)^(3/2) / (x' * y'' - y' * x'')

	const double denomin = first_deriv_x * second_deriv_y - first_deriv_y * second_deriv_x;

	if (denomin != 0) //div by 0 error checking
	{
		holder_point.radius = fabs(pow(first_deriv_x * first_deriv_x + first_deriv_y * first_deriv_y, 3.0 / 2.0) /
								   denomin);
		// Later math dies if radius is too large, FP overflow?
		holder_point.radius = std::min(holder_point.radius, max_path_radius);
	}
	else
	{
		holder_point.radius = max_path_radius;
	}

	if (fabs(holder_point.pos_x) > 100 || fabs(holder_point.pos_y) > 100)
	{
		ROS_ERROR_STREAM("resonableness exceeded with x of: " << holder_point.pos_x << " and y of: "
						 << holder_point.pos_y << " t: " << t);
	}

	holder_point.path_angle = atan2(first_deriv_y, first_deriv_x) - (holder_point.orientation -  M_PI / 2.0);
	holder_point.angular_velocity = first_deriv_orient * dtds_by_spline[which_spline] * max_wheel_dist_;
	//ROS_INFO_STREAM(__LINE__ << ": " << holder_point.angular_velocity);
	holder_point.angular_accel = fabs(second_deriv_orient * dtds_by_spline[which_spline] *
									  dtds_by_spline[which_spline] * max_wheel_dist_ * ang_accel_conv_);
}
}
