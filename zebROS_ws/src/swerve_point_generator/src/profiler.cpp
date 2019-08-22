#include <swerve_point_generator/profiler.h>
#include <iostream>
#include <cstdlib>
#include <algorithm>

namespace swerve_profile
{
//Constructor
swerve_profiler::swerve_profiler(double max_wheel_dist, double max_wheel_vel,
								 double max_wheel_mid_accel, double max_wheel_brake_accel,
								 double ang_accel_conv, double dt,
								 bool debug)
	:
	max_wheel_dist_(max_wheel_dist),
	max_wheel_vel_(max_wheel_vel),
	max_wheel_mid_accel_(max_wheel_mid_accel),
	max_wheel_brake_accel_(max_wheel_brake_accel),
	ang_accel_conv_(ang_accel_conv),
	dt_(dt),
	message_filter_(debug)
{
	ROS_INFO_STREAM("Starting swerve_profiler : max_wheel_dist_ = " << max_wheel_dist_
	<< " max_wheel_vel_ = " << max_wheel_vel_
	<< " max_wheel_mid_accel_ = " << max_wheel_mid_accel_
	<< " max_wheel_brake_accel_ = " << max_wheel_brake_accel_
	<< " ang_accel_conv_ = " << ang_accel_conv_
	<< " dt_ = " << dt
	<< " debug = " << debug);
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
		ROS_INFO_STREAM("x splines[" << i << "] " << x_splines[i]);
		x_splines_first_deriv.push_back(x_splines[i].first_derivative());
		ROS_INFO_STREAM("x splines[" << i << "] first deriv " << x_splines_first_deriv[i]);

		x_splines_second_deriv.push_back(x_splines_first_deriv[i].first_derivative());
		ROS_INFO_STREAM("x splines[" << i << "] second deriv " << x_splines_second_deriv[i]);

		ROS_INFO_STREAM("y splines[" << i << "] : " << y_splines[i]);
		y_splines_first_deriv.push_back(y_splines[i].first_derivative());
		ROS_INFO_STREAM("y splines[" << i << "] first deriv " << y_splines_first_deriv[i]);

		y_splines_second_deriv.push_back(y_splines_first_deriv[i].first_derivative());
		ROS_INFO_STREAM("y splines[" << i << "] second deriv " << y_splines_second_deriv[i]);

		ROS_INFO_STREAM("orient splines[" << i << "] : " << orient_splines[i]);
		orient_splines_first_deriv.push_back(orient_splines[i].first_derivative());
		ROS_INFO_STREAM("orient splines[" << i << "] first deriv " << orient_splines_first_deriv[i]);

		orient_splines_second_deriv.push_back(orient_splines_first_deriv[i].first_derivative());
		ROS_INFO_STREAM("orient splines[" << i << "] second deriv " << orient_splines_second_deriv[i]);
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
	//Run spline parametrizing code - also gets dtds and arc lengths
	// This turns the x,y linear positions into a single linear-path-position vs. time curve
	spline = parametrize_spline(x_splines_first_deriv, y_splines_first_deriv, end_points,
								total_arc, dtds_for_spline, arc_length_for_spline);
	size_t point_count = 0;
	std::vector<double> accelerations;
	double current_angular_velocity = 0;
	double prev_angular_velocity = 0;
	double prev_angular_position;
	calc_point(orient_splines.back(), spline(total_arc), prev_angular_position);
	std::vector<double> max_angular_velocities;
	size_t search_iterations = 0;
	//back pass
	//ROS_INFO_STREAM("total arc: " <<total_arc);
	//current_spline_position is the arc length we are at in the loop
	double curr_v = final_v;
	for (double current_spline_position = total_arc - curr_v * dt_; current_spline_position > 0; current_spline_position -= curr_v * dt_)
	{
		ROS_INFO_STREAM_FILTER(&message_filter_, "------------------------");

		velocities.push_back(curr_v); //For limiting the velocity on the back pass
		positions.push_back(current_spline_position);

		max_angular_velocities.push_back(current_angular_velocity);

		point_count++;

		//ROS_INFO_STREAM_FILTER(&message_filter_, "curr_v: " << curr_v);

		//Compute all the path info
		path_point current_spline_point;
		comp_point_characteristics(x_splines, y_splines, x_splines_first_deriv, y_splines_first_deriv,
								   x_splines_second_deriv, y_splines_second_deriv, orient_splines, orient_splines_first_deriv,
								   orient_splines_second_deriv, current_spline_point, end_points, dtds_for_spline, arc_length_for_spline,
								   spline(current_spline_position), current_spline_position);

		//ROS_INFO_STREAM_FILTER(&message_filter_, "in back pass after comp_point_characteristics orientation_velocities = " << current_spline_point.angular_velocity);

		const double prev_v = curr_v;
		// Solve for the next V using constraints
		path_point this_spline_point = current_spline_point;
		this_spline_point.angular_velocity = fabs(current_spline_point.orientation - prev_angular_position) / dt_ * max_wheel_dist_;
		this_spline_point.angular_accel = fabs(current_angular_velocity - prev_angular_velocity) / dt_ * max_wheel_dist_ * ang_accel_conv_;
		ROS_INFO_STREAM_FILTER(&message_filter_, "current_angular_velocity:" << current_angular_velocity
				<< " prev_angular_velocity: " << prev_angular_velocity
				<< " this_spline_point.angular_accel " << this_spline_point.angular_accel
				<< " current_angular_position: " << current_spline_point.orientation
				<< " prev_angular_position: " << prev_angular_position
				<< " this_spline_point.angular_velocity: " << this_spline_point.angular_velocity);
		prev_angular_position = current_spline_point.orientation;
		prev_angular_velocity = current_angular_velocity;

		if (!solve_for_next_V(current_spline_point, total_arc, curr_v, current_spline_position, max_wheel_brake_accel_, accelerations))
		{
			return false;
		}

		/** HACK STARTS HERE**/
		//arc length -> next arc length -> next time_o -> next angular velocity

		const double acceleration_fudge = .01; // TODO : perhaps redo as a percentage?

		// calculate the requested acceleration from the current spline length and the next spline length
		const double next_spline_position = std::max(0.0, current_spline_position - curr_v * dt_);

		// Calulate angular velocity using change in orientation
		double next_orientation;
		double next_angular_velocity;
		double requested_acceleration;

		calc_angular_terms(spline(next_spline_position), orient_splines, end_points,
				current_spline_point.orientation, current_angular_velocity, true,
				next_orientation, next_angular_velocity, requested_acceleration);

		ROS_INFO_STREAM_FILTER(&message_filter_, "current_spline_time = " << spline(current_spline_position) << " next_spline_time = " << spline(next_spline_position));
		ROS_INFO_STREAM_FILTER(&message_filter_, "current_spline_position = " << current_spline_position << " next_spline_position = " << next_spline_position);
		ROS_INFO_STREAM_FILTER(&message_filter_, "current_spline_velocity = " << curr_v << " prev_spline_velocity = " << prev_v);
		ROS_INFO_STREAM_FILTER(&message_filter_, "current_orientation = "  << current_spline_point.orientation << " next_orientation = " << next_orientation);
		ROS_INFO_STREAM_FILTER(&message_filter_, "current_angular_velocity = " << current_angular_velocity << " next_angular_velocity = " << next_angular_velocity);
		if(requested_acceleration > 1e-5) //this will be handed by the forward pass
		{
			ROS_WARN_STREAM_FILTER(&message_filter_, "skipping for forward pass");
			current_angular_velocity = next_angular_velocity;
			continue;
		}
		// Subtract out linear acceleration, leaving max accel left for
		// rotation. This is a rough model that excludes e.g. acceleration
		// along the path radius, but that could be added later
		double linear_accel = (curr_v - prev_v) / dt_;
		double max_acceleration = -max_wheel_brake_accel_ + fabs(linear_accel);
		ROS_INFO_STREAM_FILTER(&message_filter_, "requested acceleration = " << requested_acceleration << " max_acceleration = " << max_acceleration << " linear_accel = " << linear_accel);

		//binary search until spline length corresponds to maximum acceleration
		if(fabs(requested_acceleration) > fabs(max_acceleration))
		{
			ROS_WARN_STREAM_FILTER(&message_filter_, "=============== requested angular acceleration is greater than maximum");
			double min_spline_position = current_spline_position;
			double max_spline_position = next_spline_position;
			double midpoint_spline_position = 0;
			double midpoint_angular_velocity = 0;
			double midpoint_orientation = 0;
			//binary search
			while(true)
			{
				search_iterations += 1;
				//what angular acceleration do these two spline lengths require?
				midpoint_spline_position = (max_spline_position + min_spline_position) / 2;

				calc_angular_terms(spline(midpoint_spline_position), orient_splines, end_points,
						current_spline_point.orientation, current_angular_velocity, true,
						midpoint_orientation, midpoint_angular_velocity, requested_acceleration);

				ROS_INFO_STREAM_FILTER(&message_filter_, "min mid max:" << min_spline_position<< " " << midpoint_spline_position << " " << max_spline_position);

				// Calculate the updated path-relative linear V needed to get to the
				// new midpoint spline position.  Use that to update the accel available
				// for rotating the robot
				const double midpoint_spline_velocity = fabs(midpoint_spline_position - current_spline_position) / dt_;
				const double midpoint_spline_acceleration = (midpoint_spline_velocity - prev_v) / dt_;
				max_acceleration = -max_wheel_brake_accel_ + fabs(midpoint_spline_acceleration);

				ROS_INFO_STREAM_FILTER(&message_filter_, "current_spline_time = " << spline(current_spline_position) << " midpoint_spline_time = " << spline(midpoint_spline_position));
				ROS_INFO_STREAM_FILTER(&message_filter_, "current_orientation = "  << current_spline_point.orientation << " midpoint_orientation = " << midpoint_orientation);
				ROS_INFO_STREAM_FILTER(&message_filter_, "midpoint_spline_position = " << midpoint_spline_position << " next_spline_position = " << next_spline_position);
				ROS_INFO_STREAM_FILTER(&message_filter_, "ORIENTATION current_angular_velocity = " << current_angular_velocity << " midpoint_angular_velocity = " << midpoint_angular_velocity);
				ROS_INFO_STREAM_FILTER(&message_filter_, "ORIENTATION current_linear_velocity = " << prev_v << " midpoint_linear_velocity = " << midpoint_spline_velocity);
				ROS_INFO_STREAM_FILTER(&message_filter_, "requested acceleration = " << requested_acceleration << " max_acceleration = " << max_acceleration << " linear_acceleration = " << midpoint_spline_acceleration);

				//compare to the actual possible angular acceleration
				// There's some weirdness if acceleration is too negative - it tries to add positive rotation
				// to balance things out. That fails on paths with low rotation and pushes the search
				// in the wrong direction
				if(midpoint_spline_acceleration < -max_wheel_mid_accel_)
				{
					ROS_INFO_STREAM_FILTER(&message_filter_, "requested acceleration of " << requested_acceleration << " too large");
					min_spline_position = midpoint_spline_position;
				}
				else if(requested_acceleration > max_acceleration + acceleration_fudge)
				{
					ROS_INFO_STREAM_FILTER(&message_filter_, "requested acceleration of " << requested_acceleration << " too large");
					min_spline_position = midpoint_spline_position;
				}
				else if(requested_acceleration < max_acceleration)
				{
					ROS_INFO_STREAM_FILTER(&message_filter_, "requested acceleration of " << requested_acceleration << " too small");
					max_spline_position = midpoint_spline_position;
				}
				else
				{
					ROS_INFO_STREAM_FILTER(&message_filter_, "requested acceleration of " << requested_acceleration << " just right");
					break;
				}
			}
			//calculate the updated curr_v from the spline length at max angular acceleration
			curr_v = fabs(midpoint_spline_position - current_spline_position) / dt_;

			// Update the next current_angular_velocity to one which follows constraints
			current_angular_velocity = midpoint_angular_velocity;

			ROS_INFO_STREAM_FILTER(&message_filter_, "binary search curr_v: " << curr_v << " curr_ang_v: " << current_angular_velocity << " curr_ang_p:" << current_spline_point.orientation);
			accelerations.clear();
		}
		else
		{
			current_angular_velocity = next_angular_velocity;
			ROS_WARN_STREAM_FILTER(&message_filter_, "EVERYTHING IS FINE");
		}
		//ROS_INFO_STREAM_FILTER(&message_filter_, "in back pass after solve_for_next_V orientation_velocities = " << current_spline_point.angular_velocity);
		ROS_WARN_STREAM_FILTER(&message_filter_, "current - last angular velocity = " << current_angular_velocity - next_angular_velocity);
	}
	for(size_t i = 0; i < max_angular_velocities.size(); i++)
	{
		ROS_INFO_STREAM_FILTER(&message_filter_, max_angular_velocities[i]);
	}
	//ROS_WARN("called3");
	//ROS_INFO_STREAM_FILTER(&message_filter_, "passed loop 1");
	velocities.erase(velocities.end() - 1); //End must be erased
	positions.erase(positions.end() - 1);
	max_angular_velocities.erase(max_angular_velocities.end() - 1);

	// TODO : check this - if point_count is initialized to
	// non-0, it could run out of space below?
	point_count = out_msg.points.size();
	if (out_msg.points.size() == 0)
		out_msg.points.resize(155 / dt_); //For full auto :)  TODO: optimize
	curr_v = initial_v;
	size_t starting_point = positions.size() ? positions.size() - 1 : 0;
	ros::Duration now(0);
	ros::Duration period(dt_);
	//Same as back pass, but now forward

	// Reset variables for the forward pass
	int vel_index = 0;
	current_angular_velocity = 0;
	accelerations.clear();
	for (double current_spline_position = curr_v * dt_; current_spline_position < total_arc; current_spline_position += curr_v * dt_)
	{
		ROS_INFO_STREAM_FILTER(&message_filter_, "------------------------");

		const double current_spline_t = spline(current_spline_position);
		ROS_INFO_STREAM_FILTER(&message_filter_, "i val: " << current_spline_position << " t val: " << current_spline_t << " curr v: " << curr_v);

		path_point current_spline_point;
		comp_point_characteristics(x_splines, y_splines, x_splines_first_deriv, y_splines_first_deriv,
								   x_splines_second_deriv, y_splines_second_deriv, orient_splines, orient_splines_first_deriv,
								   orient_splines_second_deriv, current_spline_point, end_points, dtds_for_spline, arc_length_for_spline,
								   current_spline_t, current_spline_position);
		//ROS_INFO_STREAM_FILTER(&message_filter_, "current_spline_point.pos_x = " << current_spline_point.pos_x << " current_spline_point.pos_y = " << current_spline_point.pos_y);

		double prev_v = curr_v;

		//save output values
		ROS_WARN_STREAM_FILTER(&message_filter_, "current spline point at x = " << current_spline_point.pos_x);
		out_msg.points[point_count].positions.push_back(current_spline_point.pos_x);
		out_msg.points[point_count].positions.push_back(current_spline_point.pos_y);
		out_msg.points[point_count].positions.push_back(current_spline_point.orientation);
		out_msg.points[point_count].velocities.push_back(cos(current_spline_point.path_angle) * curr_v );
		out_msg.points[point_count].velocities.push_back(sin(current_spline_point.path_angle) * curr_v );
		out_msg.points[point_count].velocities.push_back(current_angular_velocity);
		out_msg.points[point_count].time_from_start = now;
		//ROS_INFO_STREAM_FILTER(&message_filter_, now);
		now += period;
		point_count++;
		//ROS_ERROR_STREAM_FILTER(&message_filter_, "1: " << curr_v);
		if (!solve_for_next_V(current_spline_point, total_arc, curr_v, current_spline_position, max_wheel_mid_accel_, accelerations))
		{
			return false;
		}
		//ROS_ERROR_STREAM_FILTER(&message_filter_, "2: " << curr_v);
		for (size_t k = 0; k < positions.size(); k++)
		{
			if (starting_point > k || positions[starting_point - k] > current_spline_position)
			{
				starting_point -= k;
				break;
			}
			//Find point
		}
		//Make sure starting point doesn't go less than 1
		// TODO : why? shouldn't it be less than zero?
		starting_point = std::max(static_cast<size_t>(1), starting_point);
		//coerce(starting_point, 1, 1000000000000);

		//Linear interpolation to get vel cap
		ROS_INFO_STREAM_FILTER(&message_filter_, "starting_point = " << starting_point);
		const double v_sp1 = velocities[starting_point + 1];
		const double v_s   = velocities[starting_point];
		const double p_sp1 = positions[starting_point + 1];
		const double p_s   = positions[starting_point];
		const double vel_cap = current_spline_position * (v_s - v_sp1) / (p_s - p_sp1) -
							   p_s * (v_s - v_sp1) / (p_s - p_sp1) + v_s;
		ROS_INFO_STREAM_FILTER(&message_filter_, "vel cap " << vel_cap);
		//if(i<p_sp1)
		//	continue;
		const double ang_vel_sp1 = max_angular_velocities[starting_point + 1];
		const double ang_vel_s = max_angular_velocities[starting_point];
		// TODO : should we use current_spline_position here?
		// Or is it just to get a ratio of distance along t?
		const double ang_cap = ang_vel_sp1 + (current_spline_position - p_sp1) * (ang_vel_s - ang_vel_sp1) / (p_s - p_sp1);
		ROS_INFO_STREAM_FILTER(&message_filter_, "ang_vel_sp1 = " << ang_vel_sp1 << " ang_vel_s" << ang_vel_s << " p_sp1 " << p_sp1 << " p_s " << p_s << " current_spline_position = " << current_spline_position);
		ROS_INFO_STREAM_FILTER(&message_filter_, "angular velocity cap = " << ang_cap);
		//Keep below back pass
		//ROS_INFO_STREAM_FILTER(&message_filter_, "pre cut max: " << curr_v);
		//if(curr_v > vel_cap)
		//{
		//    ROS_INFO_STREAM_FILTER(&message_filter_, "cut by previous vel max: " << vel_cap << " curr_v: " << curr_v);
		//}
		if (!coerce(curr_v, -100000000000, vel_cap))
		{
			if (!solve_for_next_V(current_spline_point, total_arc, curr_v, current_spline_position, max_wheel_mid_accel_, accelerations))
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

		/* HACK AGAIN STARTS HERE */
		const double acceleration_fudge = .01; // TODO : perhaps redo as a percentage?

		// calculate the requested acceleration from the current spline length and the next spline length
		const double next_spline_position = std::min(current_spline_position + curr_v * dt_, total_arc);

		// Calculate angular velocity using change in orientation
		double next_orientation;
		double next_angular_velocity;
		double requested_acceleration;
		calc_angular_terms(spline(next_spline_position), orient_splines, end_points,
				current_spline_point.orientation, current_angular_velocity, false,
				next_orientation, next_angular_velocity, requested_acceleration);

		ROS_INFO_STREAM_FILTER(&message_filter_, "current_spline_time = " << spline(current_spline_position) << " next_spline_time = " << spline(next_spline_position));
		ROS_INFO_STREAM_FILTER(&message_filter_, "current_spline_position = " << current_spline_position << " next_spline_position = " << next_spline_position);
		ROS_INFO_STREAM_FILTER(&message_filter_, " curr_v = " << curr_v);

		ROS_INFO_STREAM_FILTER(&message_filter_, "current_orientation = "  << current_spline_point.orientation << " next_orientation = " << next_orientation);
		ROS_INFO_STREAM_FILTER(&message_filter_, "ORIENTATION current_angular_velocity = " << current_angular_velocity << " next_angular_velocity = " << next_angular_velocity);

		// Subtract out linear acceleration, leaving max accel left for
		// rotation. This is a rough model that excludes e.g. acceleration
		// along the path radius, but that could be added later
		double linear_accel = (curr_v - prev_v) / dt_;
		double max_acceleration = max_wheel_mid_accel_ - fabs(linear_accel);
		ROS_INFO_STREAM_FILTER(&message_filter_, "requested acceleration = " << requested_acceleration << " max_acceleration = " << max_acceleration << " linear_accel = " << linear_accel);
		if(requested_acceleration < 0)
		{
			ROS_WARN_STREAM_FILTER(&message_filter_, "skipping for back pass");
			current_angular_velocity = ang_cap;
			continue;
		}

		//binary search until spline length corresponds to maximum acceleration
		if(fabs(requested_acceleration) > fabs(max_acceleration))
		{
			ROS_WARN_STREAM_FILTER(&message_filter_, "requested angular acceleration is greater than maximum");
			double min_spline_position = current_spline_position;
			double max_spline_position = next_spline_position;
			double midpoint_spline_position = 0;
			double midpoint_angular_velocity = 0;
			double midpoint_orientation = 0;
			//binary search
			while(true)
			{
				search_iterations += 1;
				//what angular acceleration do these two spline lengths require?
				midpoint_spline_position = (max_spline_position + min_spline_position)/2;

				calc_angular_terms(spline(midpoint_spline_position), orient_splines, end_points,
						current_spline_point.orientation, current_angular_velocity, false,
						midpoint_orientation, midpoint_angular_velocity, requested_acceleration);

				ROS_INFO_STREAM_FILTER(&message_filter_, "min mid max:" << min_spline_position<< " " << midpoint_spline_position << " " << max_spline_position);

				// Calculate the updated path-relative linear V needed to get to the
				// new midpoint spline position.  Use that to update the accel available
				// for rotating the robot
				const double midpoint_spline_velocity = (midpoint_spline_position - current_spline_position) / dt_;
				const double midpoint_spline_acceleration = (midpoint_spline_velocity - prev_v) / dt_;
				max_acceleration = max_wheel_mid_accel_ - fabs(midpoint_spline_acceleration);

				ROS_INFO_STREAM_FILTER(&message_filter_, "current_orientation = "  << current_spline_point.orientation << " midpoint_orientation = " << midpoint_orientation);
				ROS_INFO_STREAM_FILTER(&message_filter_, "diff vel = " << midpoint_angular_velocity - current_angular_velocity << " dt = " << curr_v / (midpoint_spline_position - current_spline_position));
				ROS_INFO_STREAM_FILTER(&message_filter_, "ORIENTATION current_angular_velocity = " << current_angular_velocity << " midpoint_angular_velocity = " << midpoint_angular_velocity);
				ROS_INFO_STREAM_FILTER(&message_filter_, "requested acceleration = " << requested_acceleration << " max_acceleration = " << max_acceleration << " linear_acceleration = " << midpoint_spline_acceleration);

				//compare to the actual possible angular acceleration
				if(midpoint_spline_acceleration < -max_wheel_mid_accel_)
				{
					ROS_INFO_STREAM_FILTER(&message_filter_, "requested acceleration of " << requested_acceleration << " too small");
					min_spline_position = midpoint_spline_position;
				}
				else if(requested_acceleration > max_acceleration)
				{
					ROS_INFO_STREAM_FILTER(&message_filter_, "requested acceleration of " << requested_acceleration << " too large");
					max_spline_position = midpoint_spline_position;
				}
				else if(requested_acceleration < max_acceleration - acceleration_fudge)
				{
					ROS_INFO_STREAM_FILTER(&message_filter_, "requested acceleration of " << requested_acceleration << " too small");
					min_spline_position = midpoint_spline_position;
				}
				else
				{
					ROS_INFO_STREAM_FILTER(&message_filter_, "requested acceleration of " << requested_acceleration << " just right");
					break;
				}
			}
			//calculate the correct curr_v from the spline length at max angular acceleration
			curr_v = fabs(midpoint_spline_position - current_spline_position) / dt_;

			// Update the next current_angular_velocity to one which follows constraints
			current_angular_velocity = std::min(midpoint_angular_velocity, ang_cap);
			ROS_INFO_STREAM_FILTER(&message_filter_, "max ang vel[" << vel_index <<" - " << point_count << "] = " << max_angular_velocities[vel_index - point_count]);

			ROS_INFO_STREAM_FILTER(&message_filter_, "binary search curr_v: " << curr_v << " curr_ang_v: " << current_angular_velocity);
			accelerations.clear();
		}
		else
		{
			current_angular_velocity = next_angular_velocity;
			ROS_WARN_STREAM_FILTER(&message_filter_, "EVERYTHING IS FINE");
		}
		//ROS_INFO_STREAM_FILTER(&message_filter_, "post cut max: " << curr_v);
		vel_index++;
	}
	//ROS_WARN("called3");
	//ROS_ERROR("finished raw generation");
	ROS_INFO_STREAM("time: " << point_count * dt_);
	ROS_INFO_STREAM("total_arc: " << total_arc);
	out_msg.points.erase(out_msg.points.begin() + point_count, out_msg.points.end());
	ROS_WARN_STREAM("p: " << out_msg.points.size());
	ROS_INFO_STREAM("point_count in profiler: " << point_count);
	for(size_t i = 0; i < out_msg.points.size(); i++)
	{
		ROS_INFO_STREAM("orientation velocities in profiler= " << out_msg.points[i].velocities[2]);
	}
	ROS_INFO_STREAM("Search iterations = " << search_iterations);
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

		ROS_INFO_STREAM_FILTER(&message_filter_, "accel_defined:" << accel_defined);

		//Maximum V based on maximum at wheel V
		const double path_angular_velocity_squared = path.angular_velocity * path.angular_velocity;
		const double v_general_max = max_wheel_vel_ * sqrt(1.0 /
				(1.0 + sqrt(2) * fabs(path.angular_velocity) * (cos_t + sin_t) + path_angular_velocity_squared));

		//if(current_v > v_general_max)
		//{
		//	ROS_INFO_STREAM_FILTER(&message_filter_, "cut by general max:" << v_general_max);
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

		ROS_INFO_STREAM_FILTER(&message_filter_, "solve_for_next_V :theta:" << theta << " cos_t:" << cos_t << " sin_t:" << sin_t << " path_radius_squared:" << path_radius_squared << " path_angular_velocity_squared:" << path_angular_velocity_squared << " path_angular_accel_squared:" << path_angular_accel_squared << " path_a_over_r:" << path_a_over_r);
		const double v_curve_max_2 = sqrt(accel_defined /
										  sqrt(1.0 / path_radius_squared + path_angular_accel_squared + sqrt(2) * path_a_over_r * (cos_t + sin_t)));

		//if(current_v > v_curve_max)
		//{
		//	ROS_INFO_STREAM_FILTER(&message_filter_, "cut by curve max:" << v_curve_max << " radius:" << path.radius << " eff_max_a:" << eff_max_a);
		//
		//}

		ROS_INFO_STREAM_FILTER(&message_filter_, "solve_for_next_V, current_v:" << current_v << " v_general_max:" << v_general_max << " v_curve_max:" << v_curve_max << " v_curve_max_2:" << v_curve_max_2);

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

			ROS_INFO_STREAM_FILTER(&message_filter_, "accel1:" << accel1 << " accel2:" << accel2 << " accel_final:" << accel_final);

			//Implementation of adams-bashforth:
			const size_t s = accelerations.size();
			ROS_INFO_STREAM_FILTER(&message_filter_, "accelerations.size = " << accelerations.size());
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
			ROS_INFO_STREAM_FILTER(&message_filter_, "solve_for_next_V, after adding accel current_v:" << current_v << " v_general_max:" << v_general_max << " v_curve_max:" << v_curve_max << " v_curve_max_2:" << v_curve_max_2);
			const bool b11 = coerce(current_v, -v_curve_max, v_curve_max);
			const bool b12 = coerce(current_v, -v_curve_max_2, v_curve_max_2);
			const bool b13 = coerce(current_v, -v_general_max, v_general_max);
			if (b11 || b12 || b13)
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
		ROS_INFO_STREAM_FILTER(&message_filter_, "off path +" << current_v << " accel_defined: " << accel_defined);
		coerce(current_v, -max_wheel_vel_, max_wheel_vel_);
		ROS_INFO_STREAM_FILTER(&message_filter_, "off path coerce " << current_v);
	}
	return true;
	//ROS_INFO_STREAM(__LINE__ << ": " << path.angular_velocity);
}

constexpr size_t spline_points = 1000;
tk::spline swerve_profiler::parametrize_spline(const std::vector<spline_coefs> &x_splines_first_deriv,
		const std::vector<spline_coefs> &y_splines_first_deriv,
		const std::vector<double> &end_points, double &total_arc_length,
		std::vector<double> &dtds_by_spline,
		std::vector<double> &arc_length_by_spline)
{
	total_arc_length = 0;
	double period_t = (end_points[0] - 0.0) / spline_points;
	double start = 0.0;
	double arc_before = 0.0;
	double b_val = 0.0;
	std::vector<double> t_vals;
	std::vector<double> s_vals;
	t_vals.reserve(x_splines_first_deriv.size() * (spline_points + 1));
	s_vals.reserve(x_splines_first_deriv.size() * (spline_points + 1));
	//ROS_INFO_STREAM("Running parametrize");

	//ROS_WARN_STREAM(x_splines_first_deriv.size());

	ROS_INFO_STREAM_FILTER(&message_filter_, "THE SIZE HERE IS: " << x_splines_first_deriv.size());
	for (size_t i = 0; i < x_splines_first_deriv.size(); i++)
	{
		ROS_INFO_STREAM_FILTER(&message_filter_, "endpoints: " << end_points[i]);

		if (i != 0)
		{
			period_t = (end_points[i] - end_points[i - 1]) / static_cast<double>(spline_points);
			start = end_points[i - 1];
		}
		if (i > 1)
		{
			dtds_by_spline.push_back((end_points[i - 1] - end_points[i - 2]) /
									 (total_arc_length - arc_before));
			ROS_INFO_STREAM_FILTER(&message_filter_, "dtds by spline:" << dtds_by_spline[i]);
		}
		else if (i == 1)
		{
			dtds_by_spline.push_back((end_points[0] - 0) / (total_arc_length - arc_before));
			ROS_INFO_STREAM_FILTER(&message_filter_, "dtds by spline:" << dtds_by_spline[i]);
		}
		arc_before = total_arc_length;
		ROS_INFO_STREAM_FILTER(&message_filter_, "arc_before: " << arc_before);
		for (size_t k = 0; k < spline_points; k++)
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

			//Simpsons rule
			//f(t) = sqrt((dx/dt)^2 + (dy/dt)^2)

			//ROS_INFO_STREAM_FILTER(&message_filter_, "period_t: " << period_t);
			//ROS_INFO_STREAM_FILTER(&message_filter_, "idek: " << hypot(x_at_a, y_at_a) + 4 * hypot(x_at_avg, y_at_avg) + hypot(x_at_b, y_at_b));
			total_arc_length += period_t / 6. * (hypot(x_at_a, y_at_a) + 4. * hypot(x_at_avg, y_at_avg) + hypot(x_at_b, y_at_b));
			//ROS_INFO_STREAM_FILTER(&message_filter_, "arc_now: " << total_arc_length);
			//ROS_INFO_STREAM_FILTER(&message_filter_, "Spline: " << i << " a_val: " << a_val <<"  arc_length: " << total_arc_length);
		}
		arc_length_by_spline.push_back(total_arc_length);
	}
	if (x_splines_first_deriv.size() == 1)
	{
		dtds_by_spline.push_back((end_points[x_splines_first_deriv.size() - 1] - 0)
								 /  (total_arc_length - arc_before));
		ROS_INFO_STREAM_FILTER(&message_filter_, "dtds by spline:" << dtds_by_spline.back());
	}
	else
	{
		dtds_by_spline.push_back((end_points[x_splines_first_deriv.size() - 1]
								  - end_points[x_splines_first_deriv.size() - 2]) /  (total_arc_length - arc_before));
		ROS_INFO_STREAM_FILTER(&message_filter_, "dtds by spline:" << dtds_by_spline.back());
	}

	//Put in the last values
	t_vals.push_back(b_val);
	s_vals.push_back(total_arc_length);

	//Spline fit of t in terms of s (we input a t -> s)
	tk::spline s;

	s.set_points(s_vals, t_vals);
	for (size_t i = 0; i < t_vals.size(); i++)
	{
		ROS_INFO_STREAM_FILTER(&message_filter_, "t_vals = " << t_vals[i] << " s vals = " << s_vals[i]);
		//ROS_INFO_STREAM_FILTER(&message_filter_, "s_vals = " << s_vals[i] << " s vals = " << s(s_vals[i]));
	}
	ROS_INFO_STREAM("successful parametrize spline");
	return s;
}

// Solves for x in equations of the form 0 = a * x^2 + b * x + c
// Returns true and the positive root of x if a real solution exists
// Otherwise, returns false
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
	//ROS_INFO_STREAM("calc_point spline:" << spline << " t:" << t << " t_squared:" << t_squared << " t_cubed:" << t_cubed << " t_fourth:" << t_fourth << " t_fifth:" << t_fifth << " f(t):" << returner);
}

void swerve_profiler::comp_point_characteristics(const std::vector<spline_coefs> &x_splines,
		const std::vector<spline_coefs> &y_splines, const std::vector<spline_coefs> &x_splines_first_deriv,
		const std::vector<spline_coefs> &y_splines_first_deriv, const std::vector<spline_coefs> &x_splines_second_deriv,
		const std::vector<spline_coefs> &y_splines_second_deriv, const std::vector<spline_coefs> &orient_splines,
		const std::vector<spline_coefs> &/*orient_splines_first_deriv*/,
		const std::vector<spline_coefs> &/*orient_splines_second_deriv*/, path_point &holder_point,
		const std::vector<double> &end_points, const std::vector<double> &/*dtds_by_spline*/,
		const std::vector<double> &/*arc_length_by_spline*/, const double t, const double /*arc_length*/)
{
	size_t which_spline;

	//Find the spline based on t
	for (which_spline = 0; which_spline < x_splines.size() - 1; which_spline++)
	{
		if (t < end_points[which_spline])
		{
			break;
		}
	}

#if 0
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
#endif

#if 0
	double first_deriv_orient;
	double second_deriv_orient;
#endif
	double first_deriv_x;
	double first_deriv_y;
	double second_deriv_x;
	double second_deriv_y;

	//Calculate all the points
	calc_point(x_splines[which_spline], t, holder_point.pos_x);
	calc_point(y_splines[which_spline], t, holder_point.pos_y);
	calc_point(orient_splines[which_spline], t, holder_point.orientation); // TODO

	calc_point(x_splines_first_deriv[which_spline], t, first_deriv_x);
	calc_point(y_splines_first_deriv[which_spline], t, first_deriv_y);
	calc_point(x_splines_second_deriv[which_spline], t, second_deriv_x);
	calc_point(y_splines_second_deriv[which_spline], t, second_deriv_y);
#if 0
	calc_point(orient_splines_first_deriv[which_spline], t, first_deriv_orient);
	calc_point(orient_splines_second_deriv[which_spline], t, second_deriv_orient);
#endif

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
		ROS_ERROR_STREAM("resonableness exceeded with x of: " << holder_point.pos_x << " and y of: " << holder_point.pos_y << " t: " << t);
	}

	holder_point.path_angle = atan2(first_deriv_y, first_deriv_x) ; //- (holder_point.orientation -  M_PI / 2.0);
#if 0
	holder_point.angular_velocity = first_deriv_orient /* * dtds_by_spline[which_spline] */ *   max_wheel_dist_ ; //returns things in rad/t_o
	//ROS_INFO_STREAM(__LINE__ << ": " << holder_point.angular_velocity);
	holder_point.angular_accel = fabs(second_deriv_orient * /*dtds_by_spline[which_spline] *
									  dtds_by_spline[which_spline] **/ max_wheel_dist_ * ang_accel_conv_);
#endif
}

void swerve_profiler::calc_angular_terms(const double arb_t, const std::vector<spline_coefs> &orient_splines,
										 const std::vector<double> &end_points,
										 double curr_pos, double curr_vel, bool back_pass,
										 double &next_pos, double &next_vel, double &next_acc)
{
		size_t which_spline;
		for (which_spline = 0; which_spline < end_points.size() - 1; which_spline++)
		{
			if (arb_t < end_points[which_spline])
			{
				break;
			}
		}

		const double sign = back_pass ? -1 : 1;
		calc_point(orient_splines[which_spline], arb_t, next_pos);
		next_vel = sign * (next_pos - curr_pos) / dt_;
		next_acc = sign * (next_vel - curr_vel) / dt_ * max_wheel_dist_;
}

}
