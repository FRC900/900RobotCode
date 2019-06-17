#include <Eigen/Dense>
#include <vector>
#include <array>
#include <string>
#include <ros/ros.h>
#include <swerve_point_generator/profiler.h>
#include <swerve_point_generator/FullGenCoefs.h>
#include <swerve_point_generator/GenerateSwerveProfile.h>
#include <talon_swerve_drive_controller/MotionProfile.h> //Only needed for visualization

std::shared_ptr<swerve_profile::swerve_profiler> profile_gen;

ros::ServiceClient graph_prof;
ros::ServiceClient graph_swerve_prof;

bool full_gen(swerve_point_generator::FullGenCoefs::Request &req, swerve_point_generator::FullGenCoefs::Response &res)
{
	const int k_p = 1;
	talon_swerve_drive_controller::MotionProfile graph_msg;
	for (size_t s = 0; s < req.spline_groups.size(); s++)
	{
		int priv_num = 0;
		if (s > 0)
		{
			priv_num = req.spline_groups[s - 1];
		}
		std::vector<swerve_profile::spline_coefs> x_splines;
		std::vector<swerve_profile::spline_coefs> y_splines;
		std::vector<swerve_profile::spline_coefs> orient_splines;

		const int neg_x = req.x_invert[s] ? -1 : 1;
		std::vector<double> end_points_holder;
		double shift_by = 0;
		if (s != 0)
		{
			shift_by = req.end_points[priv_num - 1];
		}

		for (int i = priv_num; i < req.spline_groups[s]; i++)
		{
			orient_splines.push_back(swerve_profile::spline_coefs(
										 req.orient_coefs[i].spline[0] * neg_x,
										 req.orient_coefs[i].spline[1] * neg_x,
										 req.orient_coefs[i].spline[2] * neg_x,
										 req.orient_coefs[i].spline[3] * neg_x,
										 req.orient_coefs[i].spline[4] * neg_x,
										 req.orient_coefs[i].spline[5] * neg_x));
			ROS_INFO_STREAM("orient_coefs[" << i << "].spline=" << orient_splines.back());

			x_splines.push_back(swerve_profile::spline_coefs(
									req.x_coefs[i].spline[0] * neg_x,
									req.x_coefs[i].spline[1] * neg_x,
									req.x_coefs[i].spline[2] * neg_x,
									req.x_coefs[i].spline[3] * neg_x,
									req.x_coefs[i].spline[4] * neg_x,
									req.x_coefs[i].spline[5] * neg_x));
			ROS_INFO_STREAM("x_coefs[" << i << "].spline=" << x_splines.back());

			y_splines.push_back(swerve_profile::spline_coefs(
									req.y_coefs[i].spline[0],
									req.y_coefs[i].spline[1],
									req.y_coefs[i].spline[2],
									req.y_coefs[i].spline[3],
									req.y_coefs[i].spline[4],
									req.y_coefs[i].spline[5]));
			ROS_INFO_STREAM("y_coefs[" << i << "].spline=" << y_splines.back());

			ROS_INFO_STREAM("hrer: " << req.end_points[i] - shift_by << " r_s: " <<  req.spline_groups[s] <<  " s: " << s);
			end_points_holder.push_back(req.end_points[i] - shift_by);
		}

		const double t_shift = req.t_shift[s];
		const bool flip_dirc = req.flip[s];
		ROS_INFO_STREAM("req.initial_v: " << req.initial_v << " req.final_v: " << req.final_v << " t_shift: " << t_shift);

		// TODO - replace with just an array of JointTrajectoryPoints since
		// the rest of the message isn't used at all
		swerve_point_generator::GenerateSwerveProfile::Response srv_msg; //TODO FIX THIS, HACK
		profile_gen->generate_profile(x_splines, y_splines, orient_splines, req.initial_v, req.final_v, srv_msg, end_points_holder, t_shift, flip_dirc);
		const int point_count = srv_msg.points.size();

		graph_msg.request.joint_trajectory.header = srv_msg.header;

		res.dt = profile_gen->getDT();

		// TODO - just for debugging
		double total_length_x = 0;
		double total_length_y = 0;
		double total_length_theta = 0;
		for(size_t i = 0; i < srv_msg.points.size(); i++)
		{
			total_length_x += srv_msg.points[i].velocities[0] * profile_gen->getDT();
			total_length_y += srv_msg.points[i].velocities[1] * profile_gen->getDT();
			total_length_theta += srv_msg.points[i].velocities[2] * profile_gen->getDT();
		}
		ROS_ERROR_STREAM("total_length_x = " << total_length_x << " total_length_y = " <<  total_length_y << " total_length_theta = " << total_length_theta);
		for(size_t i = 0; i < srv_msg.points.size(); i++)
		{
			ROS_INFO_STREAM(srv_msg.points[i].positions[0] << " " << srv_msg.points[i].positions[1] << " " << srv_msg.points[i].positions[2]);
		}

		// Bounds checking - not safe to proceed with setting up angle
		// positions and velocities if data is not as expected.
		if (srv_msg.points.size() < 2)
		{
			ROS_ERROR("Need at least 2 points");
			return false;
		}
		for (const auto point : srv_msg.points)
		{
			if (point.positions.size() < 3)
			{
				ROS_ERROR("Not enough positions in point");
				return false;
			}
		}

		ROS_INFO_STREAM("velocity vector = " << srv_msg.points[1].positions[0] - srv_msg.points[0].positions[0] <<
				" " << srv_msg.points[1].positions[1] - srv_msg.points[0].positions[1] <<
				" rotation = " << srv_msg.points[1].positions[2] - srv_msg.points[0].positions[2] <<
				" angle = " << srv_msg.points[1].positions[2]);

		size_t hold_count = round(req.wait_before_group[s] / profile_gen->getDT());
		bool skip_one_traj = false;
		if ((hold_count > 0) && (res.hold.size() == 0))
		{
			graph_msg.request.joint_trajectory.points.push_back(srv_msg.points[0]);
			res.hold.push_back(true);
			skip_one_traj = true;
		}
		for (size_t i = 0; i < hold_count; i++)
		{
			if (!skip_one_traj)
			{
				graph_msg.request.joint_trajectory.points.push_back(srv_msg.points[1]);
				res.hold.push_back(true);
			}

			skip_one_traj = false;
		}

		graph_msg.request.joint_trajectory.points.insert(graph_msg.request.joint_trajectory.points.end(), srv_msg.points.begin(), srv_msg.points.end());

		for (int i = 0; i < point_count - k_p; i++)
		{
			res.hold.push_back(false);
		}
	}

	graph_prof.call(graph_msg);
	res.joint_trajectory = graph_msg.request.joint_trajectory;
	ROS_INFO_STREAM("profile time: " << res.joint_trajectory.points.size() * profile_gen->getDT());

	//talon_swerve_drive_controller::MotionProfilePoints graph_swerve_msg;
	//graph_swerve_msg.request.points = res.points;
	//graph_swerve_prof.call(graph_swerve_msg);
	//ROS_WARN("FIN");
	return true;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "point_gen");
	ros::NodeHandle nh;

	ros::NodeHandle controller_nh(nh, "/frcrobot_jetson/swerve_drive_controller");

	double max_accel;
	double max_brake_accel;
	double ang_accel_conv;
	double max_speed;

	if (!nh.getParam("max_accel", max_accel))
		ROS_ERROR("Could not read max_accel in point_gen");
	if (!nh.getParam("max_brake_accel", max_brake_accel))
		ROS_ERROR("Could not read max_brake_accel in point_gen");
	if (!nh.getParam("ang_accel_conv", ang_accel_conv))
		ROS_ERROR("Could not read ang_accel_conv in point_gen");
	if (!nh.getParam("max_speed", max_speed))
		ROS_ERROR("Could not read max_speed in point_gen");

	constexpr size_t WHEELCOUNT = 4;
	std::array<Eigen::Vector2d, WHEELCOUNT> wheel_coords;
	if (!controller_nh.getParam("wheel_coords1x", wheel_coords[0][0]))
		ROS_ERROR("Could not read wheel_coords1x in point_gen");
	if (!controller_nh.getParam("wheel_coords2x", wheel_coords[1][0]))
		ROS_ERROR("Could not read wheel_coords2x in point_gen");
	if (!controller_nh.getParam("wheel_coords3x", wheel_coords[2][0]))
		ROS_ERROR("Could not read wheel_coords3x in point_gen");
	if (!controller_nh.getParam("wheel_coords4x", wheel_coords[3][0]))
		ROS_ERROR("Could not read wheel_coords4x in point_gen");
	if (!controller_nh.getParam("wheel_coords1y", wheel_coords[0][1]))
		ROS_ERROR("Could not read wheel_coords1y in point_gen");
	if (!controller_nh.getParam("wheel_coords2y", wheel_coords[1][1]))
		ROS_ERROR("Could not read wheel_coords2y in point_gen");
	if (!controller_nh.getParam("wheel_coords3y", wheel_coords[2][1]))
		ROS_ERROR("Could not read wheel_coords3y in point_gen");
	if (!controller_nh.getParam("wheel_coords4y", wheel_coords[3][1]))
		ROS_ERROR("Could not read wheel_coords4y in point_gen");

	//ROS_WARN("point_init");
	//ROS_INFO_STREAM("model max speed: " << model.maxSpeed << " radius: " << model.wheelRadius);

	constexpr double defined_dt = .02;
	profile_gen = std::make_shared<swerve_profile::swerve_profiler>(
						hypot(wheel_coords[0][0], wheel_coords[0][1]),
						max_speed,
						max_accel,
						max_brake_accel,
						ang_accel_conv,
						defined_dt);

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";
	graph_prof = nh.serviceClient<talon_swerve_drive_controller::MotionProfile>("visualize_profile", false, service_connection_header);
	graph_swerve_prof = nh.serviceClient<talon_swerve_drive_controller::MotionProfile>("visualize_swerve_profile", false, service_connection_header);

	// Once everything this node needs is available, open
	// it up to connections from the outside
	ros::ServiceServer service = nh.advertiseService("point_gen/command", full_gen);

	ros::spin();
}
