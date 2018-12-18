#include "path_to_goal/path_to_goal.h"
float coerce(float x)
{
	return ((x>0.05) ? x : .05);
}

bool generateTrajectory(const base_trajectory::GenerateSpline &srvBaseTrajectory, swerve_point_generator::FullGenCoefs &traj)
{
	ROS_INFO_STREAM("started generateTrajectory");
	traj.request.orient_coefs.resize(1);
	traj.request.x_coefs.resize(1);
	traj.request.y_coefs.resize(1);

	for(size_t i = 0; i < srvBaseTrajectory.response.orient_coefs[0].spline.size(); i++)
	{
		traj.request.orient_coefs[0].spline.push_back(srvBaseTrajectory.response.orient_coefs[1].spline[i]);
		traj.request.x_coefs[0].spline.push_back(srvBaseTrajectory.response.x_coefs[1].spline[i]);
		traj.request.y_coefs[0].spline.push_back(srvBaseTrajectory.response.y_coefs[1].spline[i]);
	}

	traj.request.spline_groups.push_back(1);
	traj.request.wait_before_group.push_back(.16);
	traj.request.t_shift.push_back(0);
	traj.request.flip.push_back(false);
	traj.request.end_points.push_back(1);
	traj.request.end_points.resize(1);
	traj.request.end_points[0] = srvBaseTrajectory.response.end_points[1];
	traj.request.initial_v = 0;
	traj.request.final_v = 0;
	traj.request.x_invert.push_back(0);

	if(!point_gen.call(traj))
		return false;
	else
		return true;
}

bool runTrajectory(const swerve_point_generator::FullGenCoefs::Response &traj)
{
	ROS_INFO_STREAM("started runTrajectory");
	//visualization stuff
	robot_visualizer::ProfileFollower srv_viz_msg;
	srv_viz_msg.request.joint_trajectories.push_back(traj.joint_trajectory);

	srv_viz_msg.request.start_id = 0;

	if(!VisualizeService.call(srv_viz_msg))
	{
		ROS_ERROR("failed to call viz srv");
	}
	else
	{
		ROS_ERROR("succeded in call to viz srv");
	}

	talon_swerve_drive_controller::MotionProfilePoints swerve_control_srv;
	swerve_control_srv.request.profiles.resize(1);
    swerve_control_srv.request.profiles[0].points = traj.points;
    swerve_control_srv.request.profiles[0].dt = 0.02;
    swerve_control_srv.request.buffer = true;
    swerve_control_srv.request.run = true;
    swerve_control_srv.request.profiles[0].slot = 0;

    if (!swerve_controller.call(swerve_control_srv))
		return false;
	else
		return true;
}

class PathAction
{
protected:
	actionlib::SimpleActionServer<path_to_goal::PathAction> as_;
	std::string action_name_;

	path_to_goal::PathFeedback feedback_;
	path_to_goal::PathResult result_;

public:
	PathAction(std::string name, ros::NodeHandle n_) :
		as_(n_, name, boost::bind(&PathAction::executeCB, this, _1), false),
		action_name_(name)
	{
		as_.start();
	}

	~PathAction(void)
	{
	}

	void executeCB(const path_to_goal::PathGoalConstPtr &goal) //make a state thing so that it just progresses to the next service call
	{
		bool success = true;

		base_trajectory::GenerateSpline srvBaseTrajectory;
		srvBaseTrajectory.request.points.resize(1);

		swerve_point_generator::FullGenCoefs traj;

		ros::Duration time_to_run = ros::Duration(goal->time_to_run); //TODO: make this an actual thing

                ros::spinOnce();


		switch(goal->goal_index) {
		case 0 : //user input data
		{
			//x-movement
			srvBaseTrajectory.request.points[0].positions.push_back(goal->x);
			srvBaseTrajectory.request.points[0].velocities.push_back(0);
			srvBaseTrajectory.request.points[0].accelerations.push_back(0);
			//y-movement
			srvBaseTrajectory.request.points[0].positions.push_back(goal->y);
			srvBaseTrajectory.request.points[0].velocities.push_back(0);
			srvBaseTrajectory.request.points[0].accelerations.push_back(0);
			//z-rotation
                        if (goal->rotation < 0.001)
                            srvBaseTrajectory.request.points[0].positions.push_back(0.001);
                        else
                            srvBaseTrajectory.request.points[0].positions.push_back(goal->rotation);
			srvBaseTrajectory.request.points[0].velocities.push_back(0); //velocity at the end point
			srvBaseTrajectory.request.points[0].accelerations.push_back(0); //acceleration at the end point
			//time for profile to run
			srvBaseTrajectory.request.points[0].time_from_start = time_to_run;
		}
		case 1 : //cube data
		{
                        ROS_INFO_STREAM("in callback, cube_location.size = " << cube_location.location.size());
			if (cube_location.location.size() == 0)
			{
				ROS_ERROR_STREAM("NO CUBES FOUND - generateCoefs");
				success = false;
				break;
			}

			ROS_INFO_STREAM("x = " << cube_location.location[0].x);
			ROS_INFO_STREAM("z = " << cube_location.location[0].z);

			//x-movement
			srvBaseTrajectory.request.points[0].positions.push_back(cube_location.location[0].x); //TODO: are these in the right places?
			srvBaseTrajectory.request.points[0].velocities.push_back(0);
			srvBaseTrajectory.request.points[0].accelerations.push_back(0);
			//y-movement
			srvBaseTrajectory.request.points[0].positions.push_back(cube_location.location[0].y);
			srvBaseTrajectory.request.points[0].velocities.push_back(0);
			srvBaseTrajectory.request.points[0].accelerations.push_back(0);
			//z-rotation
			srvBaseTrajectory.request.points[0].positions.push_back(0.001);
			srvBaseTrajectory.request.points[0].velocities.push_back(0); //velocity at the end point
			srvBaseTrajectory.request.points[0].accelerations.push_back(0); //acceleration at the end point
			//time for profile to run
			srvBaseTrajectory.request.points[0].time_from_start = time_to_run;
		}
		case 2 : //exchange data
		{
			if (qr_location.location.size() == 0)
			{
				ROS_ERROR_STREAM("NO eXCHANGE FOUND - generateCoefs");
				success = false;
				break;
			}

			ROS_INFO_STREAM("x = " << qr_location.location[0].x);
			ROS_INFO_STREAM("z = " << qr_location.location[0].z);

			//x-movement
			srvBaseTrajectory.request.points[0].positions.push_back(qr_location.location[0].x); //TODO: are these in the right places?
			srvBaseTrajectory.request.points[0].velocities.push_back(0);
			srvBaseTrajectory.request.points[0].accelerations.push_back(0);
			//y-movement
			srvBaseTrajectory.request.points[0].positions.push_back(qr_location.location[0].y);
			srvBaseTrajectory.request.points[0].velocities.push_back(0);
			srvBaseTrajectory.request.points[0].accelerations.push_back(0);
			//z-rotation
			srvBaseTrajectory.request.points[0].positions.push_back(0.001);
			srvBaseTrajectory.request.points[0].velocities.push_back(0); //velocity at the end point
			srvBaseTrajectory.request.points[0].accelerations.push_back(0); //acceleration at the end point
			//time for profile to run
			srvBaseTrajectory.request.points[0].time_from_start = time_to_run;
		}
		case 3 : //lidar data (INCOMPLETE)
		{
			srvBaseTrajectory.request.points[0].positions.push_back(0);
			srvBaseTrajectory.request.points[0].velocities.push_back(0);
			srvBaseTrajectory.request.points[0].accelerations.push_back(0);
			srvBaseTrajectory.request.points[0].positions.push_back(0);
	                srvBaseTrajectory.request.points[0].velocities.push_back(0);
                        srvBaseTrajectory.request.points[0].accelerations.push_back(0);
			srvBaseTrajectory.request.points[0].positions.push_back(0.001);
			srvBaseTrajectory.request.points[0].velocities.push_back(0);
			srvBaseTrajectory.request.points[0].accelerations.push_back(0);
			srvBaseTrajectory.request.points[0].time_from_start = time_to_run;
		}
		default :
			ROS_ERROR_STREAM("goal index of " << goal->goal_index << " is not recognized");
		}

		bool running = false;
		if(!spline_gen.call(srvBaseTrajectory))
		{
			ROS_ERROR_STREAM("spline_gen died");
			success = false;
		}
		else if (!generateTrajectory(srvBaseTrajectory, traj))
		{
			ROS_ERROR_STREAM("generateTrajectory died");
			success = false;
		}
		else if (!runTrajectory(traj.response))
		{
			ROS_ERROR_STREAM("runTrajectory died");
			success = false;
		}

		ros::Rate r(10);
		const double startTime = ros::Time::now().toSec();
		bool aborted = false;
		bool timed_out = false;

		while (ros::ok() && !(aborted || success || timed_out))
		{
			if (as_.isPreemptRequested())
			{
				ROS_WARN("%s: Preempted", action_name_.c_str());
				as_.setPreempted();
				aborted = true;
				break;
			}
			r.sleep();
			ros::spinOnce();
			if (outOfPoints)
				success = true;
			timed_out = timed_out || (ros::Time::now().toSec() - startTime) > goal->time_to_run;
		}
		if (!aborted)
		{
			result_.success = success;
			result_.timeout = timed_out;
			as_.setSucceeded(result_);
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_server");
	ros::NodeHandle n;
	PathAction path("path_server", n);

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = 1;
	point_gen = n.serviceClient<swerve_point_generator::FullGenCoefs>("/point_gen/command", false, service_connection_header);
	swerve_controller = n.serviceClient<talon_swerve_drive_controller::MotionProfilePoints>("/frcrobot/swerve_drive_controller/run_profile", false, service_connection_header);
	spline_gen = n.serviceClient<base_trajectory::GenerateSpline>("/base_trajectory/spline_gen", false, service_connection_header);
	VisualizeService = n.serviceClient<robot_visualizer::ProfileFollower>("/frcrobot/visualize_auto", false, service_connection_header);
	cube_sub = n.subscribe("/frcrobot/cube_detection_node/cube_detect_msg", 10, &cubeCallback);
	talon_sub = n.subscribe("/frcrobot/talon_states", 10, talonStateCallback);

	ros::spin();

	return 0;
}

void cubeCallback(cube_detection::CubeDetection sub_location)
{
	cube_location.location.resize(sub_location.location.size());

	if (sub_location.location.size() > 0)
	{
		cube_location.location[0].x = sub_location.location[0].x;
		cube_location.location[0].y = sub_location.location[0].y;
		cube_location.location[0].z = sub_location.location[0].z;
		cube_location.angle = sub_location.angle;
	}
}

void talonStateCallback(const talon_state_controller::TalonState &talon_state)
{
	static size_t bl_drive_idx = std::numeric_limits<size_t>::max();

	if (bl_drive_idx >= talon_state.name.size())
	{
		for (size_t i = 0; i < talon_state.name.size(); i++)
		{
			if(talon_state.name[i] == "bl_drive")
			{
				bl_drive_idx = i;
				break;
			}
		}
	}

	if (bl_drive_idx < talon_state.custom_profile_status.size())
		outOfPoints = talon_state.custom_profile_status[bl_drive_idx].outOfPoints;
}

void QRCallback(cube_detection::CubeDetection sub_location)
{
	qr_location.location.resize(sub_location.location.size());

	if (sub_location.location.size() > 0)
	{
		qr_location.location[0].x = sub_location.location[0].x;
		qr_location.location[0].y = sub_location.location[0].y;
		qr_location.location[0].z = sub_location.location[0].z;
		qr_location.angle = sub_location.angle;
	}
	else
	{
		ROS_ERROR_STREAM("NO EXCHANGE FOUND");
	}
}
