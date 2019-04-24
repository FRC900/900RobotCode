#include <ros/ros.h>
#include <swerve_point_generator/FullGenCoefs.h>
#include <talon_swerve_drive_controller/MotionProfile.h>
#include <base_trajectory/GenerateSpline.h>
#include <talon_state_controller/TalonState.h>
#include <robot_visualizer/ProfileFollower.h>
#include <behaviors/PathAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <angles/angles.h>

// TODO : all of these should be members of PathAction.  Move their
// initialization from main into the class constructor
ros::ServiceClient point_gen;
ros::ServiceClient swerve_controller;
ros::ServiceClient spline_gen;
ros::ServiceClient VisualizeService;
bool outOfPoints;

void talonStateCallback(const talon_state_controller::TalonState &talon_state);

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

    talon_swerve_drive_controller::MotionProfile swerve_control_srv;

    swerve_control_srv.request.joint_trajectory = traj.joint_trajectory;
    swerve_control_srv.request.hold = traj.hold;
    swerve_control_srv.request.dt = traj.dt;
    swerve_control_srv.request.slot = 0;

    swerve_control_srv.request.buffer = true;
    swerve_control_srv.request.run = true;
	swerve_control_srv.request.wipe_all = true;
	swerve_control_srv.request.brake = false;
	swerve_control_srv.request.run_slot = 0;

	/*bool wipe_all
	bool buffer
	bool run
	bool brake
	uint8 run_slot
	bool change_queue
	uint8[] new_queue*/


    if (!swerve_controller.call(swerve_control_srv))
        return false;
    else
        return true;
}

class PathAction
{
protected:
	actionlib::SimpleActionServer<behaviors::PathAction> as_;
	std::string action_name_;

public:
	PathAction(const std::string &name, ros::NodeHandle n_) :
		as_(n_, name, boost::bind(&PathAction::executeCB, this, _1), false),
		action_name_(name)
	{
		as_.start();
	}

	~PathAction(void)
	{
	}

	void executeCB(const behaviors::PathGoalConstPtr &goal) //make a state thing so that it just progresses to the next service call
	{
		bool success = true;

		base_trajectory::GenerateSpline srvBaseTrajectory;
		srvBaseTrajectory.request.points.resize(1);

		swerve_point_generator::FullGenCoefs traj;

		ros::Duration time_to_run = ros::Duration(goal->time_to_run); //TODO: make this an actual thing

		// TODO : why spin here? Is there callback data needed?
		ros::spinOnce();

		//x-movement
		srvBaseTrajectory.request.points[0].positions.push_back(goal->x);
		srvBaseTrajectory.request.points[0].velocities.push_back(0);
		srvBaseTrajectory.request.points[0].accelerations.push_back(0);
		//y-movement
		srvBaseTrajectory.request.points[0].positions.push_back(goal->y);
		srvBaseTrajectory.request.points[0].velocities.push_back(0);
		srvBaseTrajectory.request.points[0].accelerations.push_back(0);
		//z-rotation
		const double rotation = angles::normalize_angle(goal->rotation);
		if (std::abs(rotation) < 0.001)
			srvBaseTrajectory.request.points[0].positions.push_back(rotation < 0 ? -0.001 : 0.001);
		else
			srvBaseTrajectory.request.points[0].positions.push_back(rotation);
		srvBaseTrajectory.request.points[0].velocities.push_back(0); //velocity at the end point
		srvBaseTrajectory.request.points[0].accelerations.push_back(0); //acceleration at the end point
		//time for profile to run
		srvBaseTrajectory.request.points[0].time_from_start = time_to_run;

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

			// Straight assignment should work fine here - false||x == x
			timed_out = timed_out || (ros::Time::now().toSec() - startTime) > goal->time_to_run;
		}

		if (!aborted)
		{
			behaviors::PathResult result;
			result.success = success;
			result.timeout = timed_out;
			as_.setSucceeded(result);
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_server");
	ros::NodeHandle n;
	PathAction path("path_server", n);

	if (!ros::service::waitForService("point_gen/command", 15000))
	{
		ROS_ERROR("Failed waiting for point_gen/command service, exiting");
		return 0;
	}

	if (!ros::service::waitForService("/frcrobot_jetson/swerve_drive_controller/run_profile", 15000))
	{
		ROS_ERROR("Failed waiting for /frcrobot_jetson/swerve_drive_controller/run_profile service, exiting");
		return 0;
	}

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = 1;
	point_gen = n.serviceClient<swerve_point_generator::FullGenCoefs>("point_gen/command", false, service_connection_header);
	swerve_controller = n.serviceClient<talon_swerve_drive_controller::MotionProfile>("/frcrobot_jetson/swerve_drive_controller/run_profile", false, service_connection_header);
	spline_gen = n.serviceClient<base_trajectory::GenerateSpline>("base_trajectory/spline_gen", false, service_connection_header);
	VisualizeService = n.serviceClient<robot_visualizer::ProfileFollower>("visualize_auto", false, service_connection_header);
	auto talon_sub = n.subscribe("/frcrobot_jetson/talon_states", 10, talonStateCallback);

	ros::spin();

	return 0;
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

