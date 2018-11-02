#include <atomic>
#include <realtime_tools/realtime_buffer.h>
#include "teleop_joystick_control/teleop_joystick_offseason.h"
#include "elevator_controller/ElevatorControl.h"
#include "elevator_controller/ReturnElevatorCmd.h"
#include "elevator_controller/ElevatorControlS.h"
#include "elevator_controller/Intake.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "std_msgs/Bool.h"
#include "robot_visualizer/ProfileFollower.h"

static double dead_zone = .2, slow_mode = .33, max_speed = 3.6, max_rot = 8.8, joystick_scale = 3, rotation_scale = 4;
void dead_zone_check(double &val1, double &val2)
{
	if (fabs(val1) <= dead_zone && fabs(val2) <= dead_zone)
	{
		val1 = 0;
		val2 = 0;
	}
}

std::shared_ptr<actionlib::SimpleActionClient<behaviors::ArmAction>> ac;
std::shared_ptr<actionlib::SimpleActionClient<behaviors::IntakeAction>> ac_intake;
std::shared_ptr<actionlib::SimpleActionClient<behaviors::ForearmAction>> ac_arm;

static ros::Publisher JoystickRobotVel;
//static ros::Publisher JoystickRumble;
static ros::ServiceClient BrakeSrv;

std::atomic<double> navX_angle;
std::atomic<int> arm_position;

realtime_tools::RealtimeBuffer<double> most_recent_arm_command;

// Use a realtime buffer to store the odom callback data
// The main teleop code isn't technically realtime but we
// want it to be the fast part of the code, so for now
// pretend that is the realtime side of the code
/*realtime_tools::RealtimeBuffer<ElevatorPos> elevatorPos;
realtime_tools::RealtimeBuffer<CubeState> cubeState;
realtime_tools::RealtimeBuffer<ElevatorPos> elevatorCmd;*/
void navXCallback(const sensor_msgs::Imu &navXState)
{
    const tf2::Quaternion navQuat(navXState.orientation.x, navXState.orientation.y, navXState.orientation.z, navXState.orientation.w);
    double roll;
    double pitch;
    double yaw; 
    tf2::Matrix3x3(navQuat).getRPY(roll, pitch, yaw);

    if (yaw == yaw) // ignore NaN results
        navX_angle.store(yaw, std::memory_order_relaxed);
}


void evaluateCommands(const ros_control_boilerplate::JoystickState::ConstPtr &JoystickState)
{
    /* 
     * x = left and intake until line break is met
     * y = starting position
     * b = right and spit out cube
     * a = toggle arm position between right and left
     */
/*-------------------------- press x for intake until line break sensor ------------------------*/
    if (JoystickState->buttonXPress) {
        ac->cancelAllGoals();
        ac_intake->cancelAllGoals();
        ac_arm->cancelAllGoals();

        behaviors::ArmGoal arm_goal;
        arm_goal.arm_position = 0;
        arm_goal.intake_cube = true;
        arm_goal.intake_timeout = 10;
        ac->sendGoal(arm_goal);
    }

/*-------------------------- press y for starting position ------------------------*/
    if (JoystickState->buttonYPress) {
        ac->cancelAllGoals();
        ac_intake->cancelAllGoals();
        ac_arm->cancelAllGoals();

        behaviors::ForearmGoal forearm_goal;
        forearm_goal.position = 1;
        ac_arm->sendGoal(forearm_goal);
    }

/*-------------------------- press b to exchange cube ------------------------*/
    if (JoystickState->buttonBPress) {
        ac->cancelAllGoals();
        ac_intake->cancelAllGoals();
        ac_arm->cancelAllGoals();

        behaviors::ArmGoal arm_goal;
        arm_goal.arm_position = 2;
        arm_goal.intake_cube = false;
        arm_goal.intake_timeout = 10;
        ac->sendGoal(arm_goal);
    }

/*-------------------------- press a for toggle arm position ------------------------*/
    if (JoystickState->buttonAButton) {
        ac->cancelAllGoals();
        ac_intake->cancelAllGoals();
        ac_arm->cancelAllGoals();

        static int target_position;

        if(*(most_recent_arm_command.readFromRT()) != 2)
            target_position = 2;
        else
            target_position = 0;

        behaviors::ForearmGoal forearm_goal;
        forearm_goal.position = target_position;
        ac_arm->sendGoal(forearm_goal);
    }

/*-------------------------- press direction right press for toggle arm position ------------------------*/
    if (JoystickState->directionRightPress) {
        ac->cancelAllGoals();
        ac_intake->cancelAllGoals();
        ac_arm->cancelAllGoals();

        static int target_position;

        if(*(most_recent_arm_command.readFromRT()) != 2)
            target_position = 2;
        else
            target_position = 0;

        behaviors::ForearmGoal forearm_goal;
        forearm_goal.position = target_position;
        ac_arm->sendGoal(forearm_goal);
    }
///////////////////// Drivetrain and Elevator Control \\\\\\\\\\\\\\\\\\\

	double leftStickX = JoystickState->leftStickX;
	double leftStickY = JoystickState->leftStickY;

	double rightStickX = JoystickState->rightStickX;
	double rightStickY = JoystickState->rightStickY;

	dead_zone_check(leftStickX, leftStickY);
	dead_zone_check(rightStickX, rightStickY);

	leftStickX =  pow(leftStickX, joystick_scale) * max_speed;
	leftStickY = -pow(leftStickY, joystick_scale) * max_speed;

	rightStickX =  pow(rightStickX, joystick_scale);
	rightStickY = -pow(rightStickY, joystick_scale);

	double rotation = (pow(JoystickState->leftTrigger, rotation_scale) - pow(JoystickState->rightTrigger, rotation_scale)) * max_rot;

	static bool sendRobotZero = false;
	/******* Snap to angle *********/
	/*if(JoystickState->stickRightPress == true)
	{
		ROS_INFO_STREAM("outOfPoints = " << outOfPoints);
		static bool orient_running = false;
		if(!orient_running || outOfPoints.load(std::memory_order_relaxed))
		{
			orient_running = false;
			sendRobotZero = false;
			const double angle = -navX_angle.load(std::memory_order_relaxed) - M_PI / 2;
			//const double angle = M_PI; //for testing
			ROS_INFO_STREAM("angle = " << angle);
			// TODO: look at using ros::angles package
			//const double least_dist_angle = round(angle/(M_PI/2))*M_PI/2;
			const double least_dist_angle = angle + 2* M_PI;
			const double max_rotational_velocity = 8.8; //radians/sec TODO: find this in config

			ROS_INFO_STREAM("delta angle = " << least_dist_angle - angle);
			const ros::Duration time_to_run((fabs(least_dist_angle - angle) / max_rotational_velocity) * .5); //TODO: needs testing
			ROS_INFO_STREAM("time_to_run = " << time_to_run.toSec());

			base_trajectory::GenerateSpline srvBaseTrajectory;
			swerve_point_generator::FullGenCoefs traj;
			
			if (!generateCoefs(least_dist_angle - angle, time_to_run, srvBaseTrajectory)) //generate coefficients for the spline from the endpoints 
				ROS_INFO_STREAM("spline_gen died in teleopJoystickCommands generateCoefs");
			else if (!generateTrajectory(srvBaseTrajectory, traj)) //generate a motion profile from the coefs
				ROS_INFO_STREAM("point_gen died in teleopJoystickCommands generateTrajectory");
			else if (!runTrajectory(traj.response)) //run on swerve_control
				ROS_ERROR("swerve_control failed in teleopJoystickCommands runTrajectory");
			else
				orient_running = true;
		}
		else 
		{
			ROS_INFO_STREAM("Can't run orient, it's already running");
			if (outOfPoints.load(std::memory_order_relaxed))
				orient_running = false;
		}
	}*/

	if (fabs(leftStickX) == 0.0 && fabs(leftStickY) == 0.0 && rotation == 0.0)
	{
		if (!sendRobotZero)
		{
			std_srvs::Empty empty;
			if (!BrakeSrv.call(empty))
			{
				ROS_ERROR("BrakeSrv call failed in sendRobotZero");
			}
			ROS_INFO("BrakeSrv called");
			sendRobotZero = true;
		}
	}
	else // X or Y or rotation != 0 so tell the drive base to move
	{
		sendRobotZero = false;
		//Publish drivetrain messages and elevator/pivot
		Eigen::Vector2d joyVector;
		joyVector[0] = leftStickX; //intentionally flipped
		joyVector[1] = -leftStickY;
		Eigen::Rotation2Dd r(-navX_angle.load(std::memory_order_relaxed) - M_PI / 2);
		Eigen::Vector2d rotatedJoyVector = r.toRotationMatrix() * joyVector;

		geometry_msgs::Twist vel;
		vel.linear.x = rotatedJoyVector[1];
		vel.linear.y = rotatedJoyVector[0];
		vel.linear.z = 0;

		vel.angular.x = 0;
		vel.angular.y = 0;
		vel.angular.z = rotation;

		JoystickRobotVel.publish(vel);
		/*std_msgs::Header test_header;
		  test_header.stamp = JoystickState->header.stamp;
		  test_header.seq = 1;
		  JoystickTestVel.publish(test_header);*/
		sendRobotZero = false;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Joystick_controller");
	ros::NodeHandle n;

	ros::NodeHandle n_params(n, "teleop_params");

	/*if (!n_params.getParam("exchange_delay", exchange_delay))
		ROS_ERROR("Could not read exchange_delay");*/

	navX_angle = M_PI / 2;

	ac = std::make_shared<actionlib::SimpleActionClient<behaviors::ArmAction>>("arm_server", true);
	ac_intake = std::make_shared<actionlib::SimpleActionClient<behaviors::IntakeAction>>("intake_server", true);
	ac_arm = std::make_shared<actionlib::SimpleActionClient<behaviors::ForearmAction>> ("forearm_server", true);

	ros::Subscriber joystick_sub  = n.subscribe("joystick_states", 1, &evaluateCommands);
	//ros::Subscriber joint_states_sub = n.subscribe("/frcrobot/joint_states", 1, &jointStateCallback);
	ros::Subscriber talon_states_sub = n.subscribe("/frcrobot/talon_states", 1, &talonStateCallback);
    ros::Subscriber most_recent_command_sub = n.subscribe("/frcrobot/arm_controller/arm_command", 1, &most_recent_command_cb);

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";
	//JoystickRumble = n.advertise<std_msgs::Float64>("rumble_controller/command", 1);
	BrakeSrv = n.serviceClient<std_srvs::Empty>("/frcrobot/swerve_drive_controller/brake", false, service_connection_header);
	JoystickRobotVel = n.advertise<geometry_msgs::Twist>("/frcrobot/swerve_drive_controller/cmd_vel", 1);
    ros::Subscriber navX_heading  = n.subscribe("/frcrobot/navx_mxp", 1, &navXCallback);
    

	/*//JoystickTestVel = n.advertise<std_msgs::Header>("test_header", 3);
	JoystickElevatorPos = n.advertise<elevator_controller::ElevatorControl>("/frcrobot/elevator_controller/cmd_pos", 1);
	wait_proceed_pub = n.advertise<std_msgs::Bool>("/frcrobot/auto_interpreter_server/proceed", 1);

	EndGameDeploy = n.serviceClient<std_srvs::Empty>("/frcrobot/elevator_controller/end_game_deploy", false, service_connection_header);
	EndGameDeployWings = n.serviceClient<std_srvs::Empty>("/frcrobot/elevator_controller/end_game_deploy_wings", false, service_connection_header);
	ElevatorSrv = n.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS", false, service_connection_header);
	ClampSrv = n.serviceClient<std_srvs::SetBool>("/frcrobot/elevator_controller/clamp", false, service_connection_header);
	IntakeSrv = n.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake", false, service_connection_header);
	point_gen = n.serviceClient<swerve_point_generator::FullGenCoefs>("/point_gen/command", false, service_connection_header);
	swerve_control = n.serviceClient<talon_swerve_drive_controller::MotionProfilePoints>("/frcrobot/swerve_drive_controller/run_profile", false, service_connection_header);
	spline_gen = n.serviceClient<base_trajectory::GenerateSpline>("/base_trajectory/spline_gen", false, service_connection_header);

	VisualizeService = n.serviceClient<robot_visualizer::ProfileFollower>("/frcrobot/visualize_auto", false, service_connection_header);    
	ros::Subscriber navX_heading  = n.subscribe("/frcrobot/navx_mxp", 1, &navXCallback);
	ros::Subscriber elevator_odom = n.subscribe("/frcrobot/elevator_controller/odom", 1, &OdomCallback);
	ros::Subscriber elevator_cmd  = n.subscribe("/frcrobot/elevator_controller/return_cmd_pos", 1, &elevCmdCallback);
	ros::Subscriber cube_state    = n.subscribe("/frcrobot/elevator_controller/cube_state", 1, &cubeCallback); */

	ROS_WARN("joy_init");

	ros::spin();
	return 0;
}


/*void rumbleTypeConverterPublish(uint16_t leftRumble, uint16_t rightRumble)
{
	const unsigned int rumble = ((leftRumble & 0xFFFF) << 16) | (rightRumble & 0xFFFF);
	const double rumble_val = *((double *)&rumble);
	std_msgs::Float64 rumbleMsg;
	rumbleMsg.data = rumble_val;
	JoystickRumble.publish(rumbleMsg);
}

void match_data_callback(const ros_control_boilerplate::MatchSpecificData::ConstPtr &MatchData)
{
	//Joystick Rumble
	const double localMatchTimeRemaining = MatchData->matchTimeRemaining;
	matchTimeRemaining.store(localMatchTimeRemaining, std::memory_order_relaxed);
}


void navXCallback(const sensor_msgs::Imu &navXState)
{
	const tf2::Quaternion navQuat(navXState.orientation.x, navXState.orientation.y, navXState.orientation.z, navXState.orientation.w);
	double roll;
	double pitch;
	double yaw;
	tf2::Matrix3x3(navQuat).getRPY(roll, pitch, yaw);

	if (yaw == yaw) // ignore NaN results
		navX_angle.store(yaw, std::memory_order_relaxed);
}

void cube_rumble(bool has_cube)
{
	static double start_has_cube = 0;
	static bool last_has_cube = false;
	if (has_cube && !last_has_cube)
	{
		start_has_cube = ros::Time::now().toSec();
	}
	if (has_cube && ros::Time::now().toSec() < start_has_cube + 1)
	{
		const uint16_t leftRumble = 0;
		const uint16_t rightRumble = 65535;
		rumbleTypeConverterPublish(leftRumble, rightRumble);
		last_has_cube = true;
	}
	else
	{
		const uint16_t leftRumble = 0;
		const uint16_t rightRumble = 0;
		rumbleTypeConverterPublish(leftRumble, rightRumble);
	}
	if (!has_cube)
	{
		last_has_cube = false;
		start_has_cube = 0;
	}
}

void cubeCallback(const elevator_controller::CubeState &cube)
{
	cubeState.writeFromNonRT(CubeState(cube.has_cube, cube.clamp, cube.intake_low));
	cube_rumble(cube.has_cube);
}

// Grab various info from hw_interface using
// dummy joint position values
void jointStateCallback(const sensor_msgs::JointState &joint_state)
{
	static size_t clamp_idx               = std::numeric_limits<size_t>::max();
	static size_t override_arm_limits_idx = std::numeric_limits<size_t>::max();
	if ((clamp_idx               >= joint_state.name.size()) ||
		(override_arm_limits_idx >= joint_state.name.size() ))
	{
		for (size_t i = 0; i < joint_state.name.size(); i++)
		{
			if (joint_state.name[i] == "clamp")
				clamp_idx = i;
			else if (joint_state.name[i] == "override_arm_limits")
				override_arm_limits_idx = i;
		}
	}
	if (clamp_idx < joint_state.position.size())
		clamped_c.store(joint_state.position[clamp_idx] <= 0, std::memory_order_relaxed);
	if (override_arm_limits_idx < joint_state.position.size())
		disableArmLimits.store(joint_state.position[override_arm_limits_idx], std::memory_order_relaxed);
}*/

void talonStateCallback(const talon_state_controller::TalonState &talon_state)
{
	static size_t arm_joint_idx = std::numeric_limits<size_t>::max();

	if (arm_joint_idx >= talon_state.name.size())
	{
		for (size_t i = 0; i < talon_state.name.size(); i++)
		{
			if (talon_state.name[i] == "arm_joint")
			{
				arm_joint_idx = i;
				break;
			}
		}
	}

        arm_position.store(talon_state.position[arm_joint_idx], std::memory_order_relaxed);
}

void most_recent_command_cb(const std_msgs::Float64 &msg)
{
    most_recent_arm_command.writeFromNonRT(msg.data);
}
