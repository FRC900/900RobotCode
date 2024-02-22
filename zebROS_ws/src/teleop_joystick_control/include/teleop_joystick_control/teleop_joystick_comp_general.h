#ifndef INC_TELEOP_JOYSTICK_COMP_GENERAL_H
#define INC_TELEOP_JOYSTICK_COMP_GENERAL_H

#include "teleop_joystick_control/TeleopCmdVel2023.h"

#include "behavior_actions/AutoMode.h"
#include "teleop_joystick_control/RobotOrient.h"
#include "teleop_joystick_control/RobotOrientationDriver.h"
#include "frc_msgs/MatchSpecificData.h"

// There's gotta be a better name for this
class AutoModeCalculator {
public:
	virtual uint8_t calculateAutoMode() = 0;
	void register_publisher(ros::NodeHandle n) {
		auto_mode_select_pub_ = n.advertise<behavior_actions::AutoMode>("/auto/auto_mode", 1, true);
		timer_ = n.createTimer(ros::Duration(0.1), &AutoModeCalculator::publisher_callback, this);
	}	
	void publisher_callback(const ros::TimerEvent&) {
		behavior_actions::AutoMode msg;
		msg.header.stamp = ros::Time::now();
		msg.auto_mode = calculateAutoMode();
		auto_mode_select_pub_.publish(msg);
	}
	virtual ~AutoModeCalculator() = default;
private:
	ros::Publisher auto_mode_select_pub_;
	ros::Timer timer_;
};

struct DynamicReconfigVars
{
	double joystick_deadzone{0};          // "Joystick deadzone, in percent",
	double radial_deadzone{0.05};		  // "Radial deadzone, in angular distance",
	double min_speed{0};                  // "Min linear speed to get robot to overcome friction, in m/s"
	double max_speed{2.0};                // "Max linear speed, in m/s"
	double max_speed_slow{0.75};          // "Max linear speed in slow mode, in m/s"
	double max_rot{6.0};                  // "Max angular speed"
	double max_rot_slow{2.0};             // "Max angular speed in slow mode"
	double button_move_speed{0.5};        // "Linear speed when move buttons are pressed, in m/s"
	double joystick_pow{1.5};             // "Joystick Scaling Power, linear"
	double rotation_pow{1.0};             // "Joystick Scaling Power, rotation"
	double drive_rate_limit_time{200};    // "msec to go from full back to full forward"
	double rotate_rate_limit_time{500};   // "msec to go from full counterclockwise to full clockwise"
	double trigger_threshold{0.5};        // "Amount trigger has to be pressed to trigger action"
	double stick_threshold{0.5};          // "Amount stick has to be moved to trigger diag mode action"
	double imu_zero_angle{0.0};           // "Value to pass to imu/set_zero when zeroing"
	double rotation_epsilon{0.01};		  // "Threshold Z-speed deciding if the robot is stopped"
	double rotation_axis_scale{1.0};      // "Scale factor for rotation axis stick input"
	double angle_to_add{0.135};
	double angle_threshold{angles::from_degrees(1)};
	double match_time_to_park{20}; // enable auto-parking after the 0.75 second timeout if the match time left < this value
}; 

class Driver {
public:
	Driver() = delete;
	Driver(ros::NodeHandle n, DynamicReconfigVars config);
	void moveDirection(int x, int y, int z, double button_move_speed);
	void sendDirection(double button_move_speed);
	ros::Time evalateDriverCommands(const frc_msgs::JoystickState &joy_state, const DynamicReconfigVars& config);
	void setTargetOrientation(const double angle, const bool from_teleop, const double velocity = (0.0));
	bool getNoDriverInput();
	bool orientCallback(teleop_joystick_control::RobotOrient::Request &req,
						teleop_joystick_control::RobotOrient::Response & /* res*/);
	bool waitForBrakeSrv(ros::Duration startup_wait_time);

	TeleopCmdVel<DynamicReconfigVars> teleop_cmd_vel_;
private:
	ros::Publisher JoystickRobotVel_;
	
	ros::ServiceClient BrakeSrv_;
	RobotOrientationDriver robot_orientation_driver_;
	int direction_x_{0};
	int direction_y_{0};
	int direction_z_{0};
	bool sendRobotZero_{false};
	bool no_driver_input_{false};
	double old_angular_z_{0.0};
	bool sendSetAngle_{true};
};

extern bool diagnostics_mode;
extern struct DynamicReconfigVars config;
extern std::unique_ptr<Driver> driver;
extern std::vector <std::string> topic_array;
extern ros::ServiceClient ParkSrv;
extern ros::ServiceClient IMUZeroSrv;
extern ros::ServiceClient SwerveOdomZeroSrv;
extern ros::ServiceClient setCenterSrv;	
extern ros::Publisher auto_mode_select_pub;
extern bool joystick1_left_trigger_pressed;
extern bool joystick1_right_trigger_pressed;
extern uint8_t alliance_color;
extern bool called_park_endgame;

void publish_diag_cmds(void);
void zero_all_diag_commands(void);
void preemptActionlibServers(void);
void evaluateCommands(const frc_msgs::JoystickStateConstPtr& joystick_state, int joystick_id);

// Don't uncomment until we generalize the button box
//void buttonBoxCallback(const ros::MessageEvent<std_msgs::Bool const>& event);
void matchStateCallback(const frc_msgs::MatchSpecificData &msg);

struct DDRVariable {
	std::string name;
	double value;
	std::string description;
	double min;
	double max;
};

class TeleopInitializer {
public:
	void add_custom_var(DDRVariable var);
	void set_n_params(ros::NodeHandle n_params);
	void set_n(ros::NodeHandle n);
	void init();

private:
	std::vector<DDRVariable> custom_vars;
	ros::NodeHandle n_;
	ros::NodeHandle n_params_;
};

#endif