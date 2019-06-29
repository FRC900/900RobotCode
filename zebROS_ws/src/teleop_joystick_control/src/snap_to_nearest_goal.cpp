#include <atomic>
#include "std_srvs/Empty.h"
#include <hardware_interface/joint_command_interface.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "angles/angles.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

#include <vector>

ros::Publisher snapAnglePub;
std::atomic<double> navX_angle;
std::atomic<bool> has_cargo;
std::atomic<bool> has_panel;

std::vector<double> hatch_panel_angles;
std::vector<double> cargo_angles;
std::vector<double> nothing_angles;

int limit_switch_debounce_iterations;
int linebreak_debounce_iterations;


double nearest_angle(std::vector<double> angles, double cur_angle)
{
	double snap_angle;
	double smallest_distance = std::numeric_limits<double>::max();
	for(size_t i = 0; i < angles.size(); i++){
		double distance = fabs(angles::shortest_angular_distance(cur_angle, angles[i]));
		if(distance < smallest_distance) {
			smallest_distance = distance;
			snap_angle = angles[i];
		}
	}
	return snap_angle;
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

void jointStateCallback(const sensor_msgs::JointState &joint_state)
{
		static int limit_switch_true_panel_count = 0;
		static int limit_switch_false_panel_count = 0;
		static int linebreak_true_cargo_count = 0;
		static int linebreak_false_cargo_count = 0;
        //get index of limit_switch sensor for this actionlib server
        static size_t limit_switch_idx_1 = std::numeric_limits<size_t>::max();
        static size_t limit_switch_idx_2 = std::numeric_limits<size_t>::max();
        static size_t limit_switch_idx_3 = std::numeric_limits<size_t>::max();
        static size_t limit_switch_idx_4 = std::numeric_limits<size_t>::max();
        static size_t linebreak_idx_1 = std::numeric_limits<size_t>::max();

        if (limit_switch_idx_1 >= joint_state.name.size()
            || limit_switch_idx_2 >= joint_state.name.size()
            || limit_switch_idx_3 >= joint_state.name.size()
            || limit_switch_idx_4 >= joint_state.name.size()
			|| linebreak_idx_1 >= joint_state.name.size())
        {
            for (size_t i = 0; i < joint_state.name.size(); i++)
            {
                if (joint_state.name[i] == "panel_intake_limit_switch_1")
                    limit_switch_idx_1 = i;
                if (joint_state.name[i] == "panel_intake_limit_switch_2")
                    limit_switch_idx_2 = i;
                if (joint_state.name[i] == "panel_intake_limit_switch_3")
                    limit_switch_idx_3 = i;
                if (joint_state.name[i] == "panel_intake_limit_switch_4")
                    limit_switch_idx_4 = i;
                if (joint_state.name[i] == "cargo_intake_linebreak_1")
                    linebreak_idx_1 = i;
            }
        }

        //update limit_switch counts based on the value of the limit_switch sensor
        if (limit_switch_idx_1 < joint_state.position.size()
            && limit_switch_idx_2 < joint_state.position.size()
            && limit_switch_idx_3 < joint_state.position.size()
            && limit_switch_idx_4 < joint_state.position.size()
            && linebreak_idx_1 < joint_state.position.size() )
        {
            bool linebreak_true_cargo = ( joint_state.position[linebreak_idx_1] != 0);
            bool limit_switch_true_panel = ( joint_state.position[limit_switch_idx_1] != 0
                                    || joint_state.position[limit_switch_idx_2] != 0
                                    || joint_state.position[limit_switch_idx_3] != 0
                                    || joint_state.position[limit_switch_idx_4] != 0
                                );
            if(linebreak_true_cargo)
            {
                linebreak_true_cargo_count += 1;
                linebreak_false_cargo_count = 0;
            }
            else
            {
                linebreak_true_cargo_count = 0;
                linebreak_false_cargo_count += 1;
            }
            if(limit_switch_true_panel)
            {
                limit_switch_true_panel_count += 1;
                limit_switch_false_panel_count = 0;
            }
            else
            {
                limit_switch_true_panel_count = 0;
                limit_switch_false_panel_count += 1;
            }

		    if(linebreak_true_cargo_count >	linebreak_debounce_iterations) {
                has_cargo.store(true);
            }
            else {
                has_cargo.store(false);
            }
		    if(limit_switch_true_panel_count >	limit_switch_debounce_iterations) {
                has_panel.store(true);
            }
            else {
                has_panel.store(false);
            }
        }
        else
        {
            static int count = 0;
            if(count % 100 == 0)
            {
                ROS_WARN("intake line break sensor not found in joint_states");
            }
            count++;
            limit_switch_true_panel_count = 0;
            linebreak_true_cargo_count = 0;
            limit_switch_false_panel_count += 1;
            linebreak_false_cargo_count += 1;
        }
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Joystick_controller");
	ros::NodeHandle nh;
	ros::NodeHandle n_params(nh, "goal_angles");
	ros::NodeHandle n_params_teleop(nh, "/teleop/teleop_params");

	if(!n_params.getParam("hatch_panel_angles", hatch_panel_angles))
	{
		ROS_ERROR("Could not read hatch_panel_angles in teleop joystick snap to goal");
	}
	if(!n_params.getParam("cargo_angles", cargo_angles))
	{
		ROS_ERROR("Could not read cargo_angles in teleop joystick snap to goal");
	}
	if(!n_params.getParam("nothing_angles", nothing_angles))
	{
		ROS_ERROR("Could not read nothing_angles in teleop joystick snap to goal");
	}


	if(!n_params_teleop.getParam("limit_switch_debounce_iterations", limit_switch_debounce_iterations))
	{
		ROS_ERROR("Could not read limit_switch_debounce_interations in teleop joystick snap to goal");
	}
	if(!n_params_teleop.getParam("linebreak_debounce_iterations", linebreak_debounce_iterations))
	{
		ROS_ERROR("Could not read linebreak_debounce_interations in teleop joystick snap to goal");
	}


	navX_angle = M_PI / 2;
	has_panel = true;
	has_cargo = false;

	ros::Subscriber joint_states_sub_ = nh.subscribe("/frcrobot_jetson/joint_states", 1, jointStateCallback);
	ros::Subscriber navX_heading  = nh.subscribe("/frcrobot_rio/navx_mxp", 1, &navXCallback);
	ros::Publisher snapAnglePub = nh.advertise<std_msgs::Float64>("navX_pid/setpoint", 10);
	ros::Publisher navXStatePub = nh.advertise<std_msgs::Float64>("navX_pid/state", 10);
	ROS_INFO("snap_to_angle_init");

	ros::Rate r(100);
	double snap_angle;
	ros::spinOnce();

	while(ros::ok()) {
		std_msgs::Float64 angle_snap;
		std_msgs::Float64 navX_state;
		double cur_angle = angles::normalize_angle_positive(-1*navX_angle.load(std::memory_order_relaxed));
		has_panel = true;
		if(has_panel) {
			snap_angle = nearest_angle(hatch_panel_angles, cur_angle + M_PI/2) - M_PI/2; //TODO remove having to multiply negative one
		}
		else if(has_cargo) {
			snap_angle = nearest_angle(cargo_angles, cur_angle);
		}
		else {
			snap_angle = nearest_angle(nothing_angles, cur_angle);
		}
		
		//TODO make this not a hack (ASSUMES hatch panel)
		//snap_angle = nearest_angle(hatch_panel_angles, cur_angle + M_PI/2) - M_PI/2; //TODO remove having to multiply negative one
        //snap_angle = nearest_angle(cargo_angles, cur_angle);

		double heading = angles::normalize_angle(-1*navX_angle.load(std::memory_order_relaxed));
		double goal_angle = angles::normalize_angle(snap_angle);
		double angle_diff = angles::normalize_angle(goal_angle - heading);
		angle_snap.data = 0.0;
		navX_state.data = angle_diff;
		snapAnglePub.publish(angle_snap);
		//ROS_WARN_STREAM_THROTTLE(0.1, "Angle diff: " << angle_diff);
		//ROS_WARN_STREAM_THROTTLE(0.1, "Heading: " <<  heading);
		//ROS_WARN_STREAM_THROTTLE(0.1, "Goal: " <<  goal_angle);
        navXStatePub.publish(navX_state);

		r.sleep();
		ros::spinOnce();
	}
	return 0;
}
