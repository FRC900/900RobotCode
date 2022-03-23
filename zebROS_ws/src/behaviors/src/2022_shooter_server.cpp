#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behavior_actions/Shooter2022Action.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <std_msgs/Float64.h>
#include "talon_state_msgs/TalonState.h"

#define SHOOTER_INFO(x) ROS_INFO_STREAM("2022_shooter_server : " << x)
#define SHOOTER_INFO_THROTTLE(d,x) ROS_INFO_STREAM_THROTTLE((d), "2022_shooter_server : " << x)
#define SHOOTER_ERROR(x) ROS_ERROR_STREAM("2022_shooter_server : " << x)
#define SHOOTER_ERROR_THROTTLE(d,x) ROS_ERROR_STREAM_THROTTLE((d), "2022_shooter_server : " << x)

class ShooterAction2022
{
protected:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_params_;
  actionlib::SimpleActionServer<behavior_actions::Shooter2022Action> as_;
  std::string action_name_;
  // create message that is used to publish feedback
  behavior_actions::Shooter2022Feedback feedback_;

  double high_goal_speed_;
  double low_goal_speed_;
  double eject_speed_;
  double error_margin_;
  ddynamic_reconfigure::DDynamicReconfigure ddr_;

  ros::Publisher shooter_command_pub_;

  ros::Subscriber talon_states_sub_;
  ros::Subscriber speed_offset_sub_;

  double current_speed_;
  double speed_offset_ = 0;

  uint64_t close_enough_counter_;
  int shooter_wheel_checks_ = 20;

public:

  ShooterAction2022(std::string name) :
    as_(nh_, name, boost::bind(&ShooterAction2022::executeCB, this, _1), false),
    nh_params_(nh_, "shooter_server_2022"),
    action_name_(name),
    ddr_(nh_params_)
  {
    high_goal_speed_ = 335; // was 325 at start of UNCA
    ddr_.registerVariable<double>("high_goal_speed", &high_goal_speed_, "High Goal Shooting Speed", 0, 500);
    low_goal_speed_ = 200; // 180 or 200
    ddr_.registerVariable<double>("low_goal_speed", &low_goal_speed_, "Low Goal Shooting Speed", 0, 500);
    eject_speed_ = 120;
    ddr_.registerVariable<double>("eject_speed", &eject_speed_, "Eject Cargo - Shooting Speed", 0, 500);
    error_margin_ = 1;
    ddr_.registerVariable<double>("error_margin", &error_margin_, "Shooter margin of error", 0, 50);
    shooter_wheel_checks_ = 20;
    ddr_.registerVariable<int>("shooter_wheel_checks", &shooter_wheel_checks_, "Number of times to check shooter wheel speed", 0, 50);
    // change close_enough to operate with multiple samples
    // error_margin_ = 5;
    // ddr_.registerVariable<double>("samples_for_close_enough", &error_margin_, "Shooter margin of error", 0, 50);
    shooter_command_pub_ = nh_.advertise<std_msgs::Float64>("/frcrobot_jetson/shooter_controller/command", 2);
    talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states", 1, &ShooterAction2022::talonStateCallback, this);
    speed_offset_sub_ = nh_.subscribe("/shooter_speed_offset", 1, &ShooterAction2022::speedOffsetCallback, this);
    as_.start();

    ddr_.publishServicesTopics();
  }

  ~ShooterAction2022(void)
  {
  }

  void executeCB(const behavior_actions::Shooter2022GoalConstPtr &goal)
  {
    SHOOTER_INFO("Shooter action called with mode " << goal->mode);
	std_msgs::Float64 msg;
	double shooter_speed;
	switch (goal->mode) {
	  case behavior_actions::Shooter2022Goal::HIGH_GOAL:
	    shooter_speed = high_goal_speed_;
		break;
	  case behavior_actions::Shooter2022Goal::LOW_GOAL:
		shooter_speed = low_goal_speed_;
		break;
	  case behavior_actions::Shooter2022Goal::EJECT:
		shooter_speed = eject_speed_;
		break;
	  default:
		SHOOTER_ERROR("invalid goal mode (" << goal->mode << ")");
		msg.data = 0;
		shooter_command_pub_.publish(msg);
		feedback_.close_enough = false;
		as_.publishFeedback(feedback_);
		// set the action state to preempted
		as_.setPreempted();
		return;
	}
	SHOOTER_INFO("Shooter speed setpoint = " << msg.data);
  int good_samples = 0;
    ros::Rate r(100);
    while (ros::ok()) {
      ros::spinOnce();
      if (as_.isPreemptRequested() || !ros::ok())
      {
        msg.data = 0;
        shooter_command_pub_.publish(msg);
        feedback_.close_enough = false;
        as_.publishFeedback(feedback_);
        SHOOTER_INFO(" : Preempted");
        // set the action state to preempted
        as_.setPreempted();
        break;
      }
	  msg.data = shooter_speed + speed_offset_;
	  shooter_command_pub_.publish(msg);
    /* Measure if the sample is close enough to the requested shooter wheel speed */
    if(fabs(shooter_speed - fabs(current_speed_)) < error_margin_) {
      good_samples++;
    } else {
      good_samples = 0;
    }
	  feedback_.close_enough = good_samples > shooter_wheel_checks_;
      as_.publishFeedback(feedback_);
      r.sleep();
    }
  }

  void talonStateCallback(const talon_state_msgs::TalonState talon_state)
  {
    for (size_t i = 0; i < talon_state.name.size(); i++) {
      if (talon_state.name[i] == "shooter_leader") {
        current_speed_ = talon_state.speed[i];
        return;
      }
    }
    SHOOTER_ERROR_THROTTLE(0.5, "Couldn't find shooter_leader talon in /frcrobot_jetson/talon_states. :(");
  }

  void speedOffsetCallback(const std_msgs::Float64 speed_offset_msg){
    speed_offset_ = speed_offset_msg.data;
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "shooter_server_2022");

  ShooterAction2022 shooter("shooter_server_2022");
  ros::spin();

  return 0;
}
