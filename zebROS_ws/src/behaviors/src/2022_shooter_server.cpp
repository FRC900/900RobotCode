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

#define DOWNTOWN_ACTIVE    1.0
#define DOWNTOWN_INACTIVE  0.0

class ShooterAction2022
{
protected:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_params_;
  actionlib::SimpleActionServer<behavior_actions::Shooter2022Action> as_;
  std::string action_name_;
  // create message that is used to publish feedback
  behavior_actions::Shooter2022Feedback feedback_;
  // speed is now an input, these can be used for debugging with DDR
  // Not used unless they are a non zero value
  double absolute_wheel_speed_ = 0;
  double absolute_hood_wheel_speed_ = 0;
  double eject_speed_;
  double error_margin_;
  bool hood_state_;
  ddynamic_reconfigure::DDynamicReconfigure ddr_;

  ros::Publisher shooter_command_pub_;
  ros::Publisher hood_shooter_command_pub_;
  ros::Publisher downtown_command_pub_;

  ros::Subscriber talon_states_sub_;

  ros::Subscriber speed_offset_sub_;
  ros::Subscriber hood_speed_offset_sub_;

  double current_speed_;
  double hood_current_speed_;
  double speed_offset_ = 0;
  double hood_speed_offset_ = 0;
  bool hood_state_override_ = false;
  uint64_t close_enough_counter_;
  int shooter_wheel_checks_ = 20;

public:

  ShooterAction2022(std::string name) :
    as_(nh_, name, boost::bind(&ShooterAction2022::executeCB, this, _1), false),
    nh_params_(nh_, "shooter_server_2022"),
    action_name_(name),
    ddr_(nh_params_)
  {
    if (!nh_params_.getParam("absolute_wheel_speed", absolute_wheel_speed_))
    {
      absolute_wheel_speed_ = 0; // was 325 at start of UNCA, 343 at UNCP
      ROS_ERROR_STREAM("2022_shooter_server : could not find absolute_wheel_speed, defaulting to " << absolute_wheel_speed_);
      return;
    }
    ddr_.registerVariable<double>("absolute_wheel_speed", &absolute_wheel_speed_, "Speed of lower wheel (formerly high_goal_speed)", 0, 500);

    if (!nh_params_.getParam("absolute_hood_wheel_speed", absolute_hood_wheel_speed_))
    {
      absolute_hood_wheel_speed_ = 0;
      ROS_ERROR_STREAM("2022_shooter_server : could not find absolute_hood_wheel_speed_, defaulting to " << absolute_hood_wheel_speed_);
      return;
    } // 180 or 200
    ddr_.registerVariable<double>("absolute_hood_wheel_speed", &absolute_hood_wheel_speed_, "Hood wheel shooting speed for testing", 0, 500);

    if (!nh_params_.getParam("eject_speed", eject_speed_))
    {
      eject_speed_ = 120;
      ROS_ERROR_STREAM("2022_shooter_server : could not find eject_speed, defaulting to " << eject_speed_);
      return;
    }
    ddr_.registerVariable<double>("eject_speed", &eject_speed_, "Eject Cargo - Shooting Speed", 0, 500);

    if (!nh_params_.getParam("error_margin", error_margin_))
    {
      error_margin_ = 2;
      ROS_ERROR_STREAM("2022_shooter_server : could not find error_margin, defaulting to " << error_margin_);
      return;
    }
    ddr_.registerVariable<double>("error_margin", &error_margin_, "Shooter margin of error", 0, 50);

    if (!nh_params_.getParam("shooter_wheel_checks", shooter_wheel_checks_))
    {
      shooter_wheel_checks_ = 8;
      ROS_ERROR_STREAM("2022_shooter_server : could not find shooter_wheel_checks, defaulting to " << shooter_wheel_checks_);
      return;
    }
    ddr_.registerVariable<int>("shooter_wheel_checks", &shooter_wheel_checks_, "Number of times to check shooter wheel speed", 0, 50);
    ddr_.registerVariable<bool>("hood_state_override_", &hood_state_override_, "Hood state, must set absolute shooter speeds for this to be used", 0, 1);
    // change close_enough to operate with multiple samples
    // error_margin_ = 5;
    // ddr_.registerVariable<double>("samples_for_close_enough", &error_margin_, "Shooter margin of error", 0, 50);

    ddr_.publishServicesTopics();

    shooter_command_pub_ = nh_.advertise<std_msgs::Float64>("/frcrobot_jetson/shooter_controller/command", 2);
    hood_shooter_command_pub_ = nh_.advertise<std_msgs::Float64>("/frcrobot_jetson/hood_shooter_controller/command", 2);
    downtown_command_pub_ = nh_.advertise<std_msgs::Float64>("/frcrobot_jetson/downtown_solenoid_controller/command", 2);
    talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states", 1, &ShooterAction2022::talonStateCallback, this);
    speed_offset_sub_ = nh_.subscribe("/shooter_speed_offset", 1, &ShooterAction2022::speedOffsetCallback, this);
    hood_speed_offset_sub_ = nh_.subscribe("/hood_shooter_speed_offset", 1, &ShooterAction2022::hoodSpeedOffsetCallback, this);
    as_.start();
  }

  ~ShooterAction2022(void)
  {
  }

  void executeCB(const behavior_actions::Shooter2022GoalConstPtr &goal)
  {
    SHOOTER_INFO("Shooter action called with mode " << goal->mode);

    std_msgs::Float64 msg; // for lower wheels
    std_msgs::Float64 hood_msg;
    std_msgs::Float64 downtown_msg;
    downtown_msg.data = DOWNTOWN_INACTIVE;
    double shooter_speed;
    double hood_shooter_speed;
    switch (goal->mode) {
      case behavior_actions::Shooter2022Goal::HIGH_GOAL:
        shooter_speed = goal->wheel_speed;
        hood_shooter_speed = goal->hood_wheel_speed;
        if (goal->hood_position) {
          downtown_msg.data = DOWNTOWN_ACTIVE;
        }
        break;
      case behavior_actions::Shooter2022Goal::EJECT:
        shooter_speed = eject_speed_;
        break;
      default:
        SHOOTER_ERROR("invalid goal mode (" << goal->mode << ")");
        msg.data = 0;
        hood_msg.data = 0;
        shooter_command_pub_.publish(msg);
        hood_shooter_command_pub_.publish(hood_msg);
        feedback_.close_enough = false;
        as_.publishFeedback(feedback_);
        // set the action state to preempted
        as_.setPreempted();
        return;
    }
      if (absolute_wheel_speed_ && absolute_hood_wheel_speed_) {
        downtown_msg.data = hood_state_override_;
        ROS_ERROR_STREAM_THROTTLE(1, "Using absolute shooter speeds, THIS SHOULD ONLY BE USED DURING TESTING");
      }
    downtown_command_pub_.publish(downtown_msg);
    /* Offsets commented out until we need them
    shooter_speed += speed_offset_;
    hood_shooter_speed += hood_speed_offset_;
    */
    SHOOTER_INFO("Shooter speed setpoint = " << shooter_speed);
    SHOOTER_INFO("Hood shooter speed setpoint = " << hood_shooter_speed);
    int good_samples = 0;
    current_speed_ = std::numeric_limits<double>::max();
    hood_current_speed_ = std::numeric_limits<double>::max();
    ros::Rate r(100);
    while (ros::ok()) {
      ros::spinOnce();
      if (as_.isPreemptRequested() || !ros::ok())
      {
        msg.data = 0;
        hood_msg.data = 0;
        hood_shooter_command_pub_.publish(hood_msg);
        shooter_command_pub_.publish(msg);
        downtown_msg.data = DOWNTOWN_INACTIVE;
        downtown_command_pub_.publish(downtown_msg);
        feedback_.close_enough = false;
        as_.publishFeedback(feedback_);
        SHOOTER_INFO(" : Preempted");
        // set the action state to preempted
        as_.setPreempted();
        break;
      }
      if (absolute_wheel_speed_ && absolute_hood_wheel_speed_) {
        msg.data = absolute_wheel_speed_;
        hood_msg.data = absolute_hood_wheel_speed_;
        ROS_ERROR_STREAM_THROTTLE(1, "Using absolute shooter speeds, THIS SHOULD ONLY BE USED DURING TESTING");
      }
      else {
        msg.data = shooter_speed;
        hood_msg.data = hood_shooter_speed;
      }
      shooter_command_pub_.publish(msg);
      hood_shooter_command_pub_.publish(hood_msg);
      
      /* Measure if the sample is close enough to the requested shooter wheel speed */
      // Maybe add diffrent error margins?
      ROS_INFO_STREAM_THROTTLE(2, "SHOOTER SERVER shooter speed: " << msg.data << " CURRENT SHOOTER SPEED " << current_speed_ << " CURRENT HOOD SPEED " << " HOOD SPEED " << hood_current_speed_);
      if((fabs(msg.data - fabs(current_speed_)) < error_margin_) && (fabs(hood_msg.data - fabs(hood_current_speed_)) < error_margin_)) {
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
    bool checked_speed = false;
    for (size_t i = 0; i < talon_state.name.size(); i++) {
      // not sure which one will get found first, checked_speed makes sure not to return to early
      if (talon_state.name[i] == "shooter_leader") {
        current_speed_ = talon_state.speed[i];
        if (checked_speed) {return;}
        checked_speed = true;
      }
      if (talon_state.name[i] == "hood_shooter") {
        hood_current_speed_ = talon_state.speed[i];
        if (checked_speed) {return;}
        checked_speed = true;
      }
    }
    SHOOTER_ERROR_THROTTLE(0.5, "Couldn't find shooter_leader or hood_shooter talon in /frcrobot_jetson/talon_states. :(");
  }

  void speedOffsetCallback(const std_msgs::Float64 speed_offset_msg){
    speed_offset_ = speed_offset_msg.data;
  }
  void hoodSpeedOffsetCallback(const std_msgs::Float64 hood_speed_offset_msg) {
    hood_speed_offset_ = hood_speed_offset_msg.data;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "shooter_server_2022");

  ShooterAction2022 shooter("shooter_server_2022");
  ros::spin();

  return 0;
}
