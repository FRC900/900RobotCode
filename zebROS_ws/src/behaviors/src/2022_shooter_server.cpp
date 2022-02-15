#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behavior_actions/Shooter2022Action.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <std_msgs/Float64.h>
#include "talon_state_msgs/TalonState.h"

#define SHOOTER_INFO(x) ROS_INFO_STREAM("2022_shooter_server : " << x)
#define SHOOTER_INFO_THROTTLE(d,x) ROS_INFO_STREAM_THROTTLE(d, "2022_shooter_server : " << x)
#define SHOOTER_ERROR(x) ROS_ERROR_STREAM("2022_shooter_server : " << x)

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

  double current_speed_;

public:

  ShooterAction2022(std::string name) :
    as_(nh_, name, boost::bind(&ShooterAction2022::executeCB, this, _1), false),
    action_name_(name),
    nh_params_(nh_, "shooter_server_2022"),
    ddr_(nh_params_)
  {
    high_goal_speed_ = 25;
    ddr_.registerVariable<double>("high_goal_speed", &high_goal_speed_, "High Goal Shooting Speed", 0, 27);
    low_goal_speed_ = 20;
    ddr_.registerVariable<double>("low_goal_speed", &low_goal_speed_, "Low Goal Shooting Speed", 0, 27);
    eject_speed_ = 7;
    ddr_.registerVariable<double>("eject_speed", &eject_speed_, "Eject Cargo - Shooting Speed", 0, 27);
    error_margin_ = 0.25;
    ddr_.registerVariable<double>("error_margin", &error_margin_, "Shooter margin of error", 0, 2);
    shooter_command_pub_ = nh_.advertise<std_msgs::Float64>("/frcrobot_jetson/shooter_controller/command", 2);
    talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states", 1, &ShooterAction2022::talonStateCallback, this);
    as_.start();

    ddr_.publishServicesTopics();
  }

  ~ShooterAction2022(void)
  {
  }

  void executeCB(const behavior_actions::Shooter2022GoalConstPtr &goal)
  {
    SHOOTER_INFO("Shooter action called with mode " << std::to_string(goal->mode));
    ros::Rate r(100);
    while (ros::ok()) {
      ros::spinOnce();
      std_msgs::Float64 msg;
      if (as_.isPreemptRequested() || !ros::ok())
      {
        msg.data = 0;
        shooter_command_pub_.publish(msg);
        feedback_.close_enough = false;
        as_.publishFeedback(feedback_);
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        break;
      }
      switch (goal->mode) {
        case behavior_actions::Shooter2022Goal::HIGH_GOAL:
          feedback_.close_enough = fabs(high_goal_speed_ - current_speed_) < error_margin_;
          msg.data = high_goal_speed_;
          shooter_command_pub_.publish(msg);
          break;
        case behavior_actions::Shooter2022Goal::LOW_GOAL:
          feedback_.close_enough = fabs(low_goal_speed_ - current_speed_) < error_margin_;
          msg.data = low_goal_speed_;
          shooter_command_pub_.publish(msg);
          break;
        case behavior_actions::Shooter2022Goal::EJECT:
          feedback_.close_enough = fabs(eject_speed_ - current_speed_) < error_margin_;
          msg.data = eject_speed_;
          shooter_command_pub_.publish(msg);
          break;
      }
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
    SHOOTER_ERROR("Couldn't find talon in /frcrobot_jetson/talon_states. :(");
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "shooter_server_2022");

  ShooterAction2022 shooter("shooter_server_2022");
  ros::spin();

  return 0;
}
