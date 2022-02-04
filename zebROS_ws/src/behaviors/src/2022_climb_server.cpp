#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "behavior_actions/Climb2022Action.h"
#include "controllers_2022_msgs/DynamicArmSrv.h"
#include <functional>
#include <std_msgs/Float64.h>
#include "sensor_msgs/JointState.h"
#include <map>
#include "actionlib_msgs/GoalID.h"
#include "talon_state_msgs/TalonState.h"

// How to launch hardware in simulation: roslaunch controller_node 2022_compbot_combined.launch hw_or_sim:=sim
// Driver station (contains simulated inputs): rosrun rqt_driver_station_sim rqt_driver_station_sim &
// axclient.py (actionlib client GUI): rosrun actionlib axclient.py /climb_server_2022 &

class ClimbStateMachine
{
protected:
  ros::NodeHandle nh_;
  boost::function<void()> nextFunction_;
  bool waitForHuman_ = true;
  ros::Publisher dynamic_arm_piston_;
  ros::Publisher static_hook_piston_;

  ros::Subscriber joint_states_sub_;
  ros::Subscriber talon_states_sub_;

  double d1_ls; // d = dynamic, s = static, ls = limit switch
  double d2_ls;
  double s1_ls;
  double s2_ls;

  talon_state_msgs::TalonState talon_states_;

  double arm_height_;
  double lowering_hook_speed_;
  double distance_above_rung_;
  double imbalance_timeout_; // seconds

  actionlib::SimpleActionServer<behavior_actions::Climb2022Action> &as_;
  ros::ServiceClient dynamic_arm_;
  // other state data we have
public:
  uint8_t state = 0;
  uint8_t rung = 0; // 0 = ground, 1 = medium, 2 = high, 3 = traversal
  bool exited = false;
  bool success = false; // whether it exited without errors
  ClimbStateMachine(bool waitForHuman, actionlib::SimpleActionServer<behavior_actions::Climb2022Action> &as_): as_(as_)
  {
    waitForHuman_ = waitForHuman;
    nextFunction_ = boost::bind(&ClimbStateMachine::state1, this);
    dynamic_arm_piston_ = nh_.advertise<std_msgs::Float64>("/frcrobot_jetson/dynamic_arm_solenoid_controller/command", 2);
    static_hook_piston_ = nh_.advertise<std_msgs::Float64>("/frcrobot_jetson/static_hook_solenoid_controller/command", 2);
    joint_states_sub_ = nh_.subscribe("/frcrobot_jetson/joint_states", 1, &ClimbStateMachine::jointStateCallback, this);
    talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states", 1, &ClimbStateMachine::talonStateCallback, this);
    dynamic_arm_ = nh_.serviceClient<controllers_2022_msgs::DynamicArmSrv>("/frcrobot_jetson/dynamic_arm_controller/command");
    if (!nh_.getParam("arm_height", arm_height_))
    {
      ROS_ERROR_STREAM("2022_climb_server : Could not find arm_height");
      success = false;
      exited = true;
      return;
    }
    if (!nh_.getParam("lowering_hook_speed", lowering_hook_speed_))
    {
      ROS_WARN_STREAM("2022_climb_server : Could not find lowering_hook_speed, defaulting to -0.3");
      lowering_hook_speed_ = -0.3;
    }
    if (!nh_.getParam("distance_above_rung", distance_above_rung_))
    {
      ROS_ERROR_STREAM("2022_climb_server : Could not find distance_above_rung");
      success = false;
      exited = true;
      return;
    }
    if (!nh_.getParam("imbalance_timeout", imbalance_timeout_))
    {
      ROS_WARN_STREAM("2022_climb_server : Could not find imbalance_timeout, defaulting to 2.0");
      imbalance_timeout_ = 2.0;
    }
    dynamic_arm_.waitForExistence();
  }
  void next()
  {
    nextFunction_();
  }
  bool sleepCheckingForPreempt(double time) // true if preempted, false if not
  {
    ros::Rate r(100); // 100 Hz loop
    for (int i = 0; i < time * 100.0; i++)
    {
      ros::spinOnce();
      if (as_.isPreemptRequested() || !ros::ok()) {
        return true;
      }
      r.sleep();
    }
    return false;
  }
  // Sensors needed:
  /*- State 3: encoder -- fully raised doesn't have to be super accurate
    - State 5: dynamic hook limit switches
    - State 7: limit switch at bottom, and encoder
    - State 9: encoder
    Total: encoder, limit switch at bottom, limit switches on dynamic and static hooks
  */
  void state1()
  {
    state = 1;
    ROS_INFO_STREAM("2022_climb_server : State 1");
    ROS_INFO_STREAM("2022_climb_server : ---");
    ROS_INFO_STREAM("2022_climb_server : Aligning...");
    if (sleepCheckingForPreempt(2.0)) return; // simulate aligning
    ROS_INFO_STREAM("2022_climb_server : Aligned");
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state2, this);
  }
  void state2()
  {
    state = 2;
    ROS_INFO_STREAM("2022_climb_server : State 2");
    ROS_INFO_STREAM("2022_climb_server : ---");
    ROS_INFO_STREAM("2022_climb_server : Extending dynamic arm pistons");
    ros::spinOnce();
    if (as_.isPreemptRequested() || !ros::ok()) {
      return;
    }
    std_msgs::Float64 dpMsg;
    dpMsg.data = 1.0;
    dynamic_arm_piston_.publish(dpMsg);
    if (sleepCheckingForPreempt(0.8)) return;
    ROS_INFO_STREAM("2022_climb_server : Extended dynamic arm pistons");
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state3, this);
  }
  void state3()
  {
    state = 3;
    ROS_INFO_STREAM("2022_climb_server : State 3");
    ROS_INFO_STREAM("2022_climb_server : ---");
    ROS_INFO_STREAM("2022_climb_server : Raising dynamic arms fully");
    controllers_2022_msgs::DynamicArmSrv srv;
    srv.request.use_percent_output = false; // motion magic
    srv.request.data = arm_height_;
    srv.request.go_slow = false;
    if (dynamic_arm_.call(srv))
    {
      ROS_INFO_STREAM("2022_climb_server : called dynamic arm service.");
    }
    else
    {
      ROS_ERROR_STREAM("2022_climb_server : failed to call dynamic arm service. Aborting.");
      exited = true;
      return;
    }
    auto nameArray = talon_states_.name;
    int leaderIndex = -1;
    for(size_t i = 0; i < nameArray.size(); i++){
      if(nameArray[i] == "climber_dynamic_arm_leader"){
        leaderIndex = i;
      }
    }
    if (leaderIndex == -1) {
      exited = true;
      ROS_ERROR_STREAM("2022_climb_server : Couldn't find talon in /frcrobot_jetson/talon_states. Aborting climb.");
      return;
    }
    ros::Rate r(100);
    while(talon_states_.position[leaderIndex] < arm_height_-0.05) // wait
    {
      ros::spinOnce();
      r.sleep();
    }
    ROS_INFO_STREAM("2022_climb_server : Raised dynamic arms fully");
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state4, this);
  }
  void state4()
  {
    state = 4;
    ROS_INFO_STREAM("2022_climb_server : State 4");
    ROS_INFO_STREAM("2022_climb_server : ---");
    if (rung == 0)
    {
      ROS_INFO_STREAM("2022_climb_server : Driving Backwards");
      if (sleepCheckingForPreempt(1)) return;
    } else
    {
      ROS_INFO_STREAM("2022_climb_server : Extending dynamic arm pistons to hit next rung");
      ros::spinOnce();
      if (as_.isPreemptRequested() || !ros::ok()) {
        return;
      }
      std_msgs::Float64 dpMsg;
      dpMsg.data = 1.0;
      dynamic_arm_piston_.publish(dpMsg);
      if (sleepCheckingForPreempt(0.8)) return;
      ROS_INFO_STREAM("2022_climb_server : Extended dynamic arm pistons");
    }
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state5, this);
  }
  void state5()
  {
    state = 5;
    ROS_INFO_STREAM("2022_climb_server : State 5");
    ROS_INFO_STREAM("2022_climb_server : ---");
    ROS_INFO_STREAM("2022_climb_server : Lowering dynamic arms a bit to touch rung");
    // Use motor (eventually)
    ros::Rate r(100); // 100 Hz loop.
    // NOTE **don't use full speed**. Motion profiling/motion magic? Or just 20-30%
    // Define max acceleration & velocity -- trapezoidal graph
    int counter = 0;
    // TODO make sure the arms are going at about even speeds
    // (e.g. if switch 1 is hit, shut off motor 1. if switch 2 is hit, shut off motor 2. and so on)

    // Turn on motors
    controllers_2022_msgs::DynamicArmSrv srv;
    srv.request.use_percent_output = true; // percent output
    srv.request.data = lowering_hook_speed_;
    srv.request.go_slow = false;
    if (dynamic_arm_.call(srv))
    {
      ROS_INFO_STREAM("2022_climb_server : called dynamic arm service.");
    }
    else
    {
      ROS_ERROR_STREAM("2022_climb_server : failed to call dynamic arm service. Aborting.");
      exited = true;
      return;
    }

    while (!(d1_ls && d2_ls)) {
      ros::spinOnce();
      if ((d1_ls == 1) ^ (d2_ls == 1)) { // if one hook has touched but the other has not,
        if (counter >= 200) { // and it has been two seconds since the robot was imbalanced,
          exited = true;
          ROS_ERROR_STREAM("2022_climb_server : The robot is imbalanced. Aborting climb.");
          // Turn off motors
          srv.request.use_percent_output = true; // percent output
          srv.request.data = 0;
          srv.request.go_slow = false;
          if (dynamic_arm_.call(srv))
          {
            ROS_INFO_STREAM("2022_climb_server : called dynamic arm service to stop motors.");
          }
          else
          {
            ROS_ERROR_STREAM("2022_climb_server : failed to call dynamic arm service to stop motors. Aborting.");
            exited = true;
            return;
          }
          return;
        } else {
          counter++;
        }
      }
      if (as_.isPreemptRequested() || !ros::ok()) {
        exited = true;
        return;
      }
      r.sleep();
    }

    // Turn off motors
    srv.request.use_percent_output = true; // percent output
    srv.request.data = 0;
    srv.request.go_slow = false;
    if (dynamic_arm_.call(srv))
    {
      ROS_INFO_STREAM("2022_climb_server : called dynamic arm service.");
    }
    else
    {
      ROS_ERROR_STREAM("2022_climb_server : failed to call dynamic arm service. Aborting.");
      exited = true;
      return;
    }

    if (sleepCheckingForPreempt(1)) return; // stop swinging
    ROS_INFO_STREAM("2022_climb_server : Lowered dynamic arms a bit to touch rung");
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state6, this);
  }
  void state6()
  {
    state = 6;
    ROS_INFO_STREAM("2022_climb_server : State 6");
    ROS_INFO_STREAM("2022_climb_server : ---");
    ROS_INFO_STREAM("2022_climb_server : Detaching static hooks");
    ros::spinOnce();
    if (as_.isPreemptRequested() || !ros::ok()) {
      return;
    }
    std_msgs::Float64 spMsg;
    spMsg.data = 0.0;
    static_hook_piston_.publish(spMsg);
    ros::Rate r(100); // 100 Hz loop
    int counter = 0;
    while (s1_ls || s2_ls) {
      ros::spinOnce();
      if ((s1_ls == 1) ^ (s2_ls == 1)) { // if one hook has touched but the other has not,
        if (counter >= 200) { // and it has been two seconds since the robot was imbalanced,
          exited = true;
          ROS_ERROR_STREAM("2022_climb_server : The robot is imbalanced. Aborting climb.");
          return;
        } else {
          counter++;
        }
      }
      if (as_.isPreemptRequested() || !ros::ok()) {
        exited = true;
        return;
      }
      r.sleep();
    }
    ROS_INFO_STREAM("2022_climb_server : Detached static hooks");
    if (rung != 0) {
      ROS_INFO_STREAM("2022_climb_server : Waiting for robot to stop swinging");
      if (sleepCheckingForPreempt(2)) return;
      ROS_INFO_STREAM("2022_climb_server : Robot stopped swinging");
    }
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state7, this);
  }
  void state7()
  {
    state = 7;
    ROS_INFO_STREAM("2022_climb_server : State 7");
    ROS_INFO_STREAM("2022_climb_server : ---");
    ROS_INFO_STREAM("2022_climb_server : Lowering dynamic arms fully");
    controllers_2022_msgs::DynamicArmSrv srv;
    srv.request.use_percent_output = false; // motion magic
    srv.request.data = 0;
    srv.request.go_slow = true;
    // Limit switch & encoder combination. Tell motor to go to the static hook vertical position and stop when it hits the position of the static hooks or limit switch, whichever comes first.
    // Definitely want motion profiling.
    if (dynamic_arm_.call(srv))
    {
      ROS_INFO_STREAM("2022_climb_server : called dynamic arm service.");
      auto nameArray = talon_states_.name;
      int leaderIndex = -1;
      for(size_t i = 0; i < size(nameArray); i++){
        if(nameArray[i] == "climber_dynamic_arm_leader"){
          leaderIndex = i;
        }
      }
      if (leaderIndex == -1) {
        exited = true;
        ROS_ERROR_STREAM("2022_climb_server : Couldn't find talon in /frcrobot_jetson/talon_states. Aborting climb.");
        return;
      }
      ros::Rate r(100);
      while(talon_states_.reverse_limit_switch[leaderIndex] /*Limit switch is not pressed*/)
      {
          ros::spinOnce();
          r.sleep();
      }
      srv.request.use_percent_output = true; // motion magic
      srv.request.data = 0;
      srv.request.go_slow = true;
      dynamic_arm_.call(srv);
      ROS_INFO_STREAM("2022_climb_server : called dynamic arm service to stop dynamic arm movement.");
    }
    else
    {
      ROS_ERROR_STREAM("2022_climb_server : failed to call dynamic arm service. Aborting.");
      exited = true;
      return;
    }

    // Wait for this to finish.
    rung++;
    ROS_INFO_STREAM("2022_climb_server : Lowered dynamic arms fully. Robot is fully supported by rung " << std::to_string(rung) << ".");
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state8, this);
  }
  void state8()
  {
    state = 8;
    ROS_INFO_STREAM("2022_climb_server : State 8");
    ROS_INFO_STREAM("2022_climb_server : ---");
    ROS_INFO_STREAM("2022_climb_server : Attaching static hooks");
    ros::spinOnce();
    if (as_.isPreemptRequested() || !ros::ok()) {
      return;
    }
    std_msgs::Float64 spMsg;
    spMsg.data = 1.0;
    static_hook_piston_.publish(spMsg);
    ros::Rate r(100); // 100 Hz loop
    int counter = 0;
    while (!(s1_ls && s2_ls)) {
      ros::spinOnce();
      if ((s1_ls == 1) ^ (s2_ls == 1)) { // if one hook has touched but the other has not,
        if (counter >= 200) { // and it has been two seconds since the robot was imbalanced,
          exited = true;
          ROS_ERROR_STREAM("2022_climb_server : The robot is imbalanced. Aborting climb.");
          return;
        } else {
          counter++;
        }
      }
      if (as_.isPreemptRequested() || !ros::ok()) {
        exited = true;
        return;
      }
      r.sleep();
    }
    ROS_INFO_STREAM("2022_climb_server : Attached static hooks");
    if (rung == 3)
    {
      ROS_INFO_STREAM("2022_climb_server : Climb is done! Woohoo!!");
      success = true;
      exited = true;
      return;
    }
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state9, this);
  }
  void state9()
  {
    state = 9;
    ROS_INFO_STREAM("2022_climb_server : State 9");
    ROS_INFO_STREAM("2022_climb_server : ---");
    ROS_INFO_STREAM("2022_climb_server : Raising dynamic arms slightly");

    // Find motor index
    int leaderIndex = -1;
    for (size_t i = 0; i < talon_states_.name.size(); i++) {
      if (talon_states_.name[i] == "climber_dynamic_arm_leader") {
        leaderIndex = i;
      }
    }
    if (leaderIndex == -1) {
      exited = true;
      ROS_ERROR_STREAM("2022_climb_server : Couldn't find talon in /frcrobot_jetson/talon_states. Aborting climb.");
      return;
    }

    // Construct service call
    controllers_2022_msgs::DynamicArmSrv srv;
    srv.request.use_percent_output = false; // motion magic
    srv.request.data = talon_states_.position[leaderIndex] + distance_above_rung_;
    srv.request.go_slow = true;

    // Call service
    if (dynamic_arm_.call(srv))
    {
      ROS_INFO_STREAM("2022_climb_server : called dynamic arm service.");
    }
    else
    {
      ROS_ERROR_STREAM("2022_climb_server : failed to call dynamic arm service. Aborting.");
      exited = true;
      return;
    }

    ROS_INFO_STREAM("2022_climb_server : Raised dynamic arms slightly");
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state10, this);
  }
  void state10()
  {
    state = 10;
    ROS_INFO_STREAM("2022_climb_server : State 10");
    ROS_INFO_STREAM("2022_climb_server : ---");
    ROS_INFO_STREAM("2022_climb_server : Retracting dynamic arm pistons");
    ros::spinOnce();
    if (as_.isPreemptRequested() || !ros::ok()) {
      return;
    }
    std_msgs::Float64 dpMsg;
    dpMsg.data = 0.0;
    dynamic_arm_piston_.publish(dpMsg);
    if (sleepCheckingForPreempt(1)) return;
    ROS_INFO_STREAM("2022_climb_server : Retracted dynamic arm pistons");
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state3, this);
  }
  void jointStateCallback(const sensor_msgs::JointState joint_state)
  {
    std::map<std::string, double*> stateNamesToVariables = {{"climber_dynamic1_limit_switch", &d1_ls}, {"climber_dynamic2_limit_switch", &d2_ls}, {"climber_static1_limit_switch", &s1_ls}, {"climber_static2_limit_switch", &s2_ls}};
    for (auto const &nameVar : stateNamesToVariables)
    {
      // get index of sensor
      size_t index = std::numeric_limits<size_t>::max();
      for (size_t i = 0; i < joint_state.name.size(); i++)
      {
        if (joint_state.name[i] == nameVar.first)
          index = i;
      }

      //update linebreak counts based on the value of the linebreak sensor
      if (index < joint_state.position.size())
      {
        *nameVar.second = joint_state.position[index];
      }
      else
      {
        ROS_WARN_STREAM_THROTTLE(2.0, "2022_climb_server : " << nameVar.first << " sensor not found in joint_states");
      }
    }
  }
  void talonStateCallback(const talon_state_msgs::TalonState talon_state)
  {
    talon_states_ = talon_state;
  }
};

class ClimbAction2022
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<behavior_actions::Climb2022Action> as_;
  std::string action_name_;
  // create messages that are used to publish feedback/result
  behavior_actions::Climb2022Feedback feedback_;
  behavior_actions::Climb2022Result result_;

public:

  ClimbAction2022(std::string name) :
    as_(nh_, name, boost::bind(&ClimbAction2022::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~ClimbAction2022(void)
  {
  }

  void executeCB(const behavior_actions::Climb2022GoalConstPtr &goal)
  {
    ClimbStateMachine sm(true, as_);
    // start executing the action
    while (!sm.exited)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        sm.success = false;
        break;
      }
      sm.next();
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        sm.success = false;
        break;
      }
      feedback_.state = sm.state;
      feedback_.rung = sm.rung;
      as_.publishFeedback(feedback_);
      if (goal->waitingforhuman)
      {
        ROS_INFO_STREAM("2022_climb_server : Waiting for human...");
        // REPLACE WITH JOYSTICK TEMPLATE
        ROS_INFO_STREAM("2022_climb_server : Thank you human, proceeding :)");
      }
    }

    result_.success = sm.success;
    if (result_.success)
    {
      ROS_INFO("%s: Succeeded", action_name_.c_str());
    } else
    {
      ROS_INFO("%s: Failed", action_name_.c_str());
    }
    // set the action state to success or not
    as_.setSucceeded(result_);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "climb_server_2022");

  ClimbAction2022 climbAction("climb_server_2022");
  ros::spin();

  return 0;
}
