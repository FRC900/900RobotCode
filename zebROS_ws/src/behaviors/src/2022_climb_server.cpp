#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "behavior_actions/Climb2022Action.h"
#include "controllers_2022_msgs/DynamicArmSrv.h"
#include <std_msgs/Float64.h>
#include "sensor_msgs/JointState.h"
#include <map>
#include "talon_state_msgs/TalonState.h"

// How to simulate this:
/*
First, make sure the `dynamic_arm_controller` is spawned
--ENTER DOCKER--
roslaunch controller_node 2022_compbot_combined.launch hw_or_sim:=sim
--in a separate terminal--
rosrun rqt_driver_station_sim rqt_driver_station_sim & # Enable reverse limit switch on the climber leader talon, disable it once the climb server node is running and good
rosrun actionlib axclient.py /climb_server_2022 &
--in a separate terminal--
roslaunch behaviors 2022_climb.launch
*/

// piston config
constexpr double DYNAMIC_ARM_UPRIGHT =  1.0;
constexpr double DYNAMIC_ARM_TILTED  = -1.0;
constexpr double STATIC_HOOK_OPEN    = -1.0;
constexpr double STATIC_HOOK_CLOSED  =  1.0;

class ClimbStateMachine
{
protected:
  ros::NodeHandle nh_;
  boost::function<void()> nextFunction_;
  ros::Publisher dynamic_arm_piston_;
  ros::Publisher static_hook_piston_;

  ros::Subscriber joint_states_sub_;
  ros::Subscriber talon_states_sub_;
  // s = static, ls = limit switch
  double s1_ls;
  double s2_ls;

  talon_state_msgs::TalonState talon_states_;

  double full_extend_height_ground_;
  double full_extend_height_air_;
  double static_hook_distance_above_rung_;
  double imbalance_timeout_; // seconds
  double static_hook_release_height_;
  double get_to_zero_percent_output_;
  double piston_wait_time_;
  double swinging_wait_time_;

  std::vector<boost::function<void()>> state_functions_;

  actionlib::SimpleActionServer<behavior_actions::Climb2022Action> &as_;
  ros::ServiceClient dynamic_arm_;
  // other state data we have
public:
  uint8_t state = 0;
  uint8_t rung = 0; // 0 = ground, 1 = medium, 2 = high, 3 = traversal
  bool exited = false;
  bool success = false; // whether it exited without errors
  ClimbStateMachine(actionlib::SimpleActionServer<behavior_actions::Climb2022Action> &as_): as_(as_)
  {
    nextFunction_ = boost::bind(&ClimbStateMachine::state1, this);
    state_functions_ = {boost::bind(&ClimbStateMachine::state1, this), boost::bind(&ClimbStateMachine::state2, this), boost::bind(&ClimbStateMachine::state3, this), boost::bind(&ClimbStateMachine::state4, this), boost::bind(&ClimbStateMachine::state5, this), boost::bind(&ClimbStateMachine::state6, this), boost::bind(&ClimbStateMachine::state7, this), boost::bind(&ClimbStateMachine::state8, this)};
    dynamic_arm_piston_ = nh_.advertise<std_msgs::Float64>("/frcrobot_jetson/dynamic_arm_solenoid_controller/command", 2);
    static_hook_piston_ = nh_.advertise<std_msgs::Float64>("/frcrobot_jetson/static_hook_solenoid_controller/command", 2);
    joint_states_sub_ = nh_.subscribe("/frcrobot_jetson/joint_states", 1, &ClimbStateMachine::jointStateCallback, this);
    talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states", 1, &ClimbStateMachine::talonStateCallback, this);
	const std::map<std::string, std::string> service_connection_header{ {"tcp_nodelay", "1"} };
    dynamic_arm_ = nh_.serviceClient<controllers_2022_msgs::DynamicArmSrv>("/frcrobot_jetson/dynamic_arm_controller/command", false, service_connection_header);
    if (!nh_.getParam("full_extend_height_ground", full_extend_height_ground_))
    {
      ROS_ERROR_STREAM("2022_climb_server : Could not find full_extend_height_ground");
      success = false;
      exited = true;
      return;
    }
    if (!nh_.getParam("full_extend_height_air", full_extend_height_air_))
    {
      ROS_ERROR_STREAM("2022_climb_server : Could not find full_extend_height_air");
      success = false;
      exited = true;
      return;
    }
    if (!nh_.getParam("static_hook_release_height", static_hook_release_height_)) {
      ROS_ERROR_STREAM("2022_climb_server : Could not find static_hook_release_height");
      success = false;
      exited = true;
      return;
    }
    if (!nh_.getParam("static_hook_distance_above_rung", static_hook_distance_above_rung_))
    {
      ROS_ERROR_STREAM("2022_climb_server : Could not find static_hook_distance_above_rung");
      success = false;
      exited = true;
      return;
    }
    if (!nh_.getParam("get_to_zero_percent_output", get_to_zero_percent_output_))
    {
      // Used if we tell motion magic to go to 0 and we haven't hit the limit switch yet -- we keep going down at this speed until we hit the limit switch.
      // **Must be negative.**
      ROS_ERROR_STREAM("2022_climb_server : Could not find get_to_zero_percent_output");
      success = false;
      exited = true;
      return;
    }
    if (!nh_.getParam("imbalance_timeout", imbalance_timeout_))
    {
      ROS_WARN_STREAM("2022_climb_server : Could not find imbalance_timeout, defaulting to 2.0 seconds");
      imbalance_timeout_ = 2.0;
    }
    if (!nh_.getParam("piston_wait_time", piston_wait_time_))
    {
      ROS_WARN_STREAM("2022_climb_server : Could not find piston_wait_time, defaulting to 0.8 seconds");
      piston_wait_time_ = 0.8;
    }
    if (!nh_.getParam("swinging_wait_time", swinging_wait_time_))
    {
      ROS_WARN_STREAM("2022_climb_server : Could not find swinging_wait_time, defaulting to 1 second");
      swinging_wait_time_ = 1.0;
    }
    dynamic_arm_.waitForExistence();
  }
  void reset(bool singleStep, uint8_t start_state) {
    if (!singleStep) {
      state = 0;
      nextFunction_ = boost::bind(&ClimbStateMachine::state1, this);
      rung = 0;
    } else if (start_state != 0 && start_state <= state_functions_.size()) {
      state = start_state;
      nextFunction_ = state_functions_[start_state - 1];
    }
    exited = false;
    success = false;
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
        exited = true;
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
    ROS_INFO_STREAM("2022_climb_server : Aligning and opening static hooks...");
    if (sleepCheckingForPreempt(2.0)) return; // simulate aligning
    std_msgs::Float64 spMsg;
    spMsg.data = STATIC_HOOK_OPEN;
    static_hook_piston_.publish(spMsg);
    if (sleepCheckingForPreempt(piston_wait_time_)) return; // wait for pistons
    ROS_INFO_STREAM("2022_climb_server : Aligned and opened");
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
      exited = true;
      return;
    }
    std_msgs::Float64 dpMsg;
    dpMsg.data = DYNAMIC_ARM_UPRIGHT;
    dynamic_arm_piston_.publish(dpMsg);
    if (sleepCheckingForPreempt(piston_wait_time_)) return;
    ROS_INFO_STREAM("2022_climb_server : Extended dynamic arm pistons");
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state3, this);
    // TODO move zeroing to a service call and zero the arm here
  }
  void state3()
  {
    state = 3;
    ROS_INFO_STREAM("2022_climb_server : State 3");
    ROS_INFO_STREAM("2022_climb_server : ---");
    ROS_INFO_STREAM("2022_climb_server : Raising dynamic arms fully");
    controllers_2022_msgs::DynamicArmSrv srv;
    srv.request.use_percent_output = false; // motion magic
    srv.request.data = rung == 0 ? full_extend_height_ground_ : full_extend_height_air_;
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
    while(fabs(talon_states_.position[leaderIndex] - srv.request.data) > 0.05) // wait
    {
      r.sleep();
      ros::spinOnce();
      if (as_.isPreemptRequested() || !ros::ok()) {
        exited = true;
        return;
      }
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
      if (sleepCheckingForPreempt(1)) return; // TODO replace with drivetrain stuff
    } else
    {
      ROS_INFO_STREAM("2022_climb_server : Extending dynamic arm pistons to hit next rung");
      ros::spinOnce();
      if (as_.isPreemptRequested() || !ros::ok()) {
        exited = true;
        return;
      }
      std_msgs::Float64 dpMsg;
      dpMsg.data = DYNAMIC_ARM_UPRIGHT;
      dynamic_arm_piston_.publish(dpMsg);
      if (sleepCheckingForPreempt(piston_wait_time_)) return;
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
    ROS_INFO_STREAM("2022_climb_server : Lowering dynamic arms fully while releasing static hooks");
    controllers_2022_msgs::DynamicArmSrv srv;
    srv.request.use_percent_output = false; // motion magic
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
    bool opened_hooks = false;
    while (fabs(talon_states_.speed[leaderIndex]) < fabs(get_to_zero_percent_output_)) {
      r.sleep();
      ros::spinOnce();
      // wait for motors to go up to speed to not fail check in <this line + 6 lines>
    }
    while(!talon_states_.reverse_limit_switch[leaderIndex]) // wait
    {
      ROS_INFO_STREAM_THROTTLE(0.25, "2022_climb_server : waiting to hit dynamic arm limit switches");
      r.sleep();
      ros::spinOnce();
      if (fabs(talon_states_.speed[leaderIndex]) < fabs(get_to_zero_percent_output_)) {
        // if we have not hit the limit switch yet and we have stopped, keep going down slowly
        srv.request.use_percent_output = true;
        srv.request.data = -fabs(get_to_zero_percent_output_);
        srv.request.go_slow = false;
        dynamic_arm_.call(srv);
      }
      if (!opened_hooks && (talon_states_.position[leaderIndex] <= static_hook_release_height_) && !s1_ls && !s2_ls) { // if hooks haven't been opened, height < hook release height, and both hooks aren't touching anything,
        // open hooks
        std_msgs::Float64 spMsg;
        spMsg.data = STATIC_HOOK_OPEN;
        static_hook_piston_.publish(spMsg);
        ROS_INFO_STREAM("2022_climb_server : Detached static hooks.");
        ROS_INFO_STREAM("");
        opened_hooks = true;
      }
      if (as_.isPreemptRequested() || !ros::ok()) {
        exited = true;
        return;
      }
    }
    srv.request.use_percent_output = false;
    srv.request.data = 0;
    srv.request.go_slow = false;
    dynamic_arm_.call(srv);
    ROS_INFO_STREAM("2022_climb_server : called dynamic arm service to hold up in air.");
    rung++;
    ROS_INFO_STREAM("2022_climb_server : Lowered dynamic arms fully and released static hooks. Robot is fully supported by rung " << std::to_string(rung) << ".");
    if (rung == 3)
    {
      ROS_INFO_STREAM("2022_climb_server : Climb is done! Woohoo!!");
      success = true;
      exited = true;
      state = 0;
      rung = 0;
      nextFunction_ = boost::bind(&ClimbStateMachine::state1, this);
      ROS_INFO_STREAM("2022_climb_server : RESET STATE");
      return;
    }
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state6, this);
  }
  void state6()
  {
    state = 6;
    ROS_INFO_STREAM("2022_climb_server : State 6");
    ROS_INFO_STREAM("2022_climb_server : ---");
    ROS_INFO_STREAM("2022_climb_server : Attaching static hooks");
    ros::spinOnce();
    if (as_.isPreemptRequested() || !ros::ok()) {
      exited = true;
      return;
    }
    std_msgs::Float64 spMsg;
    spMsg.data = STATIC_HOOK_CLOSED;
    static_hook_piston_.publish(spMsg);
    if (sleepCheckingForPreempt(piston_wait_time_)) return;
    ROS_INFO_STREAM("2022_climb_server : Attached static hooks");
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state7, this);
  }
  void state7()
  {
    state = 7;
    ROS_INFO_STREAM("2022_climb_server : State 7");
    ROS_INFO_STREAM("2022_climb_server : ---");
    ROS_INFO_STREAM("2022_climb_server : Raising dynamic arms slightly to hang by static hooks");
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
    srv.request.data = talon_states_.position[leaderIndex] + static_hook_distance_above_rung_;
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

    ROS_INFO_STREAM("2022_climb_server : waiting for static hooks to be attached");
    ros::Rate r(100); // 100 Hz loop
    int counter = 0;
    while (!(s1_ls && s2_ls)) {
      ROS_INFO_STREAM_THROTTLE(0.25, "2022_climb_server : waiting to hit static hook limit switches");
      ros::spinOnce();
      if ((s1_ls == 1) ^ (s2_ls == 1)) { // if one hook has touched but the other has not,
        if (counter >= imbalance_timeout_ * 100) { // and it has been two seconds since the robot was imbalanced,
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

    ROS_INFO_STREAM("2022_climb_server : Raised dynamic arms slightly to hang by static hooks");
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state8, this);
  }
  void state8()
  {
    state = 8;
    ROS_INFO_STREAM("2022_climb_server : State 8");
    ROS_INFO_STREAM("2022_climb_server : ---");
    ROS_INFO_STREAM("2022_climb_server : Retracting dynamic arm pistons");
    ros::spinOnce();
    if (as_.isPreemptRequested() || !ros::ok()) {
      exited = true;
      return;
    }
    std_msgs::Float64 dpMsg;
    dpMsg.data = DYNAMIC_ARM_TILTED;
    dynamic_arm_piston_.publish(dpMsg);
    if (sleepCheckingForPreempt(piston_wait_time_)) return;
    ROS_INFO_STREAM("2022_climb_server : Retracted dynamic arm pistons");
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state3, this);
  }
  void jointStateCallback(const sensor_msgs::JointState joint_state)
  {
    std::map<std::string, double*> stateNamesToVariables = {{"climber_static1_limit_switch", &s1_ls}, {"climber_static2_limit_switch", &s2_ls}};
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
  void stopMotors() {
    // TODO may need to add stop condition to controller to make robot stay up
    // Construct service call
    controllers_2022_msgs::DynamicArmSrv srv;
    srv.request.use_percent_output = true; // percent output
    srv.request.data = 0;
    srv.request.go_slow = false; // irrelevant

    // Call service
    if (dynamic_arm_.call(srv))
    {
      ROS_INFO_STREAM("2022_climb_server : called dynamic arm service to stop motors.");
    }
    else
    {
      ROS_ERROR_STREAM("2022_climb_server : failed to call dynamic arm service.");
    }
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
  ClimbStateMachine sm;

public:

  ClimbAction2022(std::string name) :
    as_(nh_, name, boost::bind(&ClimbAction2022::executeCB, this, _1), false),
    action_name_(name),
    sm(as_)
  {
    as_.start();
  }

  ~ClimbAction2022(void)
  {
  }

  void executeCB(const behavior_actions::Climb2022GoalConstPtr &goal)
  {
    sm.reset(goal->single_step, goal->start_state);
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
      if (goal->single_step)
      {
        break;
      }
    }

    sm.success = sm.success || !sm.exited;
    if ((!goal->single_step) || (!sm.success))
    {
      sm.stopMotors();
    }
    result_.success = sm.success;
    if (result_.success)
    {
      ROS_INFO("%s: Succeeded", action_name_.c_str());
    } else
    {
      sm.reset(false, 0);
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
