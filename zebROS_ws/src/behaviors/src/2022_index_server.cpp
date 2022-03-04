#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "behavior_actions/Index2022Action.h"
#include <std_msgs/Float64.h>
#include "sensor_msgs/JointState.h"
#include <map>

class IndexStateMachine
{
protected:
  double straight_motor_percent_output_config_;
  double arc_motor_percent_output_config_;
  ros::Subscriber joint_states_sub_;
  ros::Publisher straight_motor_publisher_;
  ros::Publisher arc_motor_publisher_;
  ros::NodeHandle nh_;
  ros::Publisher ball_state_publisher_;
  boost::function<void()> nextFunction_;
  actionlib::SimpleActionServer<behavior_actions::Index2022Action> &as_;
  bool arc_sensor_pressed_;
  bool straight_sensor_pressed_;
  double straight_sensor_pressed_double_;
  double arc_sensor_pressed_double_;
  ros::Timer publish_ball_count_caller_ = nh_.createTimer(ros::Rate(5),&IndexStateMachine::publish_ball_count, this);

public:
  uint8_t state_ = 0;
  bool exited_ = false;
  bool success_ = false; // whether it exited without errors
  uint8_t num_of_balls_ = 0;
  uint8_t goal_;


  IndexStateMachine(actionlib::SimpleActionServer<behavior_actions::Index2022Action> &as_): as_(as_)
  {
    joint_states_sub_ = nh_.subscribe("/frcrobot_jetson/joint_states", 1, &IndexStateMachine::jointStateCallback, this);
    straight_motor_publisher_ = nh_.advertise<std_msgs::Float64>("/frcrobot_jetson/indexer_straight_motor_controller/command", 1);
    arc_motor_publisher_ = nh_.advertise<std_msgs::Float64>("/frcrobot_jetson/indexer_arc_motor_controller/command", 1);
    ball_state_publisher_ = nh_.advertise<std_msgs::Float64>("/2022_index_server/ball_state", 1);
    ros::NodeHandle indexer_params_nh(nh_, "indexer_actionlib_params");

  	if (!indexer_params_nh.getParam("straight_motor_percent_output", straight_motor_percent_output_config_))
  	{
  		ROS_ERROR("could not read straight_motor_percent_output");
  		straight_motor_percent_output_config_ = 0.5;
  	}
    if (!indexer_params_nh.getParam("arc_motor_percent_output", arc_motor_percent_output_config_))
  	{
  		ROS_ERROR("could not read arc_motor_percent_output");
  		arc_motor_percent_output_config_ = 0.5;
  	}
  }
  void publish_ball_count(const ros::TimerEvent&){
    if(straight_sensor_pressed_ && arc_sensor_pressed_){
      num_of_balls_= 2;
    } else if(straight_sensor_pressed_ || arc_sensor_pressed_){
      num_of_balls_ = 1;
    } else{
      num_of_balls_ = 0;
    }
    std_msgs::Float64 num_of_balls_msg;
    num_of_balls_msg.data = num_of_balls_;
    ball_state_publisher_.publish(num_of_balls_msg);
  }
  void run_straight_motor(double percent_output){
    std_msgs::Float64 straight_motor_percent_output;
    straight_motor_percent_output.data = percent_output;
    straight_motor_publisher_.publish(straight_motor_percent_output);
  }
  void run_arc_motor(double percent_output){
    std_msgs::Float64 arc_motor_percent_output;
    arc_motor_percent_output.data = percent_output;
    arc_motor_publisher_.publish(arc_motor_percent_output);
  }
  void reset() {
    nextFunction_ = boost::bind(&IndexStateMachine::state0, this);
    exited_ = false;
    success_ = false;
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
        exited_ = true;
        return true;
      }
      r.sleep();
    }
    return false;
  }

  void state0() //decides which state to go to based on sensor readings
  {
    state_ = 0;
    if(goal_ == behavior_actions::Index2022Goal::EJECT){
      nextFunction_ = boost::bind(&IndexStateMachine::state3, this); //call eject state
      return;
    }
    if(goal_ == behavior_actions::Index2022Goal::MOVE_TO_SHOOTER){
      nextFunction_ = boost::bind(&IndexStateMachine::state4, this); //call move to shooter state
      return;
    }
    if(arc_sensor_pressed_ && straight_sensor_pressed_){
      exited_ = true;
      return;
    }
    if(arc_sensor_pressed_){
      nextFunction_ = boost::bind(&IndexStateMachine::state10, this);
      return;
    }
    else{
      nextFunction_ = boost::bind(&IndexStateMachine::state1, this);
      return;
    }
  }
  void state1() //indexer is empty
  {
    state_ = 1;
    run_straight_motor(straight_motor_percent_output_config_);
    ros::Rate r(20);
    while(!arc_sensor_pressed_){ //TODO set up timeout or do it in higher level code
      r.sleep();
      ros::spinOnce();
    }
    //if timeout was reached, set num_of_balls to 0
    nextFunction_ = boost::bind(&IndexStateMachine::state10, this);
  }
  void state2() //state to exit
  {
    exited_ = true;
  }
  void state3() //eject state
  {
    state_ = 3;
    // run straight motor backwards
    run_straight_motor(-straight_motor_percent_output_config_);
    // run arc motor backwards
    run_arc_motor(-arc_motor_percent_output_config_);
    ros::Rate r(20);
    while(arc_sensor_pressed_ || straight_sensor_pressed_){
      r.sleep();
      ros::spinOnce();
    }
    nextFunction_ = boost::bind(&IndexStateMachine::state2, this);
  }
  void state4() //move to shooter state
  {
    state_ = 4;
    // run straight motor backwards
    run_straight_motor(-straight_motor_percent_output_config_);
    run_arc_motor(-arc_motor_percent_output_config_);
    // run arc motor backwards
    if(straight_sensor_pressed_ && arc_sensor_pressed_){
      ros::Rate r(20);
      while(straight_sensor_pressed_){
        r.sleep();
        ros::spinOnce();
      }
    }
    else{
      ros::Rate r(20);
      while(arc_sensor_pressed_){
        r.sleep();
        ros::spinOnce();
      }
      // wait one second
      ros::Duration(1.0).sleep();
    }

    nextFunction_ = boost::bind(&IndexStateMachine::state2, this);
  }
  void state10() //one existing ball
  {
    state_ = 10;
    run_straight_motor(straight_motor_percent_output_config_);
    ros::Rate r(20);
    while(arc_sensor_pressed_ && !straight_sensor_pressed_){
      r.sleep();
      ros::spinOnce();
    }
    nextFunction_ = boost::bind(&IndexStateMachine::state2, this);
  }

  void jointStateCallback(const sensor_msgs::JointState joint_state)
  { //TODO replace state names to whatever they are actually named
    std::map<std::string, double*> stateNamesToVariables = {{"indexer_straight_linebreak", &straight_sensor_pressed_double_}, {"indexer_arc_linebreak", &arc_sensor_pressed_double_}};

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
        ROS_WARN_STREAM_THROTTLE(2.0, "2022_index_server : " << nameVar.first << " sensor not found in joint_states");
      }
    }

    if(straight_sensor_pressed_double_ == 1.0){
      straight_sensor_pressed_ = true; //assuming 1 means that it is pressed
    } else{
      straight_sensor_pressed_ = false;
    }

    if(arc_sensor_pressed_double_ == 1.0){
      arc_sensor_pressed_ = true; //assuming 1 means that it is pressed
    } else{
      arc_sensor_pressed_ = false;
    }
  }
  void stopMotors() {
    //Stop straight motor
    run_straight_motor(0);

    //Stop arc motor
    run_arc_motor(0);
}
};

class IndexAction2022
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<behavior_actions::Index2022Action> as_;
  std::string action_name_;
  // create messages that are used to publish feedback/result
  behavior_actions::Index2022Feedback feedback_;
  behavior_actions::Index2022Result result_;
  IndexStateMachine sm;

public:

  IndexAction2022(std::string name) :
    as_(nh_, name, boost::bind(&IndexAction2022::executeCB, this, _1), false),
    action_name_(name),
    sm(as_)
  {
    as_.start();
  }

  ~IndexAction2022(void)
  {
  }

  void executeCB(const behavior_actions::Index2022GoalConstPtr &goal)
  {
    sm.goal_ = goal->goal;
    // start executing the action
    while (!sm.exited_)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        sm.success_ = false;
        break;
      }
      sm.next();
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        sm.success_ = false;
        break;
      }
      feedback_.num_of_balls = sm.num_of_balls_;
      as_.publishFeedback(feedback_);
    }

    sm.success_ = sm.success_ || !sm.exited_;
    if (!sm.success_)
    {
      sm.stopMotors();
    }
    result_.success = sm.success_;
    if (result_.success)
    {
      ROS_INFO("%s: Succeeded", action_name_.c_str());
    } else
    {
      sm.reset();
      ROS_INFO("%s: Failed", action_name_.c_str());
    }
    // set the action state to success or not
    as_.setSucceeded(result_);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "index_server_2022");

  IndexAction2022 IndexAction("index_server_2022");
  ros::spin();

  return 0;
}
