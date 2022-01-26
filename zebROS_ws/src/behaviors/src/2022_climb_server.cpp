#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "behavior_actions/Climb2022Action.h"
#include <functional>

class ClimbStateMachine
{
protected:
  ros::NodeHandle nh_;
  boost::function<void()> nextFunction_;
  bool waitForHuman_ = true;
  // other state data we have
public:
  uint8_t state = 0;
  uint8_t rung = 0; // 0 = ground, 1 = medium, 2 = high, 3 = traversal
  bool exited = false;
  ClimbStateMachine(bool waitForHuman)
  {
    waitForHuman_ = waitForHuman;
    nextFunction_ = boost::bind(&ClimbStateMachine::state1, this);
  }
  void next()
  {
    nextFunction_();
  }
  void state1()
  {
    state = 1;
    ROS_INFO_STREAM("State 1");
    ROS_INFO_STREAM("---");
    ROS_INFO_STREAM("Aligning...");
    ros::Duration(2.0).sleep(); // simulate aligning
    ROS_INFO_STREAM("Aligned");
    if (waitForHuman_)
    {
      ROS_INFO_STREAM("Waiting for human...");
      // REPLACE WITH JOYSTICK TEMPLATE
      ROS_INFO_STREAM("Thank you human, proceeding :)");
    }
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state2, this);
  }
  void state2()
  {
    state = 2;
    ROS_INFO_STREAM("State 2");
    ROS_INFO_STREAM("---");
    ROS_INFO_STREAM("Extending dynamic arm pistons");
    ros::Duration(0.8).sleep();
    ROS_INFO_STREAM("Extended dynamic arm pistons");
    if (waitForHuman_)
    {
      ROS_INFO_STREAM("Waiting for human...");
      // REPLACE WITH JOYSTICK TEMPLATE
      ROS_INFO_STREAM("Thank you human, proceeding :)");
    }
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state3, this);
  }
  void state3()
  {
    state = 3;
    ROS_INFO_STREAM("State 3");
    ROS_INFO_STREAM("---");
    ROS_INFO_STREAM("Raising dynamic arms fully");
    ros::Duration(1.5).sleep(); // simulate waiting for arms to be fully extended (limit switch? encoders?)
    ROS_INFO_STREAM("Raised dynamic arms fully");
    if (waitForHuman_)
    {
      ROS_INFO_STREAM("Waiting for human...");
      // REPLACE WITH JOYSTICK TEMPLATE
      ROS_INFO_STREAM("Thank you human, proceeding :)");
    }
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state4, this);
  }
  void state4()
  {
    state = 4;
    ROS_INFO_STREAM("State 4");
    ROS_INFO_STREAM("---");
    if (rung == 0)
    {
      ROS_INFO_STREAM("Driving Backwards");
      ros::Duration(1).sleep();
    } else
    {
      ROS_INFO_STREAM("Extending dynamic arm pistons to hit next rung");
      ros::Duration(0.8).sleep();
      ROS_INFO_STREAM("Extended dynamic arm pistons");
    }
    if (waitForHuman_)
    {
      ROS_INFO_STREAM("Waiting for human...");
      // REPLACE WITH JOYSTICK TEMPLATE
      ROS_INFO_STREAM("Thank you human, proceeding :)");
    }
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state5, this);
  }
  void state5()
  {
    state = 5;
    ROS_INFO_STREAM("State 5");
    ROS_INFO_STREAM("---");
    ROS_INFO_STREAM("Lowering dynamic arms a bit to touch rung");
    ros::Duration(1).sleep();
    ROS_INFO_STREAM("Lowered dynamic arms a bit to touch rung");
    if (waitForHuman_)
    {
      ROS_INFO_STREAM("Waiting for human...");
      // REPLACE WITH JOYSTICK TEMPLATE
      ROS_INFO_STREAM("Thank you human, proceeding :)");
    }
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state6, this);
  }
  void state6()
  {
    state = 6;
    ROS_INFO_STREAM("State 6");
    ROS_INFO_STREAM("---");
    ROS_INFO_STREAM("Detaching static hooks");
    ros::Duration(0.8).sleep();
    ROS_INFO_STREAM("Detached static hooks");
    if (rung != 0) {
      ROS_INFO_STREAM("Waiting for robot to stop swinging");
      ros::Duration(2).sleep();
      ROS_INFO_STREAM("Robot stopped swinging");
    }
    if (waitForHuman_)
    {
      ROS_INFO_STREAM("Waiting for human...");
      // REPLACE WITH JOYSTICK TEMPLATE
      ROS_INFO_STREAM("Thank you human, proceeding :)");
    }
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state7, this);
  }
  void state7()
  {
    state = 7;
    ROS_INFO_STREAM("State 7");
    ROS_INFO_STREAM("---");
    ROS_INFO_STREAM("Lowering dynamic arms fully");
    ros::Duration(1.5).sleep();
    rung++;
    ROS_INFO_STREAM("Lowered dynamic arms fully. Robot is fully supported by rung " << std::to_string(rung) << ".");
    if (waitForHuman_)
    {
      ROS_INFO_STREAM("Waiting for human...");
      // REPLACE WITH JOYSTICK TEMPLATE
      ROS_INFO_STREAM("Thank you human, proceeding :)");
    }
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state8, this);
  }
  void state8()
  {
    state = 8;
    ROS_INFO_STREAM("State 8");
    ROS_INFO_STREAM("---");
    ROS_INFO_STREAM("Attaching static hooks");
    ros::Duration(0.8).sleep();
    ROS_INFO_STREAM("Attached static hooks");
    if (rung == 3)
    {
      ROS_INFO_STREAM("Climb is done! Woohoo!!");
      exited = true;
      return;
    }
    if (waitForHuman_)
    {
      ROS_INFO_STREAM("Waiting for human...");
      // REPLACE WITH JOYSTICK TEMPLATE
      ROS_INFO_STREAM("Thank you human, proceeding :)");
    }
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state9, this);
  }
  void state9()
  {
    state = 9;
    ROS_INFO_STREAM("State 9");
    ROS_INFO_STREAM("---");
    ROS_INFO_STREAM("Raising dynamic arms slightly");
    ros::Duration(1).sleep();
    ROS_INFO_STREAM("Raised dynamic arms slightly");
    if (waitForHuman_)
    {
      ROS_INFO_STREAM("Waiting for human...");
      // REPLACE WITH JOYSTICK TEMPLATE
      ROS_INFO_STREAM("Thank you human, proceeding :)");
    }
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state10, this);
  }
  void state10()
  {
    state = 10;
    ROS_INFO_STREAM("State 10");
    ROS_INFO_STREAM("---");
    ROS_INFO_STREAM("Retracting dynamic arm pistons");
    ros::Duration(1).sleep();
    ROS_INFO_STREAM("Retracted dynamic arm pistons");
    if (waitForHuman_)
    {
      ROS_INFO_STREAM("Waiting for human...");
      // REPLACE WITH JOYSTICK TEMPLATE
      ROS_INFO_STREAM("Thank you human, proceeding :)");
    }
    ROS_INFO_STREAM("");
    nextFunction_ = boost::bind(&ClimbStateMachine::state3, this);
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
    bool success = true;
    ClimbStateMachine sm(true);
    // start executing the action
    while (!sm.exited)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      sm.next();
      feedback_.state = sm.state;
      feedback_.rung = sm.rung;
      as_.publishFeedback(feedback_);
    }

    if(success)
    {
      result_.success = true;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "climb_server_2022");

  ClimbAction2022 climbAction("climb_server_2022");
  ros::spin();

  return 0;
}
