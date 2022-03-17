#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <behavior_actions/Ejecting2022Action.h>
#include <behavior_actions/Intake2022Action.h>
#include <behavior_actions/Index2022Action.h>
#include <behavior_actions/Shooter2022Action.h>
#include "sensor_msgs/JointState.h"

class Ejecting2022Server
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<behavior_actions::Ejecting2022Action> as_;
  std::string action_name_;

  double server_timeout_;
  double shooting_timeout_;
  double wait_time_;

  behavior_actions::Ejecting2022Result result_;

  ros::Subscriber joint_states_sub_;
  bool cargo_in_straight_;
  bool cargo_in_arc_;

  // these lines of code show how these mechanisms are oriented on the robot
  actionlib::SimpleActionClient<behavior_actions::Shooter2022Action> ac_shooter_;
  actionlib::SimpleActionClient<behavior_actions::Index2022Action> ac_indexer_;
  actionlib::SimpleActionClient<behavior_actions::Intake2022Action> ac_intake_;

public:

  Ejecting2022Server(std::string name) :
    as_(nh_, name, boost::bind(&Ejecting2022Server::executeCB, this, _1), false),
    action_name_(name),
    ac_shooter_("/shooter/shooter_server_2022", true),
    ac_indexer_("/index/index_server_2022", true),
    ac_intake_("/intake/intake_server_2022", true)
  {
    if (!nh_.getParam("server_timeout", server_timeout_)) {
			ROS_WARN_STREAM("2022_ejecting_server : could not find server_timeout, defaulting to 10 seconds");
			server_timeout_ = 10;
		}
    if (!nh_.getParam("shooting_timeout", shooting_timeout_)) {
			ROS_WARN_STREAM("2022_ejecting_server : could not find shooting_timeout, defaulting to 10 seconds");
			shooting_timeout_ = 10;
		}
    if (!nh_.getParam("wait_time_before_stopping", wait_time_)) {
      // the time to wait after the indexer outputs the cargo before we stop the intake or shooter
      ROS_WARN_STREAM("2022_ejecting_server : could not find wait_time_before_stopping, defaulting to 0.5 seconds");
			wait_time_ = 0.5;
    }
    joint_states_sub_ = nh_.subscribe("/frcrobot_jetson/joint_states", 1, &Ejecting2022Server::jointStateCallback, this);
    as_.start();
  }

  ~Ejecting2022Server(void)
  {
  }

  template<class C, class S>
	bool waitForResultAndCheckForPreempt(const ros::Duration & timeout, const actionlib::SimpleActionClient<C> & ac, actionlib::SimpleActionServer<S> & as)
	{
		bool negative_timeout = false;
		if (timeout < ros::Duration(0, 0)) {
			ROS_WARN("waitForResultAndCheckForPreempt : Negative timeout, waiting forever");
			negative_timeout = true;
		}

		ros::Time timeout_time = ros::Time::now() + timeout;

		ros::Rate r(100);

		while (ros::ok() && !as.isPreemptRequested()) {
			ros::spinOnce();
			// Determine how long we should wait
			ros::Duration time_left = timeout_time - ros::Time::now();

			// Check if we're past the timeout time
			if (timeout > ros::Duration(0, 0) && time_left <= ros::Duration(0, 0) && !negative_timeout) {
				break;
			}

			if (ac.getState().isDone()) {
				break;
			}
			r.sleep();
		}

		return ac.getState().isDone();
	}

  void executeCB(const behavior_actions::Ejecting2022GoalConstPtr &goal)
  {
    result_.ejected_top_cargo = false;
    result_.ejected_bottom_cargo = false;

    if (!ac_shooter_.waitForServer(ros::Duration(server_timeout_))) {
      ROS_ERROR_STREAM("2022_ejecting_server : timed out connecting to shooter server, aborting");
      result_.timed_out = true;
      as_.setAborted(result_);
      return;
    }

    if (!ac_indexer_.waitForServer(ros::Duration(server_timeout_))) {
      ROS_ERROR_STREAM("2022_ejecting_server : timed out connecting to indexer server, aborting");
      result_.timed_out = true;
      as_.setAborted(result_);
      return;
    }

    if (!ac_intake_.waitForServer(ros::Duration(server_timeout_))) {
      ROS_ERROR_STREAM("2022_ejecting_server : timed out connecting to intake server, aborting");
      result_.timed_out = true;
      as_.setAborted(result_);
      return;
    }

    ros::spinOnce();

    if (goal->eject_bottom_cargo) {
      ROS_INFO_STREAM("2022_ejecting_server : ejecting bottom cargo");
      // check if indexer has a bottom cargo. if not, abort.
      if (!cargo_in_straight_) {
        ROS_INFO_STREAM("2022_ejecting_server : no cargo at the bottom of the indexer!");
        result_.timed_out = false;
        as_.setAborted(result_);
        ac_indexer_.cancelGoalsAtAndBeforeTime(ros::Time::now());
        return;
      }
      ROS_INFO_STREAM("2022_ejecting_server : calling intake, reverse=true, go_fast=" << (goal->speedy?"true":"false"));
      behavior_actions::Intake2022Goal intakeGoal;
      intakeGoal.reverse = true;
      intakeGoal.go_fast = goal->speedy;
      ac_intake_.sendGoal(intakeGoal);
      ROS_INFO_STREAM("2022_ejecting_server : calling indexer, goal=EJECT");
      behavior_actions::Index2022Goal indexerGoal;
      indexerGoal.goal = indexerGoal.EJECT;
      ac_indexer_.sendGoal(indexerGoal);
      if (!waitForResultAndCheckForPreempt(ros::Duration(-1), ac_indexer_, as_)) {
        ROS_INFO_STREAM("2022_ejecting_server : indexer timed out, aborting");
        result_.timed_out = true;
        as_.setAborted(result_);
        ac_indexer_.cancelGoalsAtAndBeforeTime(ros::Time::now());
        return;
      }
      ros::Duration(wait_time_).sleep();
      ac_intake_.cancelGoalsAtAndBeforeTime(ros::Time::now());
      ac_indexer_.cancelGoalsAtAndBeforeTime(ros::Time::now());
      if (ros::ok()) {
        result_.ejected_bottom_cargo = true;
      }
    }

    if (goal->eject_top_cargo) {
      ROS_INFO_STREAM("2022_ejecting_server : ejecting top cargo");
      // check if indexer has a top cargo. if not, abort.
      if (!cargo_in_arc_) {
        ROS_INFO_STREAM("2022_ejecting_server : no cargo at the top of the indexer!");
        result_.timed_out = false;
        as_.setAborted(result_);
        ac_indexer_.cancelGoalsAtAndBeforeTime(ros::Time::now());
        return;
      }
      ROS_INFO_STREAM("2022_ejecting_server : calling shooter, mode=EJECT");
      behavior_actions::Shooter2022Goal shooterGoal;
      shooterGoal.mode = shooterGoal.EJECT;
      bool isSpinningFast = false;
      ac_shooter_.sendGoal(shooterGoal,
                           actionlib::SimpleActionClient<behavior_actions::Shooter2022Action>::SimpleDoneCallback(),
                           actionlib::SimpleActionClient<behavior_actions::Shooter2022Action>::SimpleActiveCallback(),
                           [&isSpinningFast](const behavior_actions::Shooter2022FeedbackConstPtr &feedback){ isSpinningFast = feedback->close_enough; });
      ros::Rate r(100);
      ros::Time start = ros::Time::now();
      while (ros::ok() && !isSpinningFast) {
        if ((ros::Time::now() - start).toSec() >= shooting_timeout_) {
          ROS_INFO_STREAM("2022_ejecting_server : shooter timed out :(");
          result_.timed_out = true;
          ac_shooter_.cancelGoalsAtAndBeforeTime(ros::Time::now());
          return;
        }
        if (as_.isPreemptRequested()) {
          ROS_INFO_STREAM("2022_ejecting_server : preempted");
          result_.timed_out = false;
          as_.setAborted(result_);
          ac_shooter_.cancelGoalsAtAndBeforeTime(ros::Time::now());
          return;
        }
        ros::spinOnce();
        r.sleep();
      }
      ROS_INFO_STREAM("2022_ejecting_server : calling indexer, goal=EJECT");
      behavior_actions::Index2022Goal indexerGoal;
      indexerGoal.goal = indexerGoal.MOVE_TO_SHOOTER;
      ac_indexer_.sendGoal(indexerGoal);
      if (!waitForResultAndCheckForPreempt(ros::Duration(-1), ac_indexer_, as_)) {
        ROS_INFO_STREAM("2022_ejecting_server : indexer timed out, aborting");
        result_.timed_out = true;
        as_.setAborted(result_);
        ac_indexer_.cancelGoalsAtAndBeforeTime(ros::Time::now());
        return;
      }
      ros::Duration(wait_time_).sleep();
      ac_shooter_.cancelGoalsAtAndBeforeTime(ros::Time::now());
      ac_indexer_.cancelGoalsAtAndBeforeTime(ros::Time::now());
      if (ros::ok()) {
        result_.ejected_top_cargo = true;
      }
    }

    ROS_INFO_STREAM("2022_ejecting_server : succeeded :)");

    as_.setSucceeded(result_);

    // Stop shooter, intake, and indexer (maybe)
  }

  void jointStateCallback(const sensor_msgs::JointState joint_state)
  {
    std::map<std::string, bool*> stateNamesToVariables = {{"indexer_straight_linebreak", &cargo_in_straight_}, {"indexer_arc_linebreak", &cargo_in_arc_}};

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
        ROS_WARN_STREAM_THROTTLE(2.0, "2022_ejecting_server : " << nameVar.first << " sensor not found in joint_states");
      }
    }

  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ejecting_server_2022");

  Ejecting2022Server server("ejecting_server_2022");
  ros::spin();

  return 0;
}
