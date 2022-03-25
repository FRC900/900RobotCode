// Spin up shooter
// Wait for shooter
// Get cargo from indexer
// Repeat `num_cargo` times
// AS OF NOW, DO NOT RUN TWO CARGO AT ONCE
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <behavior_actions/Shooting2022Action.h>
#include <behavior_actions/Shooter2022Action.h>
#include <behavior_actions/Index2022Action.h>
#include <std_msgs/Float64.h>

class ShootingServer2022
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<behavior_actions::Shooting2022Action> as_;
  std::string action_name_;
  // create messages that are used to publish feedback/result
  behavior_actions::Shooting2022Feedback feedback_;

  behavior_actions::Shooting2022Result result_;
  actionlib::SimpleActionClient<behavior_actions::Shooter2022Action> ac_shooter_;
  actionlib::SimpleActionClient<behavior_actions::Index2022Action> ac_indexer_;

  double shooting_timeout_;
  double indexing_timeout_;

  ros::Subscriber cargo_state_sub_;
  uint8_t cargo_num_;

  bool is_spinning_fast_;

public:

  ShootingServer2022(std::string name) :
    as_(nh_, name, boost::bind(&ShootingServer2022::executeCB, this, _1), false),
    action_name_(name),
    ac_shooter_("/shooter/shooter_server_2022", true),
    ac_indexer_("/index/index_server_2022", true)
  {
    if (!nh_.getParam("shooting_timeout", shooting_timeout_))
    {
      ROS_WARN_STREAM("2022_shooting_server : Could not find shooting_timeout, defaulting to 10 seconds");
      shooting_timeout_ = 10.0;
    }
    if (!nh_.getParam("indexing_timeout", shooting_timeout_))
    {
      ROS_WARN_STREAM("2022_shooting_server : Could not find indexing_timeout, defaulting to 5 seconds");
      indexing_timeout_ = 5.0;
    }
    cargo_state_sub_ = nh_.subscribe("/2022_index_server/ball_state", 2, &ShootingServer2022::cargoStateCallback, this);
    as_.start();
  }

  ~ShootingServer2022(void)
  {
  }

  template<class C, class S>
	bool waitForResultAndCheckForPreempt(const ros::Duration & timeout, const actionlib::SimpleActionClient<C> & ac, actionlib::SimpleActionServer<S> & as)
	{
		if (timeout < ros::Duration(0, 0)) {
			ROS_WARN_NAMED("actionlib", "Timeouts can't be negative. Timeout is [%.2fs]", timeout.toSec());
		}

		ros::Time timeout_time = ros::Time::now() + timeout;

		ros::Rate r(100);

		while (ros::ok() && !as.isPreemptRequested()) {
			ros::spinOnce();
			// Determine how long we should wait
			ros::Duration time_left = timeout_time - ros::Time::now();

			// Check if we're past the timeout time
			if (timeout > ros::Duration(0, 0) && time_left <= ros::Duration(0, 0) ) {
				break;
			}

			if (ac.getState().isDone()) {
				break;
			}
			r.sleep();
		}

		return ac.getState().isDone();
	}

  void cargoStateCallback(const std_msgs::Float64 &msg)
  {
    cargo_num_ = msg.data;
  }

  bool spinUpShooter(bool &timedOut, bool low_goal) { // returns false if ros is not ok or timed out, true otherwise
    feedback_.state = feedback_.WAITING_FOR_SHOOTER;
    as_.publishFeedback(feedback_);
    ROS_INFO_STREAM("2022_shooting_server : spinning up shooter");
    is_spinning_fast_ = false;
    behavior_actions::Shooter2022Goal goal;
    goal.mode = low_goal ? goal.LOW_GOAL : goal.HIGH_GOAL;
    ac_shooter_.sendGoal(goal,
                         actionlib::SimpleActionClient<behavior_actions::Shooter2022Action>::SimpleDoneCallback(),
                         actionlib::SimpleActionClient<behavior_actions::Shooter2022Action>::SimpleActiveCallback(),
                         [&](const behavior_actions::Shooter2022FeedbackConstPtr &feedback){ ROS_INFO_STREAM_THROTTLE(0.1, "2022_shooting_server : new sample data"); is_spinning_fast_ = feedback->close_enough; });
    ros::Rate r(100);
    ros::Time start = ros::Time::now();
    while (ros::ok() && !is_spinning_fast_) {
      if ((ros::Time::now() - start).toSec() >= shooting_timeout_) {
        ROS_INFO_STREAM("2022_shooting_server : shooter timed out :(");
        timedOut = true;
        return false;
      }
      if (as_.isPreemptRequested()) {
        ROS_INFO_STREAM("2022_shooting_server : preempted");
        return false;
      }
      ros::spinOnce();
      r.sleep();
    }
    ROS_INFO_STREAM("2022_shooting_server : spun up shooter");
    return ros::ok();
  }

  bool waitForShooter(bool &timedOut) {
    feedback_.state = feedback_.WAITING_FOR_SHOOTER;
    as_.publishFeedback(feedback_);
    ros::Rate r(100);
    ros::Time start = ros::Time::now();
    while (ros::ok() && !is_spinning_fast_) {
      ROS_INFO_STREAM_THROTTLE(0.1, "2022_shooting_server : waiting for shooter");
      if ((ros::Time::now() - start).toSec() >= shooting_timeout_) {
        ROS_INFO_STREAM("2022_shooting_server : wait for shooter timed out :(");
        timedOut = true;
        return false;
      }
      if (as_.isPreemptRequested()) {
        ROS_INFO_STREAM("2022_shooting_server : preempted");
        return false;
      }
      ros::spinOnce();
      r.sleep();
    }
    ROS_INFO_STREAM("2022_shooting_server : shooter is spinning fast enough, continuing");
    return true;
  }

  bool getCargoFromIndexer(bool &timedOut) { // returns false if ros is not ok or timed out, true otherwise
    feedback_.state = feedback_.WAITING_FOR_INDEXER;
    as_.publishFeedback(feedback_);
    ROS_INFO_STREAM("2022_shooting_server : getting cargo from indexer");

    behavior_actions::Index2022Goal goal;
    goal.goal = goal.MOVE_TO_SHOOTER;
    ac_indexer_.sendGoal(goal);
    bool finished_before_timeout = waitForResultAndCheckForPreempt(ros::Duration(indexing_timeout_), ac_indexer_, as_);
    if (finished_before_timeout) {
      ROS_INFO_STREAM("2022_shooting_server : cargo should have been launched");
    } else {
      ROS_INFO_STREAM("2022_shooting_server : indexer timed out :(");
      return false;
    }
    if (ros::ok()) {
      feedback_.cargo_shot++;
      as_.publishFeedback(feedback_);
    }
    return ros::ok();
  }

  void executeCB(const behavior_actions::Shooting2022GoalConstPtr &goal)
  {
    if (!ac_indexer_.isServerConnected()) {
      ROS_ERROR_STREAM("2022_shooting_server : indexer server not running!!! this is unlikely to work");
    }
    if (!ac_shooter_.isServerConnected()) {
      ROS_ERROR_STREAM("2022_shooting_server : shooter server not running!!! this is unlikely to work");
    }
    result_.timed_out = false;
	result_.success = false;
    if (goal->num_cargo == 0) {
      ROS_ERROR_STREAM("2022_shooting_server : invalid number of cargo - must be a positive number. ");
      as_.setAborted(result_); // set the action state to aborted
      return;
    }
    bool shooterTimedOut = false;
    bool success = spinUpShooter(shooterTimedOut, goal->low_goal);
    result_.timed_out = shooterTimedOut;
    for (uint8_t i = 0; (cargo_num_ > 0) && (i < goal->num_cargo) && success; i++)
    {
      bool indexerTimedOut = false;
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO_STREAM("2022_shooting_server : preempted");
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      if (getCargoFromIndexer(indexerTimedOut)) {
        success = success && true;
      } else {
        result_.timed_out = indexerTimedOut;
        success = false;
        break;
      }
      if (goal->num_cargo == 2 && i == 0) {
        shooterTimedOut = false;
        if (waitForShooter(shooterTimedOut)) {
          success = success && true;
        } else {
          result_.timed_out = shooterTimedOut;
          success = false;
          break;
        }
      }
	    ros::spinOnce(); // update ball count, hopefully
    }
    ros::Duration(0.75).sleep();

    ac_shooter_.cancelGoal(); // stop shooter
    ac_indexer_.cancelGoal(); // stop indexer

	result_.success = success;
    if (success)
    {
      ROS_INFO_STREAM("2022_shooting_server : succeeded! yay!!");
      // publish feedback
      as_.publishFeedback(feedback_);
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
    else
    {
      ROS_INFO_STREAM("2022_shooting_server : failed :(");
      // set the action state to aborted
      as_.setAborted(result_);
    }

    feedback_ = behavior_actions::Shooting2022Feedback();
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "shooting_server_2022");

  ShootingServer2022 server("shooting_server_2022");
  ros::spin();

  return 0;
}
