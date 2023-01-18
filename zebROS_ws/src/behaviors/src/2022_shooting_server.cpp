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
#include <behaviors/interpolating_map.h>
#include <XmlRpcValue.h>
//#include <XmlRpcExeception.h>

// takes in array of three elements 
bool checkFloatArray(XmlRpc::XmlRpcValue &param)
{ 
  // checks if type is array 
  if (!param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("Shooting Server - Parameter is not an array");
    return false;
  }

  if (!param.size() == 3) {
    ROS_ERROR("Shooting Server - array is not of len 3");
    return false;
  }
  return true;
}

double readFloatParam(XmlRpc::XmlRpcValue &param)
{
  double val = -999;
  // can assume that array is of len 3 
	if (!param.valid())
		throw std::runtime_error("2022 Shooting Server - readFloatParam : param was not a valid type");
	if (param.getType() == XmlRpc::XmlRpcValue::TypeDouble)
	{
		val = static_cast<double>(param);
		return val;
	}
	else if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
	{
		val = static_cast<int>(param);
		return val;
	}
	else
		throw std::runtime_error("2022 Shooting Server - readFloatParam : A non-double value was read for param");
}

struct ShooterData {
      double wheel_speed;
      double hood_wheel_speed;
      ShooterData(double inp_wheel_speed, double inp_hood_wheel_speed) {
        wheel_speed = inp_wheel_speed;
        hood_wheel_speed = inp_hood_wheel_speed;
      }
      ShooterData operator+(const ShooterData& add) const {
        ShooterData res(wheel_speed + add.wheel_speed, hood_wheel_speed + add.hood_wheel_speed);
        return res;
      }
      ShooterData operator*(const double mul) const {
        ShooterData res(wheel_speed * mul, hood_wheel_speed * mul);
        return res;
      }
};

ShooterData operator*(const double& a, const ShooterData& obj) {
  ShooterData res(obj.wheel_speed * a, obj.hood_wheel_speed * a);
  /*
  res.wheel_speed = obj.wheel_speed * a;
  res.hood_wheel_speed = obj.hood_wheel_speed * a;
  */
  return res;
}

class ShootingServer2022
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<behavior_actions::Shooting2022Action> as_;
  // looks unused?
  std::string action_name_;
  // create messages that are used to publish feedback/result
  behavior_actions::Shooting2022Feedback feedback_;

  behavior_actions::Shooting2022Result result_;
  actionlib::SimpleActionClient<behavior_actions::Shooter2022Action> ac_shooter_;
  actionlib::SimpleActionClient<behavior_actions::Index2022Action> ac_indexer_;

  double shooting_timeout_;
  double indexing_timeout_;
  // maybe 2 meters for hood down v up?
  double MAGIC_CONSTANT_ = 2;
  // data was recorded to edge of robot, robot is 17.661 inches from center to edge
  // fender shot, 1.031m to edge + 0.4485894m to center = 1.4795894m to center
  // from center of field to center of robot
  wpi::interpolating_map<double, ShooterData>  shooter_speed_map_;
  XmlRpc::XmlRpcValue shooter_speed_map_xml_;
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
    if (!nh_.getParam("/shooting/shooter_speed_map", shooter_speed_map_xml_))
    {
      ROS_WARN_STREAM("2022_shooting_server : COULD NOT FIND SHOOTER SPEED MAP SHOOTING WILL FAIL");
      while (true) {
        // last time this failed along with a lot of other stuff, hopefully if that happens, we can see more easily whats going on.
        ROS_ERROR_STREAM_THROTTLE(5, "2022_shooting_server : NOT RUNNING - SHOOTER SPEED MAP NOT FOUND");
      }
      /* no equality operator for interpolating_map, just make sure the config works
      shooter_speed_map_ = {
        {1.48, ShooterData(175, 200)}, // hood up
        {2.48, ShooterData(120, 280)}, // hood down
        {3.48, ShooterData(140, 260)}, // down
        {3.98, ShooterData(145, 275)}, // ceiling too strong, these are guesses
        {4.48, ShooterData(150, 280)}
      }; 
      */
    }
    // kinda pointless, we are in trouble if the read fails, but this prevents a out of bounds crash
    else {
      ROS_INFO_STREAM("2022_shooting_server : Found shooter speed map");
      for (size_t i = 0; i < (unsigned) shooter_speed_map_xml_.size(); i++) {
        auto s = shooter_speed_map_xml_[i];
        if (!checkFloatArray(s)) {
          throw std::runtime_error("Shooter Speed Map is not an array or not of len 3");
        }
        shooter_speed_map_.insert(readFloatParam(s[0]), ShooterData(readFloatParam(s[1]), readFloatParam(s[2])));
        ROS_INFO_STREAM("2022_shooting_server : Inserted " << s[0] << " " << s[1] << " " << s[2]);
      }
    
    }
    shooter_speed_map_.insert(1.48, ShooterData(175, 200));
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

  bool spinUpShooter(bool &timedOut, bool eject, double distance) { // returns false if ros is not ok or timed out, true otherwise
    feedback_.state = feedback_.WAITING_FOR_SHOOTER;
    as_.publishFeedback(feedback_);
    ROS_INFO_STREAM("2022_shooting_server : spinning up shooter");
    is_spinning_fast_ = false;
    behavior_actions::Shooter2022Goal goal;
    if (eject) {
      goal.mode = goal.EJECT;}
    else {
      goal.mode = goal.HIGH_GOAL;}

    goal.wheel_speed = shooter_speed_map_[distance].wheel_speed;
    goal.hood_wheel_speed = shooter_speed_map_[distance].hood_wheel_speed;
    ROS_INFO_STREAM("2022_shooting_server : using wheel speed " << goal.wheel_speed << " and hood speed " << goal.hood_wheel_speed);
    // sets hood position to down if less than some constant up otherwise
    goal.hood_position = distance >= MAGIC_CONSTANT_;
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
    bool success = spinUpShooter(shooterTimedOut, goal->eject, goal->distance);
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
      ros::Duration(goal->distance < MAGIC_CONSTANT_ ? 0.25 : 0.5).sleep();
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
  // read ros param for interpolating map


  ShootingServer2022 server("shooting_server_2022");
  ros::spin();

  return 0;
}
