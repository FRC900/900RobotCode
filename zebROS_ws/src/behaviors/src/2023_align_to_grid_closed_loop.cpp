// so theoretically the best way to do this is to use the values in 2023_grid_locations.yaml to drive to the location knowing our field-relative location using an apriltag.
// that's just a particle filter.
// which we don't trust yet.
// so we can do the equivalent by calculating the offset between a detected apriltag (see 2023_apriltag_locations.yaml) and the desired location
// and averaging if there are multiple.
// find closest apriltag to desired location, calculate offset, call path_to_apriltag
#include "ros/ros.h"
#include "field_obj/Detection.h"
#include "geometry_msgs/Point.h"
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "behavior_actions/AlignToGridPID2023Action.h"
#include "behavior_actions/PathToAprilTagAction.h"
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <path_follower_msgs/holdPositionAction.h>
#include <sensor_msgs/Imu.h>
#include <frc_msgs/MatchSpecificData.h>
#include <behavior_actions/GamePieceState2023.h>
#include <path_follower_msgs/PathAction.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

class AlignActionAxisConfig
{
	public:
		AlignActionAxisConfig(const std::string &name,
							  const std::string &enable_pub_topic,
							  const std::string &command_pub_topic,
							  const std::string &state_pub_topic,
							  const std::string &error_sub_topic,
							  const std::string &timeout_param,
							  const std::string &error_threshold_param)
			: name_(name)
			, enable_pub_topic_(enable_pub_topic)
			, command_pub_topic_(command_pub_topic)
			, state_pub_topic_(state_pub_topic)
			, error_sub_topic_(error_sub_topic)
			, timeout_param_(timeout_param)
			, error_threshold_param_(error_threshold_param)
		{
		}
		std::string name_;
		std::string enable_pub_topic_;
		std::string command_pub_topic_;
		std::string state_pub_topic_;
		std::string error_sub_topic_;
		std::string timeout_param_;
		std::string error_threshold_param_;
};

class AlignActionAxisState
{
	public:
		AlignActionAxisState(ros::NodeHandle &nh,
							 const std::string &enable_pub_topic,
							 const std::string &command_pub_topic,
							 const std::string &state_pub_topic)
			: enable_pub_(nh.advertise<std_msgs::Bool>(enable_pub_topic, 1, true))
			, command_pub_(nh.advertise<std_msgs::Float64>(command_pub_topic, 1, true))
			, state_pub_(nh.advertise<std_msgs::Float64>(state_pub_topic, 1, true))
		{
			// Set defaults for PID node topics to prevent
			// spam of "Waiting for first setpoint message."
			std_msgs::Bool bool_msg;
			bool_msg.data = false;
			enable_pub_.publish(bool_msg);

			std_msgs::Float64 float64_msg;
			float64_msg.data = 0.0;
			command_pub_.publish(float64_msg);
			state_pub_.publish(float64_msg);
		}
		void setEnable(bool enable_state)
		{
			std_msgs::Bool enable_msg;
			enable_msg.data = enable_state;
			enable_pub_.publish(enable_msg);
		}
		void setCommand(double command)
		{
			std_msgs::Float64 command_msg;
			command_msg.data = command;
			command_pub_.publish(command_msg);
		}
		void setState(double state)
		{
			std_msgs::Float64 state_msg;
			state_msg.data = state;
			state_pub_.publish(state_msg);
		}
	private:
		ros::Publisher enable_pub_;
		ros::Publisher command_pub_;
		ros::Publisher state_pub_;
};


geometry_msgs::Point operator-(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) {
    geometry_msgs::Point p;
    p.x = lhs.x - rhs.x;
    p.y = lhs.y - rhs.y;
    p.z = lhs.z - rhs.z;
    return p;
}

geometry_msgs::Point operator+(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) {
    geometry_msgs::Point p;
    p.x = lhs.x + rhs.x;
    p.y = lhs.y + rhs.y;
    p.z = lhs.z + rhs.z;
    return p;
}

double getYaw(const geometry_msgs::Quaternion &o) {
    tf2::Quaternion q;
    tf2::fromMsg(o, q);
    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);
    return y;
}

double distance(const geometry_msgs::Point &pt) {
    return hypot(pt.x, pt.y, pt.z);
}

bool operator<(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) {
    return distance(lhs) < distance(rhs);
}

bool operator>(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) {
    return distance(lhs) > distance(rhs);
}

class AlignToGridAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<behavior_actions::AlignToGridPID2023Action> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  behavior_actions::AlignToGridPID2023Feedback feedback_;
  behavior_actions::AlignToGridPID2023Result result_;
  field_obj::Detection latest_;
  ros::Subscriber sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber match_sub_;
  actionlib::SimpleActionClient<path_follower_msgs::holdPositionAction> ac_hold_position_;
  double xOffset_;
  double holdPosTimeout_;
  double latest_yaw_{0};
  uint8_t alliance_{0};
  double percent_complete_{0}; 

  double x_error_{0};
  double y_error_{0};

  behavior_actions::GamePieceState2023 game_piece_state_;
	ros::Time latest_game_piece_time_;

	ros::Subscriber game_piece_sub_;

  std::map<std::string, AlignActionAxisState> axis_states_;

public:

  AlignToGridAction(std::string name) :
    as_(nh_, name, boost::bind(&AlignToGridAction::executeCB, this, _1), false),
    action_name_(name),
    sub_(nh_.subscribe<field_obj::Detection>("/tf_object_detection/tag_detection_world", 1, &AlignToGridAction::callback, this)),
    ac_hold_position_("/hold_position/hold_position_server", true)
  {
    game_piece_sub_ = nh_.subscribe("/game_piece/game_piece_state", 1, &AlignToGridAction::gamePieceCallback, this);

    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/zeroed_imu", 1, &AlignToGridAction::imuCb, this);
    match_sub_ = nh_.subscribe<frc_msgs::MatchSpecificData>("/frcrobot_rio/match_data", 1, &AlignToGridAction::matchCb, this);

    AlignActionAxisConfig x_axis("x", "x_position_pid/pid_enable", "x_position_pid/x_cmd_pub", "x_position_pid/x_state_pub", "x_position_pid/pid_debug", "x_timeout_param", "x_error_threshold_param");
	  AlignActionAxisConfig y_axis("y", "y_position_pid/pid_enable", "y_position_pid/y_cmd_pub", "y_position_pid/y_state_pub", "y_position_pid/pid_debug", "y_timeout_param", "y_error_threshold_param");
    if (!addAxis(x_axis))
    {
      ROS_ERROR_STREAM("Error adding x_axis to align to grid closed loop.");
      return ;
    }
    if (!addAxis(y_axis))
    {
      ROS_ERROR_STREAM("Error adding y_axis to align to grid closed loop.");
      return;
    }

    // need to load grid stations here.

    as_.start();
  }

  ~AlignToGridAction(void)
  {
  }

  void gamePieceCallback(const behavior_actions::GamePieceState2023ConstPtr &msg) {
		game_piece_state_ = *msg;
		latest_game_piece_time_ = ros::Time::now();
	}

  void imuCb(const sensor_msgs::ImuConstPtr &msg) {
    latest_yaw_ = getYaw(msg->orientation);
  }

  void matchCb(const frc_msgs::MatchSpecificDataConstPtr &msg) {
    alliance_ = msg->allianceColor;
  }

  void callback(const field_obj::DetectionConstPtr& msg) {
    // need to use transforms...
    // oh wait no we don't! we can tell spline srv what frame id it is relevant to
    latest_ = *msg;
  }

  // x & y axis are each controlled by a PID node which
  // tries to close the error between the current odom / pose
  // position and the desired location for each time step
  // along the path.  This is a helper to create a map
  // of state names to stucts which hold the 
  // topics for each PID node
  bool addAxis(const AlignActionAxisConfig &axis_config)
  {
    axis_states_.emplace(std::make_pair(axis_config.name_,
                      AlignActionAxisState(nh_,
                          axis_config.enable_pub_topic_,
                          axis_config.command_pub_topic_,
                          axis_config.state_pub_topic_)));
    return true;
  }

  std::optional<int> findClosestApriltag(const field_obj::Detection &detection) {
    std::vector<uint8_t> red_visible_tags{1,2,3,4};
    std::vector<uint8_t> blue_visible_tags{5,6,7,8};
    auto our_tags = alliance_ == 0 ? red_visible_tags : blue_visible_tags;
    double minDistance = std::numeric_limits<double>::max();
    int closestTag;
    if (detection.objects.size() == 0) {
      return std::nullopt;
    }
    for (field_obj::Object obj : detection.objects) {
      double d = distance(obj.location);
      if (d < minDistance && std::find(our_tags.begin(), our_tags.end(), std::stoi(obj.id)) != our_tags.end()) {
        minDistance = d;
        closestTag = std::stoi(obj.id);
      }
    }
    return closestTag;
  }

  void feedbackCb(const behavior_actions::PathToAprilTagFeedbackConstPtr& feedback) {
      percent_complete_ = feedback->percent_complete;
  }

  void executeCB(const behavior_actions::AlignToGridPID2023GoalConstPtr &goal)
  {
    // just to make sure to go through the loop once
    x_error_ = 100;
    y_error_ = 100;
    ros::Rate r = ros::Rate(30);
    ROS_INFO_STREAM("Execute callback");
    auto x_axis_it = axis_states_.find("x");
    auto &x_axis = x_axis_it->second;
    auto y_axis_it = axis_states_.find("y");
    auto &y_axis = y_axis_it->second;
    // TODO: do we want to find the tag closest to *us* or closest to the grid?
    // right now it finds the one closest to the robot
    // I'm not sure what is most accurate
    auto closestTag = findClosestApriltag(latest_);
    if (closestTag == std::nullopt) {
        ROS_ERROR_STREAM("2023_align_to_grid_closed_loop : No AprilTags found :(");
        as_.setPreempted();
        x_axis.setEnable(false);
        y_axis.setEnable(false);
        return;
    }
    int closestId = closestTag.value();

    geometry_msgs::Point offset;
    if (goal->location == goal->LEFT_CONE) {
      offset.y += 0.559;
    } else if (goal->location == goal->RIGHT_CONE) {
      offset.y -= 0.559;
    } else {
      // do nothing
    }

    if (ros::Time::now() - latest_game_piece_time_ > ros::Duration(0.5)) {
      
    } 
    else {
        double center_offset = game_piece_state_.offset_from_center;
        if (isnan(center_offset)) {
            //ROS_ERROR_STREAM("2023_align_to_grid : game piece offset is NaN, assuming game piece is centered");
        } 
        else {
            offset.y -= center_offset;
        }
    }

    offset.x = 1.0; // base_link not front_bumper

    geometry_msgs::Point tagLocation = offset; // just to start

    x_axis.setEnable(true);
    y_axis.setEnable(true);

    x_axis.setState(tagLocation.x);
    y_axis.setState(tagLocation.y);

    x_axis.setCommand(offset.x);
    y_axis.setCommand(offset.y);

    while (hypot(x_error_, y_error_) < goal->tolerance) {
        ros::spinOnce(); // grab latest callback data
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_ERROR_STREAM("2023_align_to_grid_closed_loop : Preempted");
            as_.setPreempted();
            x_axis.setEnable(false);
            y_axis.setEnable(false);
            return;
        }

        for (auto detection : latest_.objects) {
            if (std::stoi(detection.id) == closestId) {
                //ROS_INFO_STREAM("Found detection " << detection.id);
                tagLocation = detection.location; 
            }
        }
        
        x_axis.setState(tagLocation.x);
        y_axis.setState(tagLocation.y);

        x_error_ = fabs(offset.x - tagLocation.x);
        y_error_ = fabs(offset.y - tagLocation.y);

        r.sleep();
    }
    x_axis.setEnable(false);
    y_axis.setEnable(false);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "align_to_grid_closed_loop");

  AlignToGridAction alignToGrid("align_to_grid_closed_loop");
  ros::spin();

  return 0;
}