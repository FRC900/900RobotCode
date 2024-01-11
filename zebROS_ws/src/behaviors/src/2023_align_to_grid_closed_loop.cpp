

// so theoretically the best way to do this is to use the values in 2023_grid_locations.yaml to drive to the location knowing our field-relative location using an apriltag.
// that's just a particle filter.
// which we don't trust yet.
// so we can do the equivalent by calculating the offset between a detected apriltag (see 2023_apriltag_locations.yaml) and the desired location
// and averaging if there are multiple.
// find closest apriltag to desired location, calculate offset, call path_to_apriltag
#include "ros/ros.h"
#include "field_obj/Detection.h"
#include "geometry_msgs/Point.h"
#include <actionlib/server/simple_action_server.h>
#include "behavior_actions/AlignToGridPID2023Action.h"
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <path_follower_msgs/holdPositionAction.h>
#include <sensor_msgs/Imu.h>
#include <frc_msgs/MatchSpecificData.h>
#include <behavior_actions/GamePieceState2023.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include <std_srvs/Empty.h>

enum GamePiece { CUBE, CONE };

struct GridLocation {
  double y;
  GamePiece piece;
  GridLocation(double y = 0, GamePiece piece = CUBE) {
    this->y = y;
    this->piece = piece;
  }
};

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

  double x_error_{0};
  double y_error_{0};

  double valid_frames_config_{4};
  double missed_frames_before_exit_{20};

  behavior_actions::GamePieceState2023 game_piece_state_;
	ros::Time latest_game_piece_time_;

	ros::Subscriber game_piece_sub_;

  std::map<std::string, AlignActionAxisState> axis_states_;

  ros::Publisher cmd_vel_pub_;
  ros::Subscriber x_effort_sub_;
  double x_eff_;
  ros::Subscriber y_effort_sub_;
  double y_eff_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::map<int, GridLocation> gridLocations_; // should probably be uint8_t
  std::map<int, geometry_msgs::TransformStamped> tag_to_map_tfs_;

  ros::Publisher orientation_command_pub_;
  ros::Subscriber control_effort_sub_;
  ros::ServiceClient set_leds_green_client_;
  double orient_effort_;

  double max_terabee_distance_; // to avoid rotating when we drive forward

public:

  AlignToGridAction(std::string name) :
    as_(nh_, name, boost::bind(&AlignToGridAction::executeCB, this, _1), false),
    action_name_(name),
    sub_(nh_.subscribe<field_obj::Detection>("/tf_object_detection/tag_detection_world", 1, &AlignToGridAction::callback, this)),
    ac_hold_position_("/hold_position/hold_position_server", true),
    tf_listener_(tf_buffer_),
    orientation_command_pub_(nh_.advertise<std_msgs::Float64>("/teleop/orientation_command", 1)),
    control_effort_sub_(nh_.subscribe<std_msgs::Float64>("/teleop/orient_strafing/control_effort", 1, &AlignToGridAction::controlEffortCB, this))
  {
    game_piece_sub_ = nh_.subscribe("/game_piece/game_piece_state", 1, &AlignToGridAction::gamePieceCallback, this);
    const std::map<std::string, std::string> service_connection_header{ {"tcp_nodelay", "1"} };
    set_leds_green_client_ = nh_.serviceClient<std_srvs::Empty>("/candle_node/set_leds_green", false, service_connection_header);

    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/zeroed_imu", 1, &AlignToGridAction::imuCb, this);
    match_sub_ = nh_.subscribe<frc_msgs::MatchSpecificData>("/frcrobot_rio/match_data", 1, &AlignToGridAction::matchCb, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/align/cmd_vel", 1, false);
    x_effort_sub_ = nh_.subscribe<std_msgs::Float64>("x_position_pid/x_command", 1, [&](const std_msgs::Float64ConstPtr &msg) {x_eff_ = msg->data;});
    y_effort_sub_ = nh_.subscribe<std_msgs::Float64>("y_position_pid/y_command", 1, [&](const std_msgs::Float64ConstPtr &msg) {y_eff_ = msg->data;});

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
    XmlRpc::XmlRpcValue tagList;
    nh_.getParam("tags", tagList); 
    for (XmlRpc::XmlRpcValue::iterator tag=tagList.begin(); tag!=tagList.end(); ++tag) {
			geometry_msgs::TransformStamped map_to_tag;
			map_to_tag.transform.translation.x = tag->second[0];
			map_to_tag.transform.translation.y = tag->second[1];
			map_to_tag.transform.translation.z = tag->second[2];
			tf2::Quaternion q;
			q.setRPY(0, 0, 0);
			geometry_msgs::Quaternion q2m = tf2::toMsg(q);
			map_to_tag.transform.rotation = q2m;     
      tag_to_map_tfs_[std::stoi(tag->first.substr(3))] = map_to_tag;
      auto t = map_to_tag.transform.translation;
      ROS_INFO_STREAM("tag " << t.x << " " << t.y << " " << t.z << " #" << tag->first.substr(3));
    }

    XmlRpc::XmlRpcValue gridList;
    nh_.getParam("grid_locations", gridList);
    for (XmlRpc::XmlRpcValue::iterator grid=gridList.begin(); grid!=gridList.end(); ++grid) {
      GridLocation g;
      g.piece = grid->second[0] == "cone" ? CONE : CUBE;
      g.y = grid->second[1];
      gridLocations_[std::stoi(grid->first.substr(3))] = g;
      ROS_INFO_STREAM(std::stoi(grid->first.substr(3)) << " " << g.y << " " << g.piece);
    }
      
    nh_.getParam("min_valid_frames", valid_frames_config_);
    nh_.getParam("missed_frames_allowed", missed_frames_before_exit_);
    nh_.param<double>("max_terabee_distance", max_terabee_distance_, 0.15);
    // need to load grid stations here.

    as_.start();
  }

  ~AlignToGridAction(void)
  {
  }

  void controlEffortCB(const std_msgs::Float64ConstPtr& msg) {
    orient_effort_ = msg->data;
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
    auto our_tags = alliance_ == frc_msgs::MatchSpecificData::ALLIANCE_COLOR_RED ? red_visible_tags : blue_visible_tags;
    double minDistance = std::numeric_limits<double>::max();
    std::optional<int> closestTag = std::nullopt; 
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

  int getAllianceRelativeStationNumber(int alliance, int gridStation) {
    // 0 = red alliance, 1 = blue
    // if (alliance == 1) {
      return (8-(gridStation-1))+1;
    // }
    // return gridStation;
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
    geometry_msgs::Point offset;
    offset.x = 0.8; // base_link not front_bumper

    geometry_msgs::Point tagLocation = offset; // just to start

    x_axis.setEnable(true);
    y_axis.setEnable(true);
    auto closestTag = findClosestApriltag(latest_);
    // if (closestTag == std::nullopt) {
    //     ROS_ERROR_STREAM("2023_align_to_grid_closed_loop : Must be able to see an apriltag! :(");
    //     as_.setPreempted();
    //     x_axis.setEnable(false);
    //     y_axis.setEnable(false);
    //     return;
    // }
    int missed_frames = 0;
    std_msgs::Float64 msg;
    msg.data = M_PI;
    orientation_command_pub_.publish(msg);
    uint8_t valid_frames = 0;
    while (hypot(x_error_, y_error_) > goal->tolerance || valid_frames < valid_frames_config_) {
        ros::spinOnce(); // grab latest callback data
        orientation_command_pub_.publish(msg);
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_ERROR_STREAM("2023_align_to_grid_closed_loop : Preempted");
            as_.setPreempted();
            x_axis.setEnable(false);
            y_axis.setEnable(false);
            return;
        }

        closestTag = findClosestApriltag(latest_);
        if (closestTag == std::nullopt) {
            ROS_ERROR_STREAM("2023_align_to_grid_closed_loop : Could not find apriltag for this frame! :(");
            missed_frames++;
            if (missed_frames >= missed_frames_before_exit_) {
              ROS_ERROR_STREAM("Missed more than " << missed_frames_before_exit_ << " frames of tag! Aborting");
              x_axis.setEnable(false);
              y_axis.setEnable(false);
              as_.setPreempted();
              return;
            }
            r.sleep();
            continue;
        }
        else {
          x_axis.setEnable(true);
          y_axis.setEnable(true);
          missed_frames = 0;
        }

        int closestId = closestTag.value();
        ROS_INFO_STREAM_THROTTLE(1, "closest id " << closestId);
        bool foundTag = false;
        for (auto detection : latest_.objects) {
          ROS_INFO_STREAM_THROTTLE(2, "Id" << detection.id);
          if (std::stoi(detection.id) == closestId) {
            ROS_INFO_STREAM("Found detection " << detection.id);
            // TRANSFORM TO BASE LINK FIRST
            geometry_msgs::PointStamped point;
            point.point = detection.location;
            point.header = latest_.header;
            geometry_msgs::PointStamped point_out;
            ROS_INFO_STREAM("Latest header before base_link = " << latest_.header);
            // no need for x to be field relative so get the x value before the transform to map
            tf_buffer_.transform(point, point_out, "base_link", ros::Duration(0.1));
            tagLocation.x = point_out.point.x;
            tf2::doTransform(point_out, point_out, tag_to_map_tfs_[closestId]);
            tagLocation.y = point_out.point.y;
            foundTag = true;
          } 
        }
        if (!foundTag) {
          ROS_ERROR_STREAM_THROTTLE(1, "Should be impossible to be here");
          continue;
        }

        int gridStation = getAllianceRelativeStationNumber(alliance_, goal->grid_id);
        GridLocation gridLocation = gridLocations_[gridStation];
        // tag location and offset have to be inverted (* -1)
        // e.g. tag detected at 1.25, desired = 1
        // PID thinks it should move -0.25 but it should really move 0.25

        // this is inverted because it is where the tag is relative to us
        // so if we are at the left node the tag is to the right and vice versa
        
        offset.y = gridLocation.y;
        ROS_INFO_STREAM("Offset y = " << offset.y);

        if (ros::Time::now() - latest_game_piece_time_ > ros::Duration(0.5)) {
        } 
        else {
            double center_offset = game_piece_state_.offset_from_center;
            if (isnan(center_offset)) {
                //ROS_ERROR_STREAM("2023_align_to_grid : game piece offset is NaN, assuming game piece is centered");
            } 
            else {
                if (fabs(center_offset) > max_terabee_distance_) {
                  center_offset = (center_offset > 0) ? max_terabee_distance_ : -max_terabee_distance_;
                }
                offset.y += center_offset; // inverted because where tag is relative to us
            }
        }
        x_axis.setState(-tagLocation.x);
        y_axis.setState(-tagLocation.y);

        x_axis.setCommand(-offset.x);
        y_axis.setCommand(-offset.y);

        ROS_INFO_STREAM("Y axis state = " << tagLocation.y << " command = " << offset.y << ", effort = " << y_eff_); 

        x_error_ = fabs(offset.x - tagLocation.x);
        y_error_ = fabs(offset.y - tagLocation.y);

        ROS_INFO_STREAM("x tag = " << tagLocation.x << ", commanded x = " << offset.x << ", effort = " << x_eff_);
        geometry_msgs::Twist t;
        t.linear.x = x_eff_;
        t.linear.y = y_eff_;
        t.angular.z = orient_effort_;
        cmd_vel_pub_.publish(t);
        feedback_.x_error = x_error_;
        feedback_.y_error = y_error_;
        if (hypot(x_error_, y_error_) <= goal->tolerance) {
          valid_frames++;
        }
        as_.publishFeedback(feedback_);
        r.sleep();
        ROS_INFO_STREAM("offset = " << offset.x << " " << offset.y << " " << offset.z);
        ROS_INFO_STREAM("error = " << x_error_ << ", " << y_error_ << " = " << hypot(x_error_, y_error_));
    }
    std_srvs::Empty empty;
    if (!set_leds_green_client_.call(empty)) {
      ROS_ERROR_STREAM("Unable to set leds to green in align to grid closed loop!");
      // don't really care but good to know about
    }
    x_axis.setEnable(false);
    y_axis.setEnable(false);
    result_.success = true;
    as_.setSucceeded(result_);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "align_to_grid_closed_loop");

  AlignToGridAction alignToGrid("align_to_grid_closed_loop");
  ros::spin();

  return 0;
}