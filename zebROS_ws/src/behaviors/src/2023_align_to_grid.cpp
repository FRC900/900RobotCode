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
#include "behavior_actions/AlignToGrid2023Action.h"
#include "behavior_actions/PathToAprilTagAction.h"
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <path_follower_msgs/holdPositionAction.h>
#include <sensor_msgs/Imu.h>
#include <frc_msgs/MatchSpecificData.h>
#include <behavior_actions/GamePieceState2023.h>

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

class Tag {
public:
	geometry_msgs::Point location;
	double rotation;
	Tag(geometry_msgs::Point location = geometry_msgs::Point(), bool rotation = 0) {
		this->location = location;
		this->rotation = rotation;
	}
};

enum GamePiece { CUBE, CONE };

struct GridLocation {
  double y;
  GamePiece piece;
  GridLocation(double y = 0, GamePiece piece = CUBE) {
    this->y = y;
    this->piece = piece;
  }
};

class AlignToGridAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<behavior_actions::AlignToGrid2023Action> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  behavior_actions::AlignToGrid2023Feedback feedback_;
  behavior_actions::AlignToGrid2023Result result_;
  field_obj::Detection latest_;
  ros::Subscriber sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber match_sub_;
  std::map<int, Tag> tags_;
  std::map<int, GridLocation> gridLocations_; // should probably be uint8_t
  actionlib::SimpleActionClient<behavior_actions::PathToAprilTagAction> client_;
  actionlib::SimpleActionClient<path_follower_msgs::holdPositionAction> ac_hold_position_;
  double xOffset_;
  double holdPosTimeout_;
  double latest_yaw_{0};
  uint8_t alliance_{0};
  double percent_complete_{0}; 

  behavior_actions::GamePieceState2023 game_piece_state_;
	ros::Time latest_game_piece_time_;

	ros::Subscriber game_piece_sub_;

  double game_piece_timeout_;

public:

  AlignToGridAction(std::string name) :
    as_(nh_, name, boost::bind(&AlignToGridAction::executeCB, this, _1), false),
    action_name_(name),
    sub_(nh_.subscribe<field_obj::Detection>("/tf_object_detection/tag_detection_world", 1, &AlignToGridAction::callback, this)),
    client_("/path_to_apriltag", true),
    ac_hold_position_("/hold_position/hold_position_server", true)
  {
    game_piece_sub_ = nh_.subscribe("/game_piece/game_piece_state", 1, &AlignToGridAction::gamePieceCallback, this);

    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/zeroed_imu", 1, &AlignToGridAction::imuCb, this);
    match_sub_ = nh_.subscribe<frc_msgs::MatchSpecificData>("/frcrobot_rio/match_data", 1, &AlignToGridAction::matchCb, this);
    XmlRpc::XmlRpcValue tagList;
    nh_.getParam("tags", tagList);

    XmlRpc::XmlRpcValue gridList;
    nh_.getParam("grid_locations", gridList);

    nh_.getParam("offset_from_cube_tag_to_end", xOffset_);

    nh_.getParam("hold_position_timeout", holdPosTimeout_);

    if (!nh_.getParam("game_piece_timeout", game_piece_timeout_)) {
			ROS_WARN_STREAM("2023_align_to_grid : could not find game_piece_timeout, defaulting to 1 second");
			game_piece_timeout_ = 1;
		}

    for (XmlRpc::XmlRpcValue::iterator tag=tagList.begin(); tag!=tagList.end(); ++tag) {
      Tag t;
      geometry_msgs::Point p;
      p.x = tag->second[0];
      p.y = tag->second[1];
      p.z = tag->second[2];
      t.location = p;
      t.rotation = tag->second[3];
      tags_[std::stoi(tag->first.substr(3))] = t;
      ROS_INFO_STREAM("tag " << p.x << " " << p.y << " " << p.z << " #" << tag->first.substr(3));
    }

    for (XmlRpc::XmlRpcValue::iterator grid=gridList.begin(); grid!=gridList.end(); ++grid) {
      GridLocation g;
      g.piece = grid->second[0] == "cone" ? CONE : CUBE;
      g.y = grid->second[1];
      gridLocations_[std::stoi(grid->first.substr(3))] = g;
      ROS_INFO_STREAM(std::stoi(grid->first.substr(3)) << " " << g.y << " " << g.piece);
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

  int getAllianceRelativeStationNumber(int alliance, int gridStation) {
    // 0 = red alliance, 1 = blue
    if (alliance == 1) {
      return (8-(gridStation-1))+1;
    }
    return gridStation;
  }

  void feedbackCb(const behavior_actions::PathToAprilTagFeedbackConstPtr& feedback) {
      percent_complete_ = feedback->percent_complete;
  }

  void executeCB(const behavior_actions::AlignToGrid2023GoalConstPtr &goal)
  {
    if (!client_.isServerConnected()) {
      ROS_ERROR_STREAM("2023_align_to_grid : path to apriltag server not running!!! this is unlikely to work");
    }
    ros::spinOnce(); // grab latest callback data
    // TODO: do we want to find the tag closest to *us* or closest to the grid?
    // right now it finds the one closest to the robot
    // I'm not sure what is most accurate
    auto closestTag = findClosestApriltag(latest_);
    if (closestTag == std::nullopt) {
      as_.setAborted(result_); // no tag found
      ROS_ERROR_STREAM("2023_align_to_grid : No AprilTags found :(");
      return;
    }
    int closestId = closestTag.value();
    Tag tag = tags_[closestId];

    int gridStation = getAllianceRelativeStationNumber(alliance_, goal->grid_id);
    GridLocation gridLocation = gridLocations_[gridStation];


    // 1 is at the left, 9 is at the right

    // e.g. the tag we find is at (1,1) -- src and we want to go to (2,2) -- dest
    // need dest - src

    geometry_msgs::Point offset;
    offset.x = -xOffset_; // TODO consider going to a bit behind this and then driving forward for a preset amount of time? (in case we overshoot)
    ROS_INFO_STREAM("xOffset: " << offset.x);
    offset.y = (alliance_ == 1 ? -1 : 1) * (gridLocation.y - tag.location.y); // should be gridLocation.y - tag.location.y

    double terabee_offset = 0;
    if (!(goal->no_terabees)) {
      if (ros::Time::now() - latest_game_piece_time_ > ros::Duration(game_piece_timeout_)) {
				ROS_ERROR_STREAM("2023_align_to_grid : game piece position data too old, assuming game piece is centered");
			} else {
        double center_offset = game_piece_state_.offset_from_center;
        if (isnan(center_offset)) {
          ROS_ERROR_STREAM("2023_align_to_grid : game piece offset is NaN, assuming game piece is centered");
        } else {
          terabee_offset = center_offset;
        }
      }
    }
    offset.y -= terabee_offset;

    offset.z = 0;
    ROS_INFO_STREAM("grid " << gridLocation.y << " tag " << tag.location.y);
    ROS_INFO_STREAM("ydif: " << gridLocation.y - tag.location.y);

    geometry_msgs::Pose pose;
    pose.position = offset;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0); // facing the tag
    geometry_msgs::Quaternion qMsg = tf2::toMsg(q);

    pose.orientation = qMsg;
    // set orientation to quaternion with yaw = tagRotation, use setRPY

    behavior_actions::PathToAprilTagGoal aprilGoal;
    aprilGoal.id = closestId;
    aprilGoal.tagRotation = M_PI;
    aprilGoal.offset = pose;
    aprilGoal.frame_id = "front_bumper";

    client_.sendGoal(aprilGoal,  /* Done cb */ NULL, /*Active*/ NULL, boost::bind(&AlignToGridAction::feedbackCb, this, _1));

    ros::Rate r(30);
    while (!client_.getState().isDone()) {
        if (as_.isPreemptRequested() || !ros::ok())
        {
            client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
            as_.setPreempted();
            return;
        }
        feedback_.percent_complete = percent_complete_;
        as_.publishFeedback(feedback_); 
        ros::spinOnce();
        r.sleep();
    }

    // path_follower_msgs::holdPositionGoal holdPosGoal;
    // holdPosGoal.pose.position.x = 0.5; // drive into wall
    // q.setRPY(0, 0, tag.rotation - latest_yaw_);
    // holdPosGoal.pose.orientation = tf2::toMsg(q);

    // ac_hold_position_.sendGoal(holdPosGoal);
    // ros::Time start = ros::Time::now();

    // while (!ac_hold_position_.getState().isDone() && (ros::Time::now() - start < ros::Duration(holdPosTimeout_))) {
    //     if (as_.isPreemptRequested() || !ros::ok())
    //     {
    //         ac_hold_position_.cancelGoalsAtAndBeforeTime(ros::Time::now());
    //         as_.setPreempted();
    //         return;
    //     }
    //     r.sleep();
    // }
    // ac_hold_position_.cancelGoalsAtAndBeforeTime(ros::Time::now());

    as_.setSucceeded(result_);
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "align_to_grid");

  AlignToGridAction alignToGrid("align_to_grid");
  ros::spin();

  return 0;
}