#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behavior_actions/AlignedShooting2022Action.h>
#include <actionlib/client/simple_action_client.h>
#include <behavior_actions/Shooting2022Action.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/Imu.h"
#include <path_follower_msgs/holdPositionAction.h>

class Tag {
public:
	double x;
	double y;
	double z;
	bool inHub;
	Tag(double _x = 0, double _y = 0, double _z = 0, bool _inHub = false) {
		this->x = _x;
		this->y = _y;
		this->z = _z;
		this->inHub = _inHub;
	}
}; // copied from apriltag sim publisher

class XYCoord {
public:
  double x;
  double y;
  XYCoord(double _x = 0, double _y = 0) {
    this->x = _x;
    this->y = _y;
  }
};

class AprilTagShootingAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<behavior_actions::AlignedShooting2022Action> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  behavior_actions::AlignedShooting2022Feedback feedback_;
  behavior_actions::AlignedShooting2022Result result_;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  std::map<int, Tag> tags_;
  ros::Subscriber sub_;
  double hubX;
  double hubY;
  double imuZ;
  actionlib::SimpleActionClient<behavior_actions::Shooting2022Action> ac_;
  actionlib::SimpleActionClient<path_follower_msgs::holdPositionAction> ac_pos_;

public:

  AprilTagShootingAction(std::string name) :
    as_(nh_, name, boost::bind(&AprilTagShootingAction::executeCB, this, _1), false),
    action_name_(name),
    tfBuffer(ros::Duration(0.25)),
    tfListener(tfBuffer),
    ac_("/shooting/shooting_server_2022", true),
    ac_pos_("/hold_position/hold_position_server", true)
  {
    as_.start();
    XmlRpc::XmlRpcValue xmlTags;
    nh_.getParam("tags", xmlTags);
    for (auto pair : xmlTags) {
      tags_[std::stoi(pair.first)] = Tag(static_cast<double>(pair.second[0]), static_cast<double>(pair.second[1]), static_cast<double>(pair.second[2]), static_cast<bool>(pair.second[3]));
    }
    ROS_INFO_STREAM("2022_apriltag_shooting_server : " << tags_.size() << " tags loaded");
    XmlRpc::XmlRpcValue xmlHub;
    nh_.getParam("hub", xmlHub);
    hubX = static_cast<double>(xmlHub[0]);
    hubY = static_cast<double>(xmlHub[1]);
    sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/imu/data", 1000, boost::bind(&AprilTagShootingAction::imuCallback, this, _1));
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    tf2::Quaternion q;
    tf2::fromMsg(msg->orientation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    imuZ = yaw;
    ROS_INFO_STREAM_THROTTLE(5, "2022_apriltag_shooting_server : IMU: " << imuZ);
  }

  ~AprilTagShootingAction(void)
  {
  }

  XYCoord hubLocation(int tagId, XYCoord tagLocation) {
    // robot relative
    double deltaY = hubY - tags_[tagId].y;
    double deltaX = hubX - tags_[tagId].x;
    // ROS_INFO_STREAM("Relative to robot, tag " << tagId << " is at (" << tagLocation.x << "," << tagLocation.y << ")");
    // ROS_INFO_STREAM("Relative to tag " << tagId << ", hub is at (" << deltaX << "," << deltaY << ")");
    // if we say the hub is 1m forward but the robot is rotated to the left,
    // we have to rotate that 1m to the right.
    // this code does that using IMU data.
    // so it's very important that the IMU is zeroed correctly.
    // double dist = std::sqrt(deltaY*deltaY + deltaX*deltaX);
    // double arctan = atan2(deltaY, deltaX);
    // if (deltaX < 0) {
    //   arctan = -arctan;
    // }
    // deltaX = cos(arctan-imuZ) * dist;
    // deltaY = sin(arctan-imuZ) * dist;
    double newDeltaX = deltaX * cos(-imuZ) - deltaY * sin(-imuZ);
    double newDeltaY = deltaY * cos(-imuZ) + deltaX * sin(-imuZ);
    // ROS_INFO_STREAM("Relative to tag " << tagId << ", rotated, hub is at (" << newDeltaX << "," << newDeltaY << ")");
    return XYCoord(newDeltaX + tagLocation.x, newDeltaY + tagLocation.y);
  }

  std::pair<double, double> distanceAngle(XYCoord hubPlace) {
    // before you make fun of me for these variable names, i wrote them in a theater (during a play rehearsal) so it fits
    std::pair<double, double> toilAndTrouble; // first = distance, second = angle
    double fireBurn = std::sqrt(hubPlace.x*hubPlace.x + hubPlace.y*hubPlace.y); // distance
    // and
    double cauldronBubble = atan2(hubPlace.y, hubPlace.x); // angle
    toilAndTrouble.first = fireBurn;
    toilAndTrouble.second = cauldronBubble;
    return toilAndTrouble;
  }

  void executeCB(const behavior_actions::AlignedShooting2022GoalConstPtr &goal)
  {
    if (!ac_.isServerConnected()) {
      ROS_ERROR_STREAM("2022_apriltag_shooting_server : shooting server not running!!! this is unlikely to work");
    }
    static tf2_ros::TransformBroadcaster br;

    double averageHubX = 0;
    double averageHubY = 0;
    uint16_t apriltagsDetected = 0; // 65536 apriltags should be enough...

    for (const auto& [tag, _] : tags_) {
      geometry_msgs::TransformStamped transformStamped;
      geometry_msgs::PointStamped pointStamped;
      try{
        transformStamped = tfBuffer.lookupTransform("base_link", "36h11"+std::to_string(tag),
                                 ros::Time(0));
        XYCoord robotRelativeTagCoord = XYCoord(transformStamped.transform.translation.x, transformStamped.transform.translation.y);
        XYCoord hubGuess = hubLocation(tag, robotRelativeTagCoord);
        averageHubX += hubGuess.x;
        averageHubY += hubGuess.y;
        apriltagsDetected += 1;
        auto distAngle = distanceAngle(hubGuess);
      }
      catch (tf2::TransformException &ex) {
        // tag not found, this is ok
        continue;
      }
    }
    ROS_INFO_STREAM("2022_apriltag_shooting_server : " << apriltagsDetected << " tags found, setting angle");
    averageHubX /= (double)apriltagsDetected;
    averageHubY /= (double)apriltagsDetected;
    XYCoord hubGuess(averageHubX, averageHubY);
    auto distAngle = distanceAngle(hubGuess);

    path_follower_msgs::holdPositionGoal posGoal;
    tf2::Quaternion q;
    q.setRPY(0, 0, distAngle.second + imuZ);  // Create this quaternion from roll/pitch/yaw (in radians)
    geometry_msgs::Quaternion gq = tf2::toMsg(q);
    posGoal.pose.orientation = gq;
    posGoal.isAbsoluteCoord = true;

    bool aligned = false;

    ac_pos_.sendGoal(posGoal,
                actionlib::SimpleActionClient<path_follower_msgs::holdPositionAction>::SimpleDoneCallback(),
                actionlib::SimpleActionClient<path_follower_msgs::holdPositionAction>::SimpleActiveCallback(),
                [&](const path_follower_msgs::holdPositionFeedbackConstPtr& feedback){
                  aligned = feedback->isAligned;
                });
    ROS_INFO_STREAM("angle.second " << distAngle.second << " IMUZ " << imuZ);
    ROS_INFO_STREAM("2022_apriltag_shooting_server : angle set to " << distAngle.second + imuZ << " absolute");

    ros::Rate r(100);
    bool success = true;
    while (!aligned) {
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO_STREAM("2022_apriltag_shooting_server : preempted");
        ac_pos_.cancelGoalsAtAndBeforeTime(ros::Time::now());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      r.sleep();
    }
    ac_pos_.cancelGoalsAtAndBeforeTime(ros::Time::now());

    ROS_INFO_STREAM("2022_apriltag_shooting_server : angle reached");
    feedback_.state = feedback_.WAITING_FOR_SHOOTER;

    ROS_INFO_STREAM("2022_apriltag_shooting_server : shooting " << std::to_string(goal->num_cargo) << " cargo at a distance of " << distAngle.first << "m.");

    behavior_actions::Shooting2022Goal shootingGoal;
    shootingGoal.num_cargo = goal->num_cargo;
    shootingGoal.distance = distAngle.first;
    shootingGoal.eject = false;

    bool done = false;

    ac_.sendGoal(shootingGoal,
      [&](const actionlib::SimpleClientGoalState& state, const behavior_actions::Shooting2022ResultConstPtr& result){
        result_.success = result->success;
        as_.setSucceeded(result_);
        done = true;
      },
                actionlib::SimpleActionClient<behavior_actions::Shooting2022Action>::SimpleActiveCallback(),
                [&](const behavior_actions::Shooting2022FeedbackConstPtr& feedback){
                  feedback_.state = feedback->state;
                });

    while (!done) {
      // handle preempt. preempt shooting if this is preempted
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO_STREAM("2022_apriltag_shooting_server : preempted");
        // set the action state to preempted
        as_.setPreempted();
        ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());
        success = false;
        break;
      }
      r.sleep();
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "apriltag_shooting_server_2022");

  AprilTagShootingAction apriltagShootingServer("apriltag_shooting_server_2022");
  ros::spin();

  return 0;
}
