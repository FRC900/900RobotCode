// This node autonomously targets and drives to a configurable distance away from an object detected using vision.
// It continuously checks where the object is to update its target.
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <actionlib/server/simple_action_server.h>
#include "behavior_actions/DriveToObjectAction.h"
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/Imu.h>
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include <norfair_ros/Detections.h>
#include <cmath>
#include "axis_state/axis_state.h"

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

double dist_between_points(double x1, double y1, double x2, double y2) {
  // return sqrt(pow((x2-x1), 2) + pow((y2-y1), 2));
  return hypot(x2 - x1, y2 - y1);
}


bool operator<(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) {
    return distance(lhs) < distance(rhs);
}

bool operator>(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) {
    return distance(lhs) > distance(rhs);
}

struct WeightedCmdVel {
  geometry_msgs::Twist cmd_vel;
  double weight;
  WeightedCmdVel() {

  }
  WeightedCmdVel(const geometry_msgs::Twist &cmd_vel, double weight) {
    this->cmd_vel = cmd_vel;
    this->weight = weight;
  }
};

class DriveToObjectActionServer
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<behavior_actions::DriveToObjectAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  behavior_actions::DriveToObjectFeedback feedback_;
  behavior_actions::DriveToObjectResult result_;
  norfair_ros::Detections latest_;
  ros::Subscriber sub_;
  ros::Subscriber imu_sub_;
  double latest_yaw_{0};
  double orient_effort_{0};

  double x_error_{0};
  double angle_error_{0};

  double velocity_ramp_time_{0.5}; // time to transition from current command velocity to full PID control
  double distance_to_hold_angle_{0.5};
  double timeout_{0.25};

  std::map<std::string, AlignActionAxisStatePosition> axis_states_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber x_effort_sub_;
  double x_eff_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Publisher orientation_command_pub_;
  ros::Subscriber control_effort_sub_;

  ros::Subscriber cmd_vel_sub_;
  geometry_msgs::TwistStamped latest_cmd_vel_;

public:

  DriveToObjectActionServer(std::string name) :
    as_(nh_, name, boost::bind(&DriveToObjectActionServer::executeCB, this, _1), false),
    action_name_(name),
    sub_(nh_.subscribe<norfair_ros::Detections>("/norfair/output", 1, &DriveToObjectActionServer::update_latest_detection, this, ros::TransportHints().tcpNoDelay())),
    tf_listener_(tf_buffer_),
    orientation_command_pub_(nh_.advertise<std_msgs::Float64>("/teleop/orientation_command", 1)),
    control_effort_sub_(nh_.subscribe<std_msgs::Float64>("/teleop/orient_strafing/control_effort", 1, &DriveToObjectActionServer::controlEffortCB, this, ros::TransportHints().tcpNoDelay())),
    cmd_vel_sub_(nh_.subscribe<geometry_msgs::TwistStamped>("/frcrobot_jetson/swerve_drive_controller/cmd_vel_out", 1, &DriveToObjectActionServer::cmdVelCb, this))
  {
    const std::map<std::string, std::string> service_connection_header{ {"tcp_nodelay", "1"} };

    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/zeroed_imu", 1, &DriveToObjectActionServer::imuCb, this, ros::TransportHints().tcpNoDelay());
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/auto_note_align/cmd_vel", 1, false);
    x_effort_sub_ = nh_.subscribe<std_msgs::Float64>("x_position_pid/x_command", 1, [&](const std_msgs::Float64ConstPtr &msg) {x_eff_ = msg->data;}, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    AlignActionAxisConfig x_axis("x", "x_position_pid/pid_enable", "x_position_pid/x_cmd_pub", "x_position_pid/x_state_pub", "x_position_pid/pid_debug", "x_timeout_param", "x_error_threshold_param");
    if (!addAxis(x_axis))
    {
      ROS_ERROR_STREAM("Error adding x_axis to drive_to_object.");
      return ;
    }

    if(!nh_.getParam("velocity_ramp_time", velocity_ramp_time_)) {
      ROS_ERROR_STREAM("Could not read velocity_ramp_time_ in align_server");
    }

    if(!nh_.getParam("distance_to_hold_angle", distance_to_hold_angle_)) { 
      ROS_ERROR_STREAM("Could not read distance_to_hold_angle_ in align_server");
    }

    if(!nh_.getParam("timeout", timeout_)) {
      ROS_ERROR_STREAM("Could not read timeout_ in align_server");
    }
    // geometry_msgs::Twist t1;
    // t1.linear.x = 1.0;
    // geometry_msgs::Twist t2;
    // t2.linear.x = -0.1;
    // t2.linear.y = 1.0;

    // std::vector<std::pair<geometry_msgs::Twist, double>> pairs = {{t1, 0.8}, {t2, 0.2}};

    // ROS_INFO_STREAM(weightedAverageCmdVel(pairs));

    as_.start();
  }

  ~DriveToObjectActionServer(void)
  {
  }

  void cmdVelCb(const geometry_msgs::TwistStampedConstPtr &msg) {
    latest_cmd_vel_ = *msg;
  }

  void controlEffortCB(const std_msgs::Float64ConstPtr& msg) {
    orient_effort_ = msg->data;
  }

  void imuCb(const sensor_msgs::ImuConstPtr &msg) {
    latest_yaw_ = getYaw(msg->orientation);
  }

  void update_latest_detection(const norfair_ros::DetectionsConstPtr& msg) {
    latest_ = *msg;
  }

  bool addAxis(const AlignActionAxisConfig &axis_config)
  {
    axis_states_.emplace(std::make_pair(axis_config.name_,
                      AlignActionAxisStatePosition(nh_,
                          axis_config.enable_pub_topic_,
                          axis_config.command_pub_topic_,
                          axis_config.state_pub_topic_)));
    return true;
  }

  std::optional<norfair_ros::Detection> findClosestObject(const norfair_ros::Detections &detections, const std::string &object_id, const int &tracked_object_id) {
    double minDistance = std::numeric_limits<double>::max();
    // ROS_INFO_STREAM("Finding closest object");
    auto map_to_baselink = tf_buffer_.lookupTransform("odom", "base_link", ros::Time::now(), ros::Duration(0.1));
    double map_x = map_to_baselink.transform.translation.x;
    double map_y = map_to_baselink.transform.translation.y;
    // ROS_INFO_STREAM("HERE");
    std::optional<norfair_ros::Detection> closestObject = std::nullopt;
    if (detections.detections.size() == 0 || (ros::Time::now() - detections.header.stamp) > ros::Duration(timeout_)) {
      ROS_ERROR_STREAM("RETURING NULLOPT, time delta is " << (ros::Time::now() - detections.header.stamp).toSec());
      ROS_ERROR_STREAM("Det size " << detections.detections.size());
      return std::nullopt;
    }
    // ROS_INFO_STREAM("BEFORE LOOP");
    for (const norfair_ros::Detection &obj : detections.detections) {
      // ROS_INFO_STREAM("LOOP");
      if (obj.points.size() > 1) {
        ROS_ERROR_STREAM("SHOULD ONLY CONTAIN ONE POINT");
      }
      if (tracked_object_id == -1) {
        ROS_WARN_STREAM("First frame, finding closest object");
        // ROS_INFO_STREAM("Object with name " << obj.label << " at x,y " << obj.points[0].point[0] << "," << obj.points[0].point[1]);
        double d = dist_between_points(map_x, map_y, obj.points[0].point[0], obj.points[0].point[1]);

        if (d < minDistance && obj.label == object_id) {
          minDistance = d;
          closestObject = obj;
        }
      }
      else {
        if (obj.id == tracked_object_id) {
          ROS_INFO_STREAM("Tracked object WITH ID " << tracked_object_id);

          closestObject = obj;
        }
      }
    }

    ROS_INFO_STREAM("Map location x,y " << map_x << "," << map_y);
    if (closestObject.has_value()) {
      ROS_INFO_STREAM("Closest object is " << closestObject.value().id << closestObject.value().label << " at x,y " << closestObject.value().points[0].point[0] << "," << closestObject.value().points[0].point[1]);
    }
    else {
      ROS_WARN_STREAM("No object :(");
    }
    return closestObject;
  }

  geometry_msgs::Twist weightedAverageCmdVel(const std::vector<WeightedCmdVel> &entries) {
    // takes a vector of <cmd_vel, weight> pairs and computes their weighted average
    // weights must all add to one
    std::vector<geometry_msgs::Twist> weightedTwists;
    std::transform(entries.begin(), entries.end(), std::back_inserter(weightedTwists), [](const WeightedCmdVel &entry){
      geometry_msgs::Twist t = entry.cmd_vel;
      t.linear.x *= entry.weight;
      t.linear.y *= entry.weight;
      t.angular.z *= entry.weight;
      return t;
    });
    geometry_msgs::Twist finalTwist;
    std::for_each(weightedTwists.begin(), weightedTwists.end(), [&](const geometry_msgs::Twist &t){
      finalTwist.linear.x += t.linear.x;
      finalTwist.linear.y += t.linear.y;
      finalTwist.angular.z += t.angular.z;
    });
    return finalTwist;
  }

  void executeCB(const behavior_actions::DriveToObjectGoalConstPtr &goal)
  {
    // Probably should add a timeout 

    // just to make sure to go through the loop once
    int tracked_object_id = -1;
    x_error_ = 900;
    angle_error_ = 900;
    ros::Rate r = ros::Rate(60);
    ROS_INFO_STREAM("Execute callback");
    auto x_axis_it = axis_states_.find("x");
    auto &x_axis = x_axis_it->second;

    x_axis.setEnable(true);
    std::optional<norfair_ros::Detection> closestObject = findClosestObject(latest_, goal->id, tracked_object_id);
    if (closestObject == std::nullopt) {
      ROS_ERROR_STREAM("No inital object, exiting");
      result_.success = false;
      as_.setSucceeded(result_);
      return;
    } else {
      tracked_object_id = closestObject->id;
    }
    // we don't know this until we go once in the loop, set to large value to signal that
    double field_relative_object_angle = 900;
    std_msgs::Float64 msg;
    // msg.data = field_relative_object_angle;
    // orientation_command_pub_.publish(msg);

    geometry_msgs::Twist initial_cmd_vel = latest_cmd_vel_.twist;
    ROS_INFO_STREAM("Initial cmd vel is " << initial_cmd_vel);
    ros::Time start = ros::Time::now();

    geometry_msgs::PointStamped latest_map_relative_detection;

    while (!(abs(x_error_) <= goal->tolerance)) {
      ros::spinOnce(); // grab latest callback data
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_ERROR_STREAM("drive_to_object : Preempted");
        as_.setPreempted();
        x_axis.setEnable(false);
        return;
      }

      closestObject = findClosestObject(latest_, goal->id, tracked_object_id);
      geometry_msgs::PointStamped base_link_point;
      if (closestObject != std::nullopt) {
        geometry_msgs::PointStamped object_point;
        object_point.point.x = closestObject->points[0].point[0];
        object_point.point.y = closestObject->points[0].point[1];
        object_point.point.z = closestObject->points[0].point[2];
        object_point.header = latest_.header;
        // will go map to map but copies
        tf_buffer_.transform(object_point, latest_map_relative_detection, "odom");
      }

      x_axis.setEnable(true);
      auto map_to_baselink = tf_buffer_.lookupTransform("base_link", "odom", ros::Time::now(), ros::Duration(0.1));
      tf2::doTransform(latest_map_relative_detection, base_link_point, map_to_baselink);
      // ROS_WARN_STREAM("Base link pt " << base_);

      ROS_INFO_STREAM("Object is at x,y " << base_link_point.point.x << "," << base_link_point.point.y);
      ROS_INFO_STREAM("We are at x,y " << map_to_baselink.transform.translation.x << "," << map_to_baselink.transform.translation.y);
      ROS_INFO_STREAM("Distance away " << goal->distance_away);
      x_axis.setState(-base_link_point.point.x);

      x_axis.setCommand(-goal->distance_away);

      x_error_ = fabs(goal->distance_away - base_link_point.point.x);
      // checking if it is 900, we only want to set this value once and then keep it. 
      if (x_error_ > distance_to_hold_angle_ || field_relative_object_angle > 899) {
        field_relative_object_angle = latest_yaw_ + atan2(base_link_point.point.y, base_link_point.point.x);
        ROS_INFO_STREAM("SETTING ANGLE TO " << field_relative_object_angle);
      }

      msg.data = field_relative_object_angle;
      orientation_command_pub_.publish(msg);

      angle_error_ = fabs(field_relative_object_angle - latest_yaw_);
      
      ROS_INFO_STREAM("Angle error is " << angle_error_ << " field_rel " << field_relative_object_angle << " latest yaw " << latest_yaw_);
      // note: need to account for spinning, because that changes the direction we're pointing --> changes what command we need to send.
      // could try to do the math, but i'm not sure how we'd calculate that.
      // also, we'd probably need to control linear acceleration (since linear velocity has to dynamically change :/)
      // seems like the correct way to do it is just based on how far we've turned since the last vision update:

      geometry_msgs::Twist pid_twist;
      // t.linear.x = x_eff_ - (ros::Time::now() - latest_.header.stamp).toSec() * std::cos(orient_effort_);
      pid_twist.linear.x = x_eff_ * cos(field_relative_object_angle - latest_yaw_);
      pid_twist.linear.y = x_eff_ * sin(field_relative_object_angle - latest_yaw_);
      pid_twist.angular.z = orient_effort_;

      double timeDelta = (ros::Time::now() - start).toSec();
      double initialVelocityInfluence = timeDelta > velocity_ramp_time_ ? 0.0 : ((velocity_ramp_time_ - timeDelta) / velocity_ramp_time_);

      std::vector<WeightedCmdVel> weighted_cmd_vels;
      weighted_cmd_vels.push_back({pid_twist, 1.0 - initialVelocityInfluence});
      weighted_cmd_vels.push_back({initial_cmd_vel, initialVelocityInfluence});
      geometry_msgs::Twist t = weightedAverageCmdVel(weighted_cmd_vels);

      cmd_vel_pub_.publish(t);
      feedback_.x_error = x_error_;
      feedback_.angle_error = angle_error_;
      as_.publishFeedback(feedback_);
      r.sleep();
    }
    geometry_msgs::Twist t;
    t.linear.x = 0;
    t.linear.y = 0;
    t.angular.z = 0;
    cmd_vel_pub_.publish(t);
    x_axis.setEnable(false);
    result_.success = true;
    as_.setSucceeded(result_);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "drive_to_object");

  DriveToObjectActionServer alignToObject("drive_to_object");
  ros::spin();

  return 0;
}