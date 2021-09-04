#define USE_MATH_DEFINES_
#include "pf_localization/particle_filter.hpp"
#include "pf_localization/world_model.hpp"
#include "pf_localization/particle.hpp"
#include "pf_localization/pf_pose.h"
#include "pf_localization/pf_debug.h"
#include "field_obj/Detection.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <XmlRpcValue.h>
#include <cmath>
#include <memory>

#define VERBOSE
// #define EXTREME_VERBOSE

const std::string rot_topic = "/imu/zeroed_imu";
const std::string cmd_topic = "/frcrobot_jetson/swerve_drive_controller/cmd_vel_out";
const std::string goal_pos_topic = "/goal_detection/goal_detect_msg";

const std::string pub_debug_topic = "pf_debug";
const std::string pub_topic = "predicted_pose";

std::string odom_frame_id = "odom";
std::string map_frame_id = "map";
std::unique_ptr<tf2_ros::TransformBroadcaster> tfbr;
ros::Duration tf_tolerance;
ros::Publisher pub;
ros::Publisher pub_debug;

tf2_ros::Buffer tf_buffer_;

ros::Time last_time;
ros::Time last_measurement;
double rot = 0;
double noise_delta_t = 0;  // if the time since the last measurement is greater than this, positional noise will not be applied
std::unique_ptr<ParticleFilter> pf;

void rotCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  double roll, pitch, yaw;
  tf2::Quaternion raw;
  tf2::convert(msg -> orientation, raw);
  tf2::Matrix3x3(raw).getRPY(roll, pitch, yaw);
  pf->set_rotation(yaw);
  #ifdef EXTREME_VERBOSE
  ROS_INFO("rotCallback called");
  #endif
}

void publish_prediction(const ros::TimerEvent &/*event*/)
{
  const Particle prediction = pf->predict();

  // Publish message with predicted pose
  // TODO - see if std ROS odom message is a better fit than a custom messge
  pf_localization::pf_pose pose;
  pose.x = prediction.x_;
  pose.y = prediction.y_;
  pose.rot = prediction.rot_;
  pub.publish(pose);

  // Publish map->odom transform describing this position
  const double tmp = prediction.x_ + prediction.y_ + prediction.rot_;
  if (!std::isnan(tmp) && !std::isinf(tmp))
  {
    geometry_msgs::PoseStamped odom_to_map;
    try
    {
      // Subtract out odom->base_link from calculated prediction map->base_link,
      // leaving a map->odom transform to broadcast
      // Borrowed from similar code in
      // https://github.com/ros-planning/navigation/blob/noetic-devel/amcl/src/amcl_node.cpp
      tf2::Quaternion q;
      q.setRPY(0, 0, prediction.rot_);
      tf2::Transform tmp_tf(q, tf2::Vector3(prediction.x_, prediction.y_, 0.0));

      geometry_msgs::PoseStamped baselink_to_map;
      baselink_to_map.header.frame_id = "base_link";
      baselink_to_map.header.stamp = last_time;
      tf2::toMsg(tmp_tf.inverse(), baselink_to_map.pose);

      // baselink_to_map transformed from base_link to odom == odom->map
      tf_buffer_.transform(baselink_to_map, odom_to_map, odom_frame_id, ros::Duration(0.02));
    }
    catch (const tf2::TransformException &ex)
    {
      ROS_ERROR_STREAM("pf_localization : transform from base_link to " << odom_frame_id << " failed in map->odom broadcaser : " << ex.what());
    }

    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = last_time + tf_tolerance;
    transformStamped.header.frame_id = map_frame_id;
    transformStamped.child_frame_id = odom_frame_id;

    // Computed transform is odom->map, but need to invert
    // it to publish map->odom instead
    tf2::Transform odom_to_map_tf;
    tf2::convert(odom_to_map.pose, odom_to_map_tf);
    tf2::convert(odom_to_map_tf.inverse(), transformStamped.transform);

    tfbr->sendTransform(transformStamped);
  }

  if(pub_debug.getNumSubscribers() > 0){
    pf_localization::pf_debug debug;

    for (const Particle& p : pf->get_particles()) {
      pf_localization::pf_pose particle;
      particle.x = p.x_;
      particle.y = p.y_;
      particle.rot = p.rot_;
      debug.particles.push_back(particle);
    }

    pub_debug.publish(debug);
  }
}

// Extra arg here allows for switching between bearing only vs. 2d x,y position
// detection messages based on how the subscribe call is defined
void goalCallback(const field_obj::Detection::ConstPtr& msg, const bool bearingOnly){
  // TODO - just transform all of the detection coords to base_link here,
  // remove the need to do so inside the particle filter
  geometry_msgs::TransformStamped zed_to_baselink;
  try {
    zed_to_baselink = tf_buffer_.lookupTransform("base_link", msg->header.frame_id, ros::Time::now(), ros::Duration(0.02));
  }
  catch (const tf2::TransformException &ex){
  }

  std::vector<std::shared_ptr<BeaconBase>> measurement;
  geometry_msgs::Point location;
  for(const field_obj::Object& p : msg->objects) {
	tf2::doTransform(p.location, location,zed_to_baselink);
    if(bearingOnly) {
      measurement.push_back(std::make_shared<BearingBeacon>(location.x, location.y, p.id));
    } else {
      measurement.push_back(std::make_shared<PositionBeacon>(location.x, location.y, p.id));
    }
  }

  if (pf->assign_weights(measurement)) {
    pf->resample();
    last_measurement = ros::Time::now();
  }

  #ifdef EXTREME_VERBOSE
  ROS_INFO("goalCallback called");
  #endif
}

void cmdCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
  double timestep = (msg->header.stamp - last_time).toSec();
  double x_vel = msg->twist.linear.x;
  double y_vel = msg->twist.linear.y;

  double delta_x = x_vel * timestep;
  double delta_y = y_vel * timestep;

  last_time = msg->header.stamp;

  // TODO - check return code
  pf->motion_update(delta_x, delta_y, 0);
  if ((ros::Time::now() - last_measurement).toSec() < noise_delta_t) {
    pf->noise_pos();
    pf->noise_rot();
  }

  #ifdef EXTREME_VERBOSE
  ROS_INFO("cmdCallback called");
  #endif
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pf_localization_node");
  ros::NodeHandle nh_;
  tf2_ros::TransformListener tf_listener_(tf_buffer_);

  #ifdef VERBOSE
  ROS_INFO_STREAM(nh_.getNamespace());
  #endif

  XmlRpc::XmlRpcValue xml_beacons;
  double f_x_min, f_x_max, f_y_min, f_y_max, i_x_min, i_x_max, i_y_min, i_y_max, p_stdev, r_stdev;
  int num_particles;
  double tmp_tolerance = 0.1;

  std::vector<PositionBeacon> beacons;

  if (!nh_.getParam("noise_delta_t", noise_delta_t)) {
    ROS_ERROR("noise_delta_t not specified");
    return -1;
  }

  if (!nh_.getParam("num_particles", num_particles)) {
    ROS_ERROR("num_particles not specified");
    return -1;
  }
  if (!nh_.getParam("beacons", xml_beacons)) {
    ROS_ERROR("failed to load beacons");
    return -1;
  }

  if (!nh_.getParam("field_dims/x_min", f_x_min)) {
    ROS_ERROR("field dimension not specified");
    return -1;
  }
  if (!nh_.getParam("field_dims/x_max", f_x_max)) {
    ROS_ERROR("field dimension not specified");
    return -1;
  }
  if (!nh_.getParam("field_dims/y_min", f_y_min)) {
    ROS_ERROR("field dimension not specified");
    return -1;
  }
  if (!nh_.getParam("field_dims/y_max", f_y_max)) {
    ROS_ERROR("field dimension not specified");
    return -1;
  }

  ROS_INFO_STREAM("field dims assigned");

  if (!nh_.getParam("init_dims/x_min", i_x_min)) {
    ROS_ERROR("particle initialization dimension not specified");
    return -1;
  }
  if (!nh_.getParam("init_dims/x_max", i_x_max)) {
    ROS_ERROR("particle initialization dimension not specified");
    return -1;
  }
  if (!nh_.getParam("init_dims/y_min", i_y_min)) {
    ROS_ERROR("particle initialization dimension not specified");
    return -1;
  }
  if (!nh_.getParam("init_dims/y_max", i_y_max)) {
    ROS_ERROR("particle initialization dimension not specified");
    return -1;
  }

  ROS_INFO("initialization dims assigned");

  if (!nh_.param("noise_stdev/position", p_stdev, 0.1)) {
    ROS_WARN("no position stdev specified, using defalut");
  }
  if (!nh_.param("noise_stdev/rotation", r_stdev, 0.1)) {
    ROS_WARN("no rotation stdev specified, using defalut");
  }

  ROS_INFO("noise stdevs assigned");

  ROS_INFO_STREAM(f_x_min << ' ' << i_x_min << ' ' << p_stdev);

  nh_.param("map_frame_id", map_frame_id, std::string("map"));
  nh_.param("odom_frame_id", odom_frame_id, std::string("odom"));
  nh_.param("tf_tolerance", tmp_tolerance, 0.1);
  tf_tolerance.fromSec(tmp_tolerance);


  // TODO - I think this fails if a beacon is specified as an int
  for (size_t i = 0; i < (unsigned) xml_beacons.size(); i++) {
    PositionBeacon b {xml_beacons[i][0], xml_beacons[i][1], xml_beacons[i][2]};
    beacons.push_back(b);
  }

  WorldModel world(beacons, f_x_min, f_x_max, f_y_min, f_y_max);
  pf = std::make_unique<ParticleFilter>(world,
                                        i_x_min, i_x_max, i_y_min, i_y_max,
                                        p_stdev, r_stdev,
                                        num_particles);

  #ifdef VERBOSE
  for (const Particle& p : pf->get_particles()) {
	ROS_INFO_STREAM(p);
  }
  ROS_INFO_STREAM("\n\n");
  ROS_INFO_STREAM(pf->predict());
  ROS_INFO("pf localization initialized");
  #endif

  ros::Subscriber rot_sub = nh_.subscribe(rot_topic, 1, rotCallback);
  ros::Subscriber odom_sub = nh_.subscribe(cmd_topic, 1, cmdCallback);
  ros::Subscriber goal_sub = nh_.subscribe<field_obj::Detection>(goal_pos_topic, 1, boost::bind(goalCallback, _1, false));

  pub = nh_.advertise<pf_localization::pf_pose>(pub_topic, 1);
  pub_debug = nh_.advertise<pf_localization::pf_debug>(pub_debug_topic, 1);

  tfbr = std::make_unique<tf2_ros::TransformBroadcaster>();
  double publish_rate;
  nh_.param("publish_rate", publish_rate, 10.);
  auto pubTimer = nh_.createTimer(ros::Duration(1./ publish_rate), publish_prediction);

  const auto now = ros::Time::now();
  last_time = now;
  last_measurement = now;
  ros::spin();

  return 0;
}
