#define USE_MATH_DEFINES_
#include "pf_localization/particle_filter.hpp"
#include "pf_localization/world_model.hpp"
#include "pf_localization/particle.hpp"
#include "pf_localization/pf_msgs.hpp"
#include "pf_localization/PFDebug.h"
#include "field_obj/Detection.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include "ddynamic_reconfigure/ddynamic_reconfigure.h"
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <XmlRpcValue.h>
#include <cmath>
#include <memory>
#include <frc_msgs/MatchSpecificData.h>
#include <std_srvs/Empty.h>

#define VERBOSE
// #define EXTREME_VERBOSE

const std::string rot_topic = "/imu/zeroed_imu";
const std::string cmd_topic = "/frcrobot_jetson/swerve_drive_controller/cmd_vel_out";
const std::string goal_pos_topic = "/goal_detection/goal_detect_msg";
const std::string match_topic = "/frcrobot_rio/match_data";

const std::string pub_debug_topic = "pf_debug";
const std::string pub_topic = "predicted_pose";

const std::string reinit_pf_service = "re_init_pf";

std::string odom_frame_id = "odom";
std::string map_frame_id = "map";
std::unique_ptr<tf2_ros::TransformBroadcaster> tfbr;
ros::Duration tf_tolerance;
ros::Publisher pub;
ros::Publisher pub_debug;

tf2_ros::Buffer tf_buffer;

ros::Time last_cmd_vel;
ros::Time last_camera_data;
ros::Time last_camera_stamp;
double noise_delta_t = 0;  // if the time since the last measurement is greater than this, positional noise will not be applied
std::vector<double> sigmas; // std.dev for each dimension of detected beacon position (x&y for depth camera, angle for bearing-only)
double min_detection_confidence{0};
std::unique_ptr<ParticleFilter> pf;
double stopped_pos_noise_multiplier{1.0};
double stopped_rot_noise_multiplier{1.0};
double moving_pos_noise_multiplier{1.0};
double moving_rot_noise_multiplier{1.0};

// How long the transition from moving to stopped should
// take. This transition is for scaling between the moving
// and stopped multipliers above. There's a linear scaling
// between moving and stopped multiplers during this time to 
// let the localization converge.  0.0 means no transition and
// make a step transition between moving and stopped multipliers.
double    stopped_transition_time{0.0};
ros::Time last_moving_stamp{};

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

void publish_prediction(const ros::TimerEvent &event)
{
  const auto p = pf->predict();
  if (!p) {
    ROS_WARN_THROTTLE(1, "PF Prediction returned null");
    return;
  }
  const geometry_msgs::PoseWithCovariance &prediction = *p;
  geometry_msgs::PoseWithCovarianceStamped predictionStamped;

  predictionStamped.pose = prediction;
  predictionStamped.header.stamp = event.current_real;
  predictionStamped.header.frame_id = map_frame_id;

  pub.publish(predictionStamped);

  // Publish map->odom transform describing this position
  const tf2::Quaternion q(prediction.pose.orientation.x,
    prediction.pose.orientation.y,
    prediction.pose.orientation.z,
    prediction.pose.orientation.w);

  tf2::Vector3 pos(prediction.pose.position.x,
    prediction.pose.position.y,
    0.0);

  if (const double tmp = pos.getX() + pos.getY() + pos.getZ() + q.getX() + q.getY() + q.getZ() + q.getW();
      std::isfinite(tmp))
  {
    geometry_msgs::PoseStamped odom_to_map;
    try {
      // Subtract out odom->base_link from calculated prediction map->base_link,
      // leaving a map->odom transform to broadcast
      // Borrowed from similar code in
      // https://github.com/ros-planning/navigation/blob/noetic-devel/amcl/src/amcl_node.cpp
      tf2::Transform tmp_tf(q, pos);

      geometry_msgs::PoseStamped baselink_to_map;
      baselink_to_map.header.frame_id = "base_link";
      baselink_to_map.header.stamp = last_camera_stamp;
      tf2::toMsg(tmp_tf.inverse(), baselink_to_map.pose);

      // baselink_to_map transformed from base_link to odom == odom->map
      tf_buffer.transform(baselink_to_map, odom_to_map, odom_frame_id);

      geometry_msgs::TransformStamped transformStamped;

      transformStamped.header.stamp = last_cmd_vel + tf_tolerance;
      transformStamped.header.frame_id = map_frame_id;
      transformStamped.child_frame_id = odom_frame_id;

      // Computed transform is odom->map, but need to invert
      // it to publish map->odom instead
      tf2::Transform odom_to_map_tf;
      tf2::convert(odom_to_map.pose, odom_to_map_tf);
      tf2::convert(odom_to_map_tf.inverse(), transformStamped.transform);

      tfbr->sendTransform(transformStamped);
    } catch (const tf2::TransformException &ex) {
      ROS_ERROR_STREAM_THROTTLE(5, "pf_localization : transform from base_link to " << odom_frame_id << " failed in map->odom broadcaser : " << ex.what());
    }
  }

  if(pub_debug.getNumSubscribers() > 0){
    pf_localization::PFDebug debug;

    const auto particles = pf->get_particles();
    for (const Particle& particle : particles) {
      debug.poses.push_back(toPose(particle));
    }

    const auto beacons = pf->get_beacons_seen();
    for (const auto &b : beacons) {
      debug.beacons.push_back(toPose(b));
    }
    pf->clear_beacons_seen();

    debug.predicted_pose = prediction.pose;

    debug.header.frame_id = map_frame_id;
    debug.header.stamp = event.current_real;
    pub_debug.publish(debug);
  }
}

// Extra arg here allows for switching between bearing only vs. 2d x,y position
// detection messages based on how the subscribe call is defined
void goalCallback(const field_obj::Detection::ConstPtr& msg, const bool bearingOnly){
  last_camera_stamp = msg->header.stamp;
  // TODO - just transform all of the detection coords to base_link here,
  // remove the need to do so inside the particle filter
  geometry_msgs::TransformStamped zed_to_baselink;
  try {
    zed_to_baselink = tf_buffer.lookupTransform("base_link", msg->header.frame_id, ros::Time::now(), ros::Duration(0.02));
  }
  catch (const tf2::TransformException &ex) {
    ROS_ERROR_STREAM_THROTTLE(5, "PF localization not running - fix base_link to " << msg->header.frame_id << " transform");
    return;
  }

  std::vector<std::shared_ptr<BeaconBase>> measurement;
  geometry_msgs::Point location;
  for(const field_obj::Object& p : msg->objects) {
    if (p.confidence < min_detection_confidence)
    {
      continue;
    }
    // Filter out detections for which there is no beacon info.  This could be because
    // the object doesn't have a fixed position (game pieces or robots, etc) or maybe
    // we just don't have that beacon defined. Filtering those detections out here
    // saves a lot of time processing them later
    if (!pf->is_valid_beacon(p.id))
    {
      continue;
    }
    tf2::doTransform(p.location, location, zed_to_baselink);
    if(bearingOnly) {
      measurement.push_back(std::make_shared<BearingBeacon>(location.x, location.y, p.id));
    } else { 
      measurement.push_back(std::make_shared<PositionBeacon>(location.x, location.y, p.id));
    }
  }
  // ROS_INFO_STREAM("msg.objects = " << msg->objects.size() << " measurement = " << measurement.size());

  if (pf->assign_weights(measurement, sigmas)) {
    pf->resample();
    last_camera_data = ros::Time::now();
  }

  #ifdef EXTREME_VERBOSE
  ROS_INFO("goalCallback called");
  #endif
}

void cmdCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
  double timestep = (msg->header.stamp - last_cmd_vel).toSec();
  const double x_vel = msg->twist.linear.x;
  const double y_vel = msg->twist.linear.y;
  const double z_ang_velocity = msg->twist.angular.z;

  // TODO - use the average of the previous and current velocity?
  const double delta_x = x_vel * timestep;
  const double delta_y = y_vel * timestep;
  const double delta_angular_z = z_ang_velocity * timestep;

  last_cmd_vel = msg->header.stamp;
  // TODO - check return code
  pf->motion_update(delta_x, delta_y, delta_angular_z);

  const bool stopped = (x_vel == 0.0) && (y_vel == 0.0) && (z_ang_velocity == 0.0);
  // Stop applying noise after enough time has passed since the last camera measurement
  if ((ros::Time::now() - last_camera_data).toSec() < noise_delta_t) {
    // How far along are we in the transition from moving to stopped noise multipliers
    // 0 = still moving, apply only moving noise multipliers
    // 1 = stopped, apply only stopped noise multipliers
    // in between = linear interpolation between moving and stopped noise multipliers
    double stopped_scale;

    if (!stopped) {
      stopped_scale = 0.0;
    } else {
      if (stopped_transition_time == 0.0) { // avoid divide by zero
        stopped_scale = 1.0;
      } else {
        stopped_scale = (msg->header.stamp - last_moving_stamp).toSec() / stopped_transition_time;
        stopped_scale = std::min(stopped_scale, 1.0);
      }
    }

    pf->noise_pos(stopped_scale * stopped_pos_noise_multiplier + (1.0 - stopped_scale) * moving_pos_noise_multiplier);
    pf->noise_rot(stopped_scale * stopped_rot_noise_multiplier + (1.0 - stopped_scale) * moving_rot_noise_multiplier);
  }

  if (!stopped) {
    last_moving_stamp = msg->header.stamp;
  }

  #ifdef EXTREME_VERBOSE
  ROS_INFO("cmdCallback called");
  #endif
}

void matchCallback(const frc_msgs::MatchSpecificData::ConstPtr &MatchData)
{
  if (MatchData->allianceColor == frc_msgs::MatchSpecificData::ALLIANCE_COLOR_BLUE) // 1 = blue
	{
    //ROS_INFO("Blue Alliance");
		if (pf->allianceColorCheck(true)) {
      pf->reinit();
      ROS_WARN_STREAM("PF Alliance Color Changed to Blue... reinit");
    }
	}
	else if (MatchData->allianceColor == frc_msgs::MatchSpecificData::ALLIANCE_COLOR_RED)// 0 = red
	{
    //ROS_INFO("Red Alliance");
		// if true, means that beacons have changed so pf should reinit
    if (pf->allianceColorCheck(false)) {
      pf->reinit();
      ROS_WARN_STREAM("PF Alliance Color Changed to Red... reinit");
    }
	}
}

bool handle_re_init_pf(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  pf->reinit();
  return true;
}

bool readFloatVal(XmlRpc::XmlRpcValue &param, double &val)
{
  if (param.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    val = static_cast<double>(param);
    return true;
  } else if (param.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    val = static_cast<int>(param);
    return true;
  } else {
    throw std::runtime_error("A non-double value was read for beacon cood");
  }

  return false;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pf_localization_node");
  ros::NodeHandle nh_;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  #ifdef VERBOSE
  ROS_INFO_STREAM(nh_.getNamespace());
  #endif

  XmlRpc::XmlRpcValue xml_beacons;
  double f_x_min;
  double f_x_max;
  double f_y_min;
  double f_y_max;
  double i_x_min;
  double i_x_max;
  double i_y_min;
  double i_y_max;
  double p_stdev;
  double r_stdev;
  double rotation_threshold;
  int num_particles;
  double tmp_tolerance = 0.1;
  bool bearing_only = false;

  std::vector<PositionBeacon> beacons;
  //ros::topic::waitForMessage() 	
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
    ROS_WARN("no position stdev specified, using default");
  }
  if (!nh_.param("noise_stdev/rotation", r_stdev, 0.1)) {
    ROS_WARN("no rotation stdev specified, using default");
  }
  if (!nh_.param("noise_stdev/stopped_pos_multiplier", stopped_pos_noise_multiplier, 1.0)) {
    ROS_WARN("no stopped_pos_noise_multiplier specified, using default");
  }
  if (!nh_.param("noise_stdev/stopped_rot_multiplier", stopped_rot_noise_multiplier, 1.0)) {
    ROS_WARN("no stopped_rot_noise_multiplier specified, using default");
  }
  if (!nh_.param("noise_stdev/moving_pos_multiplier", moving_pos_noise_multiplier, 1.0)) {
    ROS_WARN("no moving_pos_noise_multiplier specified, using default");
  }
  if (!nh_.param("noise_stdev/moving_rot_multiplier", moving_rot_noise_multiplier, 1.0)) {
    ROS_WARN("no moving_rot_noise_multiplier specified, using default");
  }
  if (!nh_.param("stopped_transition_time", stopped_transition_time, 1.0)) {
    ROS_WARN("no stopped_transition_time specified, using default");
  }
  if (!nh_.param("rotation_threshold", rotation_threshold, 0.25)) {
    ROS_WARN("no rotation_threshold specified, using default");
  }
  if (!nh_.param("camera_sigmas", sigmas, {0.1, 0.1})) {
    ROS_WARN("no camera stdev specified, using default");
  }

  ROS_INFO("noise stdevs assigned");

  ROS_INFO_STREAM(f_x_min << ' ' << i_x_min << ' ' << p_stdev);

  nh_.param("map_frame_id", map_frame_id, std::string("map"));
  nh_.param("odom_frame_id", odom_frame_id, std::string("odom"));
  nh_.param("tf_tolerance", tmp_tolerance, 0.1);
  double camera_fov = angles::from_degrees(110.);
  if (!nh_.param("camera_fov", camera_fov, camera_fov)) {
    ROS_WARN("no camera_fov specified, using default");
  }


  tf_tolerance.fromSec(tmp_tolerance);

  nh_.param("bearing_only", bearing_only, false);
  nh_.param("min_detection_confidence", min_detection_confidence, 0.3);
  
  // Transforms the blue relative beacons to red relative beacons using the static transform blue0 -> red0
  for (size_t i = 0; i < static_cast<size_t>(xml_beacons.size()); i++) {
    double x;
    double y;
    readFloatVal(xml_beacons[i][0], x);
    readFloatVal(xml_beacons[i][1], y);
    PositionBeacon b {x, y, xml_beacons[i][2]};
    beacons.push_back(b);
  }
  ROS_INFO_STREAM("BLUE BEACONS");
  // itterate over blue_beacons vector and print each one out
  for (auto i = beacons.begin(); i != beacons.end(); i++)
    std::cout << i->x_ << ' ' << i->y_ << ' ' << i->type_ << std::endl;

  WorldModel world(beacons, WorldModelBoundaries(f_x_min, f_x_max, f_y_min, f_y_max), camera_fov);
  pf = std::make_unique<ParticleFilter>(world,
                                        WorldModelBoundaries(i_x_min, i_x_max, i_y_min, i_y_max),
                                        p_stdev, r_stdev, rotation_threshold,
                                        num_particles);

  ddynamic_reconfigure::DDynamicReconfigure ddr(nh_);
  ddr.registerVariable<double>("p_stdev", &p_stdev, [](double ns){pf->set_pos_stddev(ns); }, "Position noise standard deviation", 0.0, 5.0);
  ddr.registerVariable<double>("r_stdev", &r_stdev, [](double rs){pf->set_rot_stddev(rs); }, "Rotation noise standard deviation", 0.0, 5.0);
  ddr.registerVariable<double>("stopped_pos_noise_multiplier", &stopped_pos_noise_multiplier, "Multiplier for position noise when robot is stopped ", 0.0, 100.0);
  ddr.registerVariable<double>("stopped_rot_noise_multiplier", &stopped_rot_noise_multiplier, "Multiplier for rotation noise when robot is stopped ", 0.0, 100.0);
  ddr.registerVariable<double>("moving_pos_noise_multiplier", &moving_pos_noise_multiplier, "Multiplier for position noise when robot is moving ", 0.0, 100.0);
  ddr.registerVariable<double>("moving_rot_noise_multiplier", &moving_rot_noise_multiplier, "Multiplier for rotation noise when robot is moving ", 0.0, 100.0);
  ddr.registerVariable<double>("stopped_transition_time", &stopped_transition_time, "Time in seconds after stopping to lineraly interpolate between moving and stopped multipliers", 0.0, 10.);
  ddr.registerVariable<double>("rotation_threshold", &rotation_threshold, [](double rt){pf->set_rotation_threshold(rt); }, "Particle rotations more than this far away from imu angle are set to IMU + noise", 0.0, angles::from_degrees(360.0));
  ddr.registerVariable<double>("camera_sigma_x", &sigmas[0], "camera_sigma_x", 0.0, 5.0);
  ddr.registerVariable<double>("camera_sigma_y", &sigmas[1], "camera_sigma_x", 0.0, 5.0);
  ddr.registerVariable<double>("noise_delta_t", &noise_delta_t, "noise_delta_t", 0.0, 5.0);
  ddr.registerVariable<double>("tf_tolerance", &tmp_tolerance, "tf_tolerance", 0.0, 5.0);
  ddr.registerVariable<double>("camera_fov", &camera_fov, "camera_fov", 0.0, angles::from_degrees(360.0));
  ddr.publishServicesTopics();

  #ifdef VERBOSE
  for (const Particle& p : pf->get_particles()) {
	  ROS_INFO_STREAM(p);
  }
  ROS_INFO_STREAM("\n\n");
  const auto prediction = pf->predict();
  if (prediction) {
    ROS_INFO_STREAM(*prediction);
  }
  ROS_INFO("pf localization initialized");
  #endif

  ros::Subscriber rot_sub = nh_.subscribe(rot_topic, 10, rotCallback);
  ros::Subscriber odom_sub = nh_.subscribe(cmd_topic, 10, cmdCallback);
  ros::Subscriber goal_sub = nh_.subscribe<field_obj::Detection>(goal_pos_topic, 10, boost::bind(goalCallback, _1, bearing_only));
  ros::Subscriber match_sub = nh_.subscribe(match_topic, 10, matchCallback);

  pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pub_topic, 1);
  ros::ServiceServer re_init_pf_server = nh_.advertiseService(reinit_pf_service, handle_re_init_pf);

  pub_debug = nh_.advertise<pf_localization::PFDebug>(pub_debug_topic, 10);

  tfbr = std::make_unique<tf2_ros::TransformBroadcaster>();
  double publish_rate;
  nh_.param("publish_rate", publish_rate, 10.);
  auto pubTimer = nh_.createTimer(ros::Duration(1./ publish_rate), publish_prediction);

  ros::Duration(1).sleep(); // for data replay, needed to get time >= 0 using rosbag clock
  const auto now = ros::Time::now();
  last_cmd_vel = now;
  last_camera_data = now;
  last_camera_stamp = now;
  ros::spin();

  return 0;
}
