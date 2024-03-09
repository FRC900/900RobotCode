#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>

#include <std_srvs/Empty.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>


namespace tf2
{
    inline
    void convert(const geometry_msgs::Transform& trans, geometry_msgs::Pose& pose)
    {
        pose.orientation = trans.rotation;
        pose.position.x = trans.translation.x;
        pose.position.y = trans.translation.y;
        pose.position.z = trans.translation.z;
    }

    inline
    void convert(const geometry_msgs::Pose& pose, geometry_msgs::Transform& trans)
      {
        trans.rotation = pose.orientation;
        trans.translation.x = pose.position.x;
        trans.translation.y = pose.position.y;
        trans.translation.z = pose.position.z;
    }

    inline
    void convert(const geometry_msgs::TransformStamped& trans, geometry_msgs::PoseStamped& pose)
    {
        convert(trans.transform, pose.pose);
        pose.header = trans.header;
    }

    inline
    void convert(const geometry_msgs::PoseStamped& pose, geometry_msgs::TransformStamped& trans)
    {
        convert(pose.pose, trans.transform);
        trans.header = pose.header;
    }
}

std::string odom_frame_id = "odom";
std::string map_frame_id = "map";
std::string tagslam_baselink = "frc_robot";

std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tfbr;
ros::Publisher last_relocalized_pub;

tf2_ros::Buffer tf_buffer;
geometry_msgs::TransformStamped map_odom_tf;

ros::Time last_tf_pub = ros::Time(0);

double transform_timeout;
double maximum_jump;
double max_z;
double cmd_vel_threshold;
double ang_vel_threshold;
double time_stopped;
double publish_frequency;
bool localize_while_stopped;

void updateMapOdomTf() {
    ros::spinOnce();
    // lookup odom->base_link transform
    geometry_msgs::TransformStamped base_link_to_odom_tf;
    // lookup map->base_link transform
    geometry_msgs::TransformStamped base_link_to_map_tf;
    try {
      // invert base_link to odom
      base_link_to_map_tf = tf_buffer.lookupTransform(tagslam_baselink, map_frame_id, ros::Time(0));
      // get base_link -> odom transform at the timestamp of the tagslam transform
      // maybe this allows us to relocalize while moving, since we know where we were?
      base_link_to_odom_tf = tf_buffer.lookupTransform("base_link", odom_frame_id, base_link_to_map_tf.header.stamp);
    } catch (const tf2::TransformException &ex) {
      ROS_ERROR_STREAM_THROTTLE(5, "map to odom : transform from base_link to " << odom_frame_id << " failed in map->odom broadcaser : " << ex.what());
      return;
    }

    bool invalid = false;

    if ((ros::Time::now() - base_link_to_odom_tf.header.stamp) > ros::Duration(transform_timeout)) {
      ROS_ERROR_STREAM_THROTTLE(0.5, "map_to_odom: odom transform too old! " << base_link_to_odom_tf.header.stamp << " delta " << (ros::Time::now() - base_link_to_odom_tf.header.stamp));
      invalid = true;
    }

    if ((ros::Time::now() - base_link_to_map_tf.header.stamp) > ros::Duration(transform_timeout)) {
      ROS_ERROR_STREAM_THROTTLE(0.5, "map_to_odom: tagslam transform too old! " << base_link_to_map_tf.header.stamp << " delta " << (ros::Time::now() - base_link_to_map_tf.header.stamp));
      invalid = true;
    }

    if (invalid) {
      return;
    }

    geometry_msgs::PoseStamped odom_to_map; // ans
    geometry_msgs::PoseStamped base_link_to_map;
    tf2::Transform base_link_to_odom_transform;
    geometry_msgs::Pose pose;

    try {
      // Subtract out odom->base_link from calculated prediction map->base_link,
      // leaving a map->odom transform to broadcast
      // Borrowed from similar code in
      // https://github.com/ros-planning/navigation/blob/noetic-devel/amcl/src/amcl_node.cpp
      //tf2::Transform tmp_tf(q, pos);
      //geometry_msgs::PoseStamped baselink_to_map;
      //baselink_to_map.header.frame_id = "base_link";
      //baselink_to_map.header.stamp = last_camera_stamp;
      //tf2::toMsg(tmp_tf.inverse(), baselink_to_map.pose);

      // convert baselink to map to a poseStamped


      // MAP -> frcrobot
      // odom -> base link

      // baselink_to_map transformed from base_link to odom == odom->map
      tf2::convert(base_link_to_map_tf, base_link_to_map);
      ROS_INFO_STREAM_THROTTLE(2, "Base link to map " << base_link_to_map);
      tf2::convert(base_link_to_odom_tf.transform, base_link_to_odom_transform);
      ROS_INFO_STREAM_THROTTLE(2, "Base link to odom " << base_link_to_odom_tf.transform);
      //tf2::convert(base_link_to_odom_tf, base_link_to_odom_transform);
      base_link_to_odom_transform = base_link_to_odom_transform.inverse(); 

      tf2::convert(base_link_to_odom_transform, base_link_to_odom_tf.transform);
      ROS_INFO_STREAM_THROTTLE(2, "Base link to odom after inv " << base_link_to_odom_tf.transform);


      //tf_buffer.transform(baselink_to_map, odom_to_map, odom_frame_id);
      // odom to map is out param (supposdly)

	    tf2::doTransform(base_link_to_map, odom_to_map, base_link_to_odom_tf);
      
      geometry_msgs::TransformStamped transformStamped;

      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = map_frame_id;
      transformStamped.child_frame_id = odom_frame_id;

      // Computed transform is odom->map, but need to invert
      // it to publish map->odom instead
      // TODO, should zero Z right here
      tf2::Transform odom_to_map_tf;
      tf2::convert(odom_to_map.pose, odom_to_map_tf);
      tf2::convert(odom_to_map_tf.inverse(), transformStamped.transform);

      if (std::hypot(transformStamped.transform.translation.x - map_odom_tf.transform.translation.x, transformStamped.transform.translation.y - map_odom_tf.transform.translation.y) > maximum_jump) {
        ROS_ERROR_STREAM_THROTTLE(0.5, "map_to_odom: jump is too big! not saving transform");
        return;
      }

      if (fabs(transformStamped.transform.translation.z) > max_z) {
        ROS_ERROR_STREAM_THROTTLE(0.5, "map_to_odom: z value too high! " << transformStamped.transform.translation.z << " Not saving transofrm");
        return;
      }

      map_odom_tf = transformStamped;
      //tfbr->sendTransform(transformStamped);
    } catch (const tf2::TransformException &ex) {
      ROS_ERROR_STREAM_THROTTLE(5, "Line 99 Map to odom : transform from base_link to " << odom_frame_id << " failed in map->odom broadcaser : " << ex.what());
    }
}


bool service_cb(std_srvs::Empty::Request &/*req*/, std_srvs::Empty::Response &/*res*/) {
  updateMapOdomTf();
  if (map_odom_tf.header.frame_id == map_frame_id && (ros::Time::now() - last_tf_pub).toSec() > (1./publish_frequency)) {
    tfbr->sendTransform(map_odom_tf);
    std_msgs::Header msg;
    msg.stamp = ros::Time::now();
    last_relocalized_pub.publish(msg);
    last_tf_pub = ros::Time::now();
  }
  else {
    ROS_ERROR_STREAM("Condition not met to publish transform in service. now = " << ros::Time::now() << ", last = " << last_tf_pub);
  }
  ROS_WARN_STREAM("Publishing static map odom tf in service");
  return true;
}

void cmdVelCallback(const geometry_msgs::TwistStampedConstPtr &msg) {
  if (localize_while_stopped) {
    if (hypot(msg->twist.linear.x, msg->twist.linear.y) > cmd_vel_threshold && fabs(msg->twist.angular.z) > ang_vel_threshold) {
      last_tf_pub = ros::Time::now() + ros::Duration(time_stopped);
      
    }
    if (hypot(msg->twist.linear.x, msg->twist.linear.y) < cmd_vel_threshold && fabs(msg->twist.angular.z) < ang_vel_threshold) {
      updateMapOdomTf();
      if (map_odom_tf.header.frame_id == map_frame_id && (ros::Time::now() - last_tf_pub).toSec() > (1./publish_frequency)) {
        ROS_INFO_STREAM_THROTTLE(2, "RELOCALIZING"); 
        tfbr->sendTransform(map_odom_tf);
        std_msgs::Header msg;
        msg.stamp = ros::Time::now();
        last_relocalized_pub.publish(msg);
        last_tf_pub = ros::Time::now();
      }
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_to_odom_node");
  ros::NodeHandle nh_;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  // static_broadcaster.sendTransform(static_transformStamped);
  tfbr = std::make_unique<tf2_ros::StaticTransformBroadcaster>();
  
  if (!nh_.getParam("transform_timeout", transform_timeout))
  {
    ROS_ERROR_STREAM("map_to_odom: could not find transform_timeout, exiting");
    return -1;
  }

  if (!nh_.getParam("maximum_jump", maximum_jump))
  {
    ROS_ERROR_STREAM("map_to_odom: could not find maximum_jump, exiting");
    return -1;
  }

  if (!nh_.getParam("max_z", max_z))
  {
    ROS_ERROR_STREAM("map_to_odom: could not find max_z, exiting");
    return -1;
  }

  if (!nh_.getParam("localize_while_stopped", localize_while_stopped))
  {
    ROS_ERROR_STREAM("map_to_odom: could not find localize_while_stopped, exiting");
    return -1;
  }

  if (!nh_.getParam("cmd_vel_threshold", cmd_vel_threshold))
  {
    ROS_ERROR_STREAM("map_to_odom: could not find cmd_vel_threshold, exiting");
    return -1;
  }

  if (!nh_.getParam("ang_vel_threshold", ang_vel_threshold))
  {
    ROS_ERROR_STREAM("map_to_odom: could not find ang_vel_threshold, exiting");
    return -1;
  }

  if (!nh_.getParam("time_stopped", time_stopped))
  {
    ROS_ERROR_STREAM("map_to_odom: could not find time_stopped, exiting");
    return -1;
  }

  if (!nh_.getParam("publish_frequency", publish_frequency))
  {
    ROS_ERROR_STREAM("map_to_odom: could not find publish_frequency, exiting");
    return -1;
  }

  // write a timer that gets called at 25Hz
  // write a service that takes in an empty message and publishes the tf
  ros::ServiceServer service = nh_.advertiseService("tagslam_pub_map_to_odom", service_cb);
  ros::Subscriber cmd_vel_sub = nh_.subscribe("/frcrobot_jetson/swerve_drive_controller/cmd_vel_out", 1, cmdVelCallback);
  last_relocalized_pub = nh_.advertise<std_msgs::Header>("/last_relocalize", 1, true);
  updateMapOdomTf();
  if (map_odom_tf.header.frame_id == map_frame_id) {
    tfbr->sendTransform(map_odom_tf);
    std_msgs::Header msg;
    msg.stamp = ros::Time::now();
    last_relocalized_pub.publish(msg);
    last_tf_pub = ros::Time::now();
  }
  ros::spin();

  return 0;
}
