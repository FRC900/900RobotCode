#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include "behavior_actions/RelocalizePoint.h"
#include "field_obj/Detection.h"
#include <boost/circular_buffer.hpp>

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
geometry_msgs::TransformStamped relocalized_point_tf;

ros::Subscriber front_tags_sub;
ros::Subscriber back_tags_sub;

ros::Time last_tf_pub = ros::Time(0);

double transform_timeout;

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
                                                                                    // look at changing this to ros::Time(0)
      base_link_to_odom_tf = tf_buffer.lookupTransform("base_link", odom_frame_id, base_link_to_map_tf.header.stamp);
    } catch (const tf2::TransformException &ex) {
      ROS_ERROR_STREAM_THROTTLE(5, "map to odom : transform from base_link to " << odom_frame_id << " failed in map->odom broadcaser : " << ex.what());
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

      transformStamped.header.stamp = base_link_to_map_tf.header.stamp;
      transformStamped.header.frame_id = map_frame_id;
      transformStamped.child_frame_id = odom_frame_id;

      // Computed transform is odom->map, but need to invert
      // it to publish map->odom instead
      // TODO, should zero Z right here
      tf2::Transform odom_to_map_tf;
      tf2::convert(odom_to_map.pose, odom_to_map_tf);
      tf2::convert(odom_to_map_tf.inverse(), transformStamped.transform);

      // ROS_INFO_STREAM("map_to_odom: transform x = " << transformStamped.transform.translation.x << ", y = " << transformStamped.transform.translation.y << ", z = " << transformStamped.transform.translation.z << " at time " << transformStamped.header.stamp << " from " << base_link_to_map_tf.header.stamp << " to " << base_link_to_odom_tf.header.stamp);

      map_odom_tf = transformStamped;
      //tfbr->sendTransform(transformStamped);
    } catch (const tf2::TransformException &ex) {
      ROS_ERROR_STREAM_THROTTLE(5, "Line 99 Map to odom : transform from base_link to " << odom_frame_id << " failed in map->odom broadcaser : " << ex.what());
    }
    tfbr->sendTransform(map_odom_tf);
}

void timer_cb(const ros::TimerEvent& event) {
  last_tf_pub = ros::Time::now();
  updateMapOdomTf();
}

// Should never have to "fake" localization
bool relocalize_to_point_cb(behavior_actions::RelocalizePoint::Request &req, behavior_actions::RelocalizePoint::Response &/*res*/) {
  ROS_INFO_STREAM("map_to_odom: relocalizing to a point at x,y " << req.pose.position.x << ", " << req.pose.position.y);
  
  relocalized_point_tf.header.stamp = ros::Time::now();
  relocalized_point_tf.header.frame_id = map_frame_id;
  relocalized_point_tf.child_frame_id = odom_frame_id;
  relocalized_point_tf.transform.rotation = req.pose.orientation;
  relocalized_point_tf.transform.translation.x = req.pose.position.x;
  relocalized_point_tf.transform.translation.y = req.pose.position.y;

  tfbr->sendTransform(relocalized_point_tf);
  last_tf_pub = ros::Time::now();
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_to_odom_node");
  ros::NodeHandle nh_;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  tfbr = std::make_unique<tf2_ros::StaticTransformBroadcaster>();

  if (!nh_.getParam("transform_timeout", transform_timeout))
  {
    ROS_ERROR_STREAM("map_to_odom: could not find transform_timeout, exiting");
    return -1;
  }
  
  updateMapOdomTf();
  if (map_odom_tf.header.frame_id == map_frame_id) {
    tfbr->sendTransform(map_odom_tf);
    last_tf_pub = ros::Time::now();
  }
  // make a timer that runs at 250hz and updates the tagslam transform 
  ros::Timer timer = nh_.createTimer(ros::Duration(1/250), timer_cb);

  ros::spin();

  return 0;
}
