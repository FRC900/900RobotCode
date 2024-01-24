#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>

#include <std_srvs/Empty.h>


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
ros::Duration tf_tolerance = ros::Duration(0.1);
tf2_ros::Buffer tf_buffer;
geometry_msgs::TransformStamped map_odom_tf;


void updateMapOdomTf() {
    // lookup odom->base_link transform
    geometry_msgs::TransformStamped base_link_to_odom_tf;
    // lookup map->base_link transform
    geometry_msgs::TransformStamped base_link_to_map_tf;
    try {
      // dest source?
      base_link_to_odom_tf = tf_buffer.lookupTransform("base_link", odom_frame_id, ros::Time(0));
      // invert base_link to odom
      // 
      base_link_to_map_tf = tf_buffer.lookupTransform(tagslam_baselink, map_frame_id, ros::Time(0));
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
      map_odom_tf = transformStamped;
      //tfbr->sendTransform(transformStamped);
    } catch (const tf2::TransformException &ex) {
      ROS_ERROR_STREAM_THROTTLE(5, "Line 99 Map to odom : transform from base_link to " << odom_frame_id << " failed in map->odom broadcaser : " << ex.what());
    }
}

void timerCallback(const ros::TimerEvent &event) {
  map_odom_tf.header.stamp = ros::Time::now();
  // tfbr->sendTransform(map_odom_tf);
  ROS_WARN_STREAM_THROTTLE(2, "Publishing map odom tf in timer");
}

bool service_cb(std_srvs::Empty::Request &/*req*/, std_srvs::Empty::Response &/*res*/) {
  updateMapOdomTf();
  tfbr->sendTransform(map_odom_tf);
  ROS_WARN_STREAM("Publishing static map odom tf in service");
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_to_odom_node");
  ros::NodeHandle nh_;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  // static_broadcaster.sendTransform(static_transformStamped);
  tfbr = std::make_unique<tf2_ros::StaticTransformBroadcaster>();
  
  // write a timer that gets called at 25Hz
  // ros::Timer timer = nh_.createTimer(ros::Duration(0.1), timerCallback);
  // write a service that takes in an empty message and publishes the tf
  ros::ServiceServer service = nh_.advertiseService("tagslam_pub_map_to_odom", service_cb);
  updateMapOdomTf();
  tfbr->sendTransform(map_odom_tf);
  ros::spin();

  return 0;
}
