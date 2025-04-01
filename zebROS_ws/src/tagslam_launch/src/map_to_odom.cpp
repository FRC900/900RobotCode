#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

ros::Time last_published{};

void updateMapOdomTf(const tf2_ros::Buffer &tf_buffer, tf2_ros::StaticTransformBroadcaster &tfbr
) {
    // This will only be called from a timer event, meaning that we should
    // already be up to date with the latest tagslam transform
    // ros::spinOnce();

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
      if (last_published == base_link_to_map_tf.header.stamp) {
        return;
      }
      last_published = base_link_to_map_tf.header.stamp;
      base_link_to_odom_tf = tf_buffer.lookupTransform("base_link", odom_frame_id, base_link_to_map_tf.header.stamp);
    } catch (const tf2::TransformException &ex) {
      ROS_ERROR_STREAM_THROTTLE(5, "map to odom : transform from base_link to " << odom_frame_id << " failed in map->odom broadcaser : " << ex.what());
      return;
    }

    try {
      // Subtract out odom->base_link from calculated prediction map->base_link,
      // leaving a map->odom transform to broadcast
      // Borrowed from similar code in
      // https://github.com/ros-planning/navigation/blob/noetic-devel/amcl/src/amcl_node.cpp

       // MAP -> frcrobot
      // odom -> base link

      // baselink_to_map transformed from base_link to odom == odom->map
      geometry_msgs::PoseStamped base_link_to_map;
      tf2::convert(base_link_to_map_tf, base_link_to_map);
      ROS_INFO_STREAM_THROTTLE(2, "Base link to map " << base_link_to_map);

      tf2::Transform base_link_to_odom_transform;
      tf2::convert(base_link_to_odom_tf.transform, base_link_to_odom_transform);
      ROS_INFO_STREAM_THROTTLE(2, "Base link to odom " << base_link_to_odom_tf.transform);

      base_link_to_odom_transform = base_link_to_odom_transform.inverse(); 
      tf2::convert(base_link_to_odom_transform, base_link_to_odom_tf.transform);
      ROS_INFO_STREAM_THROTTLE(2, "Base link to odom after inv " << base_link_to_odom_tf.transform);

      // odom to map is out param (supposdly)
      geometry_msgs::PoseStamped odom_to_map; // ans
	    tf2::doTransform(base_link_to_map, odom_to_map, base_link_to_odom_tf);
      
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.stamp = base_link_to_map_tf.header.stamp;
      transformStamped.header.frame_id = map_frame_id;
      transformStamped.child_frame_id = odom_frame_id;

      // Computed transform is odom->map, but need to invert
      // it to publish map->odom instead
      // TODO, should zero Z right here?
      tf2::Transform odom_to_map_tf;
      tf2::convert(odom_to_map.pose, odom_to_map_tf);
      tf2::convert(odom_to_map_tf.inverse(), transformStamped.transform);

      // ROS_INFO_STREAM("map_to_odom: transform x = " << transformStamped.transform.translation.x << ", y = " << transformStamped.transform.translation.y << ", z = " << transformStamped.transform.translation.z << " at time " << transformStamped.header.stamp << " from " << base_link_to_map_tf.header.stamp << " to " << base_link_to_odom_tf.header.stamp);

      tfbr.sendTransform(transformStamped);
    } catch (const tf2::TransformException &ex) {
      ROS_ERROR_STREAM_THROTTLE(5, "Line " << __LINE__ << " Map to odom : transform from base_link to " << odom_frame_id << " failed in map->odom broadcaser : " << ex.what());
    }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_to_odom_node");
  ros::NodeHandle nh_;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  tf2_ros::StaticTransformBroadcaster tfbr;

  double map_to_odom_rate;
  nh_.param<double>("map_to_odom_rate", map_to_odom_rate, 20.0);

  // make a timer that runs at the requested rate and updates the tagslam transform 
  ros::Rate r(map_to_odom_rate);
  while (ros::ok()) {
    ros::spinOnce();
    updateMapOdomTf(tf_buffer, tfbr);
    r.sleep();
  }

  return 0;
}
