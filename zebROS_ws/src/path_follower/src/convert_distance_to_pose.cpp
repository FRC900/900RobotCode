#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Range.h>

ros::Publisher fake_pose_pub;

void conversionCB(const sensor_msgs::RangeConstPtr& x_distance, const sensor_msgs::RangeConstPtr& y_distance) {
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = x_distance->header.stamp;
  pose.pose.position.x = x_distance->range;
  pose.pose.position.y = y_distance->range;
  ros::NodeHandle nh;
  fake_pose_pub.publish(pose);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "covert_distance_to_pose");
  ros::NodeHandle nh;
  fake_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("distance_as_pose", 2);

  message_filters::Subscriber<sensor_msgs::Range> x_distance_subscriber(nh, "x_distance_input", 2);
  // Terrible hack since we only have 1 terabee working right now
  message_filters::Subscriber<sensor_msgs::Range> y_distance_subscriber(nh, "y_distance_input", 2);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), x_distance_subscriber, y_distance_subscriber);
  sync.registerCallback(boost::bind(&conversionCB, _1, _2));

  ros::spin();

  return 0;
}
