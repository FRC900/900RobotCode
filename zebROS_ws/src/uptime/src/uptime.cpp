/*
uptime_node

PUBLISHES TO:
uptime

FUNCTIONALITY:
publishes system uptime in seconds at 10 Hz
*/


#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <sys/sysinfo.h>


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "uptime_node");
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<std_msgs::UInt64>("uptime", 1);
  ros::Rate loop_rate(1);

  while (ros::ok()) {
    std_msgs::UInt64 m;
    struct sysinfo si;
    sysinfo(&si);
    m.data = si.uptime;
    pub.publish(m);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
