#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <std_msgs/Int32.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int main()
{

rosbag::Bag read_bag;
read_bag.open("_2018-11-03-11-08-20.bag.active", rosbag::bagmode::Read);

std::vector<sensor_msgs::LaserScan> laser_msg_write;

std::vector<std::string> topics;
topics.push_back(std::string("/sick_tim/scan"));

rosbag::View view_sick(read_bag, rosbag::TopicQuery(topics));

foreach(rosbag::MessageInstance const m, view_sick)
{
   sensor_msgs::LaserScan::ConstPtr laser_msg_read = m.instantiate<sensor_msgs::LaserScan>();
   if (laser_msg_read != NULL)
   {
       sensor_msgs::LaserScan temp_msg;
       temp_msg = *(laser_msg_read);
       temp_msg.header.frame_id = "sick";
       laser_msg_write.push_back(temp_msg);

       //write_bag.write("/sick_tim/scan", laser_msg_write.header.stamp, laser_msg_write);
   }
}

rosbag::Bag write_bag;
write_bag.open("_2018-11-03-11-08-20.bag.active", rosbag::bagmode::Write);

for(int i = 0; i < laser_msg_write.size(); i++)
{
    write_bag.write("/sick_tim/scan_modified", laser_msg_write[i].header.stamp, laser_msg_write[i]);
}

/*topics.pop_back();
topics.push_back(std::string("/rplidar/scan"));

rosbag::View view_rplidar(read_bag, rosbag::TopicQuery(topics));

foreach(rosbag::MessageInstance const m, view_rplidar)
{
   sensor_msgs::LaserScan::ConstPtr laser_msg_read = m.instantiate<sensor_msgs::LaserScan>();
   if (laser_msg_read != NULL)
   {
       laser_msg_write = *(laser_msg_read);
       laser_msg_write.header.frame_id = "rplidar";

       std::cout << laser_msg_write.header << std::endl;

        write_bag.write("/rplidar/scan", laser_msg_write.header.stamp, laser_msg_write);
   }
}*/

read_bag.close();
write_bag.close();

}
