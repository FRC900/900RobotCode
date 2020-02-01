/****************************************************************************
Conversion from a quaternion to roll, pitch and yaw.
Nodes:
subscribed /rotation_quaternion (message of type geometry_msgs::Quaternion)
published /rpy_angles (message oftype geometry_msgs::Vector3.h)
****************************************************************************/

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_datatypes.h"

// Here I use global publisher, since I want to access the
// publisher in the function MsgCallback:
ros::Publisher rpy_publisher;

// Function for conversion of quaternion to roll pitch and yaw. The angles
// are published here too.
void MsgCallback(const sensor_msgs::Imu msg)
{
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    // this Vector is then published:
    rpy_publisher.publish(rpy);
    ROS_INFO_THROTTLE(0.5, "r=% 3.3f p=% 3.3f y=% 3.5f vr=% 3.3f vp=% 3.3f vy=% 3.3f", rpy.x, rpy.y, rpy.z, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_to_rpy");
    ros::NodeHandle n;
    rpy_publisher = n.advertise<geometry_msgs::Vector3>("rpy_angles", 5);
	auto quat_subscriber = n.subscribe("/imu/zeroed_imu", 5, MsgCallback);

    // check for incoming quaternions untill ctrl+c is pressed
    ROS_INFO("waiting for zeroed_imu");
    ros::spin();
    return 0;
}
