#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class DemoTransformClass
{
    private: 
        ros::NodeHandle nh_;

        ros::Subscriber sub_;
        ros::Publisher pub_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

    public:
        DemoTransformClass() :
            tf_listener_(tf_buffer_)
        {
            sub_ = nh_.subscribe("/input_topic", 1, &DemoTransformClass::callback, this);
            pub_ = nh_.advertise<geometry_msgs::PointStamped>("/output_topic", 1);
        }

        void callback(geometry_msgs::PointStamped msg)
        {
            geometry_msgs::TransformStamped input_to_output_transform;
            geometry_msgs::PointStamped transformed_msg;
            try {
                input_to_output_transform = tf_buffer_.lookupTransform("output", "input", ros::Time::now());
                tf2::doTransform(msg, transformed_msg, input_to_output_transform);

				pub_.publish(transformed_msg);
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("Failed to transform message");
            }
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_tf_node");

    DemoTransformClass demo_tf;

    ros::spin();
}
