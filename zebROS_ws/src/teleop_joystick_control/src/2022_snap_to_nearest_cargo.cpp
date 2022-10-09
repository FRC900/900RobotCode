#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <field_obj/Detection.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

class SnapToCargo2022
{
    protected:

        ros::NodeHandle nh_;
        ros::Subscriber object_detection_sub_;
        ros::Publisher nearest_cargo_pub_;
        ros::Publisher nearest_opponent_cargo_pub_;
        double nearest_cargo_angle_;
        double nearest_opponent_cargo_angle_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

    public:

        SnapToCargo2022() : tf_listener_(tf_buffer_)
        {

        }

        void init()
        {
            nearest_cargo_pub_ = nh_.advertise<std_msgs::Float64>("nearest_cargo_angle", 1);
            nearest_opponent_cargo_pub_ = nh_.advertise<std_msgs::Float64>("nearest_opponent_cargo_angle", 1);
            object_detection_sub_ = nh_.subscribe("/tf_object_detection/object_detection_world_filtered", 1, &SnapToCargo2022::objectDetectionCallback, this);
            ROS_INFO_STREAM("snap_to_cargo_2022 : initialized");
        }

        void objectDetectionCallback(const field_obj::Detection &msg)
        {
            field_obj::Object closest_cargo;
            field_obj::Object closest_opponent_cargo;
            double shortest_distance = std::numeric_limits<double>::max();
            double shortest_opponent_distance = std::numeric_limits<double>::max();
            for (auto obj : msg.objects)
            {
                if (obj.id == "friendly_cargo")
                {
                    double dist = hypot(obj.location.x, obj.location.y);
                    if (dist < shortest_distance)
                    {
                        closest_cargo = obj;
                        shortest_distance = dist;
                    }
                }
                else if (obj.id == "opponent_cargo")
                {
                    double dist = hypot(obj.location.x, obj.location.y);
                    if (dist < shortest_opponent_distance)
                    {
                        closest_opponent_cargo = obj;
                        shortest_opponent_distance = dist;
                    }
                }
            }
            std_msgs::Float64 msg1;
            try
            {
                geometry_msgs::PoseStamped p1s;
                p1s.header = msg.header;
                p1s.pose.position = closest_cargo.location;
                tf2::Quaternion q1;
                q1.setRPY(0, 0, closest_cargo.angle * (M_PI / 180.0));
                geometry_msgs::Quaternion q1m = tf2::toMsg(q1);
                p1s.pose.orientation = q1m;
                p1s = tf_buffer_.transform(p1s, "base_link", ros::Duration(0.05));
                msg1.data = atan2(p1s.pose.position.y, p1s.pose.position.x);
            }
            catch (...)
            {
                ROS_WARN_STREAM_THROTTLE(0.1, "snap_to_cargo_2022 : transform to base_link failed, using untransformed angle");
                msg1.data = closest_cargo.angle * (M_PI / 180.0);
            }
            nearest_cargo_pub_.publish(msg1);

            std_msgs::Float64 msg2;
            try
            {
                geometry_msgs::PoseStamped p2s;
                p2s.header = msg.header;
                p2s.pose.position = closest_opponent_cargo.location;
                tf2::Quaternion q2;
                q2.setRPY(0, 0, closest_opponent_cargo.angle * (M_PI / 180.0));
                geometry_msgs::Quaternion q2m = tf2::toMsg(q2);
                p2s.pose.orientation = q2m;
                p2s = tf_buffer_.transform(p2s, "base_link", ros::Duration(0.05));
                msg2.data = atan2(p2s.pose.position.y, p2s.pose.position.x);
            }
            catch (...)
            {
                ROS_WARN_STREAM_THROTTLE(0.1, "snap_to_cargo_2022 : transform to base_link failed, using untransformed angle");
                msg2.data = closest_opponent_cargo.angle * (M_PI / 180.0);
            }
            nearest_opponent_cargo_pub_.publish(msg2);
        }


};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "snap_to_cargo_2022");

    SnapToCargo2022 cargo;
    cargo.init();
    ros::spin();

    return 0;
}
