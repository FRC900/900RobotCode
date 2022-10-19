
#include <ros/ros.h>
#include <cuda_apriltag_ros/AprilTagDetectionArray.h>
#include "field_obj/Detection.h"
class Apriltag2PF
{
    public:
        Apriltag2PF(ros::NodeHandle &n)
            : sub_(n.subscribe("/cuda_tag_detections", 1, &Apriltag2PF::cmdVelCallback, this))
            , pub_(n.advertise<field_obj::Detection>("/goal_detection/goal_detect_msg", 2))

        {
        }

        // Translate ar_track_alvar marker msg into our custom goal detection message
        void cmdVelCallback(const cuda_apriltag_ros::AprilTagDetectionArray &msgIn)
        {

            field_obj::Detection msgOut;
            msgOut.header = msgIn.header;
            for (const auto &m : msgIn.detections)
            {
                field_obj::Object object;
                // m.id is vector, could be more than 1 element when using tag bundles
                object.id = std::to_string(m.id);
                object.location = m.pose.pose.pose.position;
                object.angle = atan2(object.location.y, object.location.x) * 180. / M_PI;

                // need someone who knows how to turn a covarience matrix into confidence
                // http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovariance.html
                object.confidence = 1;
                msgOut.objects.push_back(object);


            }
            pub_.publish(msgOut);

        }

    private:
        ros::Subscriber sub_;
        ros::Publisher  pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_to_PF");

    ros::NodeHandle n;
    Apriltag2PF apriltagtoPF(n);

    ros::spin();
    return 0;
}
