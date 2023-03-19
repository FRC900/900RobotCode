#include "ros/ros.h"
#include "field_obj/Detection.h"
#include "geometry_msgs/Point.h"
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "behavior_actions/AlignToSubstation2023Action.h"
#include "behavior_actions/PathToAprilTagAction.h"
#include <behavior_actions/Intaking2023Action.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/Imu.h>
#include <frc_msgs/MatchSpecificData.h>
#include <path_follower_msgs/PathAction.h>
#include <optional>

class AlignToSubstationAction
{
    protected:

        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<behavior_actions::AlignToSubstation2023Action> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
        std::string action_name_;
        // create messages that are used to published feedback/result
        behavior_actions::AlignToSubstation2023Feedback feedback_;
        behavior_actions::AlignToSubstation2023Result result_;
        field_obj::Detection latest_;
        ros::Subscriber sub_;
        ros::Subscriber match_sub_;
        actionlib::SimpleActionClient<behavior_actions::PathToAprilTagAction> client_;
        actionlib::SimpleActionClient<behavior_actions::Intaking2023Action> ac_intaking_;
        double double_substation_x_offset_;
        double double_substation_offset_;
        double double_substation_tag_ids_[2];
        double double_substation_intake_percent_;
        uint8_t alliance_{0};
        double percent_complete_{0};

    public:

        AlignToSubstationAction(std::string name) :
            as_(nh_, name, boost::bind(&AlignToSubstationAction::executeCB, this, _1), false),
            action_name_(name),
            sub_(nh_.subscribe<field_obj::Detection>("/tf_object_detection/tag_detection_world", 1, &AlignToSubstationAction::callback, this)),
            client_("/path_to_apriltag", true),
            ac_intaking_("/intaking/intaking_server_2023", true)
        {

            match_sub_ = nh_.subscribe<frc_msgs::MatchSpecificData>("/frcrobot_rio/match_data", 1, &AlignToSubstationAction::matchCb, this);

            nh_.getParam("double_substation_x_offset", double_substation_x_offset_);
            nh_.getParam("double_substation_red_tag", double_substation_tag_ids_[0]);
            nh_.getParam("double_substation_blue_tag", double_substation_tag_ids_[1]);
            nh_.getParam("double_substation_offset", double_substation_offset_);
            nh_.getParam("double_substation_intake_percent", double_substation_intake_percent_);

            as_.start();
        }

        ~AlignToSubstationAction(void)
        {
        }

        void matchCb(const frc_msgs::MatchSpecificDataConstPtr &msg)
        {
            alliance_ = msg->allianceColor;
        }

        void callback(const field_obj::DetectionConstPtr &msg)
        {
            // need to use transforms...
            // oh wait no we don't! we can tell spline srv what frame id it is relevant to
            latest_ = *msg;
        }

        void feedbackCb(const behavior_actions::PathToAprilTagFeedbackConstPtr &feedback)
        {
            percent_complete_ = feedback->percent_complete;
        }

        void executeCB(const behavior_actions::AlignToSubstation2023GoalConstPtr &goal)
        {
            if (!client_.isServerConnected()) {
                ROS_ERROR_STREAM("2023_align_to_substation : path to apriltag server not running!!! this is unlikely to work");
            }
            if (!ac_intaking_.isServerConnected()) {
                ROS_ERROR_STREAM("2023_align_to_substation : intaking server not running!!! this is unlikely to work");
            }
            if (goal->substation == goal->DOUBLE)
            {
                ros::spinOnce(); // grab latest callback data
                auto detection = latest_;
                std::optional<field_obj::Object> tag = std::nullopt;
                for (const auto &obj : detection.objects)
                {
                    try
                    {
                        uint8_t id = std::stoi(obj.id);
                        if (id == double_substation_tag_ids_[alliance_])
                        {
                            tag = obj;
                        }
                    }
                    catch (...)
                    {
                        ROS_INFO_STREAM("2023_align_to_substation : Found an object with a non-numeric id, skipping");
                    }
                }
                ROS_INFO_STREAM("2023_align_to_substation : " << (tag != std::nullopt ? std::string("found a tag") : std::string("didn't find a tag")) << ", ignoring it to just intake");
                tag = std::nullopt;
                if (tag == std::nullopt)
                {
                    ROS_ERROR_STREAM("2023_align_to_substation : AprilTag " << std::to_string(double_substation_tag_ids_[alliance_]) << " not found :(");
                    ROS_INFO_STREAM("2023_align_to_substation : Just intaking! " << percent_complete_ << " > " << double_substation_intake_percent_);
                    behavior_actions::Intaking2023Goal intakingGoal;
                    intakingGoal.piece = intakingGoal.DOUBLE_SUBSTATION;
                    intakingGoal.outtake = false;
                    ac_intaking_.sendGoal(intakingGoal);
                    result_.success = true;
                    as_.setSucceeded(result_);
                    return;
                }

                geometry_msgs::Point offset;
                offset.x = -double_substation_x_offset_; // TODO consider going to a bit behind this and then driving forward for a preset amount of time? (in case we overshoot)
                offset.y = (goal->side == goal->LEFT ? 1 : -1) * double_substation_offset_;
                offset.z = 0;

                geometry_msgs::Pose pose;
                pose.position = offset;

                tf2::Quaternion q;
                q.setRPY(0, 0, 0); // facing the tag
                geometry_msgs::Quaternion qMsg = tf2::toMsg(q);

                pose.orientation = qMsg;
                // set orientation to quaternion with yaw = tagRotation, use setRPY

                behavior_actions::PathToAprilTagGoal aprilGoal;
                aprilGoal.id = double_substation_tag_ids_[alliance_];
                aprilGoal.tagRotation = 0.0;
                aprilGoal.offset = pose;
                aprilGoal.frame_id = "front_bumper";

                client_.sendGoal(aprilGoal,  /* Done cb */ NULL, /*Active*/ NULL, boost::bind(&AlignToSubstationAction::feedbackCb, this, _1));

                ros::Duration(0.05).sleep(); // wait for path follower to publish first feedback message so we don't instantly get 0.99

                bool intook = false;
                ros::Rate r(30);
                percent_complete_ = 0;
                while (!client_.getState().isDone())
                {
                    if (as_.isPreemptRequested() || !ros::ok())
                    {
                        ROS_ERROR_STREAM("2023_align_to_substation : Preempted!");
                        client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
                        ac_intaking_.cancelGoalsAtAndBeforeTime(ros::Time::now());
                        as_.setPreempted();
                        return;
                    }
                    if ((percent_complete_ > double_substation_intake_percent_) && !intook)
                    {
                        intook = true;
                        ROS_INFO_STREAM("2023_align_to_substation : Intaking! " << percent_complete_ << " > " << double_substation_intake_percent_);
                        behavior_actions::Intaking2023Goal intakingGoal;
                        intakingGoal.piece = intakingGoal.DOUBLE_SUBSTATION;
                        intakingGoal.outtake = false;
                        ac_intaking_.sendGoal(intakingGoal);
                    }
                    ros::spinOnce();
                    r.sleep();
                }

                as_.setSucceeded(result_);
                // other code has to move the intake down
            }
            else
            {
                ROS_ERROR_STREAM("2023_align_to_substation : only double substation currently supported, " << std::to_string(goal->substation) << " was requested.");
                result_.success = false;
                as_.setAborted(result_);
            }
        }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "align_to_substation");

    AlignToSubstationAction alignToGrid("align_to_substation");
    ros::spin();

    return 0;
}