#include <ros/ros.h>
#include <behaviors/PathAction.h>
#include <behaviors/PathGoal.h>
#include <behaviors/PathResult.h>
#include <actionlib/client/simple_action_client.h>
#include <path_to_goal/TwistSrv.h>

std::shared_ptr<actionlib::SimpleActionClient<behaviors::PathAction>> ac;

bool trigger_pathing_cb(path_to_goal::TwistSrv::Request &req, path_to_goal::TwistSrv::Response &res)
{
    ROS_WARN("In trigger callback");
    behaviors::PathGoal goal;
    goal.x = req.x; //blame ryan
    goal.y = req.y; //im sad
    goal.rotation = req.rotation;
    goal.time_to_run = req.time_to_run;
    ac->sendGoal(goal);

    bool finished_before_timeout = ac->waitForResult(ros::Duration(15));

    if(finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac->getState();
        ROS_INFO_STREAM("state = " << state.toString());
    }
    else
        ROS_INFO_STREAM("timed out in test_client");

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_client");
    ros::NodeHandle n;

    ac = std::make_shared<actionlib::SimpleActionClient<behaviors::PathAction>>("path_server", true);
    ros::ServiceServer trigger_pathing = n.advertiseService("trigger_pathing", &trigger_pathing_cb);

    ac->waitForServer();

    ros::spin();

    return 0;
}
