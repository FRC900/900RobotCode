#include <ros/ros.h>
#include <path_to_goal/PathAction.h>
#include <path_to_goal/PathGoal.h>
#include <path_to_goal/PathResult.h>
#include <actionlib/client/simple_action_client.h>
#include <path_to_goal/TwistSrv.h>

std::shared_ptr<actionlib::SimpleActionClient<path_to_goal::PathAction>> ac;

bool trigger_pathing_cb(path_to_goal::TwistSrv::Request &req, path_to_goal::TwistSrv::Response &res)
{
    ROS_WARN("In trigger callback");
    path_to_goal::PathGoal goal;
    goal.goal_index = 0;
    goal.x = req.y; //blame ryan
    goal.y = req.x; //im sad
    goal.rotation = req.rotation;
    goal.time_to_run = 0.5;
    ac->sendGoal(goal);

    bool finished_before_timeout = ac->waitForResult(ros::Duration(15));

    if(finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac->getState();
        ROS_INFO_STREAM("state = " << state.toString());
    }
    else
        ROS_INFO_STREAM("timed out");

    return true;
}
    

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_client");
    ros::NodeHandle n;

    ac = std::make_shared<actionlib::SimpleActionClient<path_to_goal::PathAction>>("path_server", true);
    ros::ServiceServer trigger_pathing = n.advertiseService("trigger_pathing", &trigger_pathing_cb);

    ac->waitForServer();

    ros::spin();

    return 0;
}
