#include <ros/ros.h>
#include <path_to_goal/PathAction.h>
#include <path_to_goal/PathGoal.h>
#include <path_to_goal/PathResult.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Trigger.h>

std::shared_ptr<actionlib::SimpleActionClient<path_to_goal::PathAction>> ac;

bool trigger_pathing_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    path_to_goal::PathGoal goal;
    goal.goal_index = 0;
    goal.x = 3.0;
    goal.y = 5.0;
    goal.rotation = 0.0001;
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

    return 0;
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
