#include <ros/ros.h>
#include <path_to_goal/PathAction.h>
#include <path_to_goal/PathGoal.h>
#include <path_to_goal/PathResult.h>
#include <actionlib/client/simple_action_client.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_client");
    actionlib::SimpleActionClient<path_to_goal::PathAction> ac("path_server", true);

    ac.waitForServer();

    path_to_goal::PathGoal goal;
    goal.goal_index = 0;
    goal.x = 3;
    goal.y = 10;
    goal.rotation = 0;
    goal.time_to_run = 10;
    ac.sendGoal(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(15));

    if(finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO_STREAM("state = " << state.toString());
    }
    else
        ROS_INFO_STREAM("timed out");

    return 0;
}
