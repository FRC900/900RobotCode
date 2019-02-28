#include <ros/ros.h>
#include "path_to_goal/path_to_goal.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Point32.h"

#include "behaviors/PathAction.h"
#include "behaviors/PathGoal.h"
#include "math.h"
#include <actionlib/client/simple_action_client.h>

geometry_msgs::Point32 target;
void store_vision_target(geometry_msgs::Point32 vision_target)
{
    target = vision_target;
    ROS_INFO("%f",target.x);
}

std::shared_ptr<actionlib::SimpleActionClient<behaviors::PathAction>> ac;


float distance;
float sidedistance;
float angleoffset;
bool trigger(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{


    ROS_INFO("Success!");
    behaviors::PathGoal path_goal;
    path_goal.x = target.x + distance*std::sin(target.z) + sidedistance*std::cos(target.z);
    path_goal.y = target.y - distance*std::cos(target.z) + sidedistance*std::sin(target.z);
    path_goal.rotation = target.z + angleoffset;

    path_goal.time_to_run = 5;
    ac->sendGoal(path_goal);

    return true;
}

int main(int argc,char**argv)
{
    ros::init(argc,argv,"align_server");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("vision_target",1,store_vision_target);

    ros::NodeHandle param_node(nh, "align_offset_params");
    if(!param_node.getParam("x_offset",sidedistance))
		ROS_ERROR_STREAM("Could not read x_offset in align_server");
    if(!param_node.getParam("y_offset",distance))
		ROS_ERROR_STREAM("Could not read y_offset in align_server");
    if(!param_node.getParam("angle_offset",angleoffset))
		ROS_ERROR_STREAM("Could not read angle_offset in align_server");




    ros::ServiceServer srv = nh.advertiseService("align_service",trigger);
    ac = std::make_shared<actionlib::SimpleActionClient<behaviors::PathAction>>("path_server",true);
    ros::spin();
}

