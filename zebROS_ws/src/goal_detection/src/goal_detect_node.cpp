#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_detection_node");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    nodelet.load(ros::this_node::getName(), "goal_detection/GoalDetect",
                 remap, nargv);

    ros::spin();

    return 0;
}

