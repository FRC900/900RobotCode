#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "gpu_apriltag");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    for (int i = 0; i < argc; i++) {
        nargv.emplace_back(argv[i]);
    }
    nodelet.load(ros::this_node::getName(), "image_reformatter/ImageReformatterNodelet",
                 remap, nargv);

    ros::spin();

    return 0;
}


