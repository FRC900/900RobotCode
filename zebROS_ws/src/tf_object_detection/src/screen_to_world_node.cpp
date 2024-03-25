#include <ros/ros.h>
#include <nodelet/loader.h>

int main (int argc, char **argv) {
	ros::init(argc, argv, "screen_to_world_node"); // FIXME I don't know what '@(node)' should be replaced with
	nodelet::Loader nodelet;
	nodelet::M_string remap(ros::names::getRemappings());
	nodelet::V_string nargv;
    for (int i = 0; i < argc; i++) {
        nargv.emplace_back(argv[i]);
    }
	std::string nodelet_name = ros::this_node::getName();
	nodelet.load(nodelet_name, "tf_object_detection/ScreenToWorld", remap, nargv);
	return 0;
}
