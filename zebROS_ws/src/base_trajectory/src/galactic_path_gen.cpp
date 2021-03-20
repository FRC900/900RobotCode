#include <ros/ros.h>
#include <math.h>
#include "base_trajectory_msgs/GenerateSpline.h"
#include "base_trajectory_msgs/GalacticPathGen.h"

ros::ServiceClient spline_gen_cli;

bool genPath(base_trajectory_msgs::GalacticPathGen::Request& req, base_trajectory_msgs::GalacticPathGen::Response& res)
{
    base_trajectory_msgs::GenerateSpline spline_gen_srv;

    spline_gen_srv.request.header = req.detection.header;

    const size_t objects_num = req.detection.objects.size();

    std::vector<std::pair<double, double>> points;
    points.push_back(std::make_pair(0,0)); // may need to worry about transforms between beginning/end point and intake?

    for (size_t i = 0; i < objects_num; i++) // filter out power cell detections
        if(req.detection.objects[i].id == "power_cell")
            points.push_back(std::make_pair(req.detection.objects[i].location.x, req.detection.objects[i].location.y));

    // need to determine last point
    points.push_back(std::make_pair(7.62, points[points.size()-1].second)); // probably best practice to make 7.62 (distance between start and end line) a config value at some point

    std::sort(points.begin(), points.end()); // sort on x coordinate

    size_t points_num = points.size();
    spline_gen_srv.request.points.resize(points_num);
    spline_gen_srv.request.point_frame_id.resize(points_num);
    for (size_t i = 0; i < points_num; i++) // copy points into spline request
    {
        spline_gen_srv.request.points[i].positions.resize(3);
        spline_gen_srv.request.points[i].positions[0] = points[i].first;
        spline_gen_srv.request.points[i].positions[1] = points[i].second;
        if(i > 0) 
        {
            spline_gen_srv.request.points[i].positions[2] = std::atan2(points[i].second-points[i-1].second, points[i].first-points[i-1].first); // right triangle math to calculate angle 
            spline_gen_srv.request.point_frame_id[i] = "intake";
        }
        else
        {
            spline_gen_srv.request.points[i].positions[2] = 0;
            spline_gen_srv.request.point_frame_id[i] = "";
        }
    }
    
    spline_gen_srv.request.optimize_final_velocity = true; // flag for optimized velocity

    if (!spline_gen_cli.call(spline_gen_srv))
	    ROS_ERROR_STREAM("Can't call spline gen service in path_follower_server");

    res.orient_coefs = spline_gen_srv.response.orient_coefs;
    res.x_coefs = spline_gen_srv.response.x_coefs;
    res.y_coefs = spline_gen_srv.response.y_coefs;
    res.end_points = spline_gen_srv.response.end_points;
    res.path = spline_gen_srv.response.path;

    return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "galactic_path_server");
	ros::NodeHandle nh;

    ros::ServiceServer svc = nh.advertiseService("galactic_path_gen", genPath);

    spline_gen_cli = nh.serviceClient<base_trajectory_msgs::GenerateSpline>("/base_trajectory/spline_gen");

    ros::spin();
    return 0;

}
