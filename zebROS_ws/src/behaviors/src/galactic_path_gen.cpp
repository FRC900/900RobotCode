#include <ros/ros.h>
#include <cmath>
#include <std_srvs/Trigger.h>
#include "base_trajectory_msgs/GenerateSpline.h"
#include "behavior_actions/DynamicPath.h"
#include "field_obj/Detection.h"

ros::ServiceClient spline_gen_cli;
ros::ServiceClient dynamic_path_cli;

field_obj::Detection lastObjectDetection;

void objectDetectCallback(field_obj::DetectionConstPtr msg)
{
	lastObjectDetection = *msg;
}


bool genPath(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
	res.success = false; // Default to failure
    base_trajectory_msgs::GenerateSpline spline_gen_srv;

    spline_gen_srv.request.header = lastObjectDetection.header;

    const size_t objects_num = lastObjectDetection.objects.size();
	ROS_INFO_STREAM("galactic_path_gen : objects_num: " << objects_num);

    std::vector<std::pair<double, double>> points;
    points.push_back(std::make_pair(0,0)); // may need to worry about transforms between beginning/end point and intake?

	for (size_t i = 0; i < objects_num; i++) // filter out power cell detections
		if(lastObjectDetection.objects[i].id == "power_cell")
			points.push_back(std::make_pair(lastObjectDetection.objects[i].location.x, lastObjectDetection.objects[i].location.y));

	//points.back().second *= .90; // TODO -ugh
	if (points.size() == 1) // No power cells added
	{
		ROS_INFO("galactic_path_gen : no power cells detected");
		res.message = "no power cells detected";
		return false;
	}

    std::sort(points.begin(), points.end()); // sort on x coordinate

    points.push_back(std::make_pair(7.62+1.25, points.back().second)); // probably best practice to make 7.62 (distance between start and end line) a config value at some point

    size_t points_num = points.size();
    spline_gen_srv.request.points.resize(3*points_num-4); // 3 * (point_num - 2) + 2
    spline_gen_srv.request.point_frame_id.resize(3*points_num-4);
	size_t point_index = 0;
	for (size_t i = 0; i < points_num; i++) // copy points into spline request
	{
		if ((i == 0) || (i == (points_num - 1)))
		{
			// Facing forward for the last point increases our chances
			// of running over the ball
			spline_gen_srv.request.points[point_index].positions.resize(3);
			spline_gen_srv.request.points[point_index].positions[2] = 0;
			spline_gen_srv.request.points[point_index].positions[0] = points[i].first;
			spline_gen_srv.request.points[point_index].positions[1] = points[i].second;
			spline_gen_srv.request.point_frame_id[point_index] = "intake";
			base_trajectory_msgs::PathOffsetLimit path_offset_limit;
			if (i == (points_num - 1)) // Allow moving the y position of the last waypoint
			{
				//path_offset_limit.min_y = -2.25;
				//path_offset_limit.max_y = 2.25;
			}
			spline_gen_srv.request.path_offset_limit.push_back(path_offset_limit);
			point_index++;
		}
		else
		{
			spline_gen_srv.request.points[point_index].positions.resize(3);
			spline_gen_srv.request.points[point_index].positions[2] = 0;//std::atan2(points[i].second-points[i-1].second, points[i].first-points[i-1].first) * 0.1; // right triangle math to calculate angle
			spline_gen_srv.request.points[point_index].positions[0] = points[i].first - .175; // left side of point
			spline_gen_srv.request.points[point_index].positions[1] = points[i].second;
			spline_gen_srv.request.point_frame_id[point_index] = "intake";
			base_trajectory_msgs::PathOffsetLimit path_offset_limit;
			spline_gen_srv.request.path_offset_limit.push_back(path_offset_limit);
			point_index++;

			spline_gen_srv.request.points[point_index].positions.resize(3);
			spline_gen_srv.request.points[point_index].positions[2] = 0;//std::atan2(points[i].second-points[i-1].second, points[i].first-points[i-1].first) * 0.1; // right triangle math to calculate angle
			spline_gen_srv.request.points[point_index].positions[0] = points[i].first + .175; //right side of point
			spline_gen_srv.request.points[point_index].positions[1] = points[i].second;
			spline_gen_srv.request.point_frame_id[point_index] = "intake";
			base_trajectory_msgs::PathOffsetLimit path_offset_limit_2;
			spline_gen_srv.request.path_offset_limit.push_back(path_offset_limit_2);
			point_index++;

			spline_gen_srv.request.points[point_index].positions.resize(3);
			spline_gen_srv.request.points[point_index].positions[2] = 0;//std::atan2(points[i].second-points[i-1].second, points[i].first-points[i-1].first) * 0.1; // right triangle math to calculate angle
			spline_gen_srv.request.points[point_index].positions[0] = points[i].first + .375; //right side of point
			spline_gen_srv.request.points[point_index].positions[1] = points[i].second;
			spline_gen_srv.request.point_frame_id[point_index] = "intake";
			base_trajectory_msgs::PathOffsetLimit path_offset_limit_3;
			spline_gen_srv.request.path_offset_limit.push_back(path_offset_limit_3);
			point_index++;
		}
		//prev_angle = spline_gen_srv.request.points[i].positions[2];
	}

    spline_gen_srv.request.optimize_final_velocity = true; // flag for optimized velocity
	ROS_INFO_STREAM("galactic_path_gen : reg: " << spline_gen_srv.request);

    if (!spline_gen_cli.call(spline_gen_srv))
	{
	    ROS_ERROR_STREAM("Can't call spline gen service in galactic_path_gen");
		res.message = "can't call spline gen service";
		return false;
	}
	behavior_actions::DynamicPath dynamic_path_srv;
	dynamic_path_srv.request.path_name = "galactic_search_path";
	dynamic_path_srv.request.dynamic_path = spline_gen_srv.response.path;
    if (!dynamic_path_cli.call(dynamic_path_srv))
	{
	    ROS_ERROR_STREAM("Can't call dynamic path service in galactic_path_gen");
		res.message = "can't call dynamic path service";
		return false;
	}
	res.success = true; // Default to failure
	res.message = "to infinity and beyond!";
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "galactic_path_server");
	ros::NodeHandle nh;

	ros::Subscriber powercellSubscriber = nh.subscribe("/tf_object_detection/object_detection_world", 1, objectDetectCallback);
    ros::ServiceServer svc = nh.advertiseService("galactic_path_gen", genPath);

    spline_gen_cli = nh.serviceClient<base_trajectory_msgs::GenerateSpline>("/path_follower/base_trajectory/spline_gen");
    dynamic_path_cli = nh.serviceClient<behavior_actions::DynamicPath>("/auto/dynamic_path");

    ros::spin();
    return 0;

}
