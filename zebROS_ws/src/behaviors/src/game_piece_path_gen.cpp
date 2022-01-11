#include <ros/ros.h>
#include <cmath>
#include <std_srvs/Trigger.h>
#include <algorithm>
#include "base_trajectory_msgs/GenerateSpline.h"
#include "behavior_actions/DynamicPath.h"
#include "behavior_actions/GamePiecePickup.h"
#include "field_obj/Detection.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

// Create service clients
ros::ServiceClient spline_gen_cli;
ros::ServiceClient dynamic_path_cli;

field_obj::Detection lastObjectDetection;

std::string game_piece_frame_id = "intake";

// Callback function to retrieve the most recent object detection message
void objectDetectCallback(field_obj::DetectionConstPtr msg)
{
	lastObjectDetection = *msg;
}

trajectory_msgs::JointTrajectoryPoint generateTrajectoryPoint(double x, double y, double rotation)
{
	trajectory_msgs::JointTrajectoryPoint point;
	point.positions.resize(3);
	point.positions[2] = rotation;
	point.positions[0] = x;
	point.positions[1] = y;
	return point;
}

std::vector<double> gamePieceIntakeOffsets{0};

bool genPath(behavior_actions::GamePiecePickup::Request &req, behavior_actions::GamePiecePickup::Response &res)
{
	res.success = false; // Default to failure
	base_trajectory_msgs::GenerateSpline spline_gen_srv;

	spline_gen_srv.request.header = lastObjectDetection.header;

	const size_t objects_num = std::min((int)(lastObjectDetection.objects.size()), (int)(req.max_objects));
	ROS_INFO_STREAM("game_piece_path_gen : objects_num: " << objects_num);

	std::vector<std::array<double, 3>> points;
	points.push_back({0, 0, 0});

	for (size_t i = 0; i < objects_num; i++) // filter out selected object detections
		if (lastObjectDetection.objects[i].id == req.object_id)
			points.push_back({lastObjectDetection.objects[i].location.x, lastObjectDetection.objects[i].location.y, 0});

	if (points.size() == 1) // No objects added
	{
		ROS_INFO_STREAM("game_piece_path_gen : no " << req.object_id << " objects detected");
		res.message = "no " + req.object_id + " game pieces detected";
		return false;
	}

	std::sort(points.begin(), points.end()); // sort on x coordinate

	points.push_back({req.endpoint.position.x, req.endpoint.position.y, req.endpoint.orientation.z});

	size_t points_num = points.size();
	size_t pts;
	if (points_num >= 2) {
		pts = gamePieceIntakeOffsets.size() * (points_num - 2) + 2;
	} else if (points_num == 1) {
		pts = 1;
	} else if (points_num == 0) {
		ROS_WARN_STREAM("game_piece_path_gen : points_num == 0 (??\x29");
		pts = 0;
	}
	spline_gen_srv.request.points.resize(pts);
	spline_gen_srv.request.point_frame_id.resize(pts);
	size_t point_index = 0;
	for (size_t i = 0; i < points_num; i++) // copy points into spline request
	{
		ROS_INFO_STREAM("(" << points[i][0] << ", " << points[i][1] << "), orientation: " << points[i][2]);
		if ((i == 0) || (i == (points_num - 1)))
		{
			// Facing forward for the last point increases our chances
			// of running over the ball
			spline_gen_srv.request.points[point_index] = generateTrajectoryPoint(points[i][0], points[i][1], points[i][2]);
			spline_gen_srv.request.point_frame_id[point_index] = game_piece_frame_id;
			base_trajectory_msgs::PathOffsetLimit path_offset_limit;
			spline_gen_srv.request.path_offset_limit.push_back(path_offset_limit);
			point_index++;
		}
		else
		{
			for (double offset : gamePieceIntakeOffsets) {
				spline_gen_srv.request.points[point_index] = generateTrajectoryPoint(points[i][0] + offset, points[i][1], points[i][2]);
				spline_gen_srv.request.point_frame_id[point_index] = game_piece_frame_id;
				base_trajectory_msgs::PathOffsetLimit path_offset_limit;
				spline_gen_srv.request.path_offset_limit.push_back(path_offset_limit);
				point_index++;
			}
		}
		//prev_angle = spline_gen_srv.request.points[i].positions[2];
	}

	spline_gen_srv.request.optimize_final_velocity = true; // flag for optimized velocity
	ROS_INFO_STREAM("game_piece_path_gen : reg: " << spline_gen_srv.request);

	if (!spline_gen_cli.call(spline_gen_srv))
	{
		ROS_ERROR_STREAM("Can't call spline gen service in game_piece_path_gen");
		res.message = "can't call spline gen service";
		return false;
	}
	behavior_actions::DynamicPath dynamic_path_srv;
	dynamic_path_srv.request.path_name = "game_piece_pickup_path";
	dynamic_path_srv.request.dynamic_path = spline_gen_srv.response.path;
	if (!dynamic_path_cli.call(dynamic_path_srv))
	{
		ROS_ERROR_STREAM("Can't call dynamic path service in game_piece_path_gen");
		res.message = "can't call dynamic path service";
		return false;
	}
	res.success = true; // Default to failure
	res.message = "to infinity and beyond!";
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "game_piece_path_server");
	ros::NodeHandle nh;

	ros::NodeHandle nh_params(nh, "game_piece_path_gen_params"); // node handle for a lower-down namespace

	if (nh_params.hasParam("game_piece_frame_id")) {
		if (!nh_params.getParam("game_piece_frame_id", game_piece_frame_id)) {
			ROS_WARN_STREAM("game_piece_path_gen : getting frame id failed, defaulting to \"intake\"");
		}
	}

	if (nh_params.hasParam("game_piece_intake_offsets")) {
		if (!nh_params.getParam("game_piece_intake_offsets", gamePieceIntakeOffsets)) {
			ROS_WARN_STREAM("game_piece_path_gen : getting offsets failed, defaulting to {0}");
		}
	} else {
		ROS_INFO_STREAM("game_piece_path_gen : no offsets specified");
	}

	ros::Subscriber powercellSubscriber = nh.subscribe("/tf_object_detection/object_detection_world", 1, objectDetectCallback);
	ros::ServiceServer svc = nh.advertiseService("game_piece_path_gen", genPath);

	spline_gen_cli = nh.serviceClient<base_trajectory_msgs::GenerateSpline>("/path_follower/base_trajectory/spline_gen");
	dynamic_path_cli = nh.serviceClient<behavior_actions::DynamicPath>("/auto/dynamic_path");

	ros::spin();
	return 0;

}
