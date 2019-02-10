#include "robot_visualizer/robot_visualize.h"

double width;
double length;
double wheel_base_pos_offset;

ros::Publisher robot_pub;

geometry_msgs::PolygonStamped robot_poly;

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_visualize");
    ros::NodeHandle n;

    ros::NodeHandle n_params(n, "robot_visual");

	if (!n_params.getParam("width", width))
        ROS_ERROR("could not read width in robot_visualize");
	if (!n_params.getParam("length", length))
        ROS_ERROR("Could not read length in robot_visualize");
	if (!n_params.getParam("wheel_base_pos_offset", wheel_base_pos_offset))
        ROS_ERROR("could not read wheel_base_pos_offset in robot_visualize");

	robot_pub = n.advertise<geometry_msgs::PolygonStamped>("robot_state_viz", 10);

	auto robot_state_sub = n.subscribe("robot_viz_state", 1, &plot_robot_cb);

	robot_poly.header.frame_id = "/robot_viz";
	robot_poly.polygon.points.resize(5);

	robot_poly.polygon.points[0].x = -width/2.0;
	robot_poly.polygon.points[0].y = -length/2.0 + wheel_base_pos_offset;
	robot_poly.polygon.points[0].z = 0.0;
	robot_poly.polygon.points[1].x = -width/2.0;
	robot_poly.polygon.points[1].y = length/2.0 + wheel_base_pos_offset;
	robot_poly.polygon.points[1].z = 0.0;
	robot_poly.polygon.points[2].x = width/2.0;
	robot_poly.polygon.points[2].y = length/2.0 + wheel_base_pos_offset;
	robot_poly.polygon.points[2].z = 0.0;
	robot_poly.polygon.points[3].x = width/2.0;
	robot_poly.polygon.points[3].y = -length/2.0 + wheel_base_pos_offset;
	robot_poly.polygon.points[3].z = 0.0;
	robot_poly.polygon.points[4].x = -width/2.0;
	robot_poly.polygon.points[4].y = -length/2.0 + wheel_base_pos_offset;
	robot_poly.polygon.points[4].z = 0.0;

	ros::spin();
}

void rotate_polygon(geometry_msgs::Polygon &poly, double angle)
{
	for(size_t i = 0; i < poly.points.size(); i++)
	{
		Eigen::Rotation2Dd r(angle);
		Eigen::Vector2d point = {poly.points[i].x, poly.points[i].y};
		point = r.toRotationMatrix() * point;
		poly.points[i].x = point[0];
		poly.points[i].y = point[1];
	}
}

void translate_polygon(geometry_msgs::Polygon &poly, double x, double y)
{
	for(size_t i = 0; i < poly.points.size(); i++)
	{
		poly.points[i].x = poly.points[i].x + x;
		poly.points[i].y = poly.points[i].y + y;
	}
}

void plot_robot_cb(const robot_visualizer::RobotVisualizeState &robot_viz)
{
	std::vector<geometry_msgs::PolygonStamped> pub_polys;
	pub_polys.push_back(robot_poly);

	for(size_t i = 0; i < pub_polys.size(); i++)
	{
		rotate_polygon(pub_polys[i].polygon, robot_viz.theta);
		translate_polygon(pub_polys[i].polygon, robot_viz.x, robot_viz.y);

		pub_polys[i].header.stamp = ros::Time::now();
	}
	robot_pub.publish(pub_polys[0]);
}
