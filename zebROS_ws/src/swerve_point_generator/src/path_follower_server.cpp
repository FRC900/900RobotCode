#include <ros/ros.h>
#include <swerve_point_generator/PathFollowAction.h>
#include <swerve_point_generator/PathFollowFeedback.h>
#include <swerve_point_generator/PathFollowResult.h>
#include <swerve_point_generator/PathFollowGoal.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>

//takes in a list of points and times
//throws it in cmd_vel form to the swerve_drive_controller
//
ros::Publisher cmd_vel_pub;

class PathFollowAction
{
	protected:
		actionlib::SimpleActionServer<swerve_point_generator::PathFollowAction> as_;
		std::string action_name_;

		swerve_point_generator::PathFollowFeedback feedback_;
		swerve_point_generator::PathFollowResult result_;

	public:
		PathFollowAction(std::string name, ros::NodeHandle n_) :
			as_(n_, name, boost::bind(&PathFollowAction::executeCB, this, _1), false),
			action_name_(name)
		{
			as_.start();
		}

		~PathFollowAction(void)
		{
		}

		void executeCB(const swerve_point_generator::PathFollowGoalConstPtr &goal)
		{
			double start_time = ros::Time::now().toSec();
			int point_index = 0;
			ros::Rate r(100);

			while(ros::ok())
			{
				double elapsed_time = ros::Time::now().toSec() - start_time;

				double last_time = goal->joint_trajectory.points[point_index].time_from_start.toSec();
				double next_time = goal->joint_trajectory.points[point_index + 1].time_from_start.toSec();
				std::vector<double> last_velocity = goal->joint_trajectory.points[point_index].velocities; //x, y, rotation
				std::vector<double> next_velocity  = goal->joint_trajectory.points[point_index + 1].velocities; //x, y, rotation

				if(elapsed_time < last_time || elapsed_time > next_time)
					ROS_ERROR_STREAM("elapsed time does not fall between the current trajectory points");

				geometry_msgs::Twist cmd_vel;
				cmd_vel.linear.x = last_velocity[0] + (next_velocity[0] - last_velocity[0]) * (next_time - last_time)/(elapsed_time - last_time);
				cmd_vel.linear.y = last_velocity[1] + (next_velocity[1] - last_velocity[1]) * (next_time - last_time)/(elapsed_time - last_time);
				cmd_vel.angular.z = last_velocity[2] + (next_velocity[2] - last_velocity[2]) * (next_time - last_time)/(elapsed_time - last_time);

				cmd_vel_pub.publish(cmd_vel);

				point_index++;
				ros::spinOnce();
				r.sleep();
				//set feedback
			}
			//set result
		}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_follower");
	ros::NodeHandle n;

	PathFollowAction path("path_follow_action", n);

	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1000);

	ros::spin();

	return 0;
}


