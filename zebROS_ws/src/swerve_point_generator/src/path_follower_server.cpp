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

			double last_time;
			double next_time;
			std::vector<double> last_velocities;
			std::vector<double> next_velocities;
			std::vector<double> last_positions;
			int num_points = goal->joint_trajectory.points.size();
			for(int i = 0; i < num_points; i++)
			{
				ROS_INFO_STREAM("orientation velocity: " << goal->joint_trajectory.points[i].velocities[2]);
			}

			double total_angle = 0;
			double last_elapsed_time = ros::Time::now().toSec() - start_time;

			ROS_INFO_STREAM("max_time = " << goal->joint_trajectory.points[num_points - 1].time_from_start.toSec());
			while(ros::ok())
			{
				double elapsed_time = ros::Time::now().toSec() - start_time;

				if(elapsed_time > goal->joint_trajectory.points[num_points - 1].time_from_start.toSec())
				{
					ROS_INFO_STREAM("finished executing path_follow");
					break;
				}

				for(int i = 1; i < num_points; i++)
				{
					if(goal->joint_trajectory.points[i].time_from_start.toSec() > elapsed_time)
					{
						last_time = goal->joint_trajectory.points[i - 1].time_from_start.toSec();
						next_time = goal->joint_trajectory.points[i].time_from_start.toSec();
						last_velocities = goal->joint_trajectory.points[i - 1].velocities;//x, y, rotation
						next_velocities = goal->joint_trajectory.points[i].velocities;//x, y, rotation
						last_positions = goal->joint_trajectory.points[i].positions;
						break;
					}
				}

				//if(elapsed_time < last_time || elapsed_time > next_time)
					ROS_ERROR_STREAM("elapsed time " << elapsed_time << "does not fall between the current trajectory points " << last_time << " and " << next_time);

				geometry_msgs::Twist cmd_vel;
				cmd_vel.linear.x = last_velocities[0] + (next_velocities[0] - last_velocities[0]) / (next_time - last_time) * (elapsed_time - last_time);
				cmd_vel.linear.y = last_velocities[1] + (next_velocities[1] - last_velocities[1]) / (next_time - last_time) * (elapsed_time - last_time);
				cmd_vel.angular.z = last_velocities[2] + (next_velocities[2] - last_velocities[2]) / (next_time - last_time) * (elapsed_time - last_time);
				ROS_INFO_STREAM("orientation: " << last_positions[2] << " orientation velocities: " << last_velocities[2]);
				ROS_INFO_STREAM("sending orientation " << cmd_vel.angular.z << " from path_follow");

				total_angle += cmd_vel.angular.z * (elapsed_time - last_elapsed_time);
				ROS_INFO_STREAM("total_angle = " << total_angle);

				cmd_vel_pub.publish(cmd_vel);

				last_elapsed_time = elapsed_time;
				point_index++;
				ros::spinOnce();
				r.sleep();
				//set feedback
			}
			//set resut
		}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_follower");
	ros::NodeHandle n;

	PathFollowAction path("path_follower", n);

	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/frcrobot_jetson/path_follower_server/cmd_vel", 1);

	ros::spin();

	return 0;
}


