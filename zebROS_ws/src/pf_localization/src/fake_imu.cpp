// Simple node to generate a fake IMU message in simulation
// Reads base ground truth Z orientation, adds noise, publishes an IMU message
// with that orientation
#include <random>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class FakeIMU
{
	public:
		FakeIMU(ros::NodeHandle &n)
			: rd_{}
			, gen_{rd_()}
			, zCovariance_(0.000001)
			, sub_(n.subscribe("base_pose_ground_truth", 2, &FakeIMU::cmdVelCallback, this))
			, pub_(n.advertise<sensor_msgs::Imu>("imu", 2))

		{
			n.param("zCovariance", zCovariance_, zCovariance_);
			normalDistribution_ = std::normal_distribution<double>{0, sqrt(zCovariance_)};
		}

		// Read Odom message add noise, republish
		void cmdVelCallback(const nav_msgs::OdometryConstPtr &msgIn)
		{
			sensor_msgs::Imu msgOut;
			msgOut.header.stamp = msgIn->header.stamp;
			msgOut.header.frame_id = "imu";
			tf2::Quaternion myQuaternion(msgIn->pose.pose.orientation.x,
										 msgIn->pose.pose.orientation.y,
										 msgIn->pose.pose.orientation.z,
										 msgIn->pose.pose.orientation.w);
			tf2::Quaternion randomRot;
			randomRot.setRPY(normalDistribution_(gen_),
							 normalDistribution_(gen_),
							 normalDistribution_(gen_));
			msgOut.orientation = tf2::toMsg(randomRot * myQuaternion);
			msgOut.orientation_covariance = { 0.000001, 0.0, 0.0,
											  0.0, 0.000001, 0.0,
											  0.0, 0.0, 0.000001};

			pub_.publish(msgOut);
		}

	private:
		std::random_device rd_;
		std::mt19937 gen_;
		std::normal_distribution<double> normalDistribution_;
		double zCovariance_;
		ros::Subscriber sub_;
		ros::Publisher  pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_imu");

	ros::NodeHandle n;
	FakeIMU fakeIMU(n);

	ros::spin();
	return 0;
}

