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
			tf2::Quaternion myQuaternion;
			tf2::fromMsg(msgIn->pose.pose.orientation, myQuaternion);
			tf2::Matrix3x3 m(myQuaternion);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
			myQuaternion.setRPY(roll  + normalDistribution_(gen_),
								pitch + normalDistribution_(gen_),
								yaw   + normalDistribution_(gen_));
			msgOut.orientation = tf2::toMsg(myQuaternion.normalized());
			msgOut.orientation_covariance = { zCovariance_, 0.0, 0.0,
											  0.0, zCovariance_, 0.0,
											  0.0, 0.0, zCovariance_};

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

