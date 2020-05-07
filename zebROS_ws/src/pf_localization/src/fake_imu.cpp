// Simple node to generate a fake IMU message in simulation
// Reads base ground truth Z orientation, adds noise, publishes an IMU message
// with that orientation
#include <random>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>

class FakeIMU
{
	public:
		FakeIMU(ros::NodeHandle &n)
			: rd_{}
			, gen_{rd_()}
			, zCovariance_(0.02)
			, sub_(n.subscribe("base_pose_ground_truth", 2, &FakeIMU::cmdVelCallback, this))
			, pub_(n.advertise<sensor_msgs::Imu>("imu", 2))

		{
			n.param("zCovariance", zCovariance_, zCovariance_);
			normalDistribution_ = std::normal_distribution<double>{0, zCovariance_ * zCovariance_};
		}

		// Read Twist message, save it to be republished in the pubThread below
		void cmdVelCallback(const nav_msgs::OdometryConstPtr &msgIn)
		{
			sensor_msgs::Imu msgOut;
			msgOut.header = msgIn->header;
			tf2::Quaternion myQuaternion;
			myQuaternion.setRPY( 0, 0, msgIn->pose.pose.orientation.z + normalDistribution_(gen_) );
			msgOut.orientation.x = myQuaternion.x();
			msgOut.orientation.y = myQuaternion.y();
			msgOut.orientation.z = myQuaternion.z();
			msgOut.orientation.w = myQuaternion.w();
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

