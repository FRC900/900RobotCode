// Simple node to generate a fake IMU message in simulation
// Reads base ground truth orientation, adds noise, publishes an odom message
// with that orientation to the sim imu input topic
// From that point, the sim node will put it into the sim pigeon2 data. From there,
// the rest of the code will think the orientation data is coming from an actual IMU
#include <random>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class FakeIMU
{
	public:
		explicit FakeIMU(ros::NodeHandle &n)
			// This is actually the remapped topic name of the stage ground truth odom data
			: sub_{n.subscribe("/frcrobot_jetson/swerve_drive_controller/odom", 1, &FakeIMU::callback, this, ros::TransportHints().tcpNoDelay())}
			, pub_{n.advertise<nav_msgs::Odometry>("/frcrobot_jetson/imu_0_in", 1)}

		{
			ros::NodeHandle n_param("~");
			n_param.param("zCovariance", zCovariance_, zCovariance_);
			normalDistribution_ = std::normal_distribution<double>{0, sqrt(zCovariance_)};
		}

		// Read Odom message add noise, republish
		void callback(const nav_msgs::OdometryConstPtr &msgIn)
		{
			nav_msgs::Odometry msgOut = *msgIn;
			tf2::Quaternion myQuaternion;
			tf2::fromMsg(msgIn->pose.pose.orientation, myQuaternion);
			tf2::Matrix3x3 m(myQuaternion);
			double roll;
			double pitch;
			double yaw;
			m.getRPY(roll, pitch, yaw);
			myQuaternion.setRPY(roll  + normalDistribution_(gen_),
								pitch + normalDistribution_(gen_),
								yaw   + normalDistribution_(gen_));
			msgOut.pose.pose.orientation = tf2::toMsg(myQuaternion.normalized());
			msgOut.pose.covariance = {zCovariance_, 0.0, 0.0,
									  0.0, zCovariance_, 0.0,
									  0.0, 0.0, zCovariance_};

			pub_.publish(msgOut);
		}

	private:
		std::random_device rd_{};
		std::mt19937 gen_{rd_()};
		std::normal_distribution<double> normalDistribution_;
		double zCovariance_{0.000001};
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

