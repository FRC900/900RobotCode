// Simple node to generate a periodic cmd_vel_out message by
// monitoring and republishing a cmd_vel (Twist) message
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

class FakeCmdVelOut
{
	public:
		FakeCmdVelOut(ros::NodeHandle &n) 
			: sub_(n.subscribe("cmd_vel", 2, &FakeCmdVelOut::cmdVelCallback, this))
			, thread_(std::bind(&FakeCmdVelOut::pubThread, this, std::ref(n)))
			, base_watchdog_timeout_(0.2)
		{
			n.param("base_watchdog_timeout", base_watchdog_timeout_, base_watchdog_timeout_);
		}

		~FakeCmdVelOut()
		{
			thread_.join();
		}

		// Read Twist message, save it to be republished in the pubThread below
		void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
		{
			std::lock_guard<std::mutex> l(mutex_);
			last_cmd_vel_.header.stamp = ros::Time::now();
			last_cmd_vel_.twist = *msg;
		}

		// Start a thread which loops and periodically publishes
		// a time-stamped copy of the most recent Twist message read
		// from cmd_vel
		void pubThread(ros::NodeHandle &n)
		{
			auto pub = n.advertise<geometry_msgs::TwistStamped>("cmd_vel_out", 2);
			ros::Rate r(100);
			geometry_msgs::TwistStamped msg;
			while (ros::ok())
			{
				{
					std::lock_guard<std::mutex> l(mutex_);
					msg = last_cmd_vel_;
				}
				const auto now = ros::Time::now();
				// If the message is too old, the watchdog code in stage
				// will stop the robot.  Emulate that behavior here
				if ((now - msg.header.stamp).toSec() >= base_watchdog_timeout_)
				{
					msg.twist.linear.x = 0;
					msg.twist.linear.y = 0;
					msg.twist.linear.z = 0;
					msg.twist.angular.x = 0;
					msg.twist.angular.y = 0;
					msg.twist.angular.z = 0;
				}

				msg.header.stamp = now;
				pub.publish(msg);
				r.sleep();
			}
		}

	private:
		ros::Subscriber sub_;
		std::thread thread_;
		std::mutex mutex_;
		geometry_msgs::TwistStamped last_cmd_vel_;
		double base_watchdog_timeout_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_cmd_vel_out");

	ros::NodeHandle n;
	FakeCmdVelOut fakeCmdVelOut(n);

	ros::spin();
	return 0;
}

