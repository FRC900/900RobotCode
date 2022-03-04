//    Node to republish sim laser scans as terabee distances
// Adding noise to simulate real world data

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <random>

class FakeTerabeeDist
{
  public:
    FakeTerabeeDist(ros::NodeHandle &n)
      : rd_{}
      , gen_{rd_()}           //generating a random digit
      , sub_(n.subscribe("fake_terabee_input", 2, &FakeTerabeeDist::cmdVelCallback, this))
      , pub_(n.advertise<sensor_msgs::Range>("terabee", 2))
      , normalDistribution_(std::normal_distribution<double>{0, .02}) // .02 is covariance value

    {

    }
                    //covariance??
    void cmdVelCallback(const sensor_msgs::LaserScanConstPtr &msgIn)
    {
			sensor_msgs::Range msgOut;
			msgOut.header.stamp = msgIn->header.stamp;
			msgOut.header.frame_id = msgIn->header.frame_id;
			msgOut.header.seq = msgIn->header.seq;
      msgOut.range = msgIn->ranges[0]+normalDistribution_(gen_); //adding noise
      msgOut.radiation_type = 1;
      msgOut.min_range = msgIn->range_min; //check lines 27-29
      msgOut.max_range = msgIn->range_max;
      msgOut.field_of_view = msgIn->angle_max-msgIn->angle_min;

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
  ros::init(argc, argv, "fake_terabee");

  ros::NodeHandle n;
  FakeTerabeeDist fakeTerabeeDist(n);

  ros::spin();
  return 0;

}
