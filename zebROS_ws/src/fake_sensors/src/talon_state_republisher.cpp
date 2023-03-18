#include <talon_state_msgs/TalonState.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>


class TalonStateRepublisher
{
    public:
        TalonStateRepublisher(ros::NodeHandle &n):
            sub_(n.subscribe("frcrobot_jetson/talon_states", 1, &TalonStateRepublisher::callback, this)),
            pub_(n.advertise<sensor_msgs::JointState>("talon_joint_states", 1)) {}
    

        void callback(const talon_state_msgs::TalonStateConstPtr &msgIn)
        {
            sensor_msgs::JointState msgOut;
			msgOut.header.stamp = msgIn->header.stamp;
			msgOut.header.frame_id = "";
            double four_bar_angle;
			for (size_t i = 0; i < msgIn->name.size(); i++)
            {
                if(msgIn->name[i]=="four_bar")
                {
                    four_bar_angle = - msgIn->position[i] + 0.52;
                    msgOut.position.push_back(four_bar_angle);
                }
                else
                {
                    msgOut.position.push_back(msgIn->position[i]);
                }
                
                msgOut.name.push_back(msgIn->name[i]);
                msgOut.velocity.push_back(msgIn->speed[i]);
                msgOut.effort.push_back(0);
            

            }
            msgOut.name.push_back("joint2");
            msgOut.position.push_back(0 - four_bar_angle);
            msgOut.name.push_back("joint1");
            msgOut.position.push_back(four_bar_angle);




            pub_.publish(msgOut);

        }
    private:
        ros::Subscriber sub_;
		ros::Publisher  pub_;

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "talon_state_republisher");

	ros::NodeHandle n;
	TalonStateRepublisher talonStateRepublisher(n);

	ros::spin();
	return 0;
}
