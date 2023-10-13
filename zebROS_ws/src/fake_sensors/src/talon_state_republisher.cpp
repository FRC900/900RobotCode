#include <optional>
#include <talon_state_msgs/TalonState.h>
#include <talon_state_msgs/TalonFXProState.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>


class TalonStateRepublisher
{
    public:
        TalonStateRepublisher(ros::NodeHandle &n):
            sub_(n.subscribe("frcrobot_jetson/talon_states", 1, &TalonStateRepublisher::callback<const talon_state_msgs::TalonStateConstPtr &, 0>, this)),
            fxprosub_(n.subscribe("frcrobot_jetson/talonfxpro_states", 1, &TalonStateRepublisher::callback<const talon_state_msgs::TalonFXProStateConstPtr &, 1>, this)),
            pub_(n.advertise<sensor_msgs::JointState>("talon_joint_states", 1)),
            pub_timer_(n.createTimer(ros::Duration(1. / 100.), boost::bind(&TalonStateRepublisher::publisher, this)))
            {

            }
    

        template <class T, size_t N>
        void callback(T msgIn)
        {
            auto & msgOut = msgsOut_[N];
            msgOut.name.clear();
            msgOut.position.clear();
            msgOut.velocity.clear();
            msgOut.effort.clear();
			msgOut.header.stamp = msgIn->header.stamp;
			msgOut.header.frame_id = "";
            std::optional<double> four_bar_angle = std::nullopt;
			for (size_t i = 0; i < msgIn->name.size(); i++)
            {
                if(msgIn->name[i]=="four_bar")
                {
                    four_bar_angle = - msgIn->position[i] + 0.52;
                    msgOut.position.push_back(*four_bar_angle);
                }
                else
                {
                    msgOut.position.push_back(msgIn->position[i]);
                }
                
                msgOut.name.push_back(msgIn->name[i]);
                if constexpr (std::is_same_v<T, const talon_state_msgs::TalonStateConstPtr &>)
                {
                    msgOut.velocity.push_back(msgIn->speed[i]);
                }
                else
                {
                    msgOut.velocity.push_back(msgIn->velocity[i]);
                }
                msgOut.effort.push_back(0);
            }
            if (four_bar_angle)
            {
                msgOut.name.push_back("joint2");
                msgOut.position.push_back(0 - *four_bar_angle);
                msgOut.name.push_back("joint1");
                msgOut.position.push_back(*four_bar_angle);
            }
        }

        void publisher(void)
        {
            sensor_msgs::JointState msgOut;

            for (const auto &msgsOut : msgsOut_)
            {
                if (msgsOut.name.size() > 0)
                {
                    msgOut.header = msgsOut.header;
                }
                for (size_t i = 0; i < msgsOut.name.size(); i++)
                {
                    msgOut.name.push_back(msgsOut.name[i]);
                    msgOut.position.push_back(msgsOut.position[i]);
                    msgOut.velocity.push_back(msgsOut.velocity[i]);
                    msgOut.effort.push_back(msgsOut.effort[i]);
                }
            }
            pub_.publish(msgOut);
        }

    private:
        ros::Subscriber sub_;
        ros::Subscriber fxprosub_;
		ros::Publisher  pub_;
        ros::Timer      pub_timer_;
        std::array<sensor_msgs::JointState, 2> msgsOut_;

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "talon_state_republisher");

	ros::NodeHandle n;
	TalonStateRepublisher talonStateRepublisher(n);

	ros::spin();
	return 0;
}
