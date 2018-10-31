#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behaviors/ForearmAction.h>
#include <arm_controller/SetArmState.h>

class ForearmAction
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<behaviors::ForearmAction> as_;
        std::string action_name_;
        behaviors::ForearmFeedback feedback_;
        behaviors::ForearmResult result_;

        ros::ServiceClient forearm_srv_;

    public:
        ForearmAction(std::string name) :
            as_(nh_, name, boost::bind(&ForearmAction::executeCB, this, _1), false),
            action_name_(name)
        {
            std::map<std::string, std::string> service_connection_header;
            service_connection_header["tcp_nodelay"] = "1";
            //forearm_srv_ = nh_.serviceClient<arm_controller::PositionSrv>("/frcrobot/arm_controller/position", false, service_connection_header);
            as_.start();
        }

        ~ForearmAction(void) {}

        void executeCB(const behaviors::ForearmGoalConstPtr &goal)
        {
            ROS_INFO_STREAM("forearm_server running callback");
            ros::Rate r(10);
            bool success = true;
            
            arm_controller::SetArmState srv;
            srv.request.position = goal->position;
            forearm_srv_.call(srv);

            as_.setSucceeded();
        }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "forearm_server");

  ForearmAction forearm_server("forearm_server");
  ros::spin();

  return 0;
}
