#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behaviors/IntakeAction.h>
#include <intake_controller/IntakeSrv.h>

class IntakeAction
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<behaviors::IntakeAction> as_;
        std::string action_name_;
        behaviors::IntakeFeedback feedback_;
        behaviors::IntakeResult result_;

        ros::ServiceClient intake_srv_;

    public:
        IntakeAction(std::string name) :
            as_(nh_, name, boost::bind(&IntakeAction::executeCB, this, _1), false),
            action_name_(name)
        {
            std::map<std::string, std::string> service_connection_header;
            service_connection_header["tcp_nodelay"] = "1";
            intake_srv_ = nh_.serviceClient<intake_controller::IntakeSrv>("/frcrobot/intake_controller/intake_command", false, service_connection_header);
            as_.start();
        }

        ~IntakeAction(void) {}

        void executeCB(const behaviors::IntakeGoalConstPtr &goal)
        {
            ROS_INFO_STREAM("intake_server running callback");

            ros::Rate r(10);
            bool success = true;
            
            intake_controller::IntakeSrv srv;
            srv.request.power = goal->power;
            srv.request.intake_in = goal->intake_in;
            //figure out how to deal with goal->timeout?
            intake_srv_.call(srv);

            as_.setSucceeded();
        }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "intake_server");

  IntakeAction intake_server("intake_server");
  ros::spin();

  return 0;
}
