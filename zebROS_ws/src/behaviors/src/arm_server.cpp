#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <behaviors/ArmAction.h>
#include <behaviors/IntakeGoal.h>
#include <behaviors/IntakeAction.h>
#include <behaviors/ForearmGoal.h>
#include <behaviors/ForearmAction.h>

enum ArmPosition
{
    LOWER_RIGHT,
    UPPER_RIGHT,
    STARTING,
    UPPER_LEFT,
    LOWER_LEFT,
};

class ArmAction
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<behaviors::ArmAction> as_;
        std::string action_name_;
        behaviors::ArmFeedback feedback_;
        behaviors::ArmResult result_;

        actionlib::SimpleActionClient<behaviors::IntakeAction> ai_;
        actionlib::SimpleActionClient<behaviors::ForearmAction> af_;

    public:
        ArmAction(std::string name) :
            as_(nh_, name, boost::bind(&ArmAction::executeCB, this, _1), false),
            action_name_(name),
            ai_("intake_server", true),
            af_("forearm_server", true)
        {
            /* std::map<std::string, std::string> service_connection_header;
            service_connection_header["tcp_nodelay"] = "1";
            ElevatorSrv_ = nh_.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS", false, service_connection_header);
            IntakeSrv_ = nh_.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake", false, service_connection_header);
            ClampSrv_ = nh_.serviceClient<std_srvs::SetBool>("/frcrobot/elevator_controller/clamp", false, service_connection_header);
            HighCube_ = nh_.subscribe("/frcrobot/elevator_controller/cube_state", 1, &autoAction::cubeStateCallback, this);
            BottomLimit_ = nh_.subscribe("/frcrobot/elevator_controller/bottom_limit_pivot", 1, &autoAction::bottomLimitCallback, this);*/

            as_.start();
        }

        ~ArmAction(void) {}

        void executeCB(const behaviors::ArmGoalConstPtr &goal)
        {
            ROS_INFO_STREAM("arm_server running callback");
            ros::Rate r(10);
            bool success = true;
            
            //spin in
            behaviors::IntakeGoal intake_goal;
            intake_goal.intake_cube = goal->intake_cube;
            intake_goal.intake_timeout = goal->intake_timeout;
            ai_.sendGoal(intake_goal);

            //move arm
            behaviors::ForearmGoal forearm_goal;
            forearm_goal.position = goal->arm_position;
            af_.sendGoal(forearm_goal);

            as_.setSucceeded();
        }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_server");

  ArmAction arm_server("arm_server");
  ros::spin();

  return 0;
}
