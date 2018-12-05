#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <behaviors/ArmAction.h>
#include <behaviors/IntakeGoal.h>
#include <behaviors/IntakeAction.h>
#include <behaviors/ForearmGoal.h>
#include <behaviors/ForearmAction.h>
/*
enum ArmPosition
{
    LOWER_RIGHT,
    UPPER_RIGHT,
    STARTING,
    UPPER_LEFT,
    LOWER_LEFT,
};
*/

double intake_timeout;
double arm_timeout;
double arm_angle_deadzone;

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
            double start_time = ros::Time::now().toSec();
            
            /*
            //spin in
            behaviors::IntakeGoal intake_goal;
            intake_goal.intake_cube = goal->intake_cube;
            intake_goal.time_out = goal->intake_timeout;
            ai_.sendGoal(intake_goal);
            */

            //move arm
            behaviors::ForearmGoal forearm_goal;
            forearm_goal.position = goal->arm_position;
            forearm_goal.timeout = arm_timeout;
            af_.sendGoal(forearm_goal);
	    


            /*
            bool arm_finished_before_timeout = af_.waitForResult(ros::Duration(arm_timeout));
            
            if(!arm_finished_before_timeout) {
                result_.success = false;
                as_.setAborted(result_);
                ROS_ERROR("Forearm actionlib timed out in arm_server. ABORTING!");
                return;
            }
            behaviors::ForearmResult forearm_result;
            forearm_result = *af_.getResult();
            
            if(!forearm_result.success) {
                result_.success = false;
                as_.setAborted(result_);
                ROS_ERROR("Forearm actionlib did not succeed in arm_server. Dit it succeed: %d  Did it time out: %d ABORTING!", forearm_result.success, forearm_result.timed_out);
                return;
            }
            */
            
            behaviors::IntakeGoal intake_goal;
            intake_goal.intake_cube = goal->intake_cube;
            intake_goal.timeout = intake_timeout;
            ai_.sendGoal(intake_goal);

            bool intake_finished_before_timeout = ai_.waitForResult(ros::Duration(intake_timeout)); 
            if(!intake_finished_before_timeout) {
                result_.success = false;
                as_.setSucceeded(result_); 
                ROS_ERROR("Intake actionlib timed out in arm_server. ABORTING!?");
                return;
            }
            
            behaviors::IntakeResult intake_result;
            intake_result = *ai_.getResult();

            if(intake_result.success) {
                result_.success = true;
                as_.setSucceeded(result_);
                ROS_WARN("arm_server succeeded!!");
            }
        ROS_WARN("arm_server exiting!");
        return;
      }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_server");

  ros::NodeHandle n;
  ros::NodeHandle n_params(n, "actionlib_arm_params");


  ros::NodeHandle n_params_fake(n, "actionlib_forearm_params");
  if(!n_params_fake.getParam("arm_angle_deadzone", arm_angle_deadzone))
    ROS_ERROR("Could not read arm_angle_deadzone in forearm_server");

  /*
  if(!n_params_fake.getParam("help_me", intake_timeout));
    ROS_ERROR("Could not read intake_timeout in arm_server");
  if(!n_params_fake.getParam("help_me2", arm_timeout));
    ROS_ERROR("Could not read arm_timeout in arm_server");
  */
  intake_timeout = 5;
  arm_timeout = 4;
  ArmAction arm_server("arm_server");
  ros::spin();

  return 0;
}
