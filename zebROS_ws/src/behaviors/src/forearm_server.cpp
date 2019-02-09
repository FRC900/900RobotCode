#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behaviors/ForearmAction.h>
#include <arm_controller/CurArmCommand.h>
#include <arm_controller/SetArmState.h>
#include <talon_state_controller/TalonState.h>
#include "behaviors/enumerated_elevator_indices.h"


double arm_angle_deadzone;
class ForearmAction
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<behaviors::ForearmAction> as_;
        std::string action_name_;
        behaviors::ForearmFeedback f
			eedback_;
        behaviors::ForearmResult result_;

        
        ros::ServiceClient forearm_srv_;
        ros::ServiceClient arm_cur_command_srv_;

        ros::Subscriber talon_states_sub;

        double arm_angle;
        double arm_cur_command;
    public:
        ForearmAction(std::string name) :
            as_(nh_, name, boost::bind(&ForearmAction::executeCB, this, _1), false),
            action_name_(name)
        {
            std::map<std::string, std::string> service_connection_header;
            service_connection_header["tcp_nodelay"] = "1";

            forearm_srv_ = nh_.serviceClient<arm_controller::SetArmState>("/frcrobot/arm_controller/arm_state_service", false, service_connection_header);
            arm_cur_command_srv_ = nh_.serviceClient<arm_controller::CurArmCommand>("/frcrobot/arm_controller/arm_cur_command_srv", false, service_connection_header);
            as_.start();
            talon_states_sub = nh_.subscribe("/frcrobot/talon_states",1, &ForearmAction::talonStateCallback, this);
        }

        ~ForearmAction(void) {}

        void executeCB(const behaviors::ForearmGoalConstPtr &goal)
        {
            ROS_INFO_STREAM("forearm_server running callback");
            ros::Rate r(10);
            bool success = false;
            bool timed_out = false;
            bool aborted = false;
            
            arm_controller::SetArmState srv;
            srv.request.position = goal->position;
            forearm_srv_.call(srv);
            
            arm_controller::CurArmCommand srv_arm_command;
            if(arm_cur_command_srv_.call(srv_arm_command)) {
                arm_cur_command = srv_arm_command.response.cur_command; 
            }
            else {
                ROS_ERROR("Failed to call service arm_cur_command_srv");
            }

            double start_time = ros::Time::now().toSec();
            ROS_ERROR("Timeout: %d", goal->timeout);
            while(!success && !timed_out && !aborted) {
                success = fabs(arm_angle - arm_cur_command) < arm_angle_deadzone;
                if(as_.isPreemptRequested() || !ros::ok()) {
                    ROS_WARN("%s: Preempted", action_name_.c_str());
                    as_.setPreempted();
                    aborted = true;
                    break;
                }
                r.sleep();
                ros::spinOnce();
                double cur_time = ros::Time::now().toSec();
                ROS_WARN("cur_position: %f", arm_angle);
                ROS_WARN("cur_command: %f", arm_cur_command);
                ROS_INFO("arm_server.cpp 75: time elapsed: %f", (ros::Time::now().toSec() - start_time));
                ROS_INFO("arm_server.cpp 75: time elapsed2: %f",(cur_time - start_time));
                ROS_WARN("------------------------------");
                timed_out = cur_time - start_time > goal->timeout;
            }
            result_.success = success;
            result_.timed_out = timed_out;
            as_.setSucceeded(result_);
        }
        void talonStateCallback(const talon_state_controller::TalonState &talon_state)
        {
            static size_t arm_master_idx = std::numeric_limits<size_t>::max();

            if (arm_master_idx >= talon_state.name.size())
            {    
                for (size_t i = 0; i < talon_state.name.size(); i++) 
                {
                    if (talon_state.name[i] == "arm_joint")
                    {
                        arm_master_idx = i; 
                        break;
                    }
                }
            }    

            else {
                arm_angle = talon_state.position[arm_master_idx];
            }
        }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "forearm_server");

  ros::NodeHandle n;
  ros::NodeHandle n_params(n, "actionlib_forearm_params");
  if(!n_params.getParam("arm_angle_deadzone", arm_angle_deadzone))
    ROS_ERROR("Could not read arm_angle_deadzone in forearm_server");

  ForearmAction forearm_server("forearm_server");
  ros::spin();

  return 0;
}
