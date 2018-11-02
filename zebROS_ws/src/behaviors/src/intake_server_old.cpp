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
        ros::Time last_time_;

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
            double start_time = ros::Time::now().toSec();
            bool timed_out = false;
            bool aborted = false;

            if(goal->intake_cube) {
                cube_state_true = 0;
                intake_controller::Intake srv;
                srv.request.power = intake_power;
                srv.request.other_power = intake_power;
                srv.request.just_override_power = false;
                srv.request.spring_state = 2; //soft in
                srv.request.up = false;
                if(!IntakeSrv_.call(srv)) 
                    ROS_ERROR("Srv intake call failed in auto interpreter server intake");
                //else
                    //ROS_INFO("Srv intake call OK in auto interpreter server intake");
                ros::spinOnce();
                            bool success = false;
                while(!success && !timed_out && !aborted) {
                    success = cube_state_true > linebreak_debounce_iterations && (proceed  || !goal->wait_to_proceed); 
                    if(as_.isPreemptRequested() || !ros::ok()) {
                        ROS_WARN("%s: Preempted", action_name_.c_str());
                        as_.setPreempted();
                        aborted = true;
                        break;
                    }
                    if (!aborted) {
                        r.sleep();
                        ros::spinOnce();
                        timed_out = (ros::Time::now().toSec()-startTime) > goal->time_out;
                    }
                /*
                else
                {
                    srv.request.power = -intake_hold_power;
                    srv.request.spring_state = 3; //hard in
                    srv.request.up = false;
                            if(!IntakeSrv_.call(srv)) ROS_ERROR("Srv intake call failed in auto interpreter server intake");;
                }
                */		    
                }
                srv.request.power = success ? 0.15 : 0;
                srv.request.other_power = success ? 0.15 : 0;
                srv.request.spring_state = 3; //soft in
                srv.request.up = false;
                srv.request.just_override_power = !success;
                if(!IntakeSrv_.call(srv)) 
                    ROS_ERROR("Srv intake call failed in auto interpreter server intake");
                //else
                    //ROS_INFO("Srv intake call OK in auto interpreter server intake");
                    }
                    else
                    {
                cube_state_true = 0;
                elevator_controller::Intake srv;
                srv.request.power = -1;
                srv.request.other_power = -1;
                srv.request.just_override_power = false;
                srv.request.spring_state = 2; //soft in
                srv.request.up = false;
                if(!IntakeSrv_.call(srv)) 
                    ROS_ERROR("Srv intake call failed in auto interpreter server intake");
                //else
                    //ROS_INFO("Srv intake call OK in auto interpreter server intake");
                ros::spinOnce();
                            bool success = false;
                while(!success && !timed_out && !aborted) {
                    success = cube_state_true > linebreak_debounce_iterations && (proceed  || !goal->wait_to_proceed); 
                    if(as_.isPreemptRequested() || !ros::ok()) {
                        ROS_WARN("%s: Preempted", action_name_.c_str());
                        as_.setPreempted();
                        aborted = true;
                        break;
                    }
                    r.sleep();
                    ros::spinOnce();
                    timed_out = (ros::Time::now().toSec()-startTime) > goal->time_out;
                /*
                else
                {
                    srv.request.power = -intake_hold_power;
                    srv.request.spring_state = 3; //hard in
                    srv.request.up = false;
                            if(!IntakeSrv_.call(srv)) ROS_ERROR("Srv intake call failed in auto interpreter server intake");;
                }
                */		    
                }
                srv.request.power = success ? 0.15 : 0;
                srv.request.other_power = success ? 0.15 : 0;
                srv.request.spring_state = 3; //soft in
                srv.request.up = false;
                srv.request.just_override_power = !success;
                if(!IntakeSrv_.call(srv)) 
                    ROS_ERROR("Srv intake call failed in auto interpreter server intake");
                //else
                    //ROS_INFO("Srv intake call OK in auto interpreter server intake");
                    }
            //else if goal->
            //{}
            if(timed_out)
                {
                        ROS_INFO("%s: Timed Out", action_name_.c_str());
                }
                else if(!aborted)
                {
                        ROS_INFO("%s: Succeeded", action_name_.c_str());
                }
                else
                {
                        ROS_INFO("%s: Aborted", action_name_.c_str());
                }

            result_.timed_out = timed_out;
            as_.setSucceeded(result_);
            return;
        }
        void cubeCallback(const elevator_controller::CubeState &msg) {
            if(msg.intake_low || msg.intake_high) {
                cube_state_true += 1;
            }
            else {
                    cube_state_true = 0;
        }
            }
            void proceedCallback(const std_msgs::Bool &msg)
            {
                    proceed = msg.data;
            }
            intake_controller::IntakeSrv srv;
            srv.request.power = goal->power;
            srv.request.intake_in = goal->intake_in;
            intake_srv_.call(srv);

            if(goal->intake_timeout > 0)
            {
                ros::Duration timeout(goal->intake_timeout);

                if(start_time - ros::Time::now() > timeout)
                {
                    intake_controller::IntakeSrv srv;
                    srv.request.power = 0;
                    srv.request.intake_in = true;
                    intake_srv_.call(srv);
                }
            }

            ros::Rate r(10);
            bool success = true;
            

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
