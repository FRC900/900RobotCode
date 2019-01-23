#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behaviors/ElevatorAction.h>
#include <elevator_controller.h>


double arm_angle_deadzone;
class ElevatorAction
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<behaviors::ElevatorAction> as_;
        std::string action_name_;
        behaviors::ElevatorFeedback feedback_;
        behaviors::ElevatorResult result_;

         
        ros::ServiceClient elevator_srv_;
        ros::ServiceClient Elevator_srv_;

        ros::Subscriber talon_states_sub;

        double elevator_position;
        

    public:
        ElevatorAction(std::string name) :
            as_(nh_, name, boost::bind(&ElevatorAction::executeCB, this, _1), false),
            action_name_(name)
        {
            std::map<std::string, std::string> service_connection_header;
            service_connection_header["tcp_nodelay"] = "1";

            elevator_srv_ = nh_.serviceClient<elevator_controller::SetElevatorState>("/frcrobot/elevator_controller/elevator_state_service", false, service_connection_header);
            as_.start();
            talon_states_sub = nh_.subscribe("/frcrobot/talon_states",1, &ElevatorAction::talonStateCallback, this);
        }

        ~ElevatorAction(void) {}

        void executeCB(const behaviors::ElevatorGoalConstPtr &goal)
        {
            ROS_INFO_STREAM("elevator_server running callback");
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

