#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behaviors/ElevatorAction.h>
#include <elevator_controller/ElevatorSrv.h>



//double arm_angle_deadzone;
class ElevatorAction
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<behaviors::ElevatorAction> as_;
        std::string action_name_;
        behaviors::ElevatorFeedback feedback_;
        behaviors::ElevatorResult result_;

        ros::ServiceClient elevator_client_;
		ros::ServiceClient elevator_cur_command_;

        ros::Subscriber talon_states_sub;

        double elevator_cur_command_;

    public:
        ElevatorAction(std::string name) :
            as_(nh_, name, boost::bind(&ElevatorAction::executeCB, this, _1), false),
            action_name_(name)
        {
            std::map<std::string, std::string> service_connection_header;
            service_connection_header["tcp_nodelay"] = "1";

            elevator_client_ = nh_.serviceClient<elevator_controller::SetElevatorState>("/frcrobot/elevator_controller/elevator_state_service", false, service_connection_header);
            elevator_cur_command_ = nh_.serviceClient<elevator_controller::SetElevatorState>("/frcrobot/elevator_controller/elevator_cur_command", false, service_connection_header);

			as_.start();
            talon_states_sub = nh_.subscribe("/frcrobot/talon_states",1, &ElevatorAction::talonStateCallback, this);
        }

        ~ElevatorAction(void) {}

        void executeCB(const behaviors::ElevatorGoalConstPtr &goal)
        {
            ROS_INFO_STREAM("elevator_client running callback");
            ros::Rate r(10);
            bool success = false;
            bool timed_out = false;
            bool aborted = false;

            elevator_controller::SetElevatorState srv;
            srv.request.position = goal->position;
            elevator_client_.call(srv);

            elevator_controller::CurElevatorCommand srv_elevator_command;
            if(elevator_cur_command_srv_.call(srv_arm_command)) {
                elevator_cur_command = srv_elevator_command.response.cur_command;
            }
            else {
                ROS_ERROR("Failed to call service elevator_cur_command_");
            }

            double start_time = ros::Time::now().toSec();
            ROS_ERROR("Timeout: %d", goal->timeout);
            while(!success && !timed_out && !aborted) {
                success = fabs(arm_angle - elevator_cur_command) < arm_angle_deadzone;
                if(as_.isPreemptRequested() || !ros::ok()) {
                    ROS_WARN("%s: Preempted", action_name_.c_str());

