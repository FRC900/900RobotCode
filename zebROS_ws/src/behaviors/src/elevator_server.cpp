#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behaviors/ElevatorAction.h>
#include <elevator_controller/ElevatorSrv.h>
#include <talon_state_controller/TalonState.h>

double elevator_deadzone;
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
            bool preempted = false;

            elevator_controller::SetElevatorState srv;
            srv.request.position = goal->elevator_setpoint;
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
            while(!success && !timed_out && !preempted) {
                success = fabs(goal->elevator_setpoint - elevator_cur_command) < elevator_deadzone;
				time_out = (ros::Time::now().toSec()-start_time) >intake_timeout;
                if(as_.isPreemptRequested() || !ros::ok()) {
                    ROS_WARN("%s: Preempted", action_name_.c_str());
					as_.setPreempted();
					preempted = true;
				}
				else{
					r.sleep();
					ros::spinOnce();
				}
			}
		}
		//log state of action and set result of action
		if(timed_out)
		{
			ROS_INFO("%s:Timed Out", action_name_.c_str());
		}
		else if(preempted)
		{
			ROS_INFO("%s:Preempted", action_name_.c_str());
		}
		else //implies succeeded
		{
			ROS_INFO("%s:Succeeded", action_name_.c_str());
		}

		result_.timed_out = timed_out;//timed_out refers to last controller call, but applies for whole action
		result_.success = success; //success refers to last controller call, but applies for whole action
		as_.setSucceeded(result_); //pretend it succeeded no matter what, but tell what actually happened with the result - helps with SMACH
		return;
	}
	void talonStateCallback(const talon_state_controller::TalonState &talon_state)
	{
		static size_t elevator_master_idx = std::numeric_limits<size_t>::max();
		if (elevator_master_idx >= talon_state.name.size())
		{
			for (size_t i = 0; i < talon_state.name(); i++)
			{
				if (talon_state.name[i] == "elevator_joint_") //figure out what actually goes here!!!
				{
					elevator_master_idx = i;
					break;
				}
			}
		}
		else{
			goal->elevator_setpoint = talon_state.position[elevator_master_idx];
			}
	};

int main(int argc, char** argv)
{
	//creates node
	ros::init(argc, argv,"elevator_server");

	//creates elevator actionlib server
	ElevatorAction elevator_action("elevator_server");

	//gets config values
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "actionlib_lift_params");

	if (!n_params.getParam("max_speed", max_speed))
		ROS_ERROR("Could not read max_speed in elevator_server");

	if (!n_params.getParam("timeout", timeout))
		ROS_ERROR("Could not read timeout in elevator_server");

	if (!n_params.getParam("elevator_deadzone", elevator_deadzone))
		ROS_ERROR("Could not read elevator_deadzone in elevator_server");

	ros::spin();
	return 0;
}
