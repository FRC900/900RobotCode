#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <behaviors/ElevatorAction.h>
#include <elevator_controller/ElevatorSrv.h>
#include <talon_state_controller/TalonState.h>
#include <elevator_controller/ElevatorSrv.h>
#include "behaviors/enumerated_elevator_indices.h"

//TODO: not global. namespace?
double elevator_position_deadzone;

class ElevatorAction {
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<behaviors::ElevatorAction> as_;
        std::string action_name_;

		//Define Feedback and result messages
        behaviors::ElevatorFeedback feedback_;
        behaviors::ElevatorResult result_;

		//Define service client to control elevator
        ros::ServiceClient elevator_client_;

		//Subscriber to monitor talon positions
        ros::Subscriber talon_states_sub;

        double elevator_cur_setpoint_;
		double cur_position_; //Variable used to store current elevator position


    public:
		std::vector<double> hatch_locations;
		std::vector<double> cargo_locations;
		std::vector<double> climb_locations; //vector of everything after index 4 in enumerated elevator indices, except for max index at the end

		double timeout;

        ElevatorAction(std::string name) :
            as_(nh_, name, boost::bind(&ElevatorAction::executeCB, this, _1), false),
            action_name_(name)
        {
			as_.start();

			//Service networking things?
            std::map<std::string, std::string> service_connection_header;
            service_connection_header["tcp_nodelay"] = "1";

			//Client for elevator controller
            elevator_client_ = nh_.serviceClient<elevator_controller::ElevatorSrv>("/frcrobot_jetson/elevator_controller/elevator_service", false, service_connection_header);

			//Talon states subscriber
            talon_states_sub = nh_.subscribe("/frcrobot_jetson/talon_states",1, &ElevatorAction::talonStateCallback, this);

			hatch_locations.resize(ELEVATOR_MAX_INDEX);
			cargo_locations.resize(ELEVATOR_MAX_INDEX);
			climb_locations.resize(ELEVATOR_MAX_INDEX - 5);
        }

        ~ElevatorAction(void) {}

        void executeCB(const behaviors::ElevatorGoalConstPtr &goal)
        {
            ROS_INFO_STREAM("%s: Running callback " << action_name_.c_str());
            ros::Rate r(10);

			//Define variables that will be set to true once the server finishes executing
            bool success = false;
			bool preempted = false;
            bool timed_out = false;

			//Initialize start time of execution
            double start_time = ros::Time::now().toSec();

			//define variables to store goal received by the elevator server
			double setpoint_index = goal->setpoint_index;
			bool place_cargo = goal->place_cargo;

			//Determine setpoint (elevator_cur_setpoint_)
			if(setpoint_index >= 5) //then it's a climb index
			{
				double climb_setpoint_index = setpoint_index - 5;
				if(climb_setpoint_index < climb_locations.size())
					elevator_cur_setpoint_ = climb_locations[climb_setpoint_index];
				else
					ROS_ERROR_STREAM("index out of bounds in elevator_server");
			}
			else if(place_cargo)
			{
				if(setpoint_index < cargo_locations.size())
					elevator_cur_setpoint_ = cargo_locations[setpoint_index];
				else
					ROS_ERROR_STREAM("index out of bounds in elevator_server");
			}
			else
			{
				if(setpoint_index < hatch_locations.size())
					elevator_cur_setpoint_ = hatch_locations[setpoint_index];
				else
					ROS_ERROR_STREAM("index out of bounds in elevator_server");
			}

			//send request to elevator controller
            elevator_controller::ElevatorSrv srv;
			srv.request.position = elevator_cur_setpoint_;
			srv.request.go_slow = false; //default
			if(setpoint_index >= 5) //then climbing, go slow
			{
				srv.request.go_slow = true;
			}
            elevator_client_.call(srv); //Send command to elevator controller

			//wait for elevator controller to finish
            while(!success && !timed_out && !preempted) {
                success = fabs(cur_position_ - elevator_cur_setpoint_) < elevator_position_deadzone;
				ROS_INFO_STREAM("elevator server: cur position = " << cur_position_ << " elevator_cur_setpoint " << elevator_cur_setpoint_);
				if(as_.isPreemptRequested() || !ros::ok())
				{
                    ROS_ERROR("%s: Preempted", action_name_.c_str());
					preempted = true;
				}
				else
				{
					ros::spinOnce();
					r.sleep();
				}
				timed_out = ros::Time::now().toSec() - start_time > timeout;
			}



			//log state of action and set result of action

			result_.timed_out = timed_out;//timed_out refers to last controller call, but applies for whole action
			result_.success = success; //success refers to last controller call, but applies for whole action

			if(timed_out)
			{
				ROS_INFO("%s:Timed Out", action_name_.c_str());
				as_.setSucceeded(result_);
			}
			else if(preempted)
			{
				ROS_INFO("%s:Preempted", action_name_.c_str());
				as_.setPreempted(result_);
			}
			else //implies succeeded
			{
				ROS_INFO("%s:Succeeded", action_name_.c_str());
				as_.setSucceeded(result_);
			}

			return;
		}


		void talonStateCallback(const talon_state_controller::TalonState &talon_state)
		{
			static size_t elevator_master_idx = std::numeric_limits<size_t>::max();
			if (elevator_master_idx >= talon_state.name.size())
			{
				for (size_t i = 0; i < talon_state.name.size(); i++)
				{
					if (talon_state.name[i] == "elevator_master")
					{
						elevator_master_idx = i;
						break;
					}
				}
			}
			else {
				cur_position_ = talon_state.position[elevator_master_idx];
			}
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

	//TODO: This needs to be part of the elevator controller and not the actionlib server
	//if (!n_params.getParam("max_speed", max_speed))
	//	ROS_ERROR("Could not read max_speed in elevator_server");

	if (!n_params.getParam("timeout", elevator_action.timeout))
		ROS_ERROR("Could not read timeout in elevator_server");

	if (!n_params.getParam("elevator_position_deadzone", elevator_position_deadzone))
		ROS_ERROR("Could not read elevator_deadzone in elevator_server");

	//read locations for elevator placement for HATCH PANEL
	double hatch_cargo_ship_position;
	if (!n_params.getParam("hatch/cargo_ship_position", hatch_cargo_ship_position))
		ROS_ERROR_STREAM("Could not read hatch_cargo_ship_position");
	elevator_action.hatch_locations[CARGO_SHIP] = hatch_cargo_ship_position;

	double hatch_rocket1_position;
	if (!n_params.getParam("hatch/rocket1_position", hatch_rocket1_position))
		ROS_ERROR_STREAM("Could not read hatch_rocket1_position");
	elevator_action.hatch_locations[ROCKET_1] = hatch_rocket1_position;

	double hatch_rocket2_position;
	if (!n_params.getParam("hatch/rocket2_position", hatch_rocket2_position))
		ROS_ERROR_STREAM("Could not read hatch_rocket2_position");
	elevator_action.hatch_locations[ROCKET_2] = hatch_rocket2_position;

	double hatch_rocket3_position;
	if (!n_params.getParam("hatch/rocket3_position", hatch_rocket3_position))
		ROS_ERROR_STREAM("Could not read hatch_rocket3_position");
	elevator_action.hatch_locations[ROCKET_3] = hatch_rocket3_position;

	double hatch_intake_position;
	if (!n_params.getParam("hatch/intake_position", hatch_intake_position))
		ROS_ERROR_STREAM("Could not read hatch_intake_position");
	elevator_action.hatch_locations[INTAKE] = hatch_intake_position;

	//read locations for elevator placement for CARGO
	double cargo_cargo_ship_position;
	if (!n_params.getParam("cargo/cargo_ship_position", cargo_cargo_ship_position))
		ROS_ERROR_STREAM("Could not read cargo_cargo_ship_position");
	elevator_action.cargo_locations[CARGO_SHIP] = cargo_cargo_ship_position;

	double cargo_rocket1_position;
	if (!n_params.getParam("cargo/rocket1_position", cargo_rocket1_position))
		ROS_ERROR_STREAM("Could not read cargo_rocket1_position");
	elevator_action.cargo_locations[ROCKET_1] = cargo_rocket1_position;

	double cargo_rocket2_position;
	if (!n_params.getParam("cargo/rocket2_position", cargo_rocket2_position))
		ROS_ERROR_STREAM("Could not read cargo_rocket2_position");
	elevator_action.cargo_locations[ROCKET_2] = cargo_rocket2_position;

	double cargo_rocket3_position;
	if (!n_params.getParam("cargo/rocket3_position", cargo_rocket3_position))
		ROS_ERROR_STREAM("Could not read cargo_rocket3_position");
	elevator_action.cargo_locations[ROCKET_3] = cargo_rocket3_position;

	double cargo_intake_position;
	if (!n_params.getParam("cargo/intake_position", cargo_intake_position))
		ROS_ERROR_STREAM("Could not read cargo_intake_position");
	elevator_action.cargo_locations[INTAKE] = cargo_intake_position;

	//read locations for elevator climbing setpoints
	double climb_deploy_position;
	if (!n_params.getParam("climber/deploy_position", climb_deploy_position))
		ROS_ERROR_STREAM("Could not read deploy_position");
	elevator_action.climb_locations[ELEVATOR_DEPLOY - 5] = climb_deploy_position;

	double climb_position;
	if (!n_params.getParam("climber/climb_position", climb_position))
		ROS_ERROR_STREAM("Could not read climb_position");
	elevator_action.climb_locations[ELEVATOR_CLIMB - 5] = climb_position;

	double climb_low_position;
	if (!n_params.getParam("climber/climb_low_position", climb_low_position))
		ROS_ERROR_STREAM("Could not read climb_low_position");
	elevator_action.climb_locations[ELEVATOR_CLIMB_LOW - 5] = climb_low_position;

	ros::spin();
	return 0;
}
