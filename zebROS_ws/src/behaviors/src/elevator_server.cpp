#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <behaviors/ElevatorAction.h>
#include <controllers_2019/ElevatorSrv.h>
#include <talon_state_controller/TalonState.h>
#include <controllers_2019/ElevatorSrv.h>
#include "behaviors/enumerated_elevator_indices.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Bool.h"
#include <atomic>
#include <thread>

//TODO: not global. namespace?
double elevator_position_deadzone;

constexpr size_t min_climb_idx = ELEVATOR_DEPLOY; //reference to minimum index in enumerated elevator indices, used for determining which indices are for climbing (this idx or greater)

class ElevatorAction {
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<behaviors::ElevatorAction> as_;
        std::string action_name_;


		//Define service client to control elevator
		ros::ServiceClient elevator_client_;

		ros::ServiceServer level_two_climb_;

		//Subscriber to monitor talon positions
        ros::Subscriber talon_states_sub_;

        double elevator_cur_setpoint_; //stores actual setpoint value to go to, not index
		double cur_position_; //Variable used to store current elevator position

		std::thread publishLvlThread_;
		std::atomic<bool> stopped_;

        void executeCB(const behaviors::ElevatorGoalConstPtr &goal)
        {
            ROS_INFO_STREAM("%s: Running callback " << action_name_.c_str());
            ros::Rate r(10);

			behaviors::ElevatorFeedback feedback;
            feedback.running = true;
            as_.publishFeedback(feedback);

            //Define variables that will be set to true once the server finishes executing
            bool preempted = false;
            bool timed_out = false;

			//Initialize start time of execution
            const double start_time = ros::Time::now().toSec();

			//Determine setpoint (elevator_cur_setpoint_)
			if(goal->setpoint_index >= min_climb_idx) //then it's a climb index
			{
				const size_t climb_setpoint_index = goal->setpoint_index - min_climb_idx;
				if(climb_setpoint_index < climb_locations_.size())
				{
					elevator_cur_setpoint_ = climb_locations_[climb_setpoint_index];
				}
				else
				{
					ROS_ERROR_STREAM("index out of bounds in elevator_server");
					preempted = true;
				}
			}
			else if(goal->place_cargo)
			{
				if(goal->setpoint_index < cargo_locations_.size())
					elevator_cur_setpoint_ = cargo_locations_[goal->setpoint_index];
				else
				{
					ROS_ERROR_STREAM("index out of bounds in elevator_server");
					preempted = true;
				}
			}
			else
			{
				if(goal->setpoint_index < hatch_locations_.size())
					elevator_cur_setpoint_ = hatch_locations_[goal->setpoint_index];
				else
				{
					ROS_ERROR_STREAM("index out of bounds in elevator_server");
					preempted = true;
				}
			}

			if(elevator_cur_setpoint_ < 0) //will occur if a config value wasn't read
			{
				ROS_ERROR_STREAM("Current setpoint was not read");
				preempted = true;
			}

			if(!preempted)
			{
				bool success = false;

				//send request to elevator controller
				controllers_2019::ElevatorSrv srv;
				srv.request.position = elevator_cur_setpoint_;
				srv.request.go_slow = false; //default
				if(goal->setpoint_index >= (min_climb_idx + 1)) //then climbing, go slow, except for ELEVATOR_DEPLOY
				{
					srv.request.go_slow = true;
				}
				if (!elevator_client_.call(srv)) //Send command to elevator controller
				{
					ROS_ERROR("Error calling elevator client in elevator_server");
					success = false;
				}

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
					timed_out = ros::Time::now().toSec() - start_time > timeout_;
				}
			}

			//if we preempt or time out, stop moving the elevator
			if(preempted || timed_out)
			{
				ROS_WARN_STREAM("Elevator server timed out or was preempted");
				controllers_2019::ElevatorSrv srv;
				srv.request.position = cur_position_;
				srv.request.go_slow = false; //default
				// Don't bother checking return code here, since what
				// can be done if this fails?
				elevator_client_.call(srv); //Send command to elevator controller
			}

			//log state of action and set result of action
			behaviors::ElevatorResult result;
			result.timed_out = timed_out;//timed_out refers to last controller call, but applies for whole action

			if(timed_out)
			{
				ROS_INFO("%s:Timed Out", action_name_.c_str());
				result.success = false;
				as_.setSucceeded(result);
			}
			else if(preempted)
			{
				ROS_INFO("%s:Preempted", action_name_.c_str());
				result.success = false;
				as_.setPreempted(result);
			}
			else //implies succeeded
			{
				ROS_INFO("%s:Succeeded", action_name_.c_str());
				result.success = true;
				as_.setSucceeded(result);
			}

			feedback.running = false;
			as_.publishFeedback(feedback);

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

		bool levelTwoClimbServer(std_srvs::SetBool::Request &level_two_climb,
								 std_srvs::SetBool::Response &res)
		{
			climb_locations_ = level_two_climb.data ? climb_locations_level_two_ : climb_locations_level_three_;
			ROS_WARN_STREAM("Level two climb = " << static_cast<bool>(level_two_climb.data));
			res.success = true;
			return true;
		}

		void climbLevelThread()
		{
			std_msgs::Bool level_two_msg;
			stopped_ = false;

			auto level_two_publisher = nh_.advertise<std_msgs::Bool>("level_two",1);

			ros::Rate r(20);

			while(ros::ok() && !stopped_)
			{
				level_two_msg.data = climb_locations_ == climb_locations_level_two_;

				level_two_publisher.publish(level_two_msg);
				r.sleep();
			}
		}

    public:
        ElevatorAction(const std::string &name) :
            as_(nh_, name, boost::bind(&ElevatorAction::executeCB, this, _1), false),
            action_name_(name),
			publishLvlThread_(std::bind(&ElevatorAction::climbLevelThread, this))
        {
			as_.start();

			//Service networking things?
            std::map<std::string, std::string> service_connection_header;
            service_connection_header["tcp_nodelay"] = "1";

			//Server for level two vs. level three climbs
			level_two_climb_ = nh_.advertiseService("level_two_climb_server", &ElevatorAction::levelTwoClimbServer,this);

			//Client for elevator controller
            elevator_client_ = nh_.serviceClient<controllers_2019::ElevatorSrv>("/frcrobot_jetson/elevator_controller/elevator_service", false, service_connection_header);

			//Talon states subscriber
            talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states",1, &ElevatorAction::talonStateCallback, this);

			hatch_locations_.resize(ELEVATOR_MAX_INDEX);
			cargo_locations_.resize(ELEVATOR_MAX_INDEX);
			climb_locations_.resize(ELEVATOR_MAX_INDEX - min_climb_idx);
			climb_locations_level_three_.resize(ELEVATOR_MAX_INDEX - min_climb_idx);
			climb_locations_level_two_.resize(ELEVATOR_MAX_INDEX - min_climb_idx);
        }

		~ElevatorAction(void)
		{
			stopped_ = true;
			publishLvlThread_.join();
		}

		// Make these std::arrays instead
		// TODO - member variables end with an underscore
		std::vector<double> hatch_locations_;
		std::vector<double> cargo_locations_;
		std::vector<double> climb_locations_level_three_;
		std::vector<double> climb_locations_level_two_;
		std::vector<double> climb_locations_; //vector of everything at and after min_climb_idx in enumerated elevator indices, except for max index at the end

		double timeout_;
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

	if (!n_params.getParam("timeout", elevator_action.timeout_))
	{
		ROS_ERROR("Could not read timeout in elevator_server");
		elevator_action.timeout_ = 5;
	}

	if (!n_params.getParam("elevator_position_deadzone", elevator_position_deadzone))
	{
		ROS_ERROR("Could not read elevator_deadzone in elevator_server");
		elevator_position_deadzone = 0.1;
	}

	//read locations for elevator placement for HATCH PANEL
	double hatch_intake_end_position;
	if (!n_params.getParam("hatch/intake_end_position", hatch_intake_end_position))
	{
		ROS_ERROR_STREAM("Could not read hatch_intake_end_position");
		hatch_intake_end_position = -1; //signals to server to preempt
	}
	elevator_action.hatch_locations_[CARGO_SHIP] = hatch_intake_end_position;

	double hatch_rocket1_position;
	if (!n_params.getParam("hatch/rocket1_position", hatch_rocket1_position))
	{
		ROS_ERROR_STREAM("Could not read hatch_rocket1_position");
		hatch_rocket1_position = -1; //signals to server to preempt
	}
	elevator_action.hatch_locations_[ROCKET_1] = hatch_rocket1_position;

	double hatch_rocket2_position;
	if (!n_params.getParam("hatch/rocket2_position", hatch_rocket2_position))
	{
		ROS_ERROR_STREAM("Could not read hatch_rocket2_position");
		hatch_rocket2_position = -1; //signals to server to preempt
	}
	elevator_action.hatch_locations_[ROCKET_2] = hatch_rocket2_position;

	double hatch_rocket3_position;
	if (!n_params.getParam("hatch/rocket3_position", hatch_rocket3_position))
	{
		ROS_ERROR_STREAM("Could not read hatch_rocket3_position");
		hatch_rocket3_position = -1; //signals to server to preempt
	}
	elevator_action.hatch_locations_[ROCKET_3] = hatch_rocket3_position;

	double hatch_intake_position;
	if (!n_params.getParam("hatch/intake_position", hatch_intake_position))
	{
		ROS_ERROR_STREAM("Could not read hatch_intake_position");
		hatch_intake_position = 0; //0 should be the bottom b/c of auto-zeroing
	}
	elevator_action.hatch_locations_[INTAKE] = hatch_intake_position;

	//read locations for elevator placement for CARGO
	double cargo_cargo_ship_position;
	if (!n_params.getParam("cargo/cargo_ship_position", cargo_cargo_ship_position))
	{
		ROS_ERROR_STREAM("Could not read cargo_cargo_ship_position");
		cargo_cargo_ship_position = -1; //signals to server to preempt
	}
	elevator_action.cargo_locations_[CARGO_SHIP] = cargo_cargo_ship_position;

	double cargo_rocket1_position;
	if (!n_params.getParam("cargo/rocket1_position", cargo_rocket1_position))
	{
		ROS_ERROR_STREAM("Could not read cargo_rocket1_position");
		cargo_rocket1_position = -1; //signals to server to preempt
	}
	elevator_action.cargo_locations_[ROCKET_1] = cargo_rocket1_position;

	double cargo_rocket2_position;
	if (!n_params.getParam("cargo/rocket2_position", cargo_rocket2_position))
	{
		ROS_ERROR_STREAM("Could not read cargo_rocket2_position");
		cargo_rocket2_position = -1; //signals to server to preempt
	}
	elevator_action.cargo_locations_[ROCKET_2] = cargo_rocket2_position;

	double cargo_rocket3_position;
	if (!n_params.getParam("cargo/rocket3_position", cargo_rocket3_position))
	{
		ROS_ERROR_STREAM("Could not read cargo_rocket3_position");
		cargo_rocket3_position = -1; //signals to server to preempt
	}
	elevator_action.cargo_locations_[ROCKET_3] = cargo_rocket3_position;

	double cargo_intake_position;
	if (!n_params.getParam("cargo/intake_position", cargo_intake_position))
	{
		ROS_ERROR_STREAM("Could not read cargo_intake_position");
		cargo_intake_position = 0; //should be correct b/c of auto-zeroing
	}
	elevator_action.cargo_locations_[INTAKE] = cargo_intake_position;

	//read locations for elevator climbing setpoints
	double climb_deploy_position;
	if (!n_params.getParam("climber2/deploy_position", climb_deploy_position))
	{
		ROS_ERROR_STREAM("Could not read deploy_position");
		climb_deploy_position = -1; //signals to server to preempt
	}
	elevator_action.climb_locations_level_two_[ELEVATOR_DEPLOY - min_climb_idx] = climb_deploy_position;

	if (!n_params.getParam("climber3/deploy_position", climb_deploy_position))
	{
		ROS_ERROR_STREAM("Could not read deploy_position");
		climb_deploy_position = -1; //signals to server to preempt
	}
	elevator_action.climb_locations_level_three_[ELEVATOR_DEPLOY - min_climb_idx] = climb_deploy_position;

	double climb_position;
	if (!n_params.getParam("climber2/climb_position", climb_position))
	{
		ROS_ERROR_STREAM("Could not read climb_position");
		climb_position = -1; //signals to server to preempt
	}
	elevator_action.climb_locations_level_two_[ELEVATOR_CLIMB - min_climb_idx] = climb_position;

	if (!n_params.getParam("climber3/climb_position", climb_position))
	{
		ROS_ERROR_STREAM("Could not read climb_position");
		climb_position = -1; //signals to server to preempt
	}
	elevator_action.climb_locations_level_three_[ELEVATOR_CLIMB - min_climb_idx] = climb_position;

	double climb_low_position;
	if (!n_params.getParam("climber2/climb_low_position", climb_low_position))
	{
		ROS_ERROR_STREAM("Could not read climb_low_position");
		climb_low_position = -1; //signals to server to preempt
	}
	elevator_action.climb_locations_level_two_[ELEVATOR_CLIMB_LOW - min_climb_idx] = climb_low_position;

	if (!n_params.getParam("climber3/climb_low_position", climb_low_position))
	{
		ROS_ERROR_STREAM("Could not read climb_low_position");
		climb_low_position = -1; //signals to server to preempt
	}
	elevator_action.climb_locations_level_three_[ELEVATOR_CLIMB_LOW - min_climb_idx] = climb_low_position;

	double climb_raise_position;
	if (!n_params.getParam("climber2/climb_raise_position", climb_raise_position))
	{
		ROS_ERROR_STREAM("Could not read climb_low_position");
		climb_raise_position = -1; //signals to server to preempt
	}
	elevator_action.climb_locations_level_two_[ELEVATOR_RAISE - min_climb_idx] = climb_raise_position;

	if (!n_params.getParam("climber3/climb_raise_position", climb_raise_position))
	{
		ROS_ERROR_STREAM("Could not read climb_low_position");
		climb_raise_position = -1; //signals to server to preempt
	}
	elevator_action.climb_locations_level_three_[ELEVATOR_RAISE - min_climb_idx] = climb_raise_position;

	elevator_action.climb_locations_ = elevator_action.climb_locations_level_three_;

	ros::spin();

	return 0;
}
