#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <behavior_actions/IntakeAction.h>
#include <behavior_actions/PlaceAction.h>
#include <behavior_actions/ElevatorAction.h>
#include <behavior_actions/ClimbAction.h>
#include <behavior_actions/AlignAction.h>
#include <behavior_actions/ShooterAction.h>
#include <path_follower/PathAction.h>
#include <behavior_actions/IndexerAction.h>
#include <behavior_actions/enumerated_elevator_indices.h>
#include <boost/algorithm/string.hpp>
#include <string>

double server_wait_timeout = 20.0; //how long to wait for a server to exist before exiting, in sec.
double server_exec_timeout = 20.0; //how long to wait for an actionlib server call to finish before timing out, in sec. Used for all actionlib calls


bool callElevator(int setpoint_idx)
{
	//create client to call actionlib server
	actionlib::SimpleActionClient<behavior_actions::ElevatorAction> elevator_ac("/elevator/elevator_server", true);

	ROS_INFO("Waiting for elevator server to start.");
	if(!elevator_ac.waitForServer(ros::Duration(server_wait_timeout)))
	{
		ROS_ERROR("Could not find server within %f seconds.", server_wait_timeout);
	}

	ROS_INFO("Sending goal to elevator server, setpoint = %d", setpoint_idx);
	// send a goal to the action
	behavior_actions::ElevatorGoal goal;
	goal.setpoint_index = setpoint_idx;
	elevator_ac.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = elevator_ac.waitForResult(ros::Duration(server_exec_timeout));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = elevator_ac.getState();
		ROS_INFO("Action finished with state: %s",state.toString().c_str());
		if(elevator_ac.getResult()->timed_out)
		{
			ROS_INFO("Elevator Server timed out!");
		}
		return true;
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
		return false;
	}

}

bool callIntakePowercell()
{
	//create client to call actionlib server
	actionlib::SimpleActionClient<behavior_actions::IntakeAction> intake_powercell_ac("/powercell_intake/powercell_intake_server", true);

	ROS_INFO("Waiting for powercell intake server to start.");
	if(!intake_powercell_ac.waitForServer(ros::Duration(server_wait_timeout)))
	{
		ROS_ERROR("Could not find server within %f seconds.", server_wait_timeout);
	}

	ROS_INFO("Sending goal to intake powercell server.");
	//send a goal to the action
	behavior_actions::IntakeGoal intake_powercell_goal;
	intake_powercell_ac.sendGoal(intake_powercell_goal);

	//wait for the action to return
	bool finished_before_timeout = intake_powercell_ac.waitForResult(ros::Duration(server_exec_timeout));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = intake_powercell_ac.getState();
		ROS_INFO("Action finished with state: %s", state.toString().c_str());
		if(intake_powercell_ac.getResult()->timed_out)
		{
			ROS_INFO("Intake cargo server timed out!");
		}
		return true;
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
		return false;
	}

}	
bool callIntakeCargo()
{
	//create client to call actionlib server
	actionlib::SimpleActionClient<behavior_actions::IntakeAction> intake_cargo_ac("/cargo_intake/cargo_intake_server", true);

	ROS_INFO("Waiting for cargo intake server to start.");
	if(!intake_cargo_ac.waitForServer(ros::Duration(server_wait_timeout)))
	{
		ROS_ERROR("Could not find server within %f seconds.", server_wait_timeout);
	}

	ROS_INFO("Sending goal to intake cargo server.");
	// send a goal to the action
	behavior_actions::IntakeGoal intake_cargo_goal;
	intake_cargo_ac.sendGoal(intake_cargo_goal);

	//wait for the action to return
	bool finished_before_timeout = intake_cargo_ac.waitForResult(ros::Duration(server_exec_timeout));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = intake_cargo_ac.getState();
		ROS_INFO("Action finished with state: %s",state.toString().c_str());
		if(intake_cargo_ac.getResult()->timed_out)
		{
			ROS_INFO("Intake cargo server timed out!");
		}
		return true;
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
		return false;
	}
}

bool callOuttakeCargo(int setpoint_idx)
{
	//create client to call actionlib server
	actionlib::SimpleActionClient<behavior_actions::PlaceAction> outtake_cargo_ac("/cargo_outtake/cargo_outtake_server", true);

	ROS_INFO("Waiting for cargo outtake server to start.");
	if(!outtake_cargo_ac.waitForServer(ros::Duration(server_wait_timeout)))
	{
		ROS_ERROR("Could not find server within %f seconds.", server_wait_timeout);
	}

	ROS_INFO("Sending goal to outtake cargo server.");
	// send a goal to the action
	behavior_actions::PlaceGoal outtake_cargo_goal;
	outtake_cargo_goal.setpoint_index = setpoint_idx;
	outtake_cargo_ac.sendGoal(outtake_cargo_goal);

	//wait for the action to return
	bool finished_before_timeout = outtake_cargo_ac.waitForResult(ros::Duration(server_exec_timeout));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = outtake_cargo_ac.getState();
		ROS_INFO("Action finished with state: %s",state.toString().c_str());
		if(outtake_cargo_ac.getResult()->timed_out)
		{
			ROS_INFO("Outtake cargo server timed out!");
		}
		return true;
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
		return false;
	}
}

bool callIntakeHatchPanel()
{
	//create client to call actionlib server
	actionlib::SimpleActionClient<behavior_actions::IntakeAction> intake_hatch_panel_ac("/hatch_intake/intake_hatch_panel_server", true);

	ROS_INFO("Waiting for panel intake server to start.");
	if(!intake_hatch_panel_ac.waitForServer(ros::Duration(server_wait_timeout)))
	{
		ROS_ERROR("Could not find server within %f seconds.", server_wait_timeout);
	}

	ROS_INFO("Sending goal to intake hatch panel server.");
	// send a goal to the action
	behavior_actions::IntakeGoal intake_hatch_panel_goal;
	intake_hatch_panel_ac.sendGoal(intake_hatch_panel_goal);

	//wait for the action to return
	bool finished_before_timeout = intake_hatch_panel_ac.waitForResult(ros::Duration(server_exec_timeout));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = intake_hatch_panel_ac.getState();
		ROS_INFO("Action finished with state: %s",state.toString().c_str());
		if(intake_hatch_panel_ac.getResult()->timed_out)
		{
			ROS_INFO("Panel intake server timed out!");
		}
		return true;
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
		return false;
	}
}

bool callOuttakeHatchPanel(int setpoint_idx)
{
	//create client to call actionlib server
	actionlib::SimpleActionClient<behavior_actions::PlaceAction> outtake_hatch_panel_ac("/hatch_outtake/outtake_hatch_panel_server", true);

	ROS_INFO("Waiting for panel outtake server to start.");
	if(!outtake_hatch_panel_ac.waitForServer(ros::Duration(server_wait_timeout)))
	{
		ROS_ERROR("Could not find server within %f seconds.", server_wait_timeout);
	}

	ROS_INFO("Sending goal to outtake hatch panel server.");
	// send a goal to the action
	behavior_actions::PlaceGoal outtake_hatch_panel_goal;
	outtake_hatch_panel_goal.setpoint_index = setpoint_idx;
	outtake_hatch_panel_ac.sendGoal(outtake_hatch_panel_goal);

	//wait for the action to return
	bool finished_before_timeout = outtake_hatch_panel_ac.waitForResult(ros::Duration(server_exec_timeout));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = outtake_hatch_panel_ac.getState();
		ROS_INFO("Action finished with state: %s",state.toString().c_str());
		if(outtake_hatch_panel_ac.getResult()->timed_out)
		{
			ROS_INFO("Panel outtake server timed out!");
		}
		return true;
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
		return false;
	}
}

bool callClimber(int step)
{
	//create client to call actionlib server
	actionlib::SimpleActionClient<behavior_actions::ClimbAction> climber_ac("/climber/climber_server", true);

	ROS_INFO("Waiting for climber server to start.");
	if(!climber_ac.waitForServer(ros::Duration(server_wait_timeout)))
	{
		ROS_ERROR("Could not find server within %f seconds.", server_wait_timeout);
	}

	ROS_INFO("Sending goal to climber server.");
	// send a goal to the action
	behavior_actions::ClimbGoal climb_goal;
	climb_goal.step = step;
	climber_ac.sendGoal(climb_goal);

	//wait for the action to return
	bool finished_before_timeout = climber_ac.waitForResult(ros::Duration(server_exec_timeout));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = climber_ac.getState();
		ROS_INFO("Action finished with state: %s",state.toString().c_str());
		if(climber_ac.getResult()->timed_out)
		{
			ROS_INFO("Climber server timed out!");
		}
		return true;
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
		return false;
	}
}

bool callShooter()
{
	//create client to call actionlib server
	actionlib::SimpleActionClient<behavior_actions::ShooterAction> shooter_ac("/shooter/shooter_server", true);

	ROS_INFO("Waiting for shooter server to start.");
	if(!shooter_ac.waitForServer(ros::Duration(server_wait_timeout)))
	{
		ROS_ERROR("Could not find server within %f seconds.", server_wait_timeout);
	}

	ROS_INFO("Sending goal to shooter server.");
	// send a goal to the action
	behavior_actions::ShooterGoal goal;
	shooter_ac.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = shooter_ac.waitForResult(ros::Duration(server_exec_timeout));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = shooter_ac.getState();
		ROS_INFO("Action finished with state: %s",state.toString().c_str());
		if(shooter_ac.getResult()->timed_out)
		{
			ROS_INFO("Climber server timed out!");
		}
		return true;
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
		return false;
	}
}

bool callAlignHatch()
{
	actionlib::SimpleActionClient<behavior_actions::AlignAction> align_hatch_ac("/align_hatch/align_hatch_server", true);

	ROS_INFO("Waiting for align hatch server to start.");
	if(!align_hatch_ac.waitForServer(ros::Duration(server_wait_timeout)))
	{
		ROS_ERROR("callAlignHatch : Could not find server.");
		return false;
	}

	ROS_INFO("callAlignHatch : Sending goal to the server.");
	behavior_actions::AlignGoal align_goal;
	align_hatch_ac.sendGoal(align_goal);

	//wait for the action to return
	bool finished_before_timeout = align_hatch_ac.waitForResult(ros::Duration(server_exec_timeout));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = align_hatch_ac.getState();
		ROS_INFO("callAlignHatch : Action finished with state: %s",state.toString().c_str());
		if(align_hatch_ac.getResult()->timed_out)
		{
			ROS_INFO("callAlignHatch : Align hatch server timed out!");
		}
		return true;
	}
	else
	{
		ROS_INFO("callAlignHatch : Action did not finish before the time out.");
		return false;
	}
}

bool callPath(double path_x_setpoint, double path_y_setpoint, double path_z_setpoint, double path_x2_setpoint, double path_y2_setpoint, double path_z2_setpoint)
{
        ROS_INFO_STREAM("path_x_setpoint = " << path_x_setpoint);
	actionlib::SimpleActionClient<path_follower::PathAction> path_ac("/path_follower/path_follower_server", true);

	ROS_INFO("Waiting for pure pursuit server to start.");
	if(!path_ac.waitForServer(ros::Duration(server_wait_timeout)))
	{
		ROS_ERROR("callPath: Could not find server.");
		return false;
	}

	ROS_INFO("callPath: Sending goal to the server.");
	path_follower::PathGoal path_goal;
	path_goal.points.resize(3);
	path_goal.points[0].x = 0;
	path_goal.points[0].y = 0;
	path_goal.points[0].z = 0;
	path_goal.points[1].x = path_x_setpoint;
	path_goal.points[1].y = path_y_setpoint;
	path_goal.points[1].z = path_z_setpoint;
	path_goal.points[2].x = path_x2_setpoint;
	path_goal.points[2].y = path_y2_setpoint;
	path_goal.points[2].z = path_z2_setpoint;
	path_ac.sendGoal(path_goal);

	//wait for the action to return
	bool finished_before_timeout = path_ac.waitForResult(ros::Duration(server_exec_timeout));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = path_ac.getState();
		ROS_INFO("callPath: Action finished with state: %s",state.toString().c_str());
		if(path_ac.getResult()->timed_out)
		{
			ROS_INFO("callPath: Path server timed out!");
		}
		return true;
	}
	else
	{
		ROS_INFO("callPath : Action did not finish before the time out.");
		return false;
	}
}



bool callIndexer(int indexer_action)
{
	//create client to call actionlib server
	actionlib::SimpleActionClient<behavior_actions::IndexerAction> indexer_ac("/indexer/indexer_server", true);

	ROS_INFO("Waiting for indexer server to start.");
	if(!indexer_ac.waitForServer(ros::Duration(server_wait_timeout)))
	{
		ROS_ERROR("Could not find server within %f seconds.", server_wait_timeout);
	}

	ROS_INFO("Sending goal to indexer server.");
	// send a goal to the action
	behavior_actions::IndexerGoal indexer_goal;
	indexer_goal.action = indexer_action;
	indexer_ac.sendGoal(indexer_goal);

	//wait for the action to return
	bool finished_before_timeout = indexer_ac.waitForResult(ros::Duration(server_exec_timeout));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = indexer_ac.getState();
		ROS_INFO("Action finished with state: %s",state.toString().c_str());
		if(indexer_ac.getResult()->timed_out)
		{
			ROS_INFO("Indexer server timed out!");
		}
		return true;
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
		return false;
	}
}



int main (int argc, char **argv)
{
	/*GET DATA FROM USER INPUT
	 * user will type into command line: rosrun behaviors test_actionlib run:=what_to_run setpoint:=setpoint
	 * Possibilities:
	 * What to run:
	 *	intake_cargo
	 *	outtake_cargo
	 *	intake_hatch_panel
	 *	outtake_hatch_panel
	 *	elevator
	 * Setpoint:
	 *	Same thing as in enumerated_elevator_indices.h
	 */
	std::string what_to_run;
	std::string elevator_setpoint;
	double path_x_setpoint;
	double path_y_setpoint;
	double path_z_setpoint;
	double path_x2_setpoint;
	double path_y2_setpoint;
	double path_z2_setpoint;
	int indexer_action;

	what_to_run = ros::getROSArg(argc, argv, "run"); //if can't find the argument, will default to an empty string of length 0
	boost::algorithm::to_lower(what_to_run); //convert to lower case
	//make sure user told us what to run
	if(what_to_run.length() == 0)
	{
		ROS_ERROR("You need to specify the run functionality with: rosrun behaviors test_actionlib run:=____");
		ROS_ERROR("Possible values for run: all, indexer, intake_cargo, outtake_cargo, intake_hatch_panel, outtake_hatch_panel, elevator, climber0, climber1, climber2, climber3, shooter, path");
		ROS_ERROR("Note: 'all' will not run the climber");
		return 0;
	}
	elevator_setpoint = ros::getROSArg(argc, argv, "setpoint"); //only used for elevator call or outtake call. Not used for the 'all' run option
	boost::algorithm::to_upper(elevator_setpoint); //convert to upper case

	//make sure we have the setpoint if we need it, and determine what it is
	int setpoint_idx;
	if(what_to_run == "outtake_cargo" || what_to_run == "outtake_hatch_panel" || what_to_run == "elevator")
	{
		//make sure user entered a setpoint
		if(elevator_setpoint.length() == 0)
		{
			ROS_ERROR("You need to specify an elevator setpoint with: rosrun behaviors test_actionlib run:=____ setpoint:=_____");
			ROS_ERROR("Setpoint values are those found in enumerated elevator indices");
			return 0;
		}
		//determine the setpoint index
		if(elevator_setpoint == "CARGO_SHIP") {
			setpoint_idx = CARGO_SHIP;
		}
		else if(elevator_setpoint == "ROCKET_1") {
			setpoint_idx = ROCKET_1;
		}
		else if(elevator_setpoint == "ROCKET_2") {
			setpoint_idx = ROCKET_2;
		}
		else if(elevator_setpoint == "ROCKET_3") {
			setpoint_idx = ROCKET_3;
		}
		else if(elevator_setpoint == "INTAKE") {
			setpoint_idx = INTAKE;
		}
		else if(elevator_setpoint == "ELEVATOR_DEPLOY") {
			setpoint_idx = ELEVATOR_DEPLOY;
		}
		else if(elevator_setpoint == "ELEVATOR_CLIMB") {
			setpoint_idx = ELEVATOR_CLIMB;
		}
		else if(elevator_setpoint == "ELEVATOR_CLIMB_LOW") {
			setpoint_idx = ELEVATOR_CLIMB;
		}
		else if(elevator_setpoint == "ELEVATOR_MAX_INDEX") {
			setpoint_idx = ELEVATOR_MAX_INDEX;
		}
		else {
			ROS_ERROR("Invalid elevator setpoint");
			return 0;
		}
	}
	else {
		elevator_setpoint = "N/A";
	}


	ROS_WARN("what_to_run: %s", what_to_run.c_str());
	ROS_WARN("setpoint: %s", elevator_setpoint.c_str());

	//Actually run stuff ---------------------------------

	ros::init(argc, argv, "test_actionlib");

	//determine what to run and do it
	std::string user_input;
	if(what_to_run == "all") {
		int test_idx = 0; //0 means the first thing to test, 1 the next, etc. This is incremented by user input keyboard presses
		while(ros::ok())
		{
			ROS_INFO("Next action? Type y for yes, and press enter.");
			std::getline(std::cin, user_input); //wait for user input
			if(user_input == "y")
			{
				//then do stuff:
				if(test_idx < 5)
				{
					//if(!callElevator(test_idx)) {return 0;} //this is a bit of a hack, but it will test elevator indices 0 through 4
					callElevator(test_idx);
				}
				else if(test_idx == 5)
				{
					if(!callIntakeCargo()) {return 0;}
				}
				else if(test_idx == 6)
				{
					if(!callOuttakeCargo(CARGO_SHIP)) {return 0;}
				}
				else if(test_idx == 7)
				{
					if(!callIntakeHatchPanel()) {return 0;}
				}
				else if(test_idx == 8)
				{
					if(!callOuttakeHatchPanel(INTAKE)) {return 0;}
				}
				else if(test_idx == 9)
				{
					ROS_INFO("Done testing.");
					return 0;
				}
				test_idx++;
			}
		}
	}
	else if(what_to_run == "intake_powercell"){
		callIntakePowercell();
	}
	else if(what_to_run == "intake_cargo") {
		callIntakeCargo();
	}
	else if(what_to_run == "outtake_cargo") {
		callOuttakeCargo(setpoint_idx);
	}
	else if(what_to_run == "intake_hatch_panel") {
		callIntakeHatchPanel();
	}
	else if(what_to_run == "outtake_hatch_panel") {
		callOuttakeHatchPanel(setpoint_idx);
	}
	else if(what_to_run == "elevator") {
		callElevator(setpoint_idx);
	}
	else if(what_to_run == "climber0") {
		callClimber(0);
	}
	else if(what_to_run == "climber1") {
		callClimber(1);
	}
	else if(what_to_run == "climber2") {
		callClimber(2);
	}
	else if(what_to_run == "climber3") {
		callClimber(3);
	}
	else if(what_to_run == "align_hatch")
	{
		callAlignHatch();
	}
	else if(what_to_run == "path")
	{
			std::cout << "Enter x1: ";
            std::cin >> path_x_setpoint;
			std::cout << "Enter y1: ";
            std::cin >> path_y_setpoint;
			std::cout << "Enter z1: ";
            std::cin >> path_z_setpoint;
			std::cout << "Enter x2: ";
            std::cin >> path_x2_setpoint;
			std::cout << "Enter y2: ";
            std::cin >> path_y2_setpoint;
			std::cout << "Enter z2: ";
            std::cin >> path_z2_setpoint;
            ROS_WARN_STREAM("path_x_setpoint: " << path_x_setpoint);
            ROS_WARN_STREAM("path_y_setpoint: " << path_y_setpoint);
            ROS_WARN_STREAM("path_z_setpoint: " << path_z_setpoint);
            ROS_WARN_STREAM("path_x2_setpoint: " << path_x2_setpoint);
            ROS_WARN_STREAM("path_y2_setpoint: " << path_y2_setpoint);
            ROS_WARN_STREAM("path_z2_setpoint: " << path_z2_setpoint);
            callPath(path_x_setpoint, path_y_setpoint, path_z_setpoint, path_x2_setpoint, path_y2_setpoint, path_z2_setpoint);
	}
	else if(what_to_run == "shooter")
	{
		callShooter();
	}
	else if(what_to_run == "shooter")
	{
		callShooter();
	}
	else if(what_to_run == "indexer")
	{
		std::cout << "Enter indexer action type (0-position intake, 1-intake a ball, or 2-feed a ball to shooter):";
		std::cin >> indexer_action;
		callIndexer(indexer_action);
	}
	else if(what_to_run == "shooter")
	{
		callShooter();
	}
	else {
		ROS_ERROR("Invalid run argument");
		return 0;
	}

	//exit
	return 0;
}
