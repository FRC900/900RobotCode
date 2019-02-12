#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <behaviors/IntakeAction.h>
#include <behaviors/PlaceAction.h>
#include <behaviors/ElevatorAction.h>
#include <behaviors/ClimbAction.h>
#include <behaviors/enumerated_elevator_indices.h>

int main (int argc, char **argv)
{
	ros::init(argc, argv, "test_actionlib");

	bool finished_before_timeout;

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<behaviors::IntakeAction> intake_cargo_ac("/cargo_intake/cargo_intake_server", true);
	actionlib::SimpleActionClient<behaviors::IntakeAction> intake_hatch_panel_ac("/hatch_intake/intake_hatch_panel_server", true);
	actionlib::SimpleActionClient<behaviors::PlaceAction> outtake_cargo_ac("/cargo_outtake/cargo_outtake_server", true);
	actionlib::SimpleActionClient<behaviors::PlaceAction> outtake_hatch_panel_ac("/hatch_outtake/outtake_hatch_panel_server", true);
	actionlib::SimpleActionClient<behaviors::ElevatorAction> elevator_ac("/elevator/elevator_server", true);

	ROS_INFO("Waiting for cargo intake server to start.");
	intake_cargo_ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Waiting for panel intake server to start.");
	intake_hatch_panel_ac.waitForServer();

	ROS_INFO("Waiting for cargo outtake server to start.");
	outtake_cargo_ac.waitForServer();

	ROS_INFO("Waiting for panel outtake to start.");
	outtake_hatch_panel_ac.waitForServer();

	for( int i = 0; i < 5; i++)
	{
		ROS_INFO("Sending goal to elevator server, setpoint = %d", i);
		// send a goal to the action
		behaviors::ElevatorGoal goal;
		goal.setpoint_index = i;
		elevator_ac.sendGoal(goal);

		//wait for the action to return
		finished_before_timeout = elevator_ac.waitForResult(ros::Duration(30.0));

		if (finished_before_timeout)
		{
			actionlib::SimpleClientGoalState state = elevator_ac.getState();
			ROS_INFO("Action finished: %s",state.toString().c_str());
		}
		else
		{
			ROS_INFO("Action did not finish before the time out.");

		}
	}

	ROS_INFO("Sending goal to intake hatch panel server.");
	// send a goal to the action
	behaviors::IntakeGoal intake_hatch_panel_goal;
	intake_hatch_panel_goal.motor_power = 0;
	intake_hatch_panel_ac.sendGoal(intake_hatch_panel_goal);

	//wait for the action to return
	finished_before_timeout = intake_hatch_panel_ac.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = intake_hatch_panel_ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
	}

	ROS_INFO("Sending goal to outtake hatch panel server.");
	// send a goal to the action
	behaviors::PlaceGoal outtake_hatch_panel_goal;
	outtake_hatch_panel_goal.setpoint_index = CARGO_SHIP;
	outtake_hatch_panel_ac.sendGoal(outtake_hatch_panel_goal);

	//wait for the action to return
	finished_before_timeout = outtake_hatch_panel_ac.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = outtake_hatch_panel_ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
	}

	ROS_INFO("Sending goal to intake cargo server.");
	// send a goal to the action
	behaviors::IntakeGoal intake_cargo_goal;
	intake_cargo_goal.motor_power = 1;
	intake_cargo_ac.sendGoal(intake_cargo_goal);

	//wait for the action to return
	finished_before_timeout = intake_cargo_ac.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = intake_cargo_ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
	}

	ROS_INFO("Sending goal to outtake cargo server.");
	// send a goal to the action
	behaviors::PlaceGoal outtake_cargo_goal;
	outtake_cargo_goal.setpoint_index = CARGO_SHIP;
	outtake_cargo_ac.sendGoal(outtake_cargo_goal);

	//wait for the action to return
	finished_before_timeout = outtake_cargo_ac.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = outtake_cargo_ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
	}

	//exit
	return 0;
}
