#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <behaviors/IntakeAction.h>
#include <behaviors/PlaceAction.h>
#include <behaviors/ElevatorAction.h>
#include <behaviors/ClimbAction.h>

int main (int argc, char **argv)
{
	ros::init(argc, argv, "test_actionlib");

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<behaviors::IntakeAction> intake_cargo_ac("cargo_intake_server", true);
	actionlib::SimpleActionClient<behaviors::IntakeAction> intake_hatch_panel_ac("intake_hatch_panel_server", true);
	actionlib::SimpleActionClient<behaviors::PlaceAction> outtake_cargo_ac("cargo_outtake_server", true);
	actionlib::SimpleActionClient<behaviors::PlaceAction> outtake_hatch_panel_ac("outtake_hatch_panel_server", true);

	ROS_INFO("Waiting for cargo intake server to start.");
	intake_cargo_ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Waiting for panel intake server to start.");
	intake_hatch_panel_ac.waitForServer();

	ROS_INFO("Waiting for cargo outtake server to start.");
	outtake_cargo_ac.waitForServer();

	ROS_INFO("Waiting for panel outtake to start.");
	outtake_hatch_panel_ac.waitForServer();

	ROS_INFO("Action servers started, sending goal.");
	// send a goal to the action
	behaviors::IntakeGoal goal;
	goal.motor_power = 0;
	intake_hatch_panel_ac.sendGoal(goal);
/*
	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
		ROS_INFO("Action did not finish before the time out.");

	//exit*/
	return 0;
}
