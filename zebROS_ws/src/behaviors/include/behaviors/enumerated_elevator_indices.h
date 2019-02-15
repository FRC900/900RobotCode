#pragma once

enum ElevatorLocation
{
	CARGO_SHIP,
	ROCKET_1,
	ROCKET_2,
	ROCKET_3,
	INTAKE,
	/*everything index 5 and higher, excluding ELEVATOR_MAX_INDEX, is used for climbing.*/
	ELEVATOR_DEPLOY, //height at which we can engage climber with elevator
	ELEVATOR_CLIMB, //height to make robot rise
	ELEVATOR_CLIMB_LOW, //height to boost back of robot up once on platform
	ELEVATOR_MAX_INDEX, //last index in list
};
