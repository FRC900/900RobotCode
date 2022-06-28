#include <cmath>
#include "talon_swerve_drive_controller/900Math.h"

double leastDistantAngleWithinHalfPi(double currentAngle, double targetAngle, bool &reverse)
{
	//returns the closest angle to the current angle = to x*.5*M_PI + target angle where x is any integer
	//used for turning wheels to the target angle (swerve)
	const double normalizedDiff = normalizeAngle(targetAngle) - normalizeAngle(currentAngle);

	const double withinPi = (fabs(normalizedDiff) < M_PI) ? normalizedDiff : (normalizedDiff - copysign(2. * M_PI, normalizedDiff));
	double withinHalfPi;

	if (fabs(withinPi) < (M_PI / 2.))
	{
		withinHalfPi = withinPi;
		reverse = false;
	}
	else
	{
		withinHalfPi = (withinPi - copysign(M_PI, withinPi));
		reverse = true;
	}
	return withinHalfPi + currentAngle;
}

double leastDistantAngleWithinPi(double currentAngle, double targetAngle)
{
	const double normalizedDiff = normalizeAngle(targetAngle) - normalizeAngle(currentAngle);
	const double withinPi = (fabs(normalizedDiff) < M_PI) ? normalizedDiff : (normalizedDiff - copysign(2. * M_PI, normalizedDiff));
	return withinPi + currentAngle;
}

double normalizeAngle(double angle) //normalizes between -M_PI and M_PI
{
	return angle - floor((angle + M_PI) / (2. * M_PI)) * 2. * M_PI;
}

