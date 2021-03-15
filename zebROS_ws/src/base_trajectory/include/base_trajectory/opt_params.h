// Container for storing and modifying path generation optimization parameters
#pragma once
#include <iostream>

// Optimization parameters - these are deltas added to
// the original guess for the spline generation used
// to improve the overall cost of following the spline
struct OptParams
{
	double posX_{0}; // offset from requested waypoint in x and y
	double posY_{0};
	double minOffX_{0};
	double maxOffX_{0};
	double minOffY_{0};
	double maxOffY_{0};
	double length_{0};       // These control the curveature vs. velocity at waypoints
	double minLength_{-std::numeric_limits<double>::max()};
	double maxLength_{std::numeric_limits<double>::max()};
	double lengthScale_{0.75};
	double minLengthScale_{-std::numeric_limits<double>::max()};
	double maxLengthScale_{std::numeric_limits<double>::max()};
	double minRotLength_{-std::numeric_limits<double>::max()};
	double maxRotLength_{std::numeric_limits<double>::max()};
	double rotLength_{0};
	double rotLengthScale_{0.75};
	double minRotLengthScale_{-std::numeric_limits<double>::max()};
	double maxRotLengthScale_{std::numeric_limits<double>::max()};
	double deltaVMagnitude_{0}; // Last waypoint velocity vector
	double deltaVDirection_{0};
	double minDeltaVMagnitude_{0};
	double maxDeltaVMagnitude_{0};
	double minDeltaVDirection_{0};
	double maxDeltaVDirection_{0};

	OptParams() = default;
	OptParams(double minOffX, double maxOffX, double minOffY, double maxOffY);

	void setDeltaV(double minMagnitude, double maxMagnitude, double minDirection, double maxDirection);

	void clearLengthLimits(void);

	bool IsAtMax(size_t index) const;

	bool IncrementVariable(size_t index, double value);

	// Syntax to let callers use the values in this
	// object as if they were an array. Since the optimizer
	// loops over all member vars, this turns the code
	// in there into a simple for() loop
	size_t size(void) const;

private:

	bool IncrementHelper(double &var, double value, double minOff, double maxOff);
	friend std::ostream& operator<< (std::ostream& stream, const OptParams &optParams);
};

std::ostream& operator<< (std::ostream& stream, const OptParams &optParams);

