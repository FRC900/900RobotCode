// Container for storing and modifying path generation optimization parameters
#pragma once
#include <iostream>

// Optimization parameters - these are deltas added to
// the original guess for the spline generation used
// to improve the overall cost of following the spline
template <class T>
struct OptParams
{
	T posX_{initialPosX_}; // offset from requested waypoint in x and y
	T minOffX_{0};
	T maxOffX_{0};

	T posY_{initialPosY_};
	T minOffY_{0};
	T maxOffY_{0};

	T length_{initialLength_};       // These control the curveature vs. velocity at waypoints
	T minLength_{0.2};
	T maxLength_{2};

	T lengthScale_{initialLengthScale_};
	T minLengthScale_{0};
	T maxLengthScale_{1.5};

	T rotLength_{initialRotLength_};
	T minRotLength_{0};
	T maxRotLength_{1.5};

	T rotLengthScale_{initialRotLengthScale_};
	T minRotLengthScale_{0};
	T maxRotLengthScale_{1.5};

	T deltaVMagnitude_{initialDeltaVMagnitude_}; // Last waypoint velocity vector
	T minDeltaVMagnitude_{0};
	T maxDeltaVMagnitude_{0};

	T deltaVDirection_{initialDeltaVDirection_};
	T minDeltaVDirection_{0};
	T maxDeltaVDirection_{0};

	OptParams() = default;
	OptParams(T minOffX, T maxOffX, T minOffY, T maxOffY);

	T &operator[] (size_t index);
	const T& operator[] (size_t index) const;

	void setDeltaV(T minMagnitude, T maxMagnitude, T minDirection, T maxDirection);

	void clearLengthLimits(void);
	void clearRotationLengthLimits(void);

	bool isAtMax(size_t index) const;
	bool doNotOptimize(size_t index) const;

	T getLowerLimit(size_t index) const;
	T getUpperLimit(size_t index) const;

	bool incrementVariable(size_t index, T value);

	// Bound dparam values to actual limits for the given variable
	T initialDParamGuess(size_t index, T value) const;

	// Syntax to let callers use the values in this
	// object as if they were an array. Since the optimizer
	// loops over all member vars, this turns the code
	// in there into a simple for() loop
	size_t size(void) const;

	T regularizationCost(T lambda) const;

private:

	bool incrementHelper(T &var, T value, T minOff, T maxOff);
	T initialDParamGuessHelper(size_t index, T var, T value, T minOff, T maxOff) const;
	T initialDParamMultiplier(size_t index) const;
	template <class X>
	friend std::ostream& operator<< (std::ostream& stream, const OptParams<X> &optParams);
	static constexpr T initialPosX_{0}; // offset from requested waypoint in x and y
	static constexpr T initialPosY_{0};
	static constexpr T initialLength_{0.2};       // These control the curveature vs. velocity at waypoints
	static constexpr T initialLengthScale_{0.75};
	static constexpr T initialRotLength_{0};
	static constexpr T initialRotLengthScale_{0.75};
	static constexpr T initialDeltaVMagnitude_{0}; // Last waypoint velocity vector
	static constexpr T initialDeltaVDirection_{0};
};

template <class T>
std::ostream& operator<< (std::ostream& stream, const OptParams<T> &optParams);

