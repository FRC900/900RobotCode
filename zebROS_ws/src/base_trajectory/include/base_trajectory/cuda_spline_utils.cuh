#ifndef CUDA_SPLINE_UTILS_INC_
#define CUDA_SPLINE_UTILS_INC_

#include <cstddef>
#include <type_traits>

#include "base_trajectory/cudat_def.cuh"

// Splines are represented by the start and end time for each of
// their segments plus the coefficients describing the equation in
// that range of time.  On the CUDA side, this is flattened into a
// single 1D array.  These constants are used to extract individual
// spline seg's fields from that flat array.
//constexpr size_t SPLINE_NUM_COEFS = 6; // Actually, order+1
//constexpr size_t SPLINE_VALUES_PER_SEG = SPLINE_NUM_COEFS + 2;
constexpr size_t SPLINE_VALUES_STARTTIME_OFFSET = 0;
constexpr size_t SPLINE_VALUES_ENDTIME_OFFSET = 1;
constexpr size_t SPLINE_VALUES_COEFF_OFFSET = 2;

// Class to encapsulate a vector - data array plus length - in
// a way which can be used in device code.
template <typename T>
struct CudaSpan
{
	CudaSpan(const T *array, const size_t size)
		: array_(array)
	    , size_(size)
	{
	}
    const T*     array_;
    const size_t size_;
};

template <class T>
struct SplineState
{
	T position;
	T velocity;
	T acceleration;
};

// For storing 3-d state
// TODO - maybe make dimensionaly a template param?
template <class T>
struct ThreeDState
{
	SplineState<T> x_;
	SplineState<T> y_;
	SplineState<T> t_;

};

// Grab the end time array entry for the last set of spline coeffs
template <class T, size_t O>
__device__
T trajectoryBackEndTime(const T *t, const size_t size)
{
	return t[size - 1 - (O + SPLINE_VALUES_ENDTIME_OFFSET)];
}

// Calculate the SplineState for a given spline segment at the specific time.
template <size_t O>
__device__
void sampleSplineSeg(const CudaSpan<CudaT> &trajectory, size_t index, float time, SplineState<CudaT> &state)
{
	time -= trajectory.array_[index * (O + 3) + SPLINE_VALUES_STARTTIME_OFFSET];
	const auto *coefficients = &trajectory.array_[index * (O + 3) + SPLINE_VALUES_COEFF_OFFSET];

	state.position = ((((time * coefficients[5] + coefficients[4]) *
				   time + coefficients[3]) *
				   time + coefficients[2]) *
				   time + coefficients[1]) *
				   time + coefficients[0];

	state.velocity = (((time * 5.0f * coefficients[5] + 4.0f * coefficients[4]) *
				  time + 3.0f * coefficients[3]) *
				  time + 2.0f * coefficients[2]) *
				  time + coefficients[1];

	state.acceleration = ((time * 20.0f * coefficients[5] + 12.0f * coefficients[4]) *
				   time + 6.0f * coefficients[3]) *
				   time + 2.0f * coefficients[2];
}

#if 0
// Search for a segment represnting the time specificed, evaluate the
// spline seg at that time
// Returns the SplineState - position, velocity and accel - of the spline
// at that time
template <size_t O>
__device__
void sampleSpline(const CudaSpan<CudaT> &trajectory, float time, SplineState<CudaT> &state)
{
	// Sampling before spline starts, grab first valid time
	if (time < trajectory.array_[0])
		return sampleSplineSeg<O>(trajectory, 0, 0.0, state);

	// Loop through the spline values array, looking for segment
	// where time falls between the start and end time for that seg, inclusive
	for (size_t i = 0; i < trajectory.size_; i += (O + 3))
	{
		if ((time >= trajectory.array_[i + SPLINE_VALUES_STARTTIME_OFFSET]) &&
			(time <= trajectory.array_[i + SPLINE_VALUES_ENDTIME_OFFSET]))
		{
			return sampleSplineSeg<O>(trajectory, i / (O + 3), time, state);
		}
	}
	// After the spline ends, return the last point
	sampleSplineSeg<O>(trajectory, trajectory.size_ / (O + 3) - 1, trajectoryBackEndTime<CudaT, O>(trajectory), state);
}
#endif

// Calculate the SplineState for a given spline segment at the specific time.
template <size_t O>
__device__ typename std::enable_if<O == 1, void>::type
sampleSplineSegPosition(const CudaT *trajectory, size_t index, float time, CudaT &position)
{
	time -= trajectory[index * (O + 3) + SPLINE_VALUES_STARTTIME_OFFSET];
	const auto *coefficients = &trajectory[index * (O + 3) + SPLINE_VALUES_COEFF_OFFSET];

	position = coefficients[0] + time*coefficients[1];
}

template <size_t O>
__device__ typename std::enable_if<O == 3, void>::type
sampleSplineSegPosition(const CudaT *trajectory, size_t index, float time, CudaT &position)
{
	time -= trajectory[index * (O + 3) + SPLINE_VALUES_STARTTIME_OFFSET];
	const auto *coefficients = &trajectory[index * (O + 3) + SPLINE_VALUES_COEFF_OFFSET];

	position = ((coefficients[3] * time + coefficients[2]) * time + coefficients[1]) * time + coefficients[0];
}

// Search for a segment represnting the time specificed, evaluate the
// spline seg at that time
// Returns the SplineState - position, velocity and accel - of the spline
// at that time
template <size_t O>
__device__
void sampleSplinePosition(const CudaT * const trajectory, const size_t trajectoryLength, const float time, CudaT &position)
{
	// Sampling before spline starts, grab first valid time
	if (time < trajectory[0])
	{
		sampleSplineSegPosition<O>(trajectory, 0, 0.0, position);
		return;
	}

	// Loop through the spline values array, looking for segment
	// where time falls between the start and end time for that seg, inclusive
	// TODO - binary search?
	for (size_t i = 0; i < trajectoryLength; i += (O + 3))
	{
		if ((time >= trajectory[i + SPLINE_VALUES_STARTTIME_OFFSET]) &&
			(time <= trajectory[i + SPLINE_VALUES_ENDTIME_OFFSET]))
		{
			sampleSplineSegPosition<O>(trajectory, i / (O + 3), time, position);
			return;
		}
	}
	// After the spline ends, return the last point
	sampleSplineSegPosition<O>(trajectory, trajectoryLength / (O + 3) - 1, trajectoryBackEndTime<CudaT, O>(trajectory, trajectoryLength), position);
}

// Search for a segment represnting the time specificed, evaluate the
// spline seg at that time
// Returns the SplineState - position, velocity and accel - of the spline
// at that time
template <size_t O>
__device__
void sampleSplinePosition(const CudaSpan<CudaT> &trajectory, const float time, CudaT &position)
{
	sampleSplinePosition<O>(trajectory.array_, trajectory.size_, time, position);
}



template <class T>
__device__
size_t arcLengthTimeToSegTime(T time, T endTime)
{
	// trajectory segments are still arbitrary times, each 1 unit long.
	// Thus, times 0 .. 0.99999 will be in segment 0, 1 .. 1.99999 in seg 1, and so on
	// The exception is the last time - the end time is at the end of segment t - 1
	return (time == endTime) ? static_cast<size_t>(endTime) - 1 : floorf(time);
}


#include "spline_util/spline_util.h"
template <class T, size_t O>
bool hostToDeviceTrajectory(CudaT *&host,   size_t& hostCapacity,
							CudaT *&device, size_t& deviceCapacity,
							size_t &len,
							const TrajectoryPerJoint<T, O> &t);

// Helper function for copying from CUDA spline states back to ROS-formatted version
template <class T>
void appendState(SegmentState<T> &state, const SplineState<CudaT> &s);
#endif
