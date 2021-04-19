#include "base_trajectory/cuda_mem_utils.cuh"
#include "base_trajectory/cuda_spline_utils.cuh"
#include "base_trajectory/cuda_utils.h"

// TODO - not sure if this belongs in this file?
template <class T, size_t O>
bool hostToDeviceTrajectory(CudaT *&host,   size_t& hostCapacity,
							CudaT *&device, size_t& deviceCapacity,
							size_t &len,
							const TrajectoryPerJoint<T, O> &t)
{
    bool rc;
	rc  = reallocHostPinnedMemory(host, hostCapacity, sizeof(CudaT) * t.size() * (O + 3));
	rc |= reallocDeviceMemory(device, deviceCapacity, sizeof(CudaT) * t.size() * (O + 3));
	len = 0;
	for (const auto &seg : t)
	{
		host[len++] = seg.startTime();
		host[len++] = seg.endTime();
		const auto coeffs = seg.getCoefs();
		for (const auto c : coeffs)
		{
			host[len++] = c;
		}
	}
	return rc;
}

template bool hostToDeviceTrajectory(CudaT *&host,   size_t& hostCapacity,
							CudaT *&device, size_t& deviceCapacity,
							size_t &len,
							const TrajectoryPerJoint<double, 1> &t);
template bool hostToDeviceTrajectory(CudaT *&host,   size_t& hostCapacity,
							CudaT *&device, size_t& deviceCapacity,
							size_t &len,
							const TrajectoryPerJoint<double, 3> &t);
template bool hostToDeviceTrajectory(CudaT *&host,   size_t& hostCapacity,
							CudaT *&device, size_t& deviceCapacity,
							size_t &len,
							const TrajectoryPerJoint<double, 5> &t);
template bool hostToDeviceTrajectory(CudaT *&host,   size_t& hostCapacity,
							CudaT *&device, size_t& deviceCapacity,
							size_t &len,
							const TrajectoryPerJoint<float, 1> &t);
template bool hostToDeviceTrajectory(CudaT *&host,   size_t& hostCapacity,
							CudaT *&device, size_t& deviceCapacity,
							size_t &len,
							const TrajectoryPerJoint<float, 3> &t);
template bool hostToDeviceTrajectory(CudaT *&host,   size_t& hostCapacity,
							CudaT *&device, size_t& deviceCapacity,
							size_t &len,
							const TrajectoryPerJoint<float, 5> &t);

// TODO - not sure if this belongs in this file?
// Helper function for copying from CUDA spline states back to ROS-formatted version
template <class T>
void appendState(SegmentState<T> &state, const SplineState<CudaT> &s)
{
	state.position = s.position;
	state.velocity = s.velocity;
	state.acceleration = s.acceleration;
}

//template void appendState<float>(SegmentState<float> &state, const SplineState<CudaT> &s);
template void appendState<double>(SegmentState<double> &state, const SplineState<CudaT> &s);
