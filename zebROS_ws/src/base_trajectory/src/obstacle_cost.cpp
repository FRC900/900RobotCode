#include "base_trajectory/obstacle_cost.h"
#include "base_trajectory/obstacle_cost_impl_cpu.h"
#include "base_trajectory/obstacle_cost_impl_cuda.h"

#if 0
#include <chrono>
#include <ros/console.h>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;
#endif

// Construct an obstacle gradient calculator with a given x and y grid size
template<class T>
ObstacleCost<T>::ObstacleCost(const ObstacleCostImplType type)
{
    //const auto t1 = high_resolution_clock::now();
	if (type == ObstacleCostImplType::CPU)
		impl_ = std::make_unique<ObstacleCostImplCpu<T>>();
	else if (type == ObstacleCostImplType::THREADED) // TODO - write me?
		impl_ = std::make_unique<ObstacleCostImplCpu<T>>();
	else if (type == ObstacleCostImplType::CUDA)
		impl_ = std::make_unique<ObstacleCostImplCuda<T>>();
    //const auto t2 = high_resolution_clock::now();
    //const duration<double, std::milli> ms_double = t2 - t1;
    //ROS_INFO_STREAM("ObstacleCost() took " << ms_double.count() << " mSec");
}

template<class T>
ObstacleCost<T>::~ObstacleCost() = default;

template<class T>
bool ObstacleCost<T>::calculate(
				std::vector<T> &arcSegCosts,
				std::shared_ptr<costmap_2d::Costmap2DROS> costmap,
				const geometry_msgs::TransformStamped &pathToMapTransform,
				const std::vector<unsigned int> &potentials,
				const std::vector<SegmentState<T>> &xStates,
				const std::vector<SegmentState<T>> &yStates)
{
    //const auto t1 = high_resolution_clock::now();
	auto rc = impl_->calculate(arcSegCosts, costmap, pathToMapTransform, potentials, xStates, yStates);
    //const auto t2 = high_resolution_clock::now();
	  /* Getting number of milliseconds as a double. */
    //const duration<double, std::milli> ms_double = t2 - t1;

    //ROS_INFO_STREAM("ObstacleCost::calculate() took " << ms_double.count() << " mSec");
	return rc;
}

template class ObstacleCost<double>;
//template class ObstacleCost<float>;
