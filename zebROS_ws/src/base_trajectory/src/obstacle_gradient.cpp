#include <chrono>
#include <ros/console.h>

#include "base_trajectory/obstacle_gradient.h"
#include "base_trajectory/obstacle_gradient_impl_cpu.h"
#include "base_trajectory/obstacle_gradient_impl_cuda.h"
#include "base_trajectory/obstacle_gradient_impl_threaded.h"

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

// Construct an obstacle gradient calculator with a given x and y grid size
ObstacleGradient::ObstacleGradient(int xs, int ys, uint8_t threshold, ObstacleGradientImplType implType)
{
    const auto t1 = high_resolution_clock::now();
	if (implType == ObstacleGradientImplType::CPU)
		impl_ = std::make_unique<ObstacleGradientImplCpu>(xs, ys, threshold);
	else if (implType == ObstacleGradientImplType::THREADED)
		impl_ = std::make_unique<ObstacleGradientImplThreaded>(xs, ys, threshold);
	else if (implType == ObstacleGradientImplType::CUDA)
		impl_ = std::make_unique<ObstacleGradientImplCuda>(xs, ys, threshold);
    const auto t2 = high_resolution_clock::now();
    const duration<double, std::milli> ms_double = t2 - t1;
    ROS_INFO_STREAM("ObstacleGradient() took " << ms_double.count() << " mSec");
}
ObstacleGradient::~ObstacleGradient() = default;

void ObstacleGradient::calculatePotentials(const unsigned char *costs, unsigned int *potential)
{
    //const auto t1 = high_resolution_clock::now();
	impl_->calculatePotentials(costs, potential);
    //const auto t2 = high_resolution_clock::now();
	/* Getting number of milliseconds as a double. */
    //const duration<double, std::milli> ms_double = t2 - t1;

    //ROS_INFO_STREAM("calculatePotentials() took " << ms_double.count() << " mSec");
}

void ObstacleGradient::setSize(int nx, int ny)
{
    const auto t1 = high_resolution_clock::now();
	impl_->setSize(nx, ny);
    const auto t2 = high_resolution_clock::now();
    const duration<double, std::milli> ms_double = t2 - t1;
    ROS_INFO_STREAM("setSize() took " << ms_double.count() << " mSec");
}

void ObstacleGradient::setThreshold(uint8_t threshold)
{
	impl_->setThreshold(threshold);
}

uint8_t ObstacleGradient::getThreshold(void) const
{
	return impl_->getThreshold();
}
