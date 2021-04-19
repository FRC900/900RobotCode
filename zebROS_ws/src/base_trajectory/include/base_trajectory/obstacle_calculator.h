// Class to calculate path cost from map obstacles
// There are two basic steps. The first, run at the start of the
// path generation, takes a snapshot of the costmap and generates
// a set of potential values, one per costmap grid square. These potential
// values are higher the further into the interior of an obstacle
// we go. This gives a hopefully smooth gradient to optimize to
// move the path out of the obstacle and back into a valid location
// The other step is run once per path generation to calculate the cost from
// obstacles for that path. The cost is simply the sum of the
// potential value from the first step for each map grid square hit
// by the path
#ifndef OBSTACLE_CALCULATOR_INC_
#define OBSTACLE_CALCULATOR_INC_

#include <memory>
#include <vector>
#include <costmap_2d/costmap_2d_ros.h>
//#include "base_trajectory/thread_pool.h"
#include "base_trajectory/obstacle_cost_impl_cpu.h"
#include "base_trajectory/obstacle_gradient_impl.h"
#include "spline_util/spline_util.h"

template <class T>
class ObstacleCalculator
{
public:
	ObstacleCalculator(const std::shared_ptr<costmap_2d::Costmap2DROS> &costmap, uint8_t threashold, const bool useCuda = false); // TODO - maybe mode?
	void calculatePotentials();
	void calculateCosts(std::vector<T> &arcSegCosts,
						const geometry_msgs::TransformStamped &pathToMapTransform,
						const std::vector<SegmentState<T>> &xStates,
						const std::vector<SegmentState<T>> &yStates);

	void waitForCompletion();
	int getThreshold(void) const;
	void setThreshold(int threshold);

private:
	void calculatePotentialsWorker();
	void calculateCostsWorker(std::vector<T> &arcSegCosts,
							  const geometry_msgs::TransformStamped &pathToMapTransform,
							  const std::vector<SegmentState<T>> &xStates,
							  const std::vector<SegmentState<T>> &yStates);
	//std::unique_ptr<ObstacleCostImpl<T>> obstacleCostImpl_;
	ObstacleCostImplCpu<T> obstacleCostImpl_;
	std::unique_ptr<ObstacleGradientImpl> obstacleGradientImpl_;
	std::vector<unsigned int> potentials_;
	std::shared_ptr<costmap_2d::Costmap2DROS> costmap_;
	//ThreadPool threadPool_;
};

#endif
