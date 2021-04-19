#ifndef INC_OBSTACLE_COST_
#define INC_OBSTACLE_COST_

#include <memory>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/TransformStamped.h>
#include "spline_util/spline_util.h"

template <class T> class ObstacleCostImpl;
enum class ObstacleCostImplType
{
	CPU,
	THREADED,
	CUDA
};
template <class T>
class ObstacleCost
{
	public:
		ObstacleCost(const ObstacleCostImplType type);
		~ObstacleCost();
		bool calculate(
				std::vector<T> &arcSegCosts,
				std::shared_ptr<costmap_2d::Costmap2DROS> costmap,
				const geometry_msgs::TransformStamped &pathToMapTransform,
				const std::vector<unsigned int> &potentials,
				const std::vector<SegmentState<T>> &xStates,
				const std::vector<SegmentState<T>> &yStates);

	private:
		std::unique_ptr<ObstacleCostImpl<T>> impl_;
};

#endif
