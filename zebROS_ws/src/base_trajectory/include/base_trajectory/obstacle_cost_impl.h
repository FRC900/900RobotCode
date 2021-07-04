#ifndef INC_OBSTACLE_COST_IMPL_
#define INC_OBSTACLE_COST_IMPL_

#include <memory>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/TransformStamped.h>
#include "spline_util/spline_util.h"

template <class T>
class ObstacleCostImpl
{
	public:
		ObstacleCostImpl(void) = default;
		virtual ~ObstacleCostImpl() = default;
		virtual bool calculate(std::vector<T> &arcSegCosts,
							   std::shared_ptr<costmap_2d::Costmap2DROS> costmap,
							   const geometry_msgs::TransformStamped &pathToMapTransform,
							   const std::vector<unsigned int> &potentials,
							   const std::vector<SegmentState<T>> &xStates,
							   const std::vector<SegmentState<T>> &yStates) = 0;

};

#endif
