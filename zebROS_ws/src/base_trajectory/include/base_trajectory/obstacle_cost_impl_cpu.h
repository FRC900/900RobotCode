#ifndef INC_OBSTACLE_COST_IMPL_CPU_
#define INC_OBSTACLE_COST_IMPL_CPU_

#include "base_trajectory/obstacle_cost_impl.h"

template <class T>
class ObstacleCostImplCpu : public ObstacleCostImpl<T>
{
	public:
		ObstacleCostImplCpu() = default;
		bool calculate(
					std::vector<T> &arcSegCosts,
					std::shared_ptr<costmap_2d::Costmap2DROS> costmap,
					const geometry_msgs::TransformStamped &pathToMapTransform,
					const std::vector<unsigned int> &potentials,
					const std::vector<SegmentState<T>> &xStates,
					const std::vector<SegmentState<T>> &yStates) override;

};

#endif
