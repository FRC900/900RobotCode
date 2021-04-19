#ifndef INC_OBSTACLE_COST_IMPL_CUDA_
#define INC_OBSTACLE_COST_IMPL_CUDA_

#include "cuda_runtime.h"
#include "base_trajectory/obstacle_cost_impl.h"
#include "base_trajectory/cudat_def.cuh"

struct XYCoord;
template <class T>
class ObstacleCostImplCuda : public ObstacleCostImpl<T>
{
	public:
		ObstacleCostImplCuda();
		~ObstacleCostImplCuda();
		bool calculate(
					std::vector<T> &arcSegCosts,
					std::shared_ptr<costmap_2d::Costmap2DROS> costmap,
					const geometry_msgs::TransformStamped &pathToMapTransform,
					const std::vector<unsigned int> &potentials,
					const std::vector<SegmentState<T>> &xStates,
					const std::vector<SegmentState<T>> &yStates) override;
	private:
		CudaT *hArcSegCosts_{nullptr};
		size_t hArcSegCostsCapacity_{0};
		CudaT *dArcSegCosts_{nullptr};
		size_t dArcSegCostsCapacity_{0};

		XYCoord *hXYCoords_{nullptr};
		size_t   hXYCoordsCapacity_{0};
		XYCoord *dXYCoords_{nullptr};
		size_t   dXYCoordsCapacity_{0};

		unsigned int *dPotentials_;
		size_t        dPotentialsCapacity_;

		cudaStream_t cudaStream_;
};

#endif
