#ifndef INC_OBSTACLE_GRADIENT_IMPL_CPU_
#define INC_OBSTACLE_GRADIENT_IMPL_CPU_

#include <vector>

#include "base_trajectory/obstacle_gradient_impl.h"
class ObstacleGradientImplCpu : public ObstacleGradientImpl
{
	public:
		ObstacleGradientImplCpu(int nx, int ny, uint8_t threshold);
		void calculatePotentials(const unsigned char *costs, unsigned int *potentials) override;
		void setSize(int nx, int ny) override;

	private:
		std::vector<unsigned int> h_potentials_;
		std::vector<unsigned int> v_potentials_;
};

#endif
