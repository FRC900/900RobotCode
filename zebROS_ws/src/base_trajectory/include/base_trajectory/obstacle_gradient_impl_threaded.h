#ifndef INC_OBSTACLE_GRADIENT_IMPL_THREADED_
#define INC_OBSTACLE_GRADIENT_IMPL_THREADED_

#include <vector>

#include "base_trajectory/obstacle_gradient_impl.h"
#include "base_trajectory/thread_pool.h"
class ObstacleGradientImplThreaded : public ObstacleGradientImpl
{
	public:
		ObstacleGradientImplThreaded(int nx, int ny, uint8_t threshold);
		void calculatePotentials(const unsigned char *costs, unsigned int *potentials) override;
		void setSize(int nx, int ny) override;

	private:
		void calculatePotentialsHorizontalForward(const unsigned char *costs);
		void calculatePotentialsHorizontalBackward(const unsigned char *costs);
		void calculatePotentialsVerticalForward(const unsigned char *costs);
		void calculatePotentialsVerticalBackward(const unsigned char *costs);
		void finalSum(unsigned int *potentials, const unsigned char *costs, const size_t startIdx, const size_t len) const;
		std::vector<unsigned int> h_potentials_forward_;
		std::vector<unsigned int> h_potentials_backward_;
		std::vector<unsigned int> v_potentials_forward_;
		std::vector<unsigned int> v_potentials_backward_;
		ThreadPool threadPool;
};

#endif
