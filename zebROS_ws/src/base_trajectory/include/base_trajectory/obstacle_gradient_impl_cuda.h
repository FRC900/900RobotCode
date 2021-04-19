#ifndef INC_OBSTACLE_GRADIENT_IMPL_CUDA_
#define INC_OBSTACLE_GRADIENT_IMPL_CUDA_

#include "cuda_runtime.h"
#include "base_trajectory/obstacle_gradient_impl.h"

class ObstacleGradientImplCuda : public ObstacleGradientImpl
{
	public:
		ObstacleGradientImplCuda(int nx, int ny, uint8_t threshold);
		~ObstacleGradientImplCuda();
		void calculatePotentials(const unsigned char *costs, unsigned int *potential) override;
		void setSize(int nx, int ny) override;
	private:
		void freeDeviceMemory(void);

		cudaStream_t    cudaStreamH_;
		cudaStream_t    cudaStreamV_;
		cudaStream_t    cudaStreamForGraph_;
		cudaEvent_t     cudaEventInputReady_;
		cudaEvent_t     cudaEventHKernel_;
		cudaEvent_t     cudaEventVKernel_;
		unsigned int   *deviceHPotentials_{nullptr};
		unsigned int   *deviceVPotentials_{nullptr};
		unsigned char  *deviceCosts_{nullptr};
		cudaGraph_t     cudaGraph_;
		cudaGraphExec_t cudaGraphExec_;
		bool            recordCudaGraph_{true};
};

#endif
