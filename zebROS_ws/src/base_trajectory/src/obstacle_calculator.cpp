#include "base_trajectory/obstacle_calculator.h"
#include "base_trajectory/obstacle_cost_impl_cpu.h"
#include "base_trajectory/obstacle_gradient_impl_cuda.h"
#include "base_trajectory/obstacle_gradient_impl_threaded.h"

//#define THREADED_POTENTIALS
//#define THREADED_COST
template <class T>
ObstacleCalculator<T>::ObstacleCalculator(const std::shared_ptr<costmap_2d::Costmap2DROS> &costmap, uint8_t threshold, const bool useCuda)
	: costmap_{costmap}
#ifdef THREADED_POTENTIALS
	, threadPool_{1} // Really just need a single background thread
#endif
{
	if (!costmap_)
		return;
	const auto &cm = costmap_->getCostmap();
	if (useCuda)
		obstacleGradientImpl_ = std::make_unique<ObstacleGradientImplCuda>(cm->getSizeInCellsX(),cm->getSizeInCellsY(), threshold);
	else
		obstacleGradientImpl_ = std::make_unique<ObstacleGradientImplThreaded>(cm->getSizeInCellsX(),cm->getSizeInCellsY(), threshold);
}

template <class T>
void ObstacleCalculator<T>::calculatePotentials()
{
	if (!costmap_)
		return;
	//waitForCompletion();
#ifdef THREADED_POTENTIALS
	threadPool_.enqueue(std::bind(&ObstacleCalculator<T>::calculatePotentialsWorker, this));
#else
	calculatePotentialsWorker();
#endif
}

template <class T>
void ObstacleCalculator<T>::calculatePotentialsWorker()
{
	//costmap_->stop();
	const auto &cm = costmap_->getCostmap();
	const auto xSize = cm->getSizeInCellsX();
	const auto ySize = cm->getSizeInCellsY();
	potentials_.resize(xSize * ySize);
	obstacleGradientImpl_->setSize((int)xSize, (int)ySize);
	obstacleGradientImpl_->calculatePotentials(cm->getCharMap(), potentials_.data());
	//costmap_->start();
}

template <class T>
void ObstacleCalculator<T>::calculateCosts(std::vector<T> &arcSegCosts,
		const geometry_msgs::TransformStamped &pathToMapTransform,
		const std::vector<SegmentState<T>> &xStates,
		const std::vector<SegmentState<T>> &yStates)
{
	if (!costmap_)
	{
		arcSegCosts = std::vector<T>(xStates.size(), static_cast<T>(0));
		return;
	}
#ifdef THREADED_COST
	threadPool_.enqueue(std::bind(&ObstacleCalculator::calculateCostsWorker, this,
				std::ref(arcSegCosts),
				std::cref(pathToMapTransform),
				std::cref(xStates),
				std::cref(yStates)));
#else
#ifdef THREADED_POTENTIALS
	waitForCompletion();
#endif
	calculateCostsWorker(arcSegCosts, pathToMapTransform, xStates, yStates);
#endif
}

template <class T>
void ObstacleCalculator<T>::calculateCostsWorker(std::vector<T> &arcSegCosts,
		const geometry_msgs::TransformStamped &pathToMapTransform,
		const std::vector<SegmentState<T>> &xStates,
		const std::vector<SegmentState<T>> &yStates)
{
	obstacleCostImpl_.calculate(arcSegCosts, costmap_, pathToMapTransform, potentials_, xStates, yStates);
}

template <class T>
void ObstacleCalculator<T>::waitForCompletion()
{
	if (!costmap_)
		return;
#ifdef THREADED_POTENTIALS
	threadPool_.waitFinished();
#endif
}

template <class T>
int ObstacleCalculator<T>::getThreshold(void) const
{
	if (!costmap_)
		return 0;
	return obstacleGradientImpl_->getThreshold();
}

template <class T>
void ObstacleCalculator<T>::setThreshold(int threshold)
{
	if (!costmap_)
		return;
	obstacleGradientImpl_->setThreshold(threshold);
}

template class ObstacleCalculator<double>;
//template class ObstacleCalculator<float>;
