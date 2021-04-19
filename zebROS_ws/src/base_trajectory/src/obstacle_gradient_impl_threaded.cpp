#include <limits>
#include "base_trajectory/obstacle_gradient_impl_threaded.h"

constexpr size_t NUM_THREADS = 4;
ObstacleGradientImplThreaded::ObstacleGradientImplThreaded(int nx, int ny, uint8_t threshold)
	: ObstacleGradientImpl(nx, ny, threshold)
	, threadPool(NUM_THREADS)
{
	ObstacleGradientImplThreaded::setSize(nx, ny);
}

// Given the grid map of size nx_, ny_ containing costs costs, create a potential
// map of the same space. Each grid potential is defined to be 0 if there is no
// obstacle at the grid location.  If there is an obstacle, the potential is the
// sum of the shortest horizontal and vertical distances to a grid location with
// no obstacle. This will tend to produce potential fields which look like pyramids,
// with a hopefully easy to follow gradient from obstacles into cells which are
// not obstructed.
// Split the work into 4 parallel threads, then sum the results from all 4 at the end.
void ObstacleGradientImplThreaded::calculatePotentialsHorizontalForward(const unsigned char *costs)
{
	// Initialize the grid - 0s for unoccupied map locations, max for areas with obstacles
	for (int i = 0; i < ns_; i++)
	{
		if (costs[i] <= threshold_)
		{
			h_potentials_forward_[i] = 0;
		}
		else
		{
			h_potentials_forward_[i] = std::numeric_limits<unsigned int>::max();
		}
	}
	// For each row iteratively propagate potentials from left and right of each cell
	for (int y = 0; y < ny_; y++)
	{
		for (int x = 1; x < nx_; x++)
		{
			const auto i = toIndex(x,y);
			h_potentials_forward_[i] = std::min(h_potentials_forward_[i-1] + 1, h_potentials_forward_[i]);
		}
	}
}

void ObstacleGradientImplThreaded::calculatePotentialsHorizontalBackward(const unsigned char *costs)
{
	// Initialize the grid - 0s for unoccupied map locations, max for areas with obstacles
	for (int i = 0; i < ns_; i++)
	{
		if (costs[i] <= threshold_)
		{
			h_potentials_backward_[i] = 0;
		}
		else
		{
			h_potentials_backward_[i] = std::numeric_limits<unsigned int>::max();
		}
	}
	for (int y = 0; y < ny_; y++)
	{
		for (int x = nx_ - 2; x >= 0; x--)
		{
			const auto i = toIndex(x,y);
			h_potentials_backward_[i] = std::min(h_potentials_backward_[i+1] + 1, h_potentials_backward_[i]);
		}
	}
}
void ObstacleGradientImplThreaded::calculatePotentialsVerticalForward(const unsigned char *costs)
{
	// Initialize the grid - 0s for unoccupied map locations, max for areas with obstacles
	for (int i = 0; i < ns_; i++)
	{
		if (costs[i] <= threshold_)
		{
			v_potentials_forward_[i] = 0;
		}
		else
		{
			v_potentials_forward_[i] = std::numeric_limits<unsigned int>::max();
		}
	}
	// Same process, this time propagating potentials along each column from
	// tops and bottom
	for (int x = 0; x < nx_; x++)
	{
		for (int y = 1; y < ny_; y++)
		{
			const auto i = toIndex(x,y);
			v_potentials_forward_[i] = std::min(v_potentials_forward_[i-nx_] + 1, v_potentials_forward_[i]);
		}
	}
}


void ObstacleGradientImplThreaded::calculatePotentialsVerticalBackward(const unsigned char *costs)
{
	// Initialize the grid - 0s for unoccupied map locations, max for areas with obstacles
	for (int i = 0; i < ns_; i++)
	{
		if (costs[i] <= threshold_)
		{
			v_potentials_backward_[i] = 0;
		}
		else
		{
			v_potentials_backward_[i] = std::numeric_limits<unsigned int>::max();
		}
	}
	for (int x = 0; x < nx_; x++)
	{
		for (int y = ny_ - 2; y >= 0; y--)
		{
			const auto i = toIndex(x,y);
			v_potentials_backward_[i] = std::min(v_potentials_backward_[i+nx_] + 1, v_potentials_backward_[i]);
		}
	}
}

void ObstacleGradientImplThreaded::finalSum(unsigned int *potentials, const unsigned char *costs, const size_t startIdx, const size_t len) const
{
	// And then sum things up to get the overall potential map
	const auto endIdx = startIdx + len;
	for (size_t i = startIdx; i <= endIdx; i++)
	{
		potentials[i] = (std::min(h_potentials_forward_[i], h_potentials_backward_[i]) + std::min(v_potentials_forward_[i], v_potentials_backward_[i])) * costs[i];
	}
}

void ObstacleGradientImplThreaded::calculatePotentials(const unsigned char* costs, unsigned int* potentials)
{
	threadPool.enqueue(std::bind(&ObstacleGradientImplThreaded::calculatePotentialsHorizontalForward,  this, costs));
	threadPool.enqueue(std::bind(&ObstacleGradientImplThreaded::calculatePotentialsHorizontalBackward, this, costs));
	threadPool.enqueue(std::bind(&ObstacleGradientImplThreaded::calculatePotentialsVerticalForward,    this, costs));
	threadPool.enqueue(std::bind(&ObstacleGradientImplThreaded::calculatePotentialsVerticalBackward,   this, costs));
	threadPool.waitFinished();
	const auto countPerThread = (ns_ + NUM_THREADS - 1) / NUM_THREADS;
	size_t startIdx = 0;
	for (size_t i = 0; i < NUM_THREADS; i++)
	{
		const auto len = std::min(countPerThread, ns_ - startIdx - 1);
		threadPool.enqueue(std::bind(&ObstacleGradientImplThreaded::finalSum, this, potentials, costs, startIdx, len));
		startIdx += countPerThread;
	}
	threadPool.waitFinished();
}

void ObstacleGradientImplThreaded::setSize(int nx, int ny)
{
	ObstacleGradientImpl::setSize(nx, ny);

	// These should be no-ops in the most common case where the map is static
	h_potentials_forward_.resize(ns_);
	h_potentials_backward_.resize(ns_);
	v_potentials_forward_.resize(ns_);
	v_potentials_backward_.resize(ns_);
}
