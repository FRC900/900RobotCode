#include <limits>
#include "base_trajectory/obstacle_gradient_impl_cpu.h"

ObstacleGradientImplCpu::ObstacleGradientImplCpu(int nx, int ny, uint8_t threshold)
	: ObstacleGradientImpl(nx, ny, threshold)
{
	ObstacleGradientImplCpu::setSize(nx, ny);
}

// Given the grid map of size nx_, ny_ containing costs costs, create a potential
// map of the same space. Each grid potential is defined to be 0 if there is no
// obstacle at the grid location.  If there is an obstacle, the potential is the
// sum of the shortest horizontal and vertical distances to a grid location with
// no obstacle. This will tend to produce potential fields which look like pyramids,
// with a hopefully easy to follow gradient from obstacles into cells which are
// not obstructed.
void ObstacleGradientImplCpu::calculatePotentials(const unsigned char* costs, unsigned int* potentials)
{
	// Initialize the grid - 0s for unoccupied map locations, max for areas with obstacles
	for (int i = 0; i < ns_; i++)
	{
		if (costs[i] <= threshold_)
		{
			h_potentials_[i] = 0;
			v_potentials_[i] = 0;
		}
		else
		{
			h_potentials_[i] = std::numeric_limits<unsigned int>::max();
			v_potentials_[i] = std::numeric_limits<unsigned int>::max();
		}
	}

	// For each row iteratively propagate potentials from left and right of each cell
	for (int y = 0; y < ny_; y++)
	{
		for (int x = 1; x < nx_; x++)
		{
			const auto i = toIndex(x,y);
			h_potentials_[i] = std::min(h_potentials_[i-1] + 1, h_potentials_[i]);
		}
	}
	for (int y = 0; y < ny_; y++)
	{
		for (int x = nx_ - 2; x >= 0; x--)
		{
			const auto i = toIndex(x,y);
			h_potentials_[i] = std::min(h_potentials_[i+1] + 1, h_potentials_[i]);
		}
	}

	// Same process, this time propagating potentials along each column from
	// tops and bottom
	for (int x = 0; x < nx_; x++)
	{
		for (int y = 1; y < ny_; y++)
		{
			const auto i = toIndex(x,y);
			v_potentials_[i] = std::min(v_potentials_[i-nx_] + 1, v_potentials_[i]);
		}
	}
	for (int x = 0; x < nx_; x++)
	{
		for (int y = ny_ - 2; y >= 0; y--)
		{
			const auto i = toIndex(x,y);
			v_potentials_[i] = std::min(v_potentials_[i+nx_] + 1, v_potentials_[i]);
		}
	}

	// And then sum things up to get the overall potential map
	for (int i = 0; i < ns_; i++)
	{
		potentials[i] = (h_potentials_[i] + v_potentials_[i]) * costs[i];
	}
}

void ObstacleGradientImplCpu::setSize(int nx, int ny)
{
	ObstacleGradientImpl::setSize(nx, ny);

	// These should be no-ops in the most common case where the map is static
	h_potentials_.resize(ns_);
	v_potentials_.resize(ns_);
}
