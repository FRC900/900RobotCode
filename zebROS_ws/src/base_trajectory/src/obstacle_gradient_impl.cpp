#include "base_trajectory/obstacle_gradient_impl.h"

ObstacleGradientImpl::ObstacleGradientImpl(int nx, int ny, uint8_t threshold)
	: threshold_(threshold)
{
	setSize(nx, ny);
}

void ObstacleGradientImpl::setSize(int nx, int ny)
{
	nx_ = nx;
	ny_ = ny;
	ns_ = nx * ny;
}

// Simple helper function to convert from x,y to an index in
// the 1d potentials or cost arrays
int ObstacleGradientImpl::toIndex(int nx, int ny) const
{
	return ny * nx_ + nx;
}

void ObstacleGradientImpl::setThreshold(uint8_t threshold)
{
	threshold_ = threshold;
}

uint8_t ObstacleGradientImpl::getThreshold(void) const
{
	return threshold_;
}
