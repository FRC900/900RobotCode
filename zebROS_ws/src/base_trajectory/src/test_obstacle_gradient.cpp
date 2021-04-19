#include <thread>
#include <chrono>
#include <cstring>
#include <iostream>
#include <chrono>
#include <ros/console.h>
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;
#include <memory>
#include "base_trajectory/cuda_utils.h"
#include "base_trajectory/obstacle_gradient.h"

int toIndex(int x, int y, int nx)
{
	return y * nx + x;
}
void runTest(int nx, int ny, unsigned char *costs, unsigned int *potentials, ObstacleGradientImplType type, const char *name)
{
	ObstacleGradient og(nx, ny, 50, type);
    auto t1 = high_resolution_clock::now();
	for (int i = 0; i < 100; i++)
	{
		og.calculatePotentials(costs, potentials);
	}
    auto t2 = high_resolution_clock::now();
    const duration<double, std::milli> ms_double = t2 - t1;
    ROS_INFO_STREAM(__FUNCTION__ << name << " : " << ms_double.count() << " mSec");

	for (int j = 0; j < 15; j ++)
	{
		for (int i = 0; i < 15; i ++)
		{
			std::cout << " " << (int)costs[toIndex(i, j, nx)];
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

	for (int j = 0; j < 15; j ++)
	{
		for (int i = 0; i < 15; i ++)
		{
			std::cout << " " << potentials[toIndex(i, j, nx)];
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

}
int main(void)
{
	constexpr int nx = 3000;
	constexpr int ny = 1500;
	auto costs = std::make_unique<unsigned char []>(nx * ny);
	auto potentials = std::make_unique<unsigned int[]>(nx * ny);

	memset(costs.get(), 0, sizeof(costs));

	costs[toIndex(3,3,nx)] = 253;
	costs[toIndex(3,4,nx)] = 253;
	costs[toIndex(3,5,nx)] = 253;
	costs[toIndex(3,6,nx)] = 253;

	costs[toIndex(4,3,nx)] = 253;
	costs[toIndex(4,4,nx)] = 253;
	costs[toIndex(4,5,nx)] = 253;
	costs[toIndex(4,6,nx)] = 253;

	costs[toIndex(5,3,nx)] = 253;
	costs[toIndex(5,4,nx)] = 253;
	costs[toIndex(5,5,nx)] = 253;
	costs[toIndex(5,6,nx)] = 253;

	costs[toIndex(6,3,nx)] = 253;
	costs[toIndex(6,4,nx)] = 253;
	costs[toIndex(6,5,nx)] = 253;
	costs[toIndex(6,6,nx)] = 253;

	costs[toIndex(7,3,nx)] = 253;
	costs[toIndex(7,4,nx)] = 253;
	costs[toIndex(7,5,nx)] = 253;
	costs[toIndex(7,6,nx)] = 253;

	runTest(nx, ny, costs.get(), potentials.get(), ObstacleGradientImplType::CPU, "CPU");
	runTest(nx, ny, costs.get(), potentials.get(), ObstacleGradientImplType::THREADED, "THREADED");
	if (hasCudaGPU())
	{
		runTest(nx, ny, costs.get(), potentials.get(), ObstacleGradientImplType::CUDA, "CUDA");
	}
}
