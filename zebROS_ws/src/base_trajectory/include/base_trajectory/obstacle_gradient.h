#ifndef INC_OBSTACLE_GRADIENT_
#define INC_OBSTACLE_GRADIENT_

#include <memory>

// This class creates an array of values correlated to an input
// cost map. The potential field represented it designed to add
// costs to the map for areas obstructed by an obstacle. Rather
// than just add a high cost for each grid map square containing
// an obstacle, this potential map creates a gradient of costs
// sloping away from the center of each obstacle. That way, the
// gradient descent optimizer will have a set of decreasing costs
// to follow to get out from the middle of an obstacle rather than
// seeing a uniform cost for any path through the obstacle.
enum class ObstacleGradientImplType
{
	CUDA,
	CPU,
	THREADED
};

class ObstacleGradientImpl;
class ObstacleGradient
{
	public:
		ObstacleGradient(int nx, int ny, uint8_t threshold, ObstacleGradientImplType implType);
		~ObstacleGradient();
		void calculatePotentials(const unsigned char *costs, unsigned int *potential);
		void setSize(int nx, int ny);

		void setThreshold(uint8_t threshold);
		uint8_t getThreshold(void) const;
	private:
		std::unique_ptr<ObstacleGradientImpl> impl_;
};

#endif
