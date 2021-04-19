#ifndef INC_OBSTACLE_GRADIENT_IMPL_
#define INC_OBSTACLE_GRADIENT_IMPL_
#include <cstdint>

class ObstacleGradientImpl
{
	public:
		ObstacleGradientImpl(int nx, int ny, uint8_t threshold);
		virtual ~ObstacleGradientImpl() = default;
		virtual void calculatePotentials(const unsigned char *costs, unsigned int *potential) = 0;
		virtual void setSize(int nx, int ny);
		virtual void setThreshold(uint8_t threshold);
		virtual uint8_t getThreshold(void) const;
	protected:
		virtual int toIndex(int nx, int ny) const;
		int nx_;
		int ny_;
		int ns_;
		uint8_t threshold_;
};

#endif

