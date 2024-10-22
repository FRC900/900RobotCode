#ifndef INC_SWERVE_MATH_H_
#define INC_SWERVE_MATH_H_

#include <array>
#include <ostream>

namespace swervemath
{
struct Point2d
{
	Point2d() = default;
	Point2d(const double xx, const double yy)
	: x{xx}
	, y{yy}
	{
	}
	double x{0.};
	double y{0.};
	Point2d &operator/=(const double rhs) 
	{
		x /= rhs;
		y /= rhs;
		return *this;
	}
	// Needed for use in std::map
	bool operator<(const Point2d &rhs) const
	{
		if (x < rhs.x)
		{
			return true;
		}
		if ((x == rhs.x) && (y < rhs.y))
		{
			return true;
		}
		return false;
	}
	friend std::ostream &operator<<(std::ostream &os, const Point2d &p)
	{
		os << "(" << p.x << ", " << p.y << ")";
		return os;
	}
};

struct SpeedAndAngle
{
	SpeedAndAngle() = default;
	SpeedAndAngle(const double speed, const double angle)
	: speed{speed}
	, angle{angle}
	{
	}
	double speed;
	double angle;
};

template <size_t WHEELCOUNT>
class SwerveDriveMath
{
	public:
		SwerveDriveMath() = default;
		SwerveDriveMath(const SwerveDriveMath &) = delete;
		SwerveDriveMath(SwerveDriveMath &&) noexcept = delete;
		SwerveDriveMath &operator=(const SwerveDriveMath &) = delete;
		SwerveDriveMath &operator=(SwerveDriveMath &&) noexcept = delete;
		virtual ~SwerveDriveMath() = default;

		explicit SwerveDriveMath(const std::array<Point2d, WHEELCOUNT> &wheelCoordinates);

		//Wheel multipliers would need to be rerun if wheels somehow get moved around
		std::array<Point2d, WHEELCOUNT> wheelMultipliersXY(const Point2d &rotationCenter) const;

		std::array<SpeedAndAngle, WHEELCOUNT> wheelSpeedsAngles(const std::array<Point2d, WHEELCOUNT> &wheelMultipliersXY,
																  const Point2d &linearVelocity,
																  const double angularVelocity,
																  const bool norm) const;

		double getParkingAngle(const size_t wheel) const;

		double furthestWheel(const Point2d &centerOfRotation) const;
	private:
		//only must be run once to determine the angles of the wheels in parking config
		std::array<double, WHEELCOUNT> parkingAngles(void) const;

		void normalize(std::array<double, WHEELCOUNT> &input, const bool force_norm = false) const;

		//All variables here which don't need to be accessed externally
		std::array<Point2d, WHEELCOUNT> wheelCoordinates_;
		std::array<double, WHEELCOUNT> parkingAngle_;
};

} // namespace swervemath
#endif