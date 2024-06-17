#ifndef INC_SWERVE_MATH_H_
#define INC_SWERVE_MATH_H_

#include <array>
#include <Eigen/Dense>

template <size_t WHEELCOUNT>
class swerveDriveMath
{
	public:
		swerveDriveMath() = default;
		swerveDriveMath(const swerveDriveMath &) = delete;
		swerveDriveMath(swerveDriveMath &&) noexcept = delete;
		swerveDriveMath &operator=(const swerveDriveMath &) = delete;
		swerveDriveMath &operator=(swerveDriveMath &&) noexcept = delete;
		virtual ~swerveDriveMath() = default;

		explicit swerveDriveMath(const std::array<Eigen::Vector2d, WHEELCOUNT> &wheelCoordinate);

		//Wheel multipliers would need to be rerun if wheels somehow get moved around
		std::array<Eigen::Vector2d, WHEELCOUNT> wheelMultipliersXY(const Eigen::Vector2d &rotationCenter) const;

		std::array<Eigen::Vector2d, WHEELCOUNT> wheelSpeedsAngles(const std::array<Eigen::Vector2d, WHEELCOUNT> &wheelMultipliersXY, const Eigen::Vector2d &linearVelocity, const double angularVelocity, const bool norm) const;

		double getParkingAngle(const size_t wheel) const;

	private:
		//only must be run once to determine the angles of the wheels in parking config
		std::array<double, WHEELCOUNT> parkingAngles(void) const;

		void normalize(std::array<double, WHEELCOUNT> &input, const bool force_norm = false) const;

		//All variables here which don't need to be accessed externally
		std::array<Eigen::Vector2d, WHEELCOUNT> wheelCoordinate_;
		std::array<double, WHEELCOUNT> parkingAngle_;
};

#endif