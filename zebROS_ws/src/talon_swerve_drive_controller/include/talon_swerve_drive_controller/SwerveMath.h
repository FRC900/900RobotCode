#pragma once
#include <array>
#include <vector>
#include <Eigen/Dense>

template <size_t WHEELCOUNT>
class swerveDriveMath
{
	public:
		swerveDriveMath(const std::array<Eigen::Vector2d, WHEELCOUNT> &wheelCoordinate);
		swerveDriveMath() {};
		std::array<Eigen::Vector2d, WHEELCOUNT> wheelMultipliersXY(const Eigen::Vector2d &rotationCenter) const;

		//for non field centric set angle to pi/2
		std::array<Eigen::Vector2d, WHEELCOUNT> wheelSpeedsAngles(const std::array<Eigen::Vector2d, WHEELCOUNT> &wheelMultipliersXY, const Eigen::Vector2d &velocityVector, double rotation, double angle, bool norm) const;

		double getParkingAngle(size_t wheel) const;
		//Wheel multipliers would need to be rerun if wheels somehow get moved around

	private:
		//only must be run once to determine the angles of the wheels in parking config
		std::array<double, WHEELCOUNT> parkingAngles(void) const;

		void normalize(std::array<double, WHEELCOUNT> &input, const bool force_norm = false) const;

		//All variables here which don't need to be accessed externally
		std::array<Eigen::Vector2d, WHEELCOUNT> wheelCoordinate_;
		std::array<double, WHEELCOUNT> parkingAngle_;
};
