#ifndef INC_PIGEON2_STATE_INTERFACE__
#define INC_PIGEON2_STATE_INTERFACE__

#include <array>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include "state_handle/state_handle.h"

namespace hardware_interface
{
namespace pigeon2
{

enum class AxisDirection {
	Undefined,
	PositiveZ,
	PositiveY,
	PositiveX,
	NegativeZ,
	NegativeY,
	NegativeX,
	AxisDirection_Last
};
class Pigeon2HWState
{
	public:
		Pigeon2HWState(int device_number);
		int getDeviceNumber(void) const;

		void setMountPoseForward(AxisDirection direction);
		AxisDirection getMountPoseForward(void) const;
		
		void setMountPoseUp(AxisDirection direction);
		AxisDirection getMountPoseUp(void) const;

		void setMountPoseRoll(double direction);
		double getMountPoseRoll(void) const;
		
		void setMountPosePitch(double direction);
		double getMountPosePitch(void) const;

		void setMountPoseYaw(double direction);
		double getMountPoseYaw(void) const;
		
		void setXAxisGyroError(double x_axis_gyro_error);
		double getXAxisGyroError(void) const;
		
		void setYAxisGyroError(double y_axis_gyro_error);
		double getYAxisGyroError(void) const;
		
		void setZAxisGyroError(double z_axis_gyro_error);
		double getZAxisGyroError(void) const;

		void setCompassEnable(bool compass_enable);
		bool getCompassEnable(void) const;

		void setDisableTemperatureCompensation(bool temperature_compensation);
		bool getDisableTemperatureCompensation(void) const;

		void setDisableNoMotionCalibration(bool disable_no_motion_calibration);
		bool getDisableNoMotionCalibration(void) const;

		void setGravityVector(const std::array<double, 3> &gravity_vector);
		std::array<double, 3> getGravityVector(void) const;

		void set6dQuaternion(std::array<double, 4> &quaternion);
		std::array<double, 4> get6dQuatention(void) const;

		void setYaw(double yaw);
		double getYaw(void) const;

		void setPitch(double pitch);
		double getPitch(void) const;

		void setRoll(double roll);
		double getRoll(void) const;

		void setAccumGyro(std::array<double, 3> accum_gyro);
		std::array<double, 3> getAccumGyro(void) const;

		void setAbsoluteCompassHeading(double absolute_compass_heading);
		double getAbsoluteCompassHeading(void) const;

		void setCompassHeading(double compass_heading);
		double getCompassHeading(void) const;

		void setCompassFieldStrength(double compass_field_strength);
		double getCompassFieldStrength(void) const;

		void setTemperature(double temperature);
		double getTemperature(void) const;

		void setUptime(uint32_t uptime);
		uint32_t getUptime(void) const;

		void setRawMagnetometer(std::array<int16_t, 3> &raw_magnetometer);
		std::array<int16_t, 3> getRawMagnetometer(void) const;

		void setBiasedMagnetometer(std::array<int16_t, 3> &biased_magnetometer);
		std::array<int16_t, 3> getBiasedMagnetometer(void) const;

		void setBiasedAccelerometer(std::array<int16_t, 3> &biased_accelerometer);
		std::array<int16_t, 3> getBiasedAccelerometer(void) const;

		void setRawGyro(std::array<double, 3> &raw_gyro);
		std::array<double, 3> getRawGyro(void) const;

		void setResetCount(uint32_t reset_count);
		uint32_t getResetCount(void) const;

		void setResetFlags(uint32_t reset_flags);
		uint32_t getResetFlags(void) const;

		void setFirmwareVersion(uint32_t firmware_version);
		uint32_t getFirmwareVersion(void) const;

		uint64_t getFaults(void) const;
		void setFaults(uint64_t faults);

		uint64_t getStickyFaults(void) const;
		void setStickyFaults(uint64_t sticky_faults);

	private :
		int                          device_number_;
		AxisDirection                mount_pose_forward_{AxisDirection::Undefined};
		AxisDirection                mount_pose_up_{AxisDirection::Undefined};
		double                       mount_pose_roll_{0};
		double                       mount_pose_pitch_{0};
		double                       mount_pose_yaw_{0};
		double                       x_axis_gyro_error_{0};
		double                       y_axis_gyro_error_{0};
		double                       z_axis_gyro_error_{0};
		bool                         compass_enable_{false};
		bool                         disable_temperature_compensation_{false};
		bool                         disable_no_motion_calibration_{false};
		std::array<double, 3>        gravity_vector_{0, 0, 0};
		std::array<double, 4>        quaternion_6d_{0, 0, 0, 0};
		double                       yaw_{0};
		double                       pitch_{0};
		double                       roll_{0};
		std::array<double, 3>        accum_gyro_{0};
		double                       absolute_compass_heading_{0};
		double                       compass_heading_{0};
		double                       compass_field_strength_{0};
		double                       temperature_{0};
		uint32_t                     uptime_{0};
		std::array<int16_t, 3>       raw_magnetometer_xyz_{0, 0, 0};
		std::array<int16_t, 3>       biased_magnetometer_xyz_{0, 0, 0};
		std::array<int16_t, 3>       biased_accelerometer_xyz_{0, 0, 0};
		std::array<double, 3>        raw_gyro_{0, 0, 0};
		uint32_t                     reset_count_{0};
		uint32_t                     reset_flags_{0};
		uint32_t                     firmware_version_{0};
		uint64_t                     faults_{0};
		uint64_t                     sticky_faults_{0};
};
// Glue code to let this be registered in the list of
// hardware resources on the robot.  Since state is
// read-only, allow multiple controllers to register it.
typedef StateHandle<const Pigeon2HWState> Pigeon2StateHandle;
typedef StateHandle<Pigeon2HWState>       Pigeon2WritableStateHandle;
class Pigeon2StateInterface : public HardwareResourceManager<Pigeon2StateHandle> {};
class RemotePigeon2StateInterface : public HardwareResourceManager<Pigeon2WritableStateHandle> {};

} // namespace cancoder
} // namespace hardware_interface

#endif