#ifndef INC_PIGEON2_STATE_INTERFACE__
#define INC_PIGEON2_STATE_INTERFACE__

#include <array>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include "state_handle/state_handle.h"

namespace hardware_interface::pigeon2
{
class Pigeon2HWState
{
	public:
		explicit Pigeon2HWState(const int device_number);
		int getDeviceNumber(void) const;

		void   setMountPoseYaw(const double direction);
		double getMountPoseYaw(void) const;
		void   setMountPosePitch(const double direction);
		double getMountPosePitch(void) const;
		void   setMountPoseRoll(const double direction);
		double getMountPoseRoll(void) const;

		void   setGyroTrimScalarX(const double gyro_trim_scalar_x);
		double getGyroTrimScalarX(void) const;
		void   setGyroTrimScalarY(const double gyro_trim_scalar_y);
		double getGyroTrimScalarY(void) const;
		void   setGyroTrimScalarZ(const double gyro_trim_scalar_z);
		double getGyroTrimScalarZ(void) const;	

		void setEnableCompass(const bool enable_compass);
		bool getEnableCompass(void) const;

		void setDisableTemperatureCompensation(const bool disable_temperature_compensation);
		bool getDisableTemperatureCompensation(void) const;

		void setDisableNoMotionCalibration(const bool disable_no_motion_calibration);
		bool getDisableNoMotionCalibration(void) const;

		void setNoMotionCount(const int no_motion_count);
		int getNoMotionCount(void) const;

		void setVersionMajor(const int version_major);
		int  getVersionMajor(void) const;

		void setVersionMinor(const int version_minor);
		int  getVersionMinor(void) const;

		void setVersionBugfix(const int version_bugfix);
		int  getVersionBugfix(void) const;

		void setVersionBuild(const int version_build);
		int  getVersionBuild(void) const;

		void   setYaw(const double yaw);
		double getYaw(void) const;
		void   setPitch(const double pitch);
		double getPitch(void) const;
		void   setRoll(const double roll);
		double getRoll(void) const;

		void   setQuatW(const double quat_w);
		double getQuatW(void) const;
		void   setQuatX(const double quat_x);
		double getQuatX(void) const;
		void   setQuatY(const double quat_y);
		double getQuatY(void) const;
		void   setQuatZ(const double quat_z);
		double getQuatZ(void) const;

		void   setGravityVectorX(const double gravity_vector_x);
		double getGravityVectorX(void) const;
		void   setGravityVectorY(const double gravity_vector_y);
		double getGravityVectorY(void) const;
		void   setGravityVectorZ(const double gravity_vector_z);
		double getGravityVectorZ(void) const;

		void   setTemperature(const double temperature);
		double getTemperature(void) const;

		void setUptime(const double uptime);
		double getUptime(void) const;

		void   setAccumGyroX(const double accum_gyro_x);
		double getAccumGyroX(void) const;
		void   setAccumGyroY(const double accum_gyro_y);
		double getAccumGyroY(void) const;
		void   setAccumGyroZ(const double accum_gyro_z);
		double getAccumGyroZ(void) const;

		void   setAngularVelocityX(const double angular_velocity_x);
		double getAngularVelocityX(void) const;
		void   setAngularVelocityY(const double angular_velocity_y);
		double getAngularVelocityY(void) const;
		void   setAngularVelocityZ(const double angular_velocity_z);
		double getAngularVelocityZ(void) const;

		void   setAccelerationX(const double acceleration_x);
		double getAccelerationX(void) const;
		void   setAccelerationY(const double acceleration_y);
		double getAccelerationY(void) const;
		void   setAccelerationZ(const double acceleration_z);
		double getAccelerationZ(void) const;

		void   setSupplyVoltage(const double supply_voltage);
		double getSupplyVoltage(void) const;

		void   setMagneticFieldX(const double magnetic_field_x);
		double getMagneticFieldX(void) const;
		void   setMagneticFieldY(const double magnetic_field_y);
		double getMagneticFieldY(void) const;
		void   setMagneticFieldZ(const double magnetic_field_z);
		double getMagneticFieldZ(void) const;

		void   setRawMagneticFieldX(const double raw_magnetic_field_x);
		double getRawMagneticFieldX(void) const;
		void   setRawMagneticFieldY(const double raw_magnetic_field_y);
		double getRawMagneticFieldY(void) const;
		void   setRawMagneticFieldZ(const double raw_magnetic_field_z);
		double getRawMagneticFieldZ(void) const;

		void setFaultHardware(const bool fault_hardware);
		bool getFaultHardware(void) const;
		void setFaultUndervoltage(const bool fault_undervolage);
		bool getFaultUndervoltage(void) const;
		void setFaultBootDuringEnable(const bool fault_boot_during_enable);
		bool getFaultBootDuringEnable(void) const;
		void setFaultUnlicensedFeatureInUse(const bool fault_unlicensed_feature_in_use);
		bool getFaultUnlicensedFeatureInUse(void) const;
		void setFaultBootupAccelerometer(const bool fault_bootup_accelerometer);
		bool getFaultBootupAccelerometer(void) const;
		void setFaultBootupGyroscope(const bool fault_bootup_gyroscope);
		bool getFaultBootupGyroscope(void) const;
		void setFaultBootupMagnetometer(const bool fault_bootup_magnetometer);
		bool getFaultBootupMagnetometer(void) const;
		void setFaultBootIntoMotion(const bool fault_boot_into_motion);
		bool getFaultBootIntoMotion(void) const;
		void setFaultDataAcquiredLate(const bool fault_data_acquired_late);
		bool getFaultDataAcquiredLate(void) const;
		void setFaultLoopTimeSlow(const bool fault_loop_time_slow);
		bool getFaultLoopTimeSlow(void) const;
		void setFaultSaturatedMagnetometer(const bool fault_saturated_magneter);
		bool getFaultSaturatedMagnetometer(void) const;
		void setFaultSaturatedAccelerometer(const bool fault_saturated_accelerometometer);
		bool getFaultSaturatedAccelerometer(void) const;
		void setFaultSaturatedGyroscope(const bool fault_saturated_gyroscope);
		bool getFaultSaturatedGyroscope(void) const;

		void setStickyFaultHardware(const bool sticky_fault_hardware);
		bool getStickyFaultHardware(void) const;
		void setStickyFaultUndervoltage(const bool sticky_fault_undervolage);
		bool getStickyFaultUndervoltage(void) const;
		void setStickyFaultBootDuringEnable(const bool sticky_fault_boot_during_enable);
		bool getStickyFaultBootDuringEnable(void) const;
		void setStickyFaultUnlicensedFeatureInUse(const bool sticky_fault_unlicensed_feature_in_use);
		bool getStickyFaultUnlicensedFeatureInUse(void) const;
		void setStickyFaultBootupAccelerometer(const bool sticky_fault_bootup_accelerometer);
		bool getStickyFaultBootupAccelerometer(void) const;
		void setStickyFaultBootupGyroscope(const bool sticky_fault_bootup_gyroscope);
		bool getStickyFaultBootupGyroscope(void) const;
		void setStickyFaultBootupMagnetometer(const bool sticky_fault_bootup_magnetometer);
		bool getStickyFaultBootupMagnetometer(void) const;
		void setStickyFaultBootIntoMotion(const bool sticky_fault_boot_into_motion);
		bool getStickyFaultBootIntoMotion(void) const;
		void setStickyFaultDataAcquiredLate(const bool sticky_fault_data_acquired_late);
		bool getStickyFaultDataAcquiredLate(void) const;
		void setStickyFaultLoopTimeSlow(const bool sticky_fault_loop_time_slow);
		bool getStickyFaultLoopTimeSlow(void) const;
		void setStickyFaultSaturatedMagnetometer(const bool sticky_fault_saturated_magnetometer);
		bool getStickyFaultSaturatedMagnetometer(void) const;
		void setStickyFaultSaturatedAccelerometer(const bool sticky_fault_saturated_accelerometer);
		bool getStickyFaultSaturatedAccelerometer(void) const;
		void setStickyFaultSaturatedGyroscope(const bool sticky_fault_saturated_gyrosscope);
		bool getStickyFaultSaturatedGyroscope(void) const;

	private :
		int      device_number_;
		double   mount_pose_roll_{0};
		double   mount_pose_pitch_{0};
		double   mount_pose_yaw_{0};
		double   gyro_trim_scalar_x_{0};
		double   gyro_trim_scalar_y_{0};
		double   gyro_trim_scalar_z_{0};
		bool     enable_compass_{false};
		bool     disable_temperature_compensation_{false};
		bool     disable_no_motion_calibration_{false};
		int      no_motion_count_{0};
		int      version_major_{0};
		int      version_minor_{0};
		int      version_bugfix_{0};
		int      version_build_{0};
		double   yaw_{0};
		double   pitch_{0};
		double   roll_{0};
		double   quat_w_{0};
		double   quat_x_{0};
		double   quat_y_{0};
		double   quat_z_{0};
		double   gravity_vector_x_{0};
		double   gravity_vector_y_{0};
		double   gravity_vector_z_{0};
		double   temperature_{0};
		double   uptime_{0};
		double   accum_gyro_x_{0};
		double   accum_gyro_y_{0};
		double   accum_gyro_z_{0};
		double   angular_velocity_x_{0};
		double   angular_velocity_y_{0};
		double   angular_velocity_z_{0};
		double   acceleration_x_{0};
		double   acceleration_y_{0};
		double   acceleration_z_{0};
		double   supply_voltage_{0};
		double   magnetic_field_x_{0};
		double   magnetic_field_y_{0};
		double   magnetic_field_z_{0};
		double   raw_magnetic_field_x_{0};
		double   raw_magnetic_field_y_{0};
		double   raw_magnetic_field_z_{0};

		bool     fault_hardware_{false};
		bool     fault_undervolage_{false};
		bool     fault_boot_during_enable_{false};
		bool     fault_unlicensed_feature_in_use_{false};
		bool     fault_bootup_accelerometer_{false};
		bool     fault_bootup_gyroscope_{false};
		bool     fault_bootup_magnetometer_{false};
		bool     fault_boot_into_motion_{false};
		bool     fault_data_acquired_late_{false};
		bool     fault_loop_time_slow_{false};
		bool     fault_saturated_magnetometer_{false};
		bool     fault_saturated_accelerometer_{false};
		bool     fault_saturated_gyroscope_{false};
           
		bool     sticky_fault_hardware_{false};
		bool     sticky_fault_undervolage_{false};
		bool     sticky_fault_boot_during_enable_{false};
		bool     sticky_fault_unlicensed_feature_in_use_{false};
		bool     sticky_fault_bootup_accelerometer_{false};
		bool     sticky_fault_bootup_gyroscope_{false};
		bool     sticky_fault_bootup_magnetometer_{false};
		bool     sticky_fault_boot_into_motion_{false};
		bool     sticky_fault_data_acquired_late_{false};
		bool     sticky_fault_loop_time_slow_{false};
		bool     sticky_fault_saturated_magnetometer_{false};
		bool     sticky_fault_saturated_accelerometer_{false};
		bool     sticky_fault_saturated_gyroscope_{false};
};
// Glue code to let this be registered in the list of
// hardware resources on the robot.  Since state is
// read-only, allow multiple controllers to register it.
using Pigeon2StateHandle = StateHandle<const Pigeon2HWState>;
using Pigeon2WritableStateHandle = StateHandle<Pigeon2HWState>;
class Pigeon2StateInterface : public HardwareResourceManager<Pigeon2StateHandle> {};
class RemotePigeon2StateInterface : public HardwareResourceManager<Pigeon2WritableStateHandle, ClaimResources> {};

} // namespace

#endif