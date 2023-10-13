#ifndef INC_PIGEON2_COMMAND_INTERFACE__
#define INC_PIGEON2_COMMAND_INTERFACE__

#include "pigeon2_interface/pigeon2_state_interface.h"
#include "state_handle/command_handle.h"

namespace hardware_interface
{
namespace pigeon2
{
class Pigeon2HWCommand
{
	public:
		Pigeon2HWCommand(void);

		void   setMountPoseYaw(const double direction);
		double getMountPoseYaw(void) const;
		void   setMountPosePitch(const double direction);
		double getMountPosePitch(void) const;
		void   setMountPoseRoll(const double direction);
		double getMountPoseRoll(void) const;

		bool mountPoseRPYChanged(double &yaw, double &pitch, double &roll);
		void resetMountPoseRPY(void);

		void   setGyroTrimScalarX(const double gyro_trim_scalar_x);
		double getGyroTrimScalarX(void) const;
		void   setGyroTrimScalarY(const double gyro_trim_scalar_y);
		double getGyroTrimScalarY(void) const;
		void   setGyroTrimScalarZ(const double gyro_trim_scalar_z);
		double getGyroTrimScalarZ(void) const;

		bool gyroTrimChanged(double &gyro_trim_scalar_x,
							 double &gyro_trim_scalar_y,
							 double &gyro_trim_scalar_z);
		void resetGyroTrim(void);
		
		void setEnableCompass(const bool enable_compass);
		bool getEnableCompass(void) const;

		void setDisableTemperatureCompensation(const bool disable_temperature_compensation);
		bool getDisableTemperatureCompensation(void) const;

		void setDisableNoMotionCalibration(const bool disable_no_motion_compensation);
		bool getDisableNoMotionCalibration(void) const;

		bool pigeon2FeaturesChanged(bool &enable_compass,
									bool &disable_temperature_compensation,
									bool &disable_no_motion_compensation);
		void resetPigeon2Features(void);

		void   setSetYaw(double yaw);
		double getSetYaw(void) const;
		bool   setYawChanged(double &yaw);
		void   resetSetYaw(void);

		void setClearStickyFaults(void);
		bool getClearStickyFaults(void) const;
		bool clearStickyFaultsChanged(void);

	private:
		double mount_pose_roll_{0};
		double mount_pose_pitch_{0};
		double mount_pose_yaw_{0};
		bool   mount_pose_rpy_changed_{true};
		double gyro_trim_scalar_x_{0};
		double gyro_trim_scalar_y_{0};
		double gyro_trim_scalar_z_{0};
		bool   gyro_trim_changed_{true};
		bool   enable_compass_{false};
		bool   disable_temperature_compensation_{false};
		bool   disable_no_motion_calibration_{false};
		bool   pigeon2_features_changed_{true};
		double set_yaw_{0};
		bool   set_yaw_changed_{false};
		bool   clear_sticky_faults_{false};
};

// Handle - used by each controller to get, by name of the
// corresponding joint, an interface with which to send commands
// to a Pigeon2
typedef CommandHandle<Pigeon2HWCommand, Pigeon2HWState, Pigeon2StateHandle> Pigeon2CommandHandle;


// Use ClaimResources here since we only want 1 controller
// to be able to access a given Pigeon2 at any particular time
class Pigeon2CommandInterface : public HardwareResourceManager<Pigeon2CommandHandle, ClaimResources> {};

} // namespace pigeon2
} // namespace hardware_interface

#endif