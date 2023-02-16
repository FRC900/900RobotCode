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

		void setMountPoseForward(AxisDirection direction);
		AxisDirection getMountPoseForward(void) const;

		void setMountPoseUp(AxisDirection direction);
		AxisDirection getMountPoseUp(void) const;

		bool mountPoseAxisChanged(AxisDirection &forward, AxisDirection &up);
		void resetMountPoseAxis(void);

		void setMountPoseRoll(double direction);
		double getMountPoseRoll(void) const;
		void setMountPosePitch(double direction);
		double getMountPosePitch(void) const;
		void setMountPoseYaw(double direction);
		double getMountPoseYaw(void) const;

		bool mountPoseRPYChanged(double &roll, double &pitch, double &yaw);
		void resetMountPoseRPY(void);

		void setXAxisGyroError(double x_axis_gyro_error);
		double getXAxisGyroError(void) const;
		bool xAxisGyroErrorChanged(double &x_axis_gyro_error);
		void resetXAxisGyroError(void);
		
		void setYAxisGyroError(double y_axis_gyro_error);
		double getYAxisGyroError(void) const;
		bool yAxisGyroErrorChanged(double &y_axis_gyro_error);
		void resetYAxisGyroError(void);
		
		void setZAxisGyroError(double z_axis_gyro_error);
		double getZAxisGyroError(void) const;
		bool zAxisGyroErrorChanged(double &z_axis_gyro_error);
		void resetZAxisGyroError(void);
		
		void setCompassEnable(bool compass_enable);
		bool getCompassEnable(void) const;
		bool compassEnableChanged(bool &compass_enable);
		void resetCompassEnable(void);

		void setDisableTemperatureCompensation(bool temperature_compensation);
		bool getDisableTemperatureCompensation(void) const;
		bool disableTemperatureCompensationChanged(bool &temperature_compensation);
		void resetDisableTemperatureCompensation(void);

		void setDisableNoMotionCalibration(bool disable_no_motion_compensation);
		bool getDisableNoMotionCalibration(void) const;
		bool disableNoMotionCalibrationChanged(bool &disable_no_motion_compensation);
		void resetDisableNoMotionCalibration(void);

		void setZeroGyroBiasNow(void);
		bool getZeroGyroBiasNow(void) const;
		bool zeroGyroBiasNowChanged(void);

		void setClearStickyFaults(void);
		bool getClearStickyFaults(void) const;
		bool clearStickyFaultsChanged(void);

		void setSetYaw(double yaw);
		double getSetYaw(void) const;
		bool setYawChanged(double &yaw);
		void resetSetYaw(void);

		void setAddYaw(double yaw);
		double getAddYaw(void) const;
		bool addYawChanged(double &yaw);
		void resetAddYaw(void);

		void setSetYawToCompass(void);
		bool getSetYawToCompass(void) const;
		bool setYawToCompassChanged(void);

		void setSetAccumZAngle(void);
		bool getSetAccumZAngle(void) const;
		bool setAccumZAngleChanged(void);

	private:
		AxisDirection                mount_pose_forward_{AxisDirection::Undefined};
		AxisDirection                mount_pose_up_{AxisDirection::Undefined};
		bool                         mount_pose_axis_changed_{false};
		double                       mount_pose_roll_{0};
		double                       mount_pose_pitch_{0};
		double                       mount_pose_yaw_{0};
		bool                         mount_pose_rpy_changed_{false};
		double                       x_axis_gyro_error_{0};
		bool                         x_axis_gyro_error_changed_{true};
		double                       y_axis_gyro_error_{0};
		bool                         y_axis_gyro_error_changed_{true};
		double                       z_axis_gyro_error_{0};
		bool                         z_axis_gyro_error_changed_{true};
		bool                         compass_enable_{false};
		bool                         compass_enable_changed_{true};
		bool                         disable_temperature_compensation_{false};
		bool                         disable_temperature_compensation_changed_{true};
		bool                         disable_no_motion_calibration_{false};
		bool                         disable_no_motion_calibration_changed_{true};
		bool                         clear_sticky_faults_{false};
        bool                         zero_gyro_bias_now_{false};
		double                       set_yaw_{0};
		bool                         set_yaw_changed_{false};
		double                       add_yaw_{0};
		bool                         add_yaw_changed_{false};
		bool                         set_yaw_to_compass_{false};
		bool                         set_accum_z_angle_{false};
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