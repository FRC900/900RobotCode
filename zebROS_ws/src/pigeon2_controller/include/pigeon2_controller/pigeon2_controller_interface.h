#ifndef INC_PIGEON2_CONTROLLER_INTERFACE_H__
#define INC_PIGEON2_CONTROLLER_INTERFACE_H__

#include <atomic>
#include "ddr_updater/ddr_updater.h"
#include "pigeon2_interface/pigeon2_command_interface.h"

namespace pigeon2_controller_interface
{
class Pigeon2CIParams : public ddr_updater::DDRUpdater
{
	public:
		Pigeon2CIParams(ros::NodeHandle n);

		// Generic callbacks for ddynamic reconfigure code
		template<class T>
		void setVar(T input, std::atomic<T> &val, bool update_dynamic = true);

		std::atomic<double> mount_pose_yaw_{0};
		std::atomic<double> mount_pose_pitch_{0};
		std::atomic<double> mount_pose_roll_{0};
		std::atomic<double> gyro_trim_scalar_x_{0};
		std::atomic<double> gyro_trim_scalar_y_{0};
		std::atomic<double> gyro_trim_scalar_z_{0};
		std::atomic<bool>   enable_compass_{false};
		std::atomic<bool>   disable_temperature_compensation_{false};
		std::atomic<bool>   disable_no_motion_calibration_{false};

	private:

		template <typename T>
		bool readIntoScalar(ros::NodeHandle &n, const std::string &name, std::atomic<T> &scalar)
		{
			T val;
			auto rc = n.getParam(name, val);
			if (rc)
			{
				scalar = val;
			}
			return rc;
		}
};

class Pigeon2ControllerInterface
{
	public:
		Pigeon2ControllerInterface(ros::NodeHandle &n, const std::string &joint_name, hardware_interface::pigeon2::Pigeon2CommandHandle handle);

		void update(void);

		// Functions to set the various controller-level atomoic<> private
		// member vars below
		// These could be called by e.g. service calls in a controller
		void setClearStickyFaults(void);
		void setYaw(const double yaw);

		void setDisableNoMotionCalibration(const bool flag);

        // Functions which pass through to the CIParams for this controller interface
		// Only needed if normal code needs to access them?
#if 0
		void setVelocityMeasPeriod(hardware_interface::pigeon2::SensorVelocityMeasPeriod velocity_meas_period);
#endif

	private:
		Pigeon2CIParams                                   params_;
		hardware_interface::pigeon2::Pigeon2CommandHandle handle_;
		std::atomic<bool>                                 clear_sticky_faults_{false};
		// TODO : this should probably be mutex protected to keep the 
		// value + bool trigger flag in sync
		std::atomic<double>                               set_yaw_value_{};
		std::atomic<bool>                                 set_yaw_bool_{false};
};

} // namespace pigeon2_controller_interface

#endif