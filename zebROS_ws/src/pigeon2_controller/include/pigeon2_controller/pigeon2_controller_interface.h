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

		// Callbacks for ddynamic reconfigure code
		// First two are specific to the mount pose and in addition to the 
		// normal var setting, also switch between using mount pose axis and mount
		// pose RPY.
		// The last one is a generic one for scalar types (int, bool, double)
		void setMountPoseAxis(int direction, std::atomic<hardware_interface::pigeon2::AxisDirection> &val, bool update_dynamic = true);
		void setMountPoseRPY(double direction, std::atomic<double> &val, bool update_dynamic = true);
		template<class T>
		void setVar(T input, std::atomic<T> &val, bool update_dynamic = true);

		std::atomic<hardware_interface::pigeon2::AxisDirection>                mount_pose_forward_;
		std::atomic<hardware_interface::pigeon2::AxisDirection>                mount_pose_up_;
		std::atomic<bool>                                                      use_mount_pose_axis_;
		std::atomic<double>                                                    mount_pose_roll_;
		std::atomic<double>                                                    mount_pose_pitch_;
		std::atomic<double>                                                    mount_pose_yaw_;
		std::atomic<bool>                                                      use_mount_pose_rpy_;
		std::atomic<double>                                                    x_axis_gyro_error_;
		std::atomic<double>                                                    y_axis_gyro_error_;
		std::atomic<double>                                                    z_axis_gyro_error_;
		std::atomic<bool>                                                      compass_enable_;
		std::atomic<bool>                                                      disable_temperature_compensation_;
		std::atomic<bool>                                                      disable_no_motion_calibration_;

	private:
		const std::map<std::string, int> axis_direction_enum_map_
		{
            {"Undefined", static_cast<int>(hardware_interface::pigeon2::AxisDirection::Undefined)},
			{"PositiveZ", static_cast<int>(hardware_interface::pigeon2::AxisDirection::PositiveZ)},
			{"PositiveY", static_cast<int>(hardware_interface::pigeon2::AxisDirection::PositiveY)},
			{"PositiveX", static_cast<int>(hardware_interface::pigeon2::AxisDirection::PositiveX)},
			{"NegativeZ", static_cast<int>(hardware_interface::pigeon2::AxisDirection::NegativeZ)},
			{"NegativeY", static_cast<int>(hardware_interface::pigeon2::AxisDirection::NegativeY)},
			{"NegativeX", static_cast<int>(hardware_interface::pigeon2::AxisDirection::NegativeX)},
		};

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

		template <typename T>
		bool readIntoEnum(ros::NodeHandle &n,
				const std::string &param_name,
				const std::map<std::string, int> &mymap,
				std::atomic<T>&out) const
		{
			std::string param_str;
			if (!n.getParam(param_name, param_str))
			{
				return false;
			}

			const auto it = mymap.find(param_str);
			if (it != mymap.cend())
			{
				out = static_cast<T>(it->second);
				return true;
			}
			ROS_ERROR_STREAM("Could not convert param_name "
					<< param_name
					<< " param_str="
					<< param_str
					<< " didn't match list of valid types");
			return false;
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
        void zeroGyroBiasNow(void);
		void setClearStickyFaults(void);
		void setYaw(double yaw);
		void addYaw(double yaw);
		void setYawToCompass(void);
		void setAccumZAngle(void);

        // Functions which pass through to the CIParams for this controller interface
		// Only needed if normal code needs to access them?
#if 0
		void setVelocityMeasPeriod(hardware_interface::pigeon2::SensorVelocityMeasPeriod velocity_meas_period);
#endif

	private:
		Pigeon2CIParams                                   params_;
		hardware_interface::pigeon2::Pigeon2CommandHandle handle_;
		std::atomic<bool>                                 zero_gyro_bias_now_{false};
		std::atomic<bool>                                 clear_sticky_faults_{false};
		std::atomic<double>                               set_yaw_value_{};
		std::atomic<bool>                                 set_yaw_bool_{false};
		std::atomic<double>                               add_yaw_value_{};
		std::atomic<bool>                                 add_yaw_bool_{false};
		std::atomic<bool>                                 set_yaw_to_compass_{false};
		std::atomic<bool>                                 set_accum_z_angle_{false};
};

} // namespace pigeon2_controller_interfac

#endif