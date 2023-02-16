/*
 * Original joint_state_controller Author: Wim Meeussen
 */

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <pigeon2_interface/pigeon2_state_interface.h>
#include <pigeon2_state_msgs/Pigeon2State.h>
#include <periodic_interval_counter/periodic_interval_counter.h>

namespace pigeon2_state_controller
{
class Pigeon2StateController: public controller_interface::Controller<hardware_interface::pigeon2::Pigeon2StateInterface>
{
private:
		std::vector<hardware_interface::pigeon2::Pigeon2StateHandle> pigeon2_state_;
		std::unique_ptr<realtime_tools::RealtimePublisher<pigeon2_state_msgs::Pigeon2State>> realtime_pub_;
		std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
		double publish_rate_;
		size_t num_hw_joints_; ///< Number of joints present in the Pigeon2StateInterface

public:
	bool init(hardware_interface::pigeon2::Pigeon2StateInterface *hw,
			  ros::NodeHandle                                      &root_nh,
			  ros::NodeHandle                                      &controller_nh)
	{
		// get all joint names from the hardware interface
		const std::vector<std::string> &joint_names = hw->getNames();
		num_hw_joints_ = joint_names.size();
		for (size_t i = 0; i < num_hw_joints_; i++)
			ROS_DEBUG("Got joint %s", joint_names[i].c_str());

		// get publishing period
		if (!controller_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_ERROR("Parameter 'publish_rate' not set in pigeon2 state controller");
			return false;
		}
		else if (publish_rate_ <= 0.0)
		{
			ROS_ERROR_STREAM("Invalid publish rate in pigeon2 state controller (" << publish_rate_ << ")");
			return false;
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

		// realtime publisher
		realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<pigeon2_state_msgs::Pigeon2State>>(root_nh, "pigeon2_states", 2);

		auto &m = realtime_pub_->msg_;

		// get joints and allocate message
		for (size_t i = 0; i < num_hw_joints_; i++)
		{
			m.name.push_back(joint_names[i]);
			pigeon2_state_.push_back(hw->getHandle(joint_names[i]));

			m.device_number.push_back(pigeon2_state_.back()->getDeviceNumber());
			m.mount_pose_forward.push_back("");
			m.mount_pose_up.push_back("");
			m.mount_pose_roll.push_back(0.0);
			m.mount_pose_pitch.push_back(0.0);
			m.mount_pose_yaw.push_back(0.0);
			m.x_axis_gyro_error.push_back(0.0);
			m.y_axis_gyro_error.push_back(0.0);
			m.z_axis_gyro_error.push_back(0.0);
			m.compass_enable.push_back(true);
			m.disable_temperature_compensation.push_back(true);
			m.disable_no_motion_calibration.push_back(true);
			m.gravity_vector.push_back({});
			m.quaternion_6d.push_back({});
			m.yaw.push_back(0.0);
			m.pitch.push_back(0.0);
			m.roll.push_back(0.0);
			m.accum_gyro.push_back({});
			m.absolute_compass_heading.push_back(0.0);
			m.compass_heading.push_back(0.0);
			m.compass_field_strength.push_back(0.0);
			m.temperature.push_back(0.0);
			m.uptime.push_back(0);
			m.raw_magnetometer_xyz.push_back({});
			m.biased_magnetometer_xyz.push_back({});
			m.biased_accelerometer_xyz.push_back({});
			m.raw_gyro.push_back({});
			m.reset_count.push_back(0);
			m.reset_flags.push_back(0);
			m.firmware_version.push_back(0);
			m.faults.push_back("");
			m.sticky_faults.push_back("");
		}

		return true;
	}

	void starting(const ros::Time &/*time*/)
	{
		interval_counter_->reset();
	}

	std::string axisDirectionToString(hardware_interface::pigeon2::AxisDirection direction)
	{
		switch(direction)
		{
			case hardware_interface::pigeon2::AxisDirection::Undefined:
				return "Undefined";
			case hardware_interface::pigeon2::AxisDirection::PositiveX:
				return "PositiveX";
			case hardware_interface::pigeon2::AxisDirection::PositiveY:
				return "PositiveY";
			case hardware_interface::pigeon2::AxisDirection::PositiveZ:
				return "PositiveZ";
			case hardware_interface::pigeon2::AxisDirection::NegativeX:
				return "NegativeX";
			case hardware_interface::pigeon2::AxisDirection::NegativeY:
				return "NegativeY";
			case hardware_interface::pigeon2::AxisDirection::NegativeZ:
				return "NegativeZ";
			default:
				return "Unknowm";
		}
	}
	std::string faultsToString(uint64_t faults)
	{
		std::string ret;
		uint64_t mask = 1ULL;
		if (faults & mask) ret += "ResetDuringEn ";
		mask <<= 1;
		if (faults & mask) ret += "UnderVoltage ";
		mask <<= 1;
		if (faults & mask) ret += "APIError ";
		mask <<= 1;
		if (faults & mask) ret += "HardwareFault ";

		mask = 1UL << 30;

		mask <<= 1;
		if (faults & mask) ret += "AccelFault ";
		mask <<= 1;
		if (faults & mask) ret += "GyroFault ";
		mask <<= 1;
		if (faults & mask) ret += "MagnetometerFault ";
		mask <<= 1;
		if (faults & mask) ret += "BootIntoMotion ";
		mask <<= 1 + 3; // 3 unused bits
		if (faults & mask) ret += "SaturatedMag ";
		mask <<= 1;
		if (faults & mask) ret += "SaturatedAccel ";
		mask <<= 1;
		if (faults & mask) ret += "SaturatedRotVelocity ";
		return ret;
	}

	void update(const ros::Time &time, const ros::Duration & period)
	{
		// limit rate of publishing
		if (interval_counter_->update(period))
		{
			// try to publish
			if (realtime_pub_->trylock())
			{
				// populate joint state message:
				// - fill only joints that are present in the JointStateInterface, i.e. indices [0, num_hw_joints_)
				// - leave unchanged extra joints, which have static values, i.e. indices from num_hw_joints_ onwards
				auto &m = realtime_pub_->msg_;
				m.header.stamp = time;
				for (size_t i = 0; i < num_hw_joints_; i++)
				{
					auto &ps = pigeon2_state_[i];
					m.mount_pose_forward[i] = axisDirectionToString(ps->getMountPoseForward());
					m.mount_pose_up[i] = axisDirectionToString(ps->getMountPoseForward());
					m.mount_pose_roll[i] = ps->getMountPoseRoll();
					m.mount_pose_pitch[i] = ps->getMountPosePitch();
					m.mount_pose_yaw[i] = ps->getMountPoseYaw();
					m.x_axis_gyro_error[i] = ps->getXAxisGyroError();
					m.y_axis_gyro_error[i] = ps->getYAxisGyroError();
					m.z_axis_gyro_error[i] = ps->getZAxisGyroError();
					m.compass_enable[i] = ps->getCompassEnable();
					m.disable_temperature_compensation[i] = ps->getDisableTemperatureCompensation();
					m.disable_no_motion_calibration[i] = ps->getDisableNoMotionCalibration();
					const auto gravity_vector = ps->getGravityVector();
					m.gravity_vector[i].x = gravity_vector[0];
					m.gravity_vector[i].y = gravity_vector[1];
					m.gravity_vector[i].z = gravity_vector[2];
					const auto quaternion_6d = ps->get6dQuatention();
					m.quaternion_6d[i].w = quaternion_6d[0];
					m.quaternion_6d[i].x = quaternion_6d[1];
					m.quaternion_6d[i].y = quaternion_6d[2];
					m.quaternion_6d[i].z = quaternion_6d[3];
					m.yaw[i] = ps->getYaw();
					m.pitch[i] = ps->getPitch();
					m.roll[i] = ps->getRoll();
					const auto accum_gyro = ps->getAccumGyro();
					m.accum_gyro[i].x = accum_gyro[0];
					m.accum_gyro[i].y = accum_gyro[1];
					m.accum_gyro[i].z = accum_gyro[2];
					m.absolute_compass_heading[i] = ps->getAbsoluteCompassHeading();
					m.compass_heading[i] = ps->getCompassHeading();
					m.compass_field_strength[i] = ps->getCompassFieldStrength();
					m.temperature[i] = ps->getTemperature();
					m.uptime[i] = ps->getUptime();
					const auto raw_magnetometer = ps->getRawMagnetometer();
					m.raw_magnetometer_xyz[i].x = raw_magnetometer[0] * 0.6 / 1000.;
					m.raw_magnetometer_xyz[i].y = raw_magnetometer[1] * 0.6 / 1000.;
					m.raw_magnetometer_xyz[i].z = raw_magnetometer[2] * 0.6 / 1000.;
					const auto biased_magnetometer = ps->getBiasedMagnetometer();
					m.biased_magnetometer_xyz[i].x = biased_magnetometer[0] * 0.6 / 1000.;
					m.biased_magnetometer_xyz[i].y = biased_magnetometer[1] * 0.6 / 1000.;
					m.biased_magnetometer_xyz[i].z = biased_magnetometer[2] * 0.6 / 1000.;
					auto from_q2_14 = [](uint16_t in)
					{
						double ret = in >> 14;
						ret += (in & 0x3fff) / static_cast<double>(1U << 14);

						return in / 9.80665;
					};
					const auto biased_accelerometer = ps->getBiasedAccelerometer();
					m.biased_accelerometer_xyz[i].x = from_q2_14(biased_accelerometer[0]);
					m.biased_accelerometer_xyz[i].y = from_q2_14(biased_accelerometer[1]);
					m.biased_accelerometer_xyz[i].z = from_q2_14(biased_accelerometer[2]);
					const auto raw_gyro = ps->getRawGyro();
					m.raw_gyro[i].x = raw_gyro[0];
					m.raw_gyro[i].y = raw_gyro[1];
					m.raw_gyro[i].z = raw_gyro[2];

					m.reset_count[i] = ps->getResetCount();
					m.reset_flags[i] = ps->getResetFlags();
					m.firmware_version[i] = ps->getFirmwareVersion();

#if 0
					if (fw_ver >= 0)
					{
						char str[256];
						sprintf(str, "2.2%d.%2.2d", (fw_ver >> 8) & 0xFF, fw_ver & 0xFF);
						m.firmware_version[i] = str;
					}
#endif
					m.faults[i] = faultsToString(ps->getFaults());
					m.sticky_faults[i] = faultsToString(ps->getStickyFaults());
				}
				realtime_pub_->unlockAndPublish();
			}
			else
			{
				interval_counter_->force_publish();
			}
		}
	}

	void stopping(const ros::Time & /*time*/)
	{
	}

}; // class

} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pigeon2_state_controller::Pigeon2StateController, controller_interface::ControllerBase)
