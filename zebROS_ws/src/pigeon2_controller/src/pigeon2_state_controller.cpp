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
			  ros::NodeHandle                                      &controller_nh) override
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

			m.mount_pose_yaw.push_back(0);
			m.mount_pose_pitch.push_back(0);
			m.mount_pose_roll.push_back(0);
			m.gyro_trim_scalar_x.push_back(0);
			m.gyro_trim_scalar_y.push_back(0);
			m.gyro_trim_scalar_z.push_back(0);
			m.enable_compass.push_back(false);
			m.disable_temperature_compensation.push_back(false);
			m.disable_no_motion_calibration.push_back(false);
			m.no_motion_count.push_back(0);
			m.version_major.push_back(0);
			m.version_minor.push_back(0);
			m.version_bugfix.push_back(0);
			m.version_build.push_back(0);
			m.yaw.push_back(0);
			m.pitch.push_back(0);
			m.roll.push_back(0);
			m.quat_w.push_back(0);
			m.quat_x.push_back(0);
			m.quat_y.push_back(0);
			m.quat_z.push_back(0);
			m.gravity_vector_x.push_back(0);
			m.gravity_vector_y.push_back(0);
			m.gravity_vector_z.push_back(0);
			m.temperature.push_back(0);
			m.uptime.push_back(0);
			m.accum_gyro_x.push_back(0);
			m.accum_gyro_y.push_back(0);
			m.accum_gyro_z.push_back(0);
			m.angular_velocity_x.push_back(0);
			m.angular_velocity_y.push_back(0);
			m.angular_velocity_z.push_back(0);
			m.acceleration_x.push_back(0);
			m.acceleration_y.push_back(0);
			m.acceleration_z.push_back(0);
			m.supply_voltage.push_back(0);
			m.magnetic_field_x.push_back(0);
			m.magnetic_field_y.push_back(0);
			m.magnetic_field_z.push_back(0);
			m.raw_magnetic_field_x.push_back(0);
			m.raw_magnetic_field_y.push_back(0);
			m.raw_magnetic_field_z.push_back(0);

			m.fault_hardware.push_back(false);
			m.fault_undervolage.push_back(false);
			m.fault_boot_during_enable.push_back(false);
			m.fault_unlicensed_feature_in_use.push_back(false);
			m.fault_bootup_accelerometer.push_back(false);
			m.fault_bootup_gyroscope.push_back(false);
			m.fault_bootup_magnetometer.push_back(false);
			m.fault_boot_into_motion.push_back(false);
			m.fault_data_acquired_late.push_back(false);
			m.fault_loop_time_slow.push_back(false);
			m.fault_saturated_magnetometer.push_back(false);
			m.fault_saturated_accelerometer.push_back(false);
			m.fault_saturated_gyroscope.push_back(false);

			m.sticky_fault_hardware.push_back(false);
			m.sticky_fault_undervolage.push_back(false);
			m.sticky_fault_boot_during_enable.push_back(false);
			m.sticky_fault_unlicensed_feature_in_use.push_back(false);
			m.sticky_fault_bootup_accelerometer.push_back(false);
			m.sticky_fault_bootup_gyroscope.push_back(false);
			m.sticky_fault_bootup_magnetometer.push_back(false);
			m.sticky_fault_boot_into_motion.push_back(false);
			m.sticky_fault_data_acquired_late.push_back(false);
			m.sticky_fault_loop_time_slow.push_back(false);
			m.sticky_fault_saturated_magnetometer.push_back(false);
			m.sticky_fault_saturated_accelerometer.push_back(false);
			m.sticky_fault_saturated_gyroscope.push_back(false);

		}

		return true;
	}

	void starting(const ros::Time &/*time*/) override
	{
		interval_counter_->reset();
	}

	void update(const ros::Time &time, const ros::Duration & period) override
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
					const auto &ps = pigeon2_state_[i];

					m.mount_pose_yaw[i] = ps->getMountPoseYaw();
					m.mount_pose_pitch[i] = ps->getMountPosePitch();
					m.mount_pose_roll[i] = ps->getMountPoseRoll();
					m.gyro_trim_scalar_x[i] = ps->getGyroTrimScalarX();
					m.gyro_trim_scalar_y[i] = ps->getGyroTrimScalarY();
					m.gyro_trim_scalar_z[i] = ps->getGyroTrimScalarZ();
					m.enable_compass[i] = ps->getEnableCompass();
					m.disable_temperature_compensation[i] = ps->getDisableTemperatureCompensation();
					m.disable_no_motion_calibration[i] = ps->getDisableNoMotionCalibration();
					m.no_motion_count[i] = ps->getNoMotionCount();
					m.version_major[i] = ps->getVersionMajor();
					m.version_minor[i] = ps->getVersionMinor();
					m.version_bugfix[i] = ps->getVersionBugfix();
					m.version_build[i] = ps->getVersionBuild();
					m.yaw[i] = ps->getYaw();
					m.pitch[i] = ps->getPitch();
					m.roll[i] = ps->getRoll();
					m.quat_w[i] = ps->getQuatW();
					m.quat_x[i] = ps->getQuatX();
					m.quat_y[i] = ps->getQuatY();
					m.quat_z[i] = ps->getQuatZ();
					m.gravity_vector_x[i] = ps->getGravityVectorX();
					m.gravity_vector_y[i] = ps->getGravityVectorY();
					m.gravity_vector_z[i] = ps->getGravityVectorZ();
					m.temperature[i] = ps->getTemperature();
					m.uptime[i] = ps->getUptime();
					m.accum_gyro_x[i] = ps->getAccumGyroX();
					m.accum_gyro_y[i] = ps->getAccumGyroY();
					m.accum_gyro_z[i] = ps->getAccumGyroZ();
					m.angular_velocity_x[i] = ps->getAngularVelocityX();
					m.angular_velocity_y[i] = ps->getAngularVelocityY();
					m.angular_velocity_z[i] = ps->getAngularVelocityZ();
					m.acceleration_x[i] = ps->getAccelerationX();
					m.acceleration_y[i] = ps->getAccelerationY();
					m.acceleration_z[i] = ps->getAccelerationZ();
					m.supply_voltage[i] = ps->getSupplyVoltage();
					m.magnetic_field_x[i] = ps->getMagneticFieldX();
					m.magnetic_field_y[i] = ps->getMagneticFieldY();
					m.magnetic_field_z[i] = ps->getMagneticFieldZ();
					m.raw_magnetic_field_x[i] = ps->getRawMagneticFieldX();
					m.raw_magnetic_field_y[i] = ps->getRawMagneticFieldY();
					m.raw_magnetic_field_z[i] = ps->getRawMagneticFieldZ();

					m.fault_hardware[i] = ps->getFaultHardware();
					m.fault_undervolage[i] = ps->getFaultUndervoltage();
					m.fault_boot_during_enable[i] = ps->getFaultBootDuringEnable();
					m.fault_unlicensed_feature_in_use[i] = ps->getFaultUnlicensedFeatureInUse();
					m.fault_bootup_accelerometer[i] = ps->getFaultBootupAccelerometer();
					m.fault_bootup_gyroscope[i] = ps->getFaultBootupGyroscope();
					m.fault_bootup_magnetometer[i] = ps->getFaultBootupMagnetometer();
					m.fault_boot_into_motion[i] = ps->getFaultBootIntoMotion();
					m.fault_data_acquired_late[i] = ps->getFaultDataAcquiredLate();
					m.fault_loop_time_slow[i] = ps->getFaultLoopTimeSlow();
					m.fault_saturated_magnetometer[i] = ps->getFaultSaturatedMagnetometer();
					m.fault_saturated_accelerometer[i] = ps->getFaultSaturatedAccelometer();
					m.fault_saturated_gyroscope[i] = ps->getFaultSaturatedGyroscope();

					m.sticky_fault_hardware[i] = ps->getStickyFaultHardware();
					m.sticky_fault_undervolage[i] = ps->getStickyFaultUndervoltage();
					m.sticky_fault_boot_during_enable[i] = ps->getStickyFaultBootDuringEnable();
					m.sticky_fault_unlicensed_feature_in_use[i] = ps->getStickyFaultUnlicensedFeatureInUse();
					m.sticky_fault_bootup_accelerometer[i] = ps->getStickyFaultBootupAccelerometer();
					m.sticky_fault_bootup_gyroscope[i] = ps->getStickyFaultBootupGyroscope();
					m.sticky_fault_bootup_magnetometer[i] = ps->getStickyFaultBootupMagnetometer();
					m.sticky_fault_boot_into_motion[i] = ps->getStickyFaultBootIntoMotion();
					m.sticky_fault_data_acquired_late[i] = ps->getStickyFaultDataAcquiredLate();
					m.sticky_fault_loop_time_slow[i] = ps->getStickyFaultLoopTimeSlow();
					m.sticky_fault_saturated_magnetometer[i] = ps->getStickyFaultSaturatedMagnetometer();
					m.sticky_fault_saturated_accelerometer[i] = ps->getStickyFaultSaturatedAccelometer();
					m.sticky_fault_saturated_gyroscope[i] = ps->getStickyFaultSaturatedGyroscope();
				}
				realtime_pub_->unlockAndPublish();
			}
			else
			{
				interval_counter_->force_publish();
			}
		}
	}

	void stopping(const ros::Time & /*time*/) override
	{
	}

}; // class

} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pigeon2_state_controller::Pigeon2StateController, controller_interface::ControllerBase)
