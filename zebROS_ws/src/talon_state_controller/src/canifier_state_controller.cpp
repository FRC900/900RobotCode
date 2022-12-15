/*
 * Original joint_state_controller Author: Wim Meeussen
 */

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <talon_interface/canifier_state_interface.h>
#include <talon_state_msgs/CANifierState.h>
#include <periodic_interval_counter/periodic_interval_counter.h>
namespace canifier_state_controller
{

/**
 * \brief Controller that publishes the state of all talon&victor motor controller on a robot.
 *
 * This controller publishes the state of all resources registered to a \c hardware_interface::CANifierStateInterface to a
 * topic of type \c sensor_msgs/CANifierState. The following is a basic configuration of the controller.
 *
 * \code
 * canifier_state_controller:
 *   type: canifier_state_controller/JointStateController
 *   publish_rate: 50
 * \endcode
 *
 */
class CANifierStateController: public controller_interface::Controller<hardware_interface::canifier::CANifierStateInterface>
{

private:
	std::vector<hardware_interface::canifier::CANifierStateHandle> canifier_state_;
	std::unique_ptr<realtime_tools::RealtimePublisher<talon_state_msgs::CANifierState>> realtime_pub_;
	std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
	double publish_rate_;
	size_t num_hw_joints_; ///< Number of joints present in the CANifierStateInterface

public:	
	bool init(hardware_interface::canifier::CANifierStateInterface *hw,
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
			ROS_ERROR("Parameter 'publish_rate' not set in canifier state controller");
			return false;
		}
		else if (publish_rate_ <= 0.0)
		{
			ROS_ERROR_STREAM("Invalid publish rate in canifier state controller (" << publish_rate_ << ")");
			return false;
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

		// realtime publisher
		realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<talon_state_msgs::CANifierState>>(root_nh, "canifier_states", 2);

		auto &m = realtime_pub_->msg_;

		// get joints and allocate message
		for (size_t i = 0; i < num_hw_joints_; i++)
		{
			m.name.push_back(joint_names[i]);

			m.can_id.push_back(0);
			m.led_output_a.push_back(0);
			m.led_output_b.push_back(0);
			m.led_output_c.push_back(0);
			m.general_pin_output_enable_quad_idx.push_back(false);
			m.general_pin_output_enable_quad_b.push_back(false);
			m.general_pin_output_enable_quad_a.push_back(false);
			m.general_pin_output_enable_limr.push_back(false);
			m.general_pin_output_enable_limf.push_back(false);
			m.general_pin_output_enable_sda.push_back(false);
			m.general_pin_output_enable_scl.push_back(false);
			m.general_pin_output_enable_spi_cs_pwm3.push_back(false);
			m.general_pin_output_enable_spi_miso_pwm2.push_back(false);
			m.general_pin_output_enable_spi_mosi_pwm1.push_back(false);
			m.general_pin_output_enable_spi_clk_pwm0.push_back(false);
			m.general_pin_output_quad_idx.push_back(false);
			m.general_pin_output_quad_b.push_back(false);
			m.general_pin_output_quad_a.push_back(false);
			m.general_pin_output_limr.push_back(false);
			m.general_pin_output_limf.push_back(false);
			m.general_pin_output_sda.push_back(false);
			m.general_pin_output_scl.push_back(false);
			m.general_pin_output_spi_cs_pwm3.push_back(false);
			m.general_pin_output_spi_miso_pwm2.push_back(false);
			m.general_pin_output_spi_mosi_pwm1.push_back(false);
			m.general_pin_output_spi_clk_pwm0.push_back(false);
			m.general_pin_input_quad_idx.push_back(false);
			m.general_pin_input_quad_b.push_back(false);
			m.general_pin_input_quad_a.push_back(false);
			m.general_pin_input_limr.push_back(false);
			m.general_pin_input_limf.push_back(false);
			m.general_pin_input_sda.push_back(false);
			m.general_pin_input_scl.push_back(false);
			m.general_pin_input_spi_cs_pwm3.push_back(false);
			m.general_pin_input_spi_miso_pwm2.push_back(false);
			m.general_pin_input_spi_mosi_pwm1.push_back(false);
			m.general_pin_input_spi_clk_pwm0.push_back(false);
			m.quadrature_position.push_back(0);
			m.quadrature_velocity.push_back(0);
			m.velocity_measurement_period.push_back(0);
			m.velocity_measurement_window.push_back(0);
			m.clear_position_on_limit_f.push_back(false);
			m.clear_position_on_limit_r.push_back(false);
			m.clear_position_on_quad_idx.push_back(false);
			m.bus_voltage.push_back(0);
			m.pwm_output_enable_0.push_back(false);
			m.pwm_output_enable_1.push_back(false);
			m.pwm_output_enable_2.push_back(false);
			m.pwm_output_enable_3.push_back(false);
			m.pwm_output_0.push_back(0);
			m.pwm_output_1.push_back(0);
			m.pwm_output_2.push_back(0);
			m.pwm_output_3.push_back(0);
			m.pwm_input_0_duty_cycle.push_back(0);
			m.pwm_input_0_period.push_back(0);
			m.pwm_input_1_duty_cycle.push_back(0);
			m.pwm_input_1_period.push_back(0);
			m.pwm_input_2_duty_cycle.push_back(0);
			m.pwm_input_2_period.push_back(0);
			m.pwm_input_3_duty_cycle.push_back(0);
			m.pwm_input_3_period.push_back(0);
			m.status_1_general_period.push_back(0);
			m.status_2_general_period.push_back(0);
			m.status_3_pwminputs0_period.push_back(0);
			m.status_4_pwminputs1_period.push_back(0);
			m.status_5_pwminputs2_period.push_back(0);
			m.status_6_pwminputs3_period.push_back(0);
			m.status_8_misc_period.push_back(0);
			m.control_1_general_period.push_back(0);
			m.control_2_pwmoutput_period.push_back(0);
			m.faults.push_back(0);
			m.sticky_faults.push_back(0);
			m.encoder_ticks_per_rotation.push_back(0);
			m.conversion_factor.push_back(0);

			canifier_state_.push_back(hw->getHandle(joint_names[i]));
		}

		return true;
	}

	void starting(const ros::Time &time)
	{
		interval_counter_->reset();
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
					auto &cs = canifier_state_[i];
					m.can_id[i] = cs->getCANId();
					m.led_output_a[i] = cs->getLEDOutput(hardware_interface::canifier::LEDChannel::LEDChannelA);
					m.led_output_b[i] = cs->getLEDOutput(hardware_interface::canifier::LEDChannel::LEDChannelB);
					m.led_output_c[i] = cs->getLEDOutput(hardware_interface::canifier::LEDChannel::LEDChannelC);
					m.general_pin_output_enable_quad_idx[i] = cs->getGeneralPinOutputEnable(hardware_interface::canifier::GeneralPin::QUAD_IDX);
					m.general_pin_output_enable_quad_b[i] = cs->getGeneralPinOutputEnable(hardware_interface::canifier::GeneralPin::QUAD_B);
					m.general_pin_output_enable_quad_a[i] = cs->getGeneralPinOutputEnable(hardware_interface::canifier::GeneralPin::QUAD_A);
					m.general_pin_output_enable_limr[i] = cs->getGeneralPinOutputEnable(hardware_interface::canifier::GeneralPin::LIMR);
					m.general_pin_output_enable_limf[i] = cs->getGeneralPinOutputEnable(hardware_interface::canifier::GeneralPin::LIMF);
					m.general_pin_output_enable_sda[i] = cs->getGeneralPinOutputEnable(hardware_interface::canifier::GeneralPin::SDA);
					m.general_pin_output_enable_scl[i] = cs->getGeneralPinOutputEnable(hardware_interface::canifier::GeneralPin::SCL);
					m.general_pin_output_enable_spi_cs_pwm3[i] = cs->getGeneralPinOutputEnable(hardware_interface::canifier::GeneralPin::SPI_CS);
					m.general_pin_output_enable_spi_miso_pwm2[i] = cs->getGeneralPinOutputEnable(hardware_interface::canifier::GeneralPin::SPI_MISO_PWM2P);
					m.general_pin_output_enable_spi_mosi_pwm1[i] = cs->getGeneralPinOutputEnable(hardware_interface::canifier::GeneralPin::SPI_MOSI_PWM1P);
					m.general_pin_output_enable_spi_clk_pwm0[i] = cs->getGeneralPinOutputEnable(hardware_interface::canifier::GeneralPin::SPI_CLK_PWM0P);
					m.general_pin_output_quad_idx[i] = cs->getGeneralPinOutput(hardware_interface::canifier::GeneralPin::QUAD_IDX);
					m.general_pin_output_quad_b[i] = cs->getGeneralPinOutput(hardware_interface::canifier::GeneralPin::QUAD_B);
					m.general_pin_output_quad_a[i] = cs->getGeneralPinOutput(hardware_interface::canifier::GeneralPin::QUAD_A);
					m.general_pin_output_limr[i] = cs->getGeneralPinOutput(hardware_interface::canifier::GeneralPin::LIMR);
					m.general_pin_output_limf[i] = cs->getGeneralPinOutput(hardware_interface::canifier::GeneralPin::LIMF);
					m.general_pin_output_sda[i] = cs->getGeneralPinOutput(hardware_interface::canifier::GeneralPin::SDA);
					m.general_pin_output_scl[i] = cs->getGeneralPinOutput(hardware_interface::canifier::GeneralPin::SCL);
					m.general_pin_output_spi_cs_pwm3[i] = cs->getGeneralPinOutput(hardware_interface::canifier::GeneralPin::SPI_CS);
					m.general_pin_output_spi_miso_pwm2[i] = cs->getGeneralPinOutput(hardware_interface::canifier::GeneralPin::SPI_MISO_PWM2P);
					m.general_pin_output_spi_mosi_pwm1[i] = cs->getGeneralPinOutput(hardware_interface::canifier::GeneralPin::SPI_MOSI_PWM1P);
					m.general_pin_output_spi_clk_pwm0[i] = cs->getGeneralPinOutput(hardware_interface::canifier::GeneralPin::SPI_CLK_PWM0P);
					m.general_pin_input_quad_idx[i] = cs->getGeneralPinInput(hardware_interface::canifier::GeneralPin::QUAD_IDX);
					m.general_pin_input_quad_b[i] = cs->getGeneralPinInput(hardware_interface::canifier::GeneralPin::QUAD_B);
					m.general_pin_input_quad_a[i] = cs->getGeneralPinInput(hardware_interface::canifier::GeneralPin::QUAD_A);
					m.general_pin_input_limr[i] = cs->getGeneralPinInput(hardware_interface::canifier::GeneralPin::LIMR);
					m.general_pin_input_limf[i] = cs->getGeneralPinInput(hardware_interface::canifier::GeneralPin::LIMF);
					m.general_pin_input_sda[i] = cs->getGeneralPinInput(hardware_interface::canifier::GeneralPin::SDA);
					m.general_pin_input_scl[i] = cs->getGeneralPinInput(hardware_interface::canifier::GeneralPin::SCL);
					m.general_pin_input_spi_cs_pwm3[i] = cs->getGeneralPinInput(hardware_interface::canifier::GeneralPin::SPI_CS);
					m.general_pin_input_spi_miso_pwm2[i] = cs->getGeneralPinInput(hardware_interface::canifier::GeneralPin::SPI_MISO_PWM2P);
					m.general_pin_input_spi_mosi_pwm1[i] = cs->getGeneralPinInput(hardware_interface::canifier::GeneralPin::SPI_MOSI_PWM1P);
					m.general_pin_input_spi_clk_pwm0[i] = cs->getGeneralPinInput(hardware_interface::canifier::GeneralPin::SPI_CLK_PWM0P);
					m.quadrature_position[i] = cs->getQuadraturePosition();
					m.quadrature_velocity[i] = cs->getQuadratureVelocity();
					switch (cs->getVelocityMeasurementPeriod())
					{
						case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_1Ms:
							m.velocity_measurement_period[i] = 1;
							break;
						case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_2Ms:
							m.velocity_measurement_period[i] = 2;
							break;
						case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_5Ms:
							m.velocity_measurement_period[i] = 5;
							break;
						case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_10Ms:
							m.velocity_measurement_period[i] = 10;
							break;
						case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_20Ms:
							m.velocity_measurement_period[i] = 20;
							break;
						case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_25Ms:
							m.velocity_measurement_period[i] = 25;
							break;
						case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_50Ms:
							m.velocity_measurement_period[i] = 50;
							break;
						case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_100Ms:
							m.velocity_measurement_period[i] = 100;
							break;
						default :
							ROS_ERROR_STREAM("Unknown CANifier velocity measurement period");
							m.velocity_measurement_period[i] = 0;
					}
					m.velocity_measurement_window[i] = cs->getVelocityMeasurementWindow();
					m.clear_position_on_limit_f[i] = cs->getClearPositionOnLimitF();
					m.clear_position_on_limit_r[i] = cs->getClearPositionOnLimitR();
					m.clear_position_on_quad_idx[i] = cs->getClearPositionOnQuadIdx();
					m.bus_voltage[i] = cs->getBusVoltage();
					m.pwm_output_enable_0[i] = cs->getPWMOutputEnable(hardware_interface::canifier::PWMChannel::PWMChannel0);
					m.pwm_output_enable_1[i] = cs->getPWMOutputEnable(hardware_interface::canifier::PWMChannel::PWMChannel1);
					m.pwm_output_enable_2[i] = cs->getPWMOutputEnable(hardware_interface::canifier::PWMChannel::PWMChannel2);
					m.pwm_output_enable_3[i] = cs->getPWMOutputEnable(hardware_interface::canifier::PWMChannel::PWMChannel3);
					m.pwm_output_0[i] = cs->getPWMOutput(hardware_interface::canifier::PWMChannel::PWMChannel0);
					m.pwm_output_1[i] = cs->getPWMOutput(hardware_interface::canifier::PWMChannel::PWMChannel1);
					m.pwm_output_2[i] = cs->getPWMOutput(hardware_interface::canifier::PWMChannel::PWMChannel2);
					m.pwm_output_3[i] = cs->getPWMOutput(hardware_interface::canifier::PWMChannel::PWMChannel3);
					const auto pwm_input_0 = cs->getPWMInput(hardware_interface::canifier::PWMChannel::PWMChannel0);
					m.pwm_input_0_duty_cycle[i] = pwm_input_0[0];
					m.pwm_input_0_period[i] = pwm_input_0[1];
					const auto pwm_input_1 = cs->getPWMInput(hardware_interface::canifier::PWMChannel::PWMChannel1);
					m.pwm_input_1_duty_cycle[i] = pwm_input_1[0];
					m.pwm_input_1_period[i] = pwm_input_1[1];
					const auto pwm_input_2 = cs->getPWMInput(hardware_interface::canifier::PWMChannel::PWMChannel2);
					m.pwm_input_2_duty_cycle[i] = pwm_input_2[0];
					m.pwm_input_2_period[i] = pwm_input_2[1];
					const auto pwm_input_3 = cs->getPWMInput(hardware_interface::canifier::PWMChannel::PWMChannel3);
					m.pwm_input_3_duty_cycle[i] = pwm_input_3[0];
					m.pwm_input_3_period[i] = pwm_input_3[1];
					m.status_1_general_period[i] = cs->getStatusFramePeriod(hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_1_General);
					m.status_2_general_period[i] = cs->getStatusFramePeriod(hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_2_General);
					m.status_3_pwminputs0_period[i] = cs->getStatusFramePeriod(hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_3_PwmInputs0);
					m.status_4_pwminputs1_period[i] = cs->getStatusFramePeriod(hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_4_PwmInputs1);
					m.status_5_pwminputs2_period[i] = cs->getStatusFramePeriod(hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_5_PwmInputs2);
					m.status_6_pwminputs3_period[i] = cs->getStatusFramePeriod(hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_6_PwmInputs3);
					m.status_8_misc_period[i] = cs->getStatusFramePeriod(hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_8_Misc);
					m.control_1_general_period[i] = cs->getControlFramePeriod(hardware_interface::canifier::CANifier_Control_1_General);
					m.control_2_pwmoutput_period[i] = cs->getControlFramePeriod(hardware_interface::canifier::CANifier_Control_2_PwmOutput);
					m.faults[i] = cs->getFaults();
					m.sticky_faults[i] = cs->getStickyFaults();
					m.encoder_ticks_per_rotation[i] = cs->getEncoderTicksPerRotation();
					m.conversion_factor[i] = cs->getConversionFactor();
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
	{}

}; // class

} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(canifier_state_controller::CANifierStateController, controller_interface::ControllerBase)