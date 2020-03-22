#include "canifier_controller/canifier_controller_interface.h"

namespace canifier_controller_interface
{
CANifierCIParams::CANifierCIParams(ros::NodeHandle n)
	: ddr_(n)
{
	// First set defaults for all params
	for (auto &lo : led_output_)
		lo = 0;
	for (auto &gpoe : general_pin_output_enable_)
		gpoe = false;
	for (auto &gp : general_pin_output_)
		gp = false;

	velocity_measurement_period_ = hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_100Ms;
	velocity_measurement_window_ = 64;
	clear_position_on_limit_f_   = false;
	clear_position_on_limit_r_   = false;
	clear_position_on_quad_idx_  = false;
	// Disable pwm output by defalt
	for (auto &pe : pwm_output_enable_)
		pe = false;
	for (auto &po : pwm_output_)
		po = 0.0;

	status_frame_period_[hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_1_General] = hardware_interface::canifier::canifierstatusframe_status_1_general_default;
	status_frame_period_[hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_2_General] = hardware_interface::canifier::canifierstatusframe_status_2_general_default;
	status_frame_period_[hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_3_PwmInputs0] = hardware_interface::canifier::canifierstatusframe_status_3_pwminputs0_default;
	status_frame_period_[hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_4_PwmInputs1] = hardware_interface::canifier::canifierstatusframe_status_4_pwminputs1_default;
	status_frame_period_[hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_5_PwmInputs2] = hardware_interface::canifier::canifierstatusframe_status_5_pwminputs2_default;
	status_frame_period_[hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_6_PwmInputs3] = hardware_interface::canifier::canifierstatusframe_status_6_pwminputs3_default;
	status_frame_period_[hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_8_Misc] = hardware_interface::canifier::canifierstatusframe_status_8_misc_default;
	control_frame_period_[hardware_interface::canifier::CANifierControlFrame::CANifier_Control_1_General] = hardware_interface::canifier::canifier_control_1_general_default;
	control_frame_period_[hardware_interface::canifier::CANifierControlFrame::CANifier_Control_2_PwmOutput] = hardware_interface::canifier::canifier_control_2_pwmoutput_default;

	// Then read in values from config file
	readIntoArray(n, "led_channela_output", hardware_interface::canifier::LEDChannel::LEDChannelA, led_output_);
	readIntoArray(n, "led_channelb_output", hardware_interface::canifier::LEDChannel::LEDChannelB, led_output_);
	readIntoArray(n, "led_channelc_output", hardware_interface::canifier::LEDChannel::LEDChannelC, led_output_);

	readIntoArray(n, "quadidx_output_enable", hardware_interface::canifier::GeneralPin::QUAD_IDX, general_pin_output_enable_);
	readIntoArray(n, "quadb_output_enable", hardware_interface::canifier::GeneralPin::QUAD_B, general_pin_output_enable_);
	readIntoArray(n, "quada_output_enable", hardware_interface::canifier::GeneralPin::QUAD_A, general_pin_output_enable_);
	readIntoArray(n, "limr_output_enable", hardware_interface::canifier::GeneralPin::LIMR, general_pin_output_enable_);
	readIntoArray(n, "limf_output_enable", hardware_interface::canifier::GeneralPin::LIMF, general_pin_output_enable_);
	readIntoArray(n, "sda_output_enable", hardware_interface::canifier::GeneralPin::SDA, general_pin_output_enable_);
	readIntoArray(n, "scl_output_enable", hardware_interface::canifier::GeneralPin::SCL, general_pin_output_enable_);
	readIntoArray(n, "spics_output_enable", hardware_interface::canifier::GeneralPin::SPI_CS, general_pin_output_enable_);
	readIntoArray(n, "spimisopwm2p_output_enable", hardware_interface::canifier::GeneralPin::SPI_MISO_PWM2P, general_pin_output_enable_);
	readIntoArray(n, "spimosipwm1p_output_enable", hardware_interface::canifier::GeneralPin::SPI_MOSI_PWM1P, general_pin_output_enable_);
	readIntoArray(n, "spiclkpwm0p_output_enable", hardware_interface::canifier::GeneralPin::SPI_CLK_PWM0P, general_pin_output_enable_);
	readIntoArray(n, "quadidx_output", hardware_interface::canifier::GeneralPin::QUAD_IDX, general_pin_output_);
	readIntoArray(n, "quadb_output", hardware_interface::canifier::GeneralPin::QUAD_B, general_pin_output_);
	readIntoArray(n, "quada_output", hardware_interface::canifier::GeneralPin::QUAD_A, general_pin_output_);
	readIntoArray(n, "limr_output", hardware_interface::canifier::GeneralPin::LIMR, general_pin_output_);
	readIntoArray(n, "limf_output", hardware_interface::canifier::GeneralPin::LIMF, general_pin_output_);
	readIntoArray(n, "sda_output", hardware_interface::canifier::GeneralPin::SDA, general_pin_output_);
	readIntoArray(n, "scl_output", hardware_interface::canifier::GeneralPin::SCL, general_pin_output_);
	readIntoArray(n, "spics_output", hardware_interface::canifier::GeneralPin::SPI_CS, general_pin_output_);
	readIntoArray(n, "spimisopwm2p_output", hardware_interface::canifier::GeneralPin::SPI_MISO_PWM2P, general_pin_output_);
	readIntoArray(n, "spimosipwm1p_output", hardware_interface::canifier::GeneralPin::SPI_MOSI_PWM1P, general_pin_output_);
	readIntoArray(n, "spiclkpwm0p_output", hardware_interface::canifier::GeneralPin::SPI_CLK_PWM0P, general_pin_output_);

	readIntoEnum(n, "velocity_measurement_period_", velocity_measurement_period_enum_map_, velocity_measurement_period_);
	readIntoScalar(n, "velocity_measurement_window_", velocity_measurement_window_);
	readIntoScalar(n, "clear_position_on_limit_f_", clear_position_on_limit_f_);
	readIntoScalar(n, "clear_position_on_limit_r_", clear_position_on_limit_r_);
	readIntoScalar(n, "clear_position_on_quad_idx_" ,clear_position_on_quad_idx_);

	readIntoArray(n, "pwm_channel0_output_enable", hardware_interface::canifier::PWMChannel::PWMChannel0, pwm_output_enable_);
	readIntoArray(n, "pwm_channel1_output_enable", hardware_interface::canifier::PWMChannel::PWMChannel1, pwm_output_enable_);
	readIntoArray(n, "pwm_channel2_output_enable", hardware_interface::canifier::PWMChannel::PWMChannel2, pwm_output_enable_);
	readIntoArray(n, "pwm_channel3_output_enable", hardware_interface::canifier::PWMChannel::PWMChannel3, pwm_output_enable_);
	readIntoArray(n, "pwm_channel0_output", hardware_interface::canifier::PWMChannel::PWMChannel0, pwm_output_);
	readIntoArray(n, "pwm_channel1_output", hardware_interface::canifier::PWMChannel::PWMChannel1, pwm_output_);
	readIntoArray(n, "pwm_channel2_output", hardware_interface::canifier::PWMChannel::PWMChannel2, pwm_output_);
	readIntoArray(n, "pwm_channel3_output", hardware_interface::canifier::PWMChannel::PWMChannel3, pwm_output_);

	readIntoArray(n, "status_1_general_period", hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_1_General, status_frame_period_);
	readIntoArray(n, "status_2_general_period", hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_2_General, status_frame_period_);
	readIntoArray(n, "status_3_pwminputs0_period", hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_3_PwmInputs0, status_frame_period_);
	readIntoArray(n, "status_4_pwminputs1_period", hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_4_PwmInputs1, status_frame_period_);
	readIntoArray(n, "status_5_pwminputs2_period", hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_5_PwmInputs2, status_frame_period_);
	readIntoArray(n, "status_6_pwminputs3_period", hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_6_PwmInputs3, status_frame_period_);
	readIntoArray(n, "status_8_misc_period", hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_8_Misc, status_frame_period_);

	readIntoArray(n, "control_1_general_period", hardware_interface::canifier::CANifierControlFrame::CANifier_Control_1_General, control_frame_period_);
	readIntoArray(n, "control_2_pwm_output_period", hardware_interface::canifier::CANifierControlFrame::CANifier_Control_2_PwmOutput, control_frame_period_);

	// Then hook them up to dynamic reconfigure options
	ddr_.registerVariable<double>("led_channela_output", boost::bind(&CANifierCIParams::getLEDOutput, this, hardware_interface::canifier::LEDChannel::LEDChannelA), boost::bind(&CANifierCIParams::setLEDOutput, this, hardware_interface::canifier::LEDChannel::LEDChannelA, _1, false), "LED Channel A output", 0.0, 1.0);
	ddr_.registerVariable<double>("led_channelb_output", boost::bind(&CANifierCIParams::getLEDOutput, this, hardware_interface::canifier::LEDChannel::LEDChannelB), boost::bind(&CANifierCIParams::setLEDOutput, this, hardware_interface::canifier::LEDChannel::LEDChannelB, _1, false), "LED Channel B output", 0.0, 1.0);
	ddr_.registerVariable<double>("led_channelc_output", boost::bind(&CANifierCIParams::getLEDOutput, this, hardware_interface::canifier::LEDChannel::LEDChannelC), boost::bind(&CANifierCIParams::setLEDOutput, this, hardware_interface::canifier::LEDChannel::LEDChannelC, _1, false), "LED Channel C output", 0.0, 1.0);
	ddr_.registerVariable<bool>("quadidx_output_enable", boost::bind(&CANifierCIParams::getGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::QUAD_IDX), boost::bind(&CANifierCIParams::setGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::QUAD_IDX, _1, false), "General Pins QUAD_IDX output enable");
	ddr_.registerVariable<bool>("quadb_output_enable", boost::bind(&CANifierCIParams::getGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::QUAD_B), boost::bind(&CANifierCIParams::setGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::QUAD_B, _1, false), "General Pins QUAD_B output enable");
	ddr_.registerVariable<bool>("quada_output_enable", boost::bind(&CANifierCIParams::getGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::QUAD_A), boost::bind(&CANifierCIParams::setGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::QUAD_A, _1, false), "General Pins QUAD_A output enable");
	ddr_.registerVariable<bool>("limr_output_enable", boost::bind(&CANifierCIParams::getGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::LIMR), boost::bind(&CANifierCIParams::setGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::LIMR, _1, false), "General Pins limr output enable");
	ddr_.registerVariable<bool>("limf_output_enable", boost::bind(&CANifierCIParams::getGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::LIMF), boost::bind(&CANifierCIParams::setGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::LIMF, _1, false), "General Pins limf output enable");
	ddr_.registerVariable<bool>("sda_output_enable", boost::bind(&CANifierCIParams::getGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::SDA), boost::bind(&CANifierCIParams::setGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::SDA, _1, false), "General Pins SDA output enable");
	ddr_.registerVariable<bool>("scl_output_enable", boost::bind(&CANifierCIParams::getGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::SCL), boost::bind(&CANifierCIParams::setGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::SCL, _1, false), "General Pins SCL output enable");
	ddr_.registerVariable<bool>("spics_output_enable", boost::bind(&CANifierCIParams::getGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::SPI_CS), boost::bind(&CANifierCIParams::setGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::SPI_CS, _1, false), "General Pins SPI_CS output enable");
	ddr_.registerVariable<bool>("spimisopwm2p_output_enable", boost::bind(&CANifierCIParams::getGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::SPI_MISO_PWM2P), boost::bind(&CANifierCIParams::setGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::SPI_MISO_PWM2P, _1, false), "General Pins SPI?MISO_PWM2P output enable");
	ddr_.registerVariable<bool>("spimosipwm1p_output_enable", boost::bind(&CANifierCIParams::getGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::SPI_MOSI_PWM1P), boost::bind(&CANifierCIParams::setGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::SPI_MOSI_PWM1P, _1, false), "General Pins SCL output enable");
	ddr_.registerVariable<bool>("spiclkpwm0p_output_enable", boost::bind(&CANifierCIParams::getGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::SPI_CLK_PWM0P), boost::bind(&CANifierCIParams::setGeneralPinOutputEnable, this, hardware_interface::canifier::GeneralPin::SPI_CLK_PWM0P, _1, false), "General Pins SCL output enable");
	ddr_.registerVariable<bool>("quadidx_output", boost::bind(&CANifierCIParams::getGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::QUAD_IDX), boost::bind(&CANifierCIParams::setGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::QUAD_IDX, _1, false), "General Pins QUAD_IDX output ");
	ddr_.registerVariable<bool>("quadb_output", boost::bind(&CANifierCIParams::getGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::QUAD_B), boost::bind(&CANifierCIParams::setGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::QUAD_B, _1, false), "General Pins QUAD_B output ");
	ddr_.registerVariable<bool>("quada_output", boost::bind(&CANifierCIParams::getGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::QUAD_A), boost::bind(&CANifierCIParams::setGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::QUAD_A, _1, false), "General Pins QUAD_A output ");
	ddr_.registerVariable<bool>("limr_output", boost::bind(&CANifierCIParams::getGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::LIMR), boost::bind(&CANifierCIParams::setGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::LIMR, _1, false), "General Pins limr output ");
	ddr_.registerVariable<bool>("limf_output", boost::bind(&CANifierCIParams::getGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::LIMF), boost::bind(&CANifierCIParams::setGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::LIMF, _1, false), "General Pins limf output ");
	ddr_.registerVariable<bool>("sda_output", boost::bind(&CANifierCIParams::getGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::SDA), boost::bind(&CANifierCIParams::setGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::SDA, _1, false), "General Pins SDA output ");
	ddr_.registerVariable<bool>("scl_output", boost::bind(&CANifierCIParams::getGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::SCL), boost::bind(&CANifierCIParams::setGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::SCL, _1, false), "General Pins SCL output ");
	ddr_.registerVariable<bool>("spics_output", boost::bind(&CANifierCIParams::getGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::SPI_CS), boost::bind(&CANifierCIParams::setGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::SPI_CS, _1, false), "General Pins SPI_CS output ");
	ddr_.registerVariable<bool>("spimisopwm2p_output", boost::bind(&CANifierCIParams::getGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::SPI_MISO_PWM2P), boost::bind(&CANifierCIParams::setGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::SPI_MISO_PWM2P, _1, false), "General Pins SPI?MISO_PWM2P output ");
	ddr_.registerVariable<bool>("spimosipwm1p_output", boost::bind(&CANifierCIParams::getGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::SPI_MOSI_PWM1P), boost::bind(&CANifierCIParams::setGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::SPI_MOSI_PWM1P, _1, false), "General Pins SCL output ");
	ddr_.registerVariable<bool>("spiclkpwm0p_output", boost::bind(&CANifierCIParams::getGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::SPI_CLK_PWM0P), boost::bind(&CANifierCIParams::setGeneralPinOutput, this, hardware_interface::canifier::GeneralPin::SPI_CLK_PWM0P, _1, false), "General Pins SCL output ");

	ddr_.registerEnumVariable<int>("velocity_measurment_period", boost::bind(&CANifierCIParams::getVelocityMeasurementPeriod, this), boost::bind(&CANifierCIParams::setVelocityMeasurementPeriod, this, _1, false), "Velocity Measurement Period", velocity_measurement_period_enum_map_);
	ddr_.registerVariable<int>("velocity_measurement_window_",  boost::bind(&CANifierCIParams::getVelocityMeasurementWindow, this), boost::bind(&CANifierCIParams::setVelocityMeasurementWindow, this, _1, false), "Velocity measurement window (samples)", 0, 512);

	ddr_.registerVariable<bool>("clear_position_on_limit_f_", boost::bind(&CANifierCIParams::getClearPositionOnLimitF, this), boost::bind(&CANifierCIParams::setClearPositionOnLimitF, this, _1, false), "Clear position when forward limit switch hit");
	ddr_.registerVariable<bool>("clear_position_on_limit_r_", boost::bind(&CANifierCIParams::getClearPositionOnLimitR, this), boost::bind(&CANifierCIParams::setClearPositionOnLimitR, this, _1, false), "Clear position when reverse limit switch hit");
	ddr_.registerVariable<bool>("clear_position_on_quad_idx_", boost::bind(&CANifierCIParams::getClearPositionOnQuadIdx, this), boost::bind(&CANifierCIParams::setClearPositionOnQuadIdx, this, _1, false), "Clear position when quad encoder index hit");

	ddr_.registerVariable<bool>("pwm_channel0_output_enable", boost::bind(&CANifierCIParams::getPWMOutputEnable, this, hardware_interface::canifier::PWMChannel::PWMChannel0), boost::bind(&CANifierCIParams::setPWMOutputEnable, this, hardware_interface::canifier::PWMChannel::PWMChannel0, _1, false), "PWM Channel 0 output enable");
	ddr_.registerVariable<bool>("pwm_channel1_output_enable", boost::bind(&CANifierCIParams::getPWMOutputEnable, this, hardware_interface::canifier::PWMChannel::PWMChannel1), boost::bind(&CANifierCIParams::setPWMOutputEnable, this, hardware_interface::canifier::PWMChannel::PWMChannel1, _1, false), "PWM Channel 1 output enable");
	ddr_.registerVariable<bool>("pwm_channel2_output_enable", boost::bind(&CANifierCIParams::getPWMOutputEnable, this, hardware_interface::canifier::PWMChannel::PWMChannel2), boost::bind(&CANifierCIParams::setPWMOutputEnable, this, hardware_interface::canifier::PWMChannel::PWMChannel2, _1, false), "PWM Channel 2 output enable");
	ddr_.registerVariable<bool>("pwm_channel3_output_enable", boost::bind(&CANifierCIParams::getPWMOutputEnable, this, hardware_interface::canifier::PWMChannel::PWMChannel3), boost::bind(&CANifierCIParams::setPWMOutputEnable, this, hardware_interface::canifier::PWMChannel::PWMChannel3, _1, false), "PWM Channel 3 output enable");
	ddr_.registerVariable<bool>("pwm_channel0_output", boost::bind(&CANifierCIParams::getPWMOutput, this, hardware_interface::canifier::PWMChannel::PWMChannel0), boost::bind(&CANifierCIParams::setPWMOutput, this, hardware_interface::canifier::PWMChannel::PWMChannel0, _1, false), "PWM Channel 0 output ", 0.0, 1.0);
	ddr_.registerVariable<bool>("pwm_channel1_output", boost::bind(&CANifierCIParams::getPWMOutput, this, hardware_interface::canifier::PWMChannel::PWMChannel1), boost::bind(&CANifierCIParams::setPWMOutput, this, hardware_interface::canifier::PWMChannel::PWMChannel1, _1, false), "PWM Channel 1 output ", 0.0, 1.0);
	ddr_.registerVariable<bool>("pwm_channel2_output", boost::bind(&CANifierCIParams::getPWMOutput, this, hardware_interface::canifier::PWMChannel::PWMChannel2), boost::bind(&CANifierCIParams::setPWMOutput, this, hardware_interface::canifier::PWMChannel::PWMChannel2, _1, false), "PWM Channel 2 output ", 0.0, 1.0);
	ddr_.registerVariable<bool>("pwm_channel3_output", boost::bind(&CANifierCIParams::getPWMOutput, this, hardware_interface::canifier::PWMChannel::PWMChannel3), boost::bind(&CANifierCIParams::setPWMOutput, this, hardware_interface::canifier::PWMChannel::PWMChannel3, _1, false), "PWM Channel 3 output ", 0.0, 1.0);

	ddr_.registerVariable<int>("status_1_general_period", boost::bind(&CANifierCIParams::getStatusFramePeriod, this, hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_1_General), boost::bind(&CANifierCIParams::setStatusFramePeriod, this, hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_1_General, _1, false), "Status 1 general frame period", 0, 255);
	ddr_.registerVariable<int>("status_2_general_period", boost::bind(&CANifierCIParams::getStatusFramePeriod, this, hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_2_General), boost::bind(&CANifierCIParams::setStatusFramePeriod, this, hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_2_General, _1, false), "Status 2 general frame period", 0, 255);
	ddr_.registerVariable<int>("status_3_pwminputs0_period", boost::bind(&CANifierCIParams::getStatusFramePeriod, this, hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_3_PwmInputs0), boost::bind(&CANifierCIParams::setStatusFramePeriod, this, hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_3_PwmInputs0, _1, false), "Status 3 PWM inputs 0 frame period", 0, 255);
	ddr_.registerVariable<int>("status_4_pwminputs1_period", boost::bind(&CANifierCIParams::getStatusFramePeriod, this, hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_4_PwmInputs1), boost::bind(&CANifierCIParams::setStatusFramePeriod, this, hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_4_PwmInputs1, _1, false), "Status 4 PWM inputs 1 frame period", 0, 255);
	ddr_.registerVariable<int>("status_5_pwminputs2_period", boost::bind(&CANifierCIParams::getStatusFramePeriod, this, hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_5_PwmInputs2), boost::bind(&CANifierCIParams::setStatusFramePeriod, this, hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_5_PwmInputs2, _1, false), "Status 5 PWM inputs 2 frame period", 0, 255);
	ddr_.registerVariable<int>("status_6_pwminputs3_period", boost::bind(&CANifierCIParams::getStatusFramePeriod, this, hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_6_PwmInputs3), boost::bind(&CANifierCIParams::setStatusFramePeriod, this, hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_6_PwmInputs3, _1, false), "Status 6 PWM inputs 3 frame period", 0, 255);
	ddr_.registerVariable<int>("status_8_misc_period", boost::bind(&CANifierCIParams::getStatusFramePeriod, this, hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_8_Misc), boost::bind(&CANifierCIParams::setStatusFramePeriod, this, hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_8_Misc, _1, false), "Status 8 PWM misc frame period", 0, 255);

	ddr_.registerVariable<int>("control_1_general_period", boost::bind(&CANifierCIParams::getControlFramePeriod, this, hardware_interface::canifier::CANifierControlFrame::CANifier_Control_1_General), boost::bind(&CANifierCIParams::setControlFramePeriod, this, hardware_interface::canifier::CANifierControlFrame::CANifier_Control_1_General, _1, false), "Control 1 general frame period", 0, 255);
	ddr_.registerVariable<int>("control_2_pwm_output_period", boost::bind(&CANifierCIParams::getControlFramePeriod, this, hardware_interface::canifier::CANifierControlFrame::CANifier_Control_2_PwmOutput), boost::bind(&CANifierCIParams::setControlFramePeriod, this, hardware_interface::canifier::CANifierControlFrame::CANifier_Control_2_PwmOutput, _1, false), "Control 1 general frame period", 0, 255);

	ddr_.publishServicesTopics();
}


void CANifierCIParams::setLEDOutput(hardware_interface::canifier::LEDChannel index, double value, bool update_dynamic)
{
	if ((index <= hardware_interface::canifier::LEDChannel::LEDChannelFirst) ||
	    (index >= hardware_interface::canifier::LEDChannel::LEDChannelLast) )
		return;
	const bool update_published_info = update_dynamic && (led_output_[index] != value);
	led_output_[index] = value;
	if (update_published_info)
	{
		ddr_.updatePublishedInformation();
	}
}
void CANifierCIParams::setGeneralPinOutputEnable(hardware_interface::canifier::GeneralPin index, bool value, bool update_dynamic)
{
	if ((index <= hardware_interface::canifier::GeneralPin::GeneralPin_FIRST) ||
		(index >= hardware_interface::canifier::GeneralPin::GeneralPin_LAST))
		return;
	const bool update_published_info = update_dynamic && (general_pin_output_enable_[index] != value);
	general_pin_output_enable_[index] = value;
	if (update_published_info)
	{
		ddr_.updatePublishedInformation();
	}
}

void CANifierCIParams::setGeneralPinOutput(hardware_interface::canifier::GeneralPin index, bool value, bool update_dynamic)
{
	if ((index <= hardware_interface::canifier::GeneralPin::GeneralPin_FIRST) ||
		(index >= hardware_interface::canifier::GeneralPin::GeneralPin_LAST))
		return;
	const bool update_published_info = update_dynamic && (general_pin_output_[index] != value);
	general_pin_output_[index] = value;
	if (update_published_info)
	{
		ddr_.updatePublishedInformation();
	}
}
void CANifierCIParams::setVelocityMeasurementPeriod(int period, bool update_dynamic)
{
	if ((period < hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_1Ms) ||
	    (period > hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_100Ms))
	{
		ROS_ERROR_STREAM("Internal error in " << __PRETTY_FUNCTION__ << " : period out of bounds");
		return;
	}
	const bool update_published_info = update_dynamic && (velocity_measurement_period_ != static_cast<hardware_interface::canifier::CANifierVelocityMeasPeriod>(period));
	velocity_measurement_period_ = static_cast<hardware_interface::canifier::CANifierVelocityMeasPeriod>(period);
	if (update_published_info)
	{
		ddr_.updatePublishedInformation();
	}
}

void CANifierCIParams::setVelocityMeasurementWindow(int window, bool update_dynamic)
{
	const bool update_published_info = update_dynamic && (velocity_measurement_window_ != window);
	velocity_measurement_window_ = window;
	if (update_published_info)
	{
		ddr_.updatePublishedInformation();
	}
}
void CANifierCIParams::setClearPositionOnLimitF(bool value, bool update_dynamic)
{
	const bool update_published_info = update_dynamic && (clear_position_on_limit_f_ != value);
	clear_position_on_limit_f_ = value;
	if (update_published_info)
	{
		ddr_.updatePublishedInformation();
	}
}
void CANifierCIParams::setClearPositionOnLimitR(bool value, bool update_dynamic)
{
	const bool update_published_info = update_dynamic && (clear_position_on_limit_r_ != value);
	clear_position_on_limit_r_ = value;
	if (update_published_info)
	{
		ddr_.updatePublishedInformation();
	}
}
void CANifierCIParams::setClearPositionOnQuadIdx(bool value, bool update_dynamic)
{
	const bool update_published_info = update_dynamic && (clear_position_on_quad_idx_ != value);
	clear_position_on_quad_idx_ = value;
	if (update_published_info)
	{
		ddr_.updatePublishedInformation();
	}
}
void CANifierCIParams::setPWMOutputEnable(hardware_interface::canifier::PWMChannel index, bool value, bool update_dynamic)
{
	if ((index <= hardware_interface::canifier::PWMChannel::PWMChannelFirst) ||
		(index >= hardware_interface::canifier::PWMChannel::PWMChannelLast))
	{
		ROS_ERROR_STREAM("Error in " << __PRETTY_FUNCTION__ << " : index out of bounds");
		return;
	}
	const bool update_published_info = update_dynamic && (pwm_output_enable_[index] != value);
	pwm_output_enable_[index] = value;
	if (update_published_info)
	{
		ddr_.updatePublishedInformation();
	}
}
void CANifierCIParams::setPWMOutput(hardware_interface::canifier::PWMChannel index, double value, bool update_dynamic)
{
	if ((index <= hardware_interface::canifier::PWMChannel::PWMChannelFirst) ||
		(index >= hardware_interface::canifier::PWMChannel::PWMChannelLast))
	{
		ROS_ERROR_STREAM("Error in " << __PRETTY_FUNCTION__ << " : index out of bounds");
		return;
	}
	const bool update_published_info = update_dynamic && (pwm_output_[index] != value);
	pwm_output_[index] = value;
	if (update_published_info)
	{
		ddr_.updatePublishedInformation();
	}
}
void CANifierCIParams::setStatusFramePeriod(hardware_interface::canifier::CANifierStatusFrame frame_id, int period, bool update_dynamic)
{
	if ((frame_id <= hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_First) ||
	    (frame_id >= hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Last))
	{
		ROS_ERROR_STREAM("Error in " << __PRETTY_FUNCTION__ << " : frame_id out of bounds");
		return;
	}
	const bool update_published_info = update_dynamic && (status_frame_period_[frame_id] != period);
	status_frame_period_[frame_id] = period;
	if (update_published_info)
	{
		ddr_.updatePublishedInformation();
	}

}
void CANifierCIParams::setControlFramePeriod(hardware_interface::canifier::CANifierControlFrame frame_id, int period, bool update_dynamic)
{
	if ((frame_id <= hardware_interface::canifier::CANifierControlFrame::CANifier_Control_First) ||
		(frame_id >= hardware_interface::canifier::CANifierControlFrame::CANifier_Control_Last))
	{
		ROS_ERROR_STREAM("Error in " << __PRETTY_FUNCTION__ << " : frame_id out of bounds");
		return;
	}
	const bool update_published_info = update_dynamic && (control_frame_period_[frame_id] != period);
	control_frame_period_[frame_id] = period;
	if (update_published_info)
	{
		ddr_.updatePublishedInformation();
	}
}

double CANifierCIParams::getLEDOutput(hardware_interface::canifier::LEDChannel index) const
{
	if ((index <= hardware_interface::canifier::LEDChannel::LEDChannelFirst) ||
	    (index >= hardware_interface::canifier::LEDChannel::LEDChannelLast) )
	{
		ROS_ERROR_STREAM("Error in " << __PRETTY_FUNCTION__ << " : index out of bounds");
		return -std::numeric_limits<double>::max();
	}
	return led_output_[index];
}
bool CANifierCIParams::getGeneralPinOutputEnable(hardware_interface::canifier::GeneralPin index) const
{
	if ((index <= hardware_interface::canifier::GeneralPin::GeneralPin_FIRST) ||
		(index >= hardware_interface::canifier::GeneralPin::GeneralPin_LAST))
	{
		ROS_ERROR_STREAM("Error in " << __PRETTY_FUNCTION__ << " : index out of bounds");
		return false;
	}
	return general_pin_output_enable_[index];
}
bool CANifierCIParams::getGeneralPinOutput(hardware_interface::canifier::GeneralPin index) const
{
	if ((index <= hardware_interface::canifier::GeneralPin::GeneralPin_FIRST) ||
		(index >= hardware_interface::canifier::GeneralPin::GeneralPin_LAST))
	{
		ROS_ERROR_STREAM("Error in " << __PRETTY_FUNCTION__ << " : index out of bounds");
		return false;
	}
	return general_pin_output_[index];
}
hardware_interface::canifier::CANifierVelocityMeasPeriod CANifierCIParams::getVelocityMeasurementPeriod(void) const
{
	return velocity_measurement_period_;
}
int CANifierCIParams::getVelocityMeasurementWindow(void) const
{
	return velocity_measurement_window_;
}
bool CANifierCIParams::getClearPositionOnLimitF(void) const
{
	return clear_position_on_limit_f_;
}
bool CANifierCIParams::getClearPositionOnLimitR(void) const
{
	return clear_position_on_limit_r_;
}
bool CANifierCIParams::getClearPositionOnQuadIdx(void) const
{
	return clear_position_on_quad_idx_;
}
bool CANifierCIParams::getPWMOutputEnable(hardware_interface::canifier::PWMChannel index) const
{
	if ((index <= hardware_interface::canifier::PWMChannel::PWMChannelFirst) ||
		(index >= hardware_interface::canifier::PWMChannel::PWMChannelLast))
	{
		ROS_ERROR_STREAM("Error in " << __PRETTY_FUNCTION__ << " : index out of bounds");
		return false;
	}
	return pwm_output_enable_[index];
}
double CANifierCIParams::getPWMOutput(hardware_interface::canifier::PWMChannel index) const
{
	if ((index <= hardware_interface::canifier::PWMChannel::PWMChannelFirst) ||
		(index >= hardware_interface::canifier::PWMChannel::PWMChannelLast))
	{
		ROS_ERROR_STREAM("Error in " << __PRETTY_FUNCTION__ << " : index out of bounds");
		return false;
	}
	return pwm_output_[index];
}
int CANifierCIParams::getStatusFramePeriod(hardware_interface::canifier::CANifierStatusFrame frame_id) const
{
	if ((frame_id <= hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_First) ||
	    (frame_id >= hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Last))
	{
		ROS_ERROR_STREAM("Error in " << __PRETTY_FUNCTION__ << " : frame_id out of bounds");
		return 0;
	}
	return status_frame_period_[frame_id];
}
int CANifierCIParams::getControlFramePeriod(hardware_interface::canifier::CANifierControlFrame frame_id) const
{
	if ((frame_id <= hardware_interface::canifier::CANifierControlFrame::CANifier_Control_First) ||
		(frame_id >= hardware_interface::canifier::CANifierControlFrame::CANifier_Control_Last))
	{
		ROS_ERROR_STREAM("Error in " << __PRETTY_FUNCTION__ << " : frame_id out of bounds");
		return 0;
	}
	return control_frame_period_[frame_id];
}

CANifierControllerInterface::CANifierControllerInterface(ros::NodeHandle &n, const std::string &joint_name, hardware_interface::canifier::CANifierCommandHandle handle)
	: params_(ros::NodeHandle(n, joint_name))
	, handle_(handle)
	, set_quadrature_position_{false}
	, new_quadrature_position_{0.0}
	, clear_sticky_faults_{false}
{
}

void CANifierControllerInterface::update(void)
{
	for (size_t i = hardware_interface::canifier::LEDChannel::LEDChannelFirst + 1; i < hardware_interface::canifier::LEDChannel::LEDChannelLast; i++)
	{
		const auto led_channel = static_cast<hardware_interface::canifier::LEDChannel>(i);
		handle_->setLEDOutput(led_channel, params_.getLEDOutput(led_channel));
	}
	for (size_t i = hardware_interface::canifier::GeneralPin::GeneralPin_FIRST + 1; i < hardware_interface::canifier::GeneralPin::GeneralPin_LAST; i++)
	{
		const auto general_pin = static_cast<hardware_interface::canifier::GeneralPin>(i);
		handle_->setGeneralPinOutputEnable(general_pin, params_.getGeneralPinOutputEnable(general_pin));
		handle_->setGeneralPinOutput(general_pin, params_.getGeneralPinOutput(general_pin));
	}

	{
		std::unique_lock<std::mutex> l(set_quadrature_position_mutex_, std::try_to_lock);

		if (l.owns_lock() && set_quadrature_position_)
		{
			handle_->setQuadraturePosition(new_quadrature_position_);
			set_quadrature_position_ = false;
		}
	}

	handle_->setVelocityMeasurementPeriod(params_.getVelocityMeasurementPeriod());
	handle_->setVelocityMeasurementWindow(params_.getVelocityMeasurementWindow());
	handle_->setClearPositionOnLimitF(params_.getClearPositionOnLimitF());
	handle_->setClearPositionOnLimitR(params_.getClearPositionOnLimitR());
	handle_->setClearPositionOnQuadIdx(params_.getClearPositionOnQuadIdx());

	for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
	{
		const auto pwm_channel = static_cast<hardware_interface::canifier::PWMChannel>(i);
		handle_->setPWMOutputEnable(pwm_channel, params_.getPWMOutputEnable(pwm_channel));
		handle_->setPWMOutput(pwm_channel, params_.getPWMOutput(pwm_channel));
	}
	for (size_t i = hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_First + 1; i < hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Last; i++)
	{
		const auto frame_id = static_cast<hardware_interface::canifier::CANifierStatusFrame>(i);
		handle_->setStatusFramePeriod(frame_id, params_.getStatusFramePeriod(frame_id));
	}
	for (size_t i = hardware_interface::canifier::CANifierControlFrame::CANifier_Control_First + 1; i < hardware_interface::canifier::CANifierControlFrame::CANifier_Control_Last; i++)
	{
		const auto frame_id = static_cast<hardware_interface::canifier::CANifierControlFrame>(i);
		handle_->setControlFramePeriod(frame_id, params_.getControlFramePeriod(frame_id));
	}
	if (clear_sticky_faults_)
	{
		handle_->setClearStickyFaults();
		clear_sticky_faults_ = false;
	}
}

void CANifierControllerInterface::setQuadraturePosition(double new_quadrature_position)
{
	std::lock_guard<std::mutex> l(set_quadrature_position_mutex_);
	set_quadrature_position_ = true;
	new_quadrature_position_ = new_quadrature_position;
}

void CANifierControllerInterface::setClearStickyFaults(void)
{
	clear_sticky_faults_ = true;
}
void CANifierControllerInterface::setLEDOutput(hardware_interface::canifier::LEDChannel index, double value)
{
	params_.setLEDOutput(index, value);
}
void CANifierControllerInterface::setGeneralPinOutputEnable(hardware_interface::canifier::GeneralPin index, bool value)
{
	params_.setGeneralPinOutputEnable(index, value);
}
void CANifierControllerInterface::setGeneralPinOutput(hardware_interface::canifier::GeneralPin index, bool value)
{
	params_.setGeneralPinOutput(index, value);
}
void CANifierControllerInterface::setVelocityMeasurementPeriod(int period)
{
	params_.setVelocityMeasurementPeriod(period);
}
void CANifierControllerInterface::setVelocityMeasurementWindow(int window)
{
	params_.setVelocityMeasurementWindow(window);
}
void CANifierControllerInterface::setClearPositionOnLimitF(bool value)
{
	params_.setClearPositionOnLimitF(value);
}
void CANifierControllerInterface::setClearPositionOnLimitR(bool value)
{
	params_.setClearPositionOnLimitR(value);
}
void CANifierControllerInterface::setClearPositionOnQuadIdx(bool value)
{
	params_.setClearPositionOnQuadIdx(value);
}
void CANifierControllerInterface::setPWMOutputEnable(hardware_interface::canifier::PWMChannel index, bool value)
{
	params_.setPWMOutputEnable(index, value);
}
void CANifierControllerInterface::setPWMOutput(hardware_interface::canifier::PWMChannel index, double value)
{
	params_.setPWMOutput(index, value);
}
void CANifierControllerInterface::setStatusFramePeriod(hardware_interface::canifier::CANifierStatusFrame frame_id, int period)
{
	params_.setStatusFramePeriod(frame_id, period);
}
void CANifierControllerInterface::setControlFramePeriod(hardware_interface::canifier::CANifierControlFrame frame_id, int period)
{
	params_.setControlFramePeriod(frame_id, period);
}

}; // namespace canifier_controller_interface

