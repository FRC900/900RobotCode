#pragma once

#include "talon_interface/canifier_state_interface.h"
#include "state_handle/command_handle.h"

namespace hardware_interface
{
namespace canifier
{
class CANifierHWCommand
{
	public:
		CANifierHWCommand(void);

		void   setLEDOutput(LEDChannel led_channel, double percentOutput);
		double getLEDOutput(LEDChannel led_channel) const;
		bool   ledOutputChanged(LEDChannel led_channel, double &percentOutput);
		void   resetLEDOutput(LEDChannel led_channel);

		void   setGeneralPinOutput(GeneralPin pin, bool value);
		bool   getGeneralPinOutput(GeneralPin pin) const;
		void   setGeneralPinOutputEnable(GeneralPin pin, bool output_enable);
		bool   getGeneralPinOutputEnable(GeneralPin pin) const;
		bool   generalPinOutputChanged(GeneralPin pin, bool &value, bool &output_enable);
		void   resetGeneralPinOutput(GeneralPin pin);

		void   setQuadraturePosition(double position);
		double getQuadraturePosition(void) const;
		bool   quadraturePositionChanged(double &position);
		void   resetQuadraturePosition(void);

		void   setVelocityMeasurementPeriod(CANifierVelocityMeasPeriod period);
		CANifierVelocityMeasPeriod getVelocityMeasurementPeriod(void) const;
		bool   velocityMeasurementPeriodChanged(CANifierVelocityMeasPeriod &period);
		void   resetVelocityMeasurementPeriod(void);

		void   setVelocityMeasurementWindow(int window);
		int    getVelocityMeasurementWindow(void) const;
		bool   velocityMeasurementWindowChanged(int &window);
		void   resetVelocityMeasurementWindow(void);

		void   setClearPositionOnLimitF(bool value);
		bool   getClearPositionOnLimitF(void) const;
		bool   clearPositionOnLimitFChanged(bool &value);
		void   resetClearPositionOnLimitF(void);

		void   setClearPositionOnLimitR(bool value);
		bool   getClearPositionOnLimitR(void) const;
		bool   clearPositionOnLimitRChanged(bool &value);
		void   resetClearPositionOnLimitR(void);

		void   setClearPositionOnQuadIdx(bool value);
		bool   getClearPositionOnQuadIdx(void) const;
		bool   clearPositionOnQuadIdxChanged(bool &value);
		void   resetClearPositionOnQuadIdx(void);

		void   setPWMOutput(PWMChannel channel, double value);
		double getPWMOutput(PWMChannel channel) const;
		bool   pwmOutputChanged(PWMChannel channel, double &value);
		void   resetPWMOutput(PWMChannel channel);

		void   setPWMOutputEnable(PWMChannel channel, bool value);
		bool   getPWMOutputEnable(PWMChannel channel) const;
		bool   pwmOutputEnableChanged(PWMChannel channel, bool &value);
		void   resetPWMOutputEnable(PWMChannel channel);

		void   setStatusFramePeriod(CANifierStatusFrame frame_id, int period);
		int    getStatusFramePeriod(CANifierStatusFrame frame_id) const;
		bool   statusFramePeriodChanged(CANifierStatusFrame frame_id, int &period);
		void   resetStatusFramePeriod(CANifierStatusFrame frame_id);

		void   setControlFramePeriod(CANifierControlFrame frame_id, int period);
		int    getControlFramePeriod(CANifierControlFrame frame_id) const;
		bool   controlFramePeriodChanged(CANifierControlFrame frame_id, int &period);
		void   resetControlFramePeriod(CANifierControlFrame frame_id);

		void   setClearStickyFaults(void);
		bool   getClearStickyFaults(void) const;
		bool   clearStickyFaultsChanged(void);

		void   setEncoderTicksPerRotation(unsigned int encoder_ticks_per_rotation);
		unsigned int getEncoderTicksPerRotation(void) const;

		void   setConversionFactor(double conversion_factor);
		double getConversionFactor(void) const;

	private:
		std::array<double, LEDChannel::LEDChannelLast>                    led_output_;
		std::array<bool,   LEDChannel::LEDChannelLast>                    led_output_changed_;
		std::array<bool,   GeneralPin::GeneralPin_LAST>                   general_pin_output_;
		std::array<bool,   GeneralPin::GeneralPin_LAST>                   general_pin_output_enable_;
		std::array<bool,   GeneralPin::GeneralPin_LAST>                   general_pin_changed_;
		double                                                            quadrature_position_;
		bool                                                              quadrature_position_changed_;
		CANifierVelocityMeasPeriod                                        velocity_measurement_period_;
		bool                                                              velocity_measurement_period_changed_;
		int                                                               velocity_measurement_window_;
		bool                                                              velocity_measurement_window_changed_;
		bool                                                              clear_position_on_limit_f_;
		bool                                                              clear_position_on_limit_f_changed_;
		bool                                                              clear_position_on_limit_r_;
		bool                                                              clear_position_on_limit_r_changed_;
		bool                                                              clear_position_on_quad_idx_;
		bool                                                              clear_position_on_quad_idx_changed_;
		std::array<double, PWMChannel::PWMChannelLast>                    pwm_output_;
		std::array<bool,   PWMChannel::PWMChannelLast>                    pwm_output_changed_;
		std::array<bool,   PWMChannel::PWMChannelLast>                    pwm_output_enable_;
		std::array<bool,   PWMChannel::PWMChannelLast>                    pwm_output_enable_changed_;
		std::array<int,    CANifierStatusFrame::CANifierStatusFrame_Last> status_frame_period_;
		std::array<bool,   CANifierStatusFrame::CANifierStatusFrame_Last> status_frame_period_changed_;
		std::array<int,    CANifierControlFrame::CANifier_Control_Last>   control_frame_period_;
		std::array<bool,   CANifierControlFrame::CANifier_Control_Last>   control_frame_period_changed_;
		bool                                                              clear_sticky_faults_;
		unsigned int                                                      encoder_ticks_per_rotation_;
		double                                                            conversion_factor_;
}; // class CANifierHWCommand

typedef CommandHandle<CANifierHWCommand, CANifierHWState, CANifierStateHandle> CANifierCommandHandle;

// Use ClaimResources here since we only want 1 controller
// to be able to access a given CANifier at any particular time
class CANifierCommandInterface : public HardwareResourceManager<CANifierCommandHandle, ClaimResources> {};


} // namespace canifier
} // namespace hardware_interface


