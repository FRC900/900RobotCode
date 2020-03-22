#pragma once
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>
namespace hardware_interface
{
namespace canifier
{

enum LEDChannel
{
	LEDChannelFirst = -1,
	LEDChannelA,
	LEDChannelB,
	LEDChannelC,
	LEDChannelLast
};

enum PWMChannel
{
	PWMChannelFirst = -1,
	PWMChannel0,
	PWMChannel1,
	PWMChannel2,
	PWMChannel3,
	PWMChannelLast
};

constexpr size_t PWMChannelCount = 4;

enum GeneralPin
{
	GeneralPin_FIRST = -1,
	QUAD_IDX,
	QUAD_B,
	QUAD_A,
	LIMR,
	LIMF,
	SDA,
	SCL,
	SPI_CS,
	SPI_MISO_PWM2P,
	SPI_MOSI_PWM1P,
	SPI_CLK_PWM0P,
	GeneralPin_LAST
};

/**
 * Structure to hold the pin values.
 */
struct PinValues
{
	bool QUAD_IDX;
	bool QUAD_B;
	bool QUAD_A;
	bool LIMR;
	bool LIMF;
	bool SDA;
	bool SCL;
	bool SPI_CS_PWM3;
	bool SPI_MISO_PWM2;
	bool SPI_MOSI_PWM1;
	bool SPI_CLK_PWM0;
};

enum CANifierVelocityMeasPeriod
{
	Period_1Ms,
	Period_2Ms,
	Period_5Ms,
	Period_10Ms,
	Period_20Ms,
	Period_25Ms,
	Period_50Ms,
	Period_100Ms,
	Period_Last
};

enum CANifierStatusFrame
{
	CANifierStatusFrame_First = -1,
	CANifierStatusFrame_Status_1_General,
	CANifierStatusFrame_Status_2_General,
	CANifierStatusFrame_Status_3_PwmInputs0,
	CANifierStatusFrame_Status_4_PwmInputs1,
	CANifierStatusFrame_Status_5_PwmInputs2,
	CANifierStatusFrame_Status_6_PwmInputs3,
	CANifierStatusFrame_Status_8_Misc,
	CANifierStatusFrame_Last
};

// TODO : Guessing on 8_misc
constexpr int canifierstatusframe_status_1_general_default = 100;
constexpr int canifierstatusframe_status_2_general_default = 10;
constexpr int canifierstatusframe_status_3_pwminputs0_default = 100;
constexpr int canifierstatusframe_status_4_pwminputs1_default = 100;
constexpr int canifierstatusframe_status_5_pwminputs2_default = 100;
constexpr int canifierstatusframe_status_6_pwminputs3_default = 100;
constexpr int canifierstatusframe_status_8_misc_default = 100;

enum CANifierControlFrame {
	CANifier_Control_First = -1,
	CANifier_Control_1_General,
	CANifier_Control_2_PwmOutput,
	CANifier_Control_Last
};

// TODO : fix me
constexpr int canifier_control_1_general_default = 0;
constexpr int canifier_control_2_pwmoutput_default = 0;

class CANifierHWState
{
	public:
		CANifierHWState(int can_id);

		int    getCANId(void) const;

		void   setLEDOutput(LEDChannel led_channel, double percentOutput);
		double getLEDOutput(LEDChannel led_channel) const;

		void   setGeneralPinOutput(GeneralPin pin, bool value);
		bool   getGeneralPinOutput(GeneralPin pin) const;
		void   setGeneralPinOutputEnable(GeneralPin pin, bool output_enable);
		bool   getGeneralPinOutputEnable(GeneralPin pin) const;
		void   setGeneralPinInput(GeneralPin pin, bool value);
		bool   getGeneralPinInput(GeneralPin pin) const;

		void   setQuadraturePosition(double position);
		double getQuadraturePosition(void) const;
		void   setQuadratureVelocity(double velocity);
		double getQuadratureVelocity(void) const;

		void   setVelocityMeasurementPeriod(CANifierVelocityMeasPeriod period);
		CANifierVelocityMeasPeriod getVelocityMeasurementPeriod(void) const;

		void   setVelocityMeasurementWindow(int period);
		int    getVelocityMeasurementWindow(void) const;

		void   setClearPositionOnLimitF(bool value);
		bool   getClearPositionOnLimitF(void) const;

		void   setClearPositionOnLimitR(bool value);
		bool   getClearPositionOnLimitR(void) const;

		void   setClearPositionOnQuadIdx(bool value);
		bool   getClearPositionOnQuadIdx(void) const;

		void   setBusVoltage(double bus_voltage);
		double getBusVoltage(void) const;

		void   setPWMOutput(PWMChannel channel, double value);
		double getPWMOutput(PWMChannel channel) const;
		void   setPWMOutputEnable(PWMChannel channel, bool value);
		bool   getPWMOutputEnable(PWMChannel channel) const;
		void   setPWMInput(PWMChannel channel, const std::array<double, 2> &value);
		std::array<double, 2> getPWMInput(PWMChannel channel) const;

		void   setStatusFramePeriod(CANifierStatusFrame frame_id, int period);
		int    getStatusFramePeriod(CANifierStatusFrame frame_id) const;

		void   setControlFramePeriod(CANifierControlFrame frame_id, int period);
		int    getControlFramePeriod(CANifierControlFrame frame_id) const;

		void     setFaults(unsigned faults);
		unsigned int getFaults(void) const;
		void     setStickyFaults(unsigned faults);
		unsigned int getStickyFaults(void) const;

		void   setEncoderTicksPerRotation(unsigned int encoder_ticks_per_rotation);
		unsigned int getEncoderTicksPerRotation(void) const;

		void   setConversionFactor(double conversion_factor);
		double getConversionFactor(void) const;

	private:
		int                                                               can_id_;
		std::array<double, LEDChannel::LEDChannelLast>                    led_output_;
		std::array<bool,   GeneralPin::GeneralPin_LAST>                   general_pin_output_;
		std::array<bool,   GeneralPin::GeneralPin_LAST>                   general_pin_output_enable_;
		std::array<bool,   GeneralPin::GeneralPin_LAST>                   general_pin_input_;
		double                                                            quadrature_position_;
		double                                                            quadrature_velocity_;
		CANifierVelocityMeasPeriod                                        velocity_measurement_period_;
		int                                                               velocity_measurement_window_;
		bool                                                              clear_position_on_limit_f_;
		bool                                                              clear_position_on_limit_r_;
		bool                                                              clear_position_on_quad_idx_;
		double                                                            bus_voltage_;
		std::array<double, PWMChannel::PWMChannelLast>                    pwm_output_;
		std::array<bool,   PWMChannel::PWMChannelLast>                    pwm_output_enable_;
		std::array<std::array<double, 2>, PWMChannel::PWMChannelLast>     pwm_input_;
		std::array<int,    CANifierStatusFrame::CANifierStatusFrame_Last> status_frame_period_;
		std::array<int,    CANifierControlFrame::CANifier_Control_Last>   control_frame_period_;
		unsigned int                                                      faults_;
		unsigned int                                                      sticky_faults_;
		unsigned int                                                      encoder_ticks_per_rotation_;
		double                                                            conversion_factor_;

}; // class CANifierHWCommand

typedef StateHandle<const CANifierHWState> CANifierStateHandle;
typedef StateHandle<CANifierHWState> CANifierWritableStateHandle;
class CANifierStateInterface : public HardwareResourceManager<CANifierStateHandle> {};
class RemoteCANifierStateInterface : public HardwareResourceManager<canifier::CANifierWritableStateHandle, ClaimResources> {};

} // namespace canifier
} // namespace hardware_interface
