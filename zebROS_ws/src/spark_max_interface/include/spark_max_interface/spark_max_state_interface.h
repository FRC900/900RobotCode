#ifndef SPARK_MAX_STATE_INTERFACE_INC_
#define SPARK_MAX_STATE_INTERFACE_INC_

#include <array>

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>

namespace hardware_interface
{

enum SensorType { kNoSensor, kHallSensor, kQuadrature };

enum IdleMode { kCoast, kBrake };

// TODO : Fault to string?
enum FaultID {
	kBrownout, kOvercurrent, kIWDTReset, kMotorFault,
	kSensorFault, kStall, kEEPROMCRC, kCANTX,
	kCANRX, kHasReset, kDRVFault, kOtherFault,
	kSoftLimitFwd, kSoftLimitRev, kHardLimitFwd, kHardLimitRev
};

enum LimitSwitchPolarity { kNormallyOpen, kNormallyClosed };
enum MotorType { kBrushed, kBrushless };
enum ControlType {
    kDutyCycle,
    kVelocity,
    kVoltage,
    kPosition,
    kSmartMotion,
    kCurrent,
    kSmartVelocity
};

enum ExternalFollower { kFollowerDisabled, kFollowerSparkMax, kFollowerPhoenix };
enum AccelStrategy { kTrapezoidal, kSCurve };
enum class ArbFFUnits { kVoltage, kPercentOut };

constexpr size_t SPARK_MAX_PID_SLOTS = 4;

class SparkMaxHWState
{
	public:
		SparkMaxHWState(int device_id, MotorType motor_type);

		int getDeviceId(void) const;
		MotorType getMotorType(void) const;
		void setSetPoint(double set_point);
		double getSetPoint(void) const;

		void setInverted(bool inverted);
		bool getInverted(void) const;

		void setPosition(double position);
		double getPosition(void) const;

		void setVelocity(double velocity);
		double getVelocity(void) const;

		void setPGain(size_t slot, double p_gain);
		double getPGain(size_t slot) const;

		void setIGain(size_t slot, double i_gain);
		double getIGain(size_t slot) const;

		void setDGain(size_t slot, double d_gain);
		double getDGain(size_t slot) const;

		void setFGain(size_t slot, double f_gain);
		double getFGain(size_t slot) const;

		void setIZone(size_t slot, double i_zone);
		double getIZone(size_t slot) const;

		void setDFilter(size_t slot, double d_filter);
		double getDFilter(size_t slot) const;

		void setPIDFOutputMin(size_t slot, double pidf_output_min);
		double getPIDFOutputMin(size_t slot) const;

		void setPIDFOutputMax(size_t slot, double pidf_output_max);
		double getPIDFOutputMax(size_t slot) const;

		void setPIDFReferenceOutput(size_t slot, double pidf_reference_value);
		double getPIDFReferenceOutput(size_t slot) const;

		void setPIDFReferenceCtrl(size_t slot, ControlType pidf_reference_ctrl);
		ControlType getPIDFReferenceCtrl(size_t slot) const;

		void setPIDFArbFeedForward(size_t slot, double pidf_arb_feed_forward);
		double getPIDFArbFeedForward(size_t slot) const;

		void setPIDFArbFeedForwardUnits(size_t slot, ArbFFUnits pidf_arb_feed_forward_units);
		ArbFFUnits getPIDFArbFeedForwardUnits(size_t slot) const;
		void setPIDFReferenceSlot(size_t slot);
		int getPIDFReferenceSlot(void) const;


		void setForwardLimitSwitchPolarity(LimitSwitchPolarity forward_limit_switch_polarity);
		LimitSwitchPolarity getForwardLimitSwitchPolarity(void) const;

		void setForwardLimitSwitchEnabled(bool forward_limit_switch_enabled);
		bool getForwardLimitSwitchEnabled(void) const;

		void setForwardLimitSwitch(bool forward_limit_switch);
		bool getForwardLimitSwitch(void) const;

		void setReverseLimitSwitchPolarity(LimitSwitchPolarity reverse_limit_switch_polarity);
		LimitSwitchPolarity getReverseLimitSwitchPolarity(void) const;

		void setReverseLimitSwitchEnabled(bool reverse_limit_switch_enabled);
		bool getReverseLimitSwitchEnabled(void) const;

		void setReverseLimitSwitch(bool reverse_limit_switch);
		bool getReverseLimitSwitch(void) const;

		void setCurrentLimit(unsigned int current_limit);
		unsigned int getCurrentLimit(void) const;

		void setCurrentLimitFree(unsigned int current_limit_free);
		unsigned int getCurrentLimitFree(void) const;

		void setCurrentLimitStall(unsigned int current_limit_stall);
		unsigned int getCurrentLimitStall(void) const;

		void setCurrentLimitRPM(unsigned int current_limit_rpm);
		unsigned int getCurrentLimitRPM(void) const;

		void setSecondaryCurrentLimit(double secondary_current_limit);
		double getSecondaryCurrentLimit(void) const;

		void setSecondaryCurrentLimitCycles(unsigned int secondary_current_limit_cycles);
		unsigned int getSecondaryCurrentLimitCycles(void) const;

		void setIdleMode(IdleMode idle_mode);
		IdleMode getIdleMode(void) const;

		void setVoltageCompensationEnable(bool enable);
		bool getVoltageCompensationEnable(void) const;

		void setVoltageCompensationNominalVoltage(double nominal_voltage);
		bool getVoltageCompensationNominalVoltage(void) const;

		void setOpenLoopRampRate(double open_loop_ramp_rate);
		double getOpenLoopRampRate(void) const;

		void setClosedLoopRampRate(double closed_loop_ramp_rate);
		double getClosedLoopRampRate(void) const;

		void setForwardSoftlimitEnable(bool enable);
		bool getForwardSoftlimitEnable(void) const;
		void setForwardSoftlimit(double limit);
		double getForwardSoftlimit(void) const;

		void setReverseSoftlimitEnable(bool enable);
		bool getReverseSoftlimitEnable(void) const;
		void setReverseSoftlimit(double limit);
		double getReverseSoftlimit(void) const;

		void setFollowerType(ExternalFollower follower_type);
		ExternalFollower getFollowerType(void) const;

		void setFollowerID(int follower_id);
		int getFollowerID(void) const;

		void setFollowerInvert(bool follower_invert);
		bool getFollowerInvert(void) const;

		void setFaults(uint16_t faults);
		uint16_t getFaults(void) const;

		void setStickyFaults(uint16_t sticky_faults);
		uint16_t getStickyFaults(void) const;

		void setBusVoltage(double bus_voltage);
		double getBusVoltage(void) const;

		void setAppliedOutput(double applied_output);
		double getAppliedOutput(void) const;

		void setOutputCurrent(double output_current);
		double getOutputCurrent(void) const;

		void setMotorTemperature(double motor_temperature);
		double getMotorTemperature(void) const;

		unsigned int getEncoderTicksPerRotation(void) const;
		void setEncoderTicksPerRotation(unsigned int encoder_ticks_per_rotation);

		void setEncoderType(SensorType encoder_type);
		SensorType getEncoderType(void) const;

	private:
		int                 device_id_;
		MotorType           motor_type_;
		double              set_point_{0};
		bool                inverted_{false};

		// Encoder
		double              position_{0};
		double              velocity_{0};

		// PID Controller
		std::array<double, SPARK_MAX_PID_SLOTS>      p_gain_;
		std::array<double, SPARK_MAX_PID_SLOTS>      i_gain_;
		std::array<double, SPARK_MAX_PID_SLOTS>      d_gain_;
		std::array<double, SPARK_MAX_PID_SLOTS>      f_gain_;
		std::array<double, SPARK_MAX_PID_SLOTS>      i_zone_;
		std::array<double, SPARK_MAX_PID_SLOTS>      d_filter_;
		std::array<double, SPARK_MAX_PID_SLOTS>      pidf_output_min_;
		std::array<double, SPARK_MAX_PID_SLOTS>      pidf_output_max_;
		std::array<double, SPARK_MAX_PID_SLOTS>      pidf_reference_value_;
		std::array<ControlType, SPARK_MAX_PID_SLOTS> pidf_reference_ctrl_;
		std::array<double, SPARK_MAX_PID_SLOTS>      pidf_arb_feed_forward_;
		std::array<ArbFFUnits, SPARK_MAX_PID_SLOTS>  pidf_arb_feed_forward_units_;
		size_t              pidf_reference_slot_;

		// Forward and Reverse Limit switches
		LimitSwitchPolarity forward_limit_switch_polarity_{kNormallyOpen};
		bool                forward_limit_switch_enabled_{false};
		bool                forward_limit_switch_{false};
		LimitSwitchPolarity reverse_limit_switch_polarity_{kNormallyOpen};
		bool                reverse_limit_switch_enabled_{false};
		bool                reverse_limit_switch_{false};

		// Something for current limit mode?
		// TODO - better defaults
		unsigned int        current_limit_{0};
		unsigned int        current_limit_stall_{0};
		unsigned int        current_limit_free_{0};
		unsigned int        current_limit_rpm_{0};
		double              secondary_current_limit_{0};
		int                 secondary_current_limit_cycles_{0};

		IdleMode            idle_mode_{kCoast};

		bool                voltage_compensation_enable_{false};
		double              voltage_compensation_nominal_voltage_{12.};

		double              open_loop_ramp_rate_{0.0};
		double              closed_loop_ramp_rate_{0.0};

		ExternalFollower    follower_type_{kFollowerDisabled};
		int                 follower_id_{-1};
		bool                follower_invert_{false};

		bool                forward_softlimit_enable_{false};
		double              forward_softlimit_{0.0};

		bool                reverse_softlimit_enable_{false};
		double              reverse_softlimit_{0.0};

		uint16_t            faults_{0};
		uint16_t            sticky_faults_{0};

		double              bus_voltage_{0.0};
		double              applied_output_{0.0};
		double              output_current_{0.0};
		double              motor_temperature_{0.0};

		unsigned int        encoder_ticks_per_rotation_{4096};
		SensorType          encoder_type_{kNoSensor};
};

typedef StateHandle<const SparkMaxHWState> SparkMaxStateHandle;
typedef StateHandle<SparkMaxHWState> SparkMaxWritableStateHandle;

class SparkMaxStateInterface : public HardwareResourceManager<SparkMaxStateHandle> {};
class RemoteSparkMaxStateInterface : public HardwareResourceManager<SparkMaxWritableStateHandle, ClaimResources> {};
} // namespace

#endif

