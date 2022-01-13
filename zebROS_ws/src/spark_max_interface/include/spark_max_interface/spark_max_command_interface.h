#ifndef SPARK_MAX_COMMAND_INTERFACE_INC_
#define SPARK_MAX_COMMAND_INTERFACE_INC_

#include <spark_max_interface/spark_max_state_interface.h>
#include "state_handle/command_handle.h"
namespace hardware_interface
{

class SparkMaxHWCommand
{
	public:
		SparkMaxHWCommand();

		void setSetPoint(double set_point);
		double getSetPoint(void) const;
		bool changedSetPoint(double &set_point);
		void resetSetPoint(void);

		void setInverted(bool inverted);
		bool getInverted(void) const;
		bool changedInverted(bool &inverted);
		void resetInverted(void);

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

		bool changedPIDFConstants(size_t slot,
				double &p_gain, double &i_gain,
				double &d_gain, double &f_gain,
				double &i_zone, double &d_filter);
		void resetPIDFConstants(size_t slot);

		void setPIDFOutputMin(size_t slot, double pidf_output_min);
		double getPIDFOutputMin(size_t slot) const;

		void setPIDFOutputMax(size_t slot, double pidf_output_max);
		double getPIDFOutputMax(size_t slot) const;
		bool changedPIDOutputRange(size_t slot,
				double &output_min,
				double &output_max);
		void resetPIDOutputRange(size_t slot);

		void setPIDFReferenceValue(size_t slot, double pidf_reference_value);
		double getPIDFReferenceValue(size_t slot) const;

		void setPIDFReferenceCtrl(size_t slot, ControlType pidf_reference_ctrl);
		ControlType getPIDFReferenceCtrl(size_t slot) const;
		void setPIDFArbFeedForward(size_t slot, double pidf_arb_feed_forward);
		double getPIDFArbFeedForward(size_t slot) const;
		void setPIDFArbFeedForwardUnits(size_t slot, ArbFFUnits pidf_arb_feed_forward);
		ArbFFUnits getPIDFArbFeedForwardUnits(size_t slot) const;
		bool changedPIDFReference(size_t slot,
				double &pidf_reference_value,
				ControlType &pidf_reference_ctrl,
				double &pidf_arb_feed_forward,
				ArbFFUnits &pidf_arb_feed_forward_units);
		void resetPIDReference(size_t slot);

		void setPIDFReferenceSlot(size_t slot);
		int getPIDFReferenceSlot(void) const;

		bool changedPIDFReferenceSlot(size_t &pidf_reference_slot);
		void resetPIDFReferenceSlot(void);

		void setForwardLimitSwitchPolarity(LimitSwitchPolarity forward_limit_switch_polarity);
		LimitSwitchPolarity getForwardLimitSwitchPolarity(void) const;

		void setForwardLimitSwitchEnabled(bool forward_limit_switch_enabled);
		bool getForwardLimitSwitchEnabled(void) const;

		bool changedForwardLimitSwitch(
				LimitSwitchPolarity &forward_limit_switch_polarity,
				bool &forward_limit_switch_enabled);
		void resetForwardLimitSwitch(void);

		void setReverseLimitSwitchPolarity(LimitSwitchPolarity reverse_limit_switch_polarity);
		LimitSwitchPolarity getReverseLimitSwitchPolarity(void) const;

		void setReverseLimitSwitchEnabled(bool reverse_limit_switch_enabled);
		bool getReverseLimitSwitchEnabled(void) const;

		bool changedReverseLimitSwitch(
				LimitSwitchPolarity &reverse_limit_switch_polarity,
				bool &reverse_limit_switch_enabled);
		void resetReverseLimitSwitch(void);

		void setCurrentLimit(unsigned int current_limit);
		unsigned int getCurrentLimit(void) const;
		bool changedCurrentLimitOne(unsigned int &current_limit);
		void resetCurrentLimitOne(void);

		void setCurrentLimitStall(unsigned int current_limit_stall);
		unsigned int getCurrentLimitStall(void) const;

		void setCurrentLimitFree(unsigned int current_limit_free);
		unsigned int getCurrentLimitFree(void) const;

		void setCurrentLimitRPM(unsigned int current_limit_rpm);
		unsigned int getCurrentLimitRPM(void) const;

		bool changedCurrentLimit(
				unsigned int &current_limit_stall,
				unsigned int &current_limit_free,
				unsigned int &current_limit_rpm);
		void resetCurrentLimit(void);

		void setSecondaryCurrentLimit(double secondary_current_limit);
		double getSecondaryCurrentLimit(void) const;

		void setSecondaryCurrentLimitCycles(unsigned int secondary_current_limit_cycles);
		unsigned int getSecondaryCurrentLimitCycles(void) const;

		bool changedSecondaryCurrentLimits(double &secondary_current_limit,
				unsigned int &secondary_current_limit_cycles);
		void resetSecondaryCurrentLimits(void);

		void setIdleMode(IdleMode idle_mode);
		IdleMode getIdleMode(void) const;
		bool changedIdleMode(IdleMode &idle_mode);
		void resetIdleMode(void);

		void setVoltageCompensationEnable(bool enable);
		bool getVoltageCompensationEnable(void);

		void setVoltageCompensationNominalVoltage(double nominal_voltage);
		bool getVoltageCompensationNominalVoltage(void);
		bool changedVoltageCompensation(bool &enable,
				double &nominal_voltage);
		void resetVoltageCompensation(void);

		void setOpenLoopRampRate(double open_loop_ramp_rate);
		double getOpenLoopRampRate(void) const;
		bool changedOpenLoopRampRate(double &open_loop_ramp_rate);
		void resetOpenLoopRampRate(void);

		void setClosedLoopRampRate(double closed_loop_ramp_rate);
		double getClosedLoopRampRate(void) const;
		bool changedClosedLoopRampRate(double &closed_loop_ramp_rate);
		void resetClosedLoopRampRate(void);

		void setFollowerType(ExternalFollower follower_type);
		ExternalFollower getFollowerType(void) const;

		void setFollowerID(double follower_id);
		double getFollowerID(void) const;

		void setFollowerInvert(double follower_invert);
		double getFollowerInvert(void) const;

		bool changedFollower(ExternalFollower &follower_type,
				int &follower_id,
				bool &follower_invert);
		void resetFollower(void);

		void setForwardSoftlimitEnable(bool enable);
		bool getForwardSoftlimitEnable(void) const;
		void setForwardSoftlimit(double limit);
		double getForwardSoftlimit(void) const;
		bool changedForwardSoftlimit(bool &enable, double &limit);
		void resetForwardSoftlimit(void);

		void setReverseSoftlimitEnable(bool enable);
		bool getReverseSoftlimitEnable(void) const;
		void setReverseSoftlimit(double limit);
		double getReverseSoftlimit(void) const;
		bool changedReverseSoftlimit(bool &enable, double &limit);
		void resetReverseSoftlimit(void);
		unsigned int getEncoderTicksPerRotation(void) const;

		void setEncoderTicksPerRotation(unsigned int encoder_ticks_per_rotation);

		void setEncoderType(SensorType encoder_type);
		SensorType getEncoderType(void) const;

		bool changedEncoderType(SensorType &encoder_type);
		void resetEncoderType(void);

	private:
		double              set_point_{0};
		bool                set_point_changed_{false};
		bool                inverted_{false};
		bool                inverted_changed_{false};

		// PID Controller
		std::array<double, SPARK_MAX_PID_SLOTS>      p_gain_;
		std::array<double, SPARK_MAX_PID_SLOTS>      i_gain_;
		std::array<double, SPARK_MAX_PID_SLOTS>      d_gain_;
		std::array<double, SPARK_MAX_PID_SLOTS>      f_gain_;
		std::array<double, SPARK_MAX_PID_SLOTS>      i_zone_;
		std::array<double, SPARK_MAX_PID_SLOTS>      d_filter_;
		std::array<bool  , SPARK_MAX_PID_SLOTS>      pidf_constants_changed_;

		std::array<double, SPARK_MAX_PID_SLOTS>      pidf_output_min_;
		std::array<double, SPARK_MAX_PID_SLOTS>      pidf_output_max_;
		std::array<bool  , SPARK_MAX_PID_SLOTS>      pidf_output_range_changed_;

		std::array<double     , SPARK_MAX_PID_SLOTS> pidf_reference_value_;
		std::array<ControlType, SPARK_MAX_PID_SLOTS> pidf_reference_ctrl_;
		std::array<double     , SPARK_MAX_PID_SLOTS> pidf_arb_feed_forward_;
		std::array<ArbFFUnits , SPARK_MAX_PID_SLOTS> pidf_arb_feed_forward_units_;
		std::array<bool       , SPARK_MAX_PID_SLOTS> pidf_reference_changed_;

		size_t              pidf_reference_slot_{0};
		bool                pidf_reference_slot_changed_{false};

		// Forward and Reverse Limit switches
		LimitSwitchPolarity forward_limit_switch_polarity_{kNormallyOpen};
		bool                forward_limit_switch_enabled_{false};
		bool                forward_limit_switch_changed_{false};
		LimitSwitchPolarity reverse_limit_switch_polarity_{kNormallyOpen};
		bool                reverse_limit_switch_enabled_{false};
		bool                reverse_limit_switch_changed_{false};

		// Something for current limit mode?
		//TODO - better defaults
		unsigned int        current_limit_{0};
		bool                current_limit_one_changed_{false};
		unsigned int        current_limit_stall_{0};
		unsigned int        current_limit_free_{0};
		unsigned int        current_limit_rpm_{0};
		bool                current_limit_changed_{false};
		double              secondary_current_limit_{0};
		unsigned int        secondary_current_limit_cycles_{0};
		bool                secondary_current_limit_changed_{false};

		IdleMode            idle_mode_{kCoast};
		bool                idle_mode_changed_{false};

		bool                voltage_compensation_enable_{false};
		double              voltage_compensation_nominal_voltage_{12.};
		bool                voltage_compensation_changed_{false};

		double              open_loop_ramp_rate_{0};
		bool                open_loop_ramp_rate_changed_{false};
		double              closed_loop_ramp_rate_{0};
		bool                closed_loop_ramp_rate_changed_{false};

		ExternalFollower    follower_type_{kFollowerDisabled};
		int                 follower_id_{-1};
		bool                follower_invert_{false};
		bool                follower_changed_{false};

		bool                forward_softlimit_enable_{false};
		double              forward_softlimit_{0.0};
		bool                forward_softlimit_changed_{false};

		bool                reverse_softlimit_enable_{false};
		double              reverse_softlimit_{0.0};
		bool                reverse_softlimit_changed_{false};

		unsigned int        encoder_ticks_per_rotation_{4096};
		SensorType          encoder_type_{kNoSensor};
		bool                encoder_type_changed_{false};
};

// Create a handle pointing to a type TalonHWCommand / TalonHWState pair
typedef CommandHandle<SparkMaxHWCommand, SparkMaxHWState, SparkMaxStateHandle> SparkMaxCommandHandle;


// Use ClaimResources here since we only want 1 controller
// to be able to access a given SparkMax at any particular time
class SparkMaxCommandInterface : public HardwareResourceManager<SparkMaxCommandHandle, ClaimResources> {};

} // namespace hardware_interface
#endif
