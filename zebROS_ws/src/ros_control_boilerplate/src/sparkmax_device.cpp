#include <optional>
#include <thread>

#include "hal/DriverStation.h"

#include "rev/CANSparkMax.h"

#include "spark_max_interface/spark_max_command_interface.h"
#include "ros_control_boilerplate/sparkmax_device.h"
#include "ros_control_boilerplate/tracer.h"

static bool convertMotorType(const hardware_interface::MotorType input,
                             rev::CANSparkMaxLowLevel::MotorType &output);
static bool convertLimitSwitchPolarity(const hardware_interface::LimitSwitchPolarity input,
                                       rev::CANDigitalInput::LimitSwitchPolarity &output);
static bool convertEncoderType(const hardware_interface::SensorType input,
                               rev::CANEncoder::EncoderType &output);
static bool convertControlType(const hardware_interface::ControlType input,
                               rev::ControlType &output);
static bool convertArbFFUnits(const hardware_interface::ArbFFUnits input,
                              rev::CANPIDController::ArbFFUnits &output);
static bool convertIdleMode(const hardware_interface::IdleMode input,
                            rev::CANSparkMax::IdleMode &output);
static bool convertExternalFollower(const hardware_interface::ExternalFollower input,
                                    rev::CANSparkMax::ExternalFollower &output);

#define safeSparkMaxCall(error_code, call_string) \
    SIMFLAG ? true : safeCall(error_code, call_string)

template <bool SIMFLAG>
SparkMaxDevice<SIMFLAG>::SparkMaxDevice(const std::string &name_space,
                                        const int joint_index,
                                        const std::string &joint_name,
                                        const int can_id,
                                        const hardware_interface::MotorType motor_type,
                                        const bool local_hardware,
                                        const bool local_update,
                                        const double read_hz_)
    : name_{joint_name}
    , can_id_{can_id}
    , local_hardware_{local_hardware}
    , local_update_{local_update}
    , can_spark_max_{nullptr}
    , state_{std::make_unique<hardware_interface::SparkMaxHWState>(can_id, motor_type)}
    , command_{std::make_unique<hardware_interface::SparkMaxHWCommand>()}
    , read_thread_state_{nullptr}
    , read_state_mutex_{nullptr}
    , read_thread_{nullptr}
{
    ROS_INFO_STREAM_NAMED("frc_robot_interface",
                         "Loading joint " << joint_index << "=" << name_ <<
                         (local_update_ ? " local" : " remote") << " update, " <<
                         (local_hardware_ ? "local" : "remote") << " hardware" <<
                         " as SparkMax CAN id " << can_id_);

    if (local_hardware_)
    {
        rev::CANSparkMaxLowLevel::MotorType rev_motor_type;
        convertMotorType(motor_type, rev_motor_type);
        if constexpr (!SIMFLAG)
        {
            can_spark_max_ = std::make_unique<rev::CANSparkMax>(can_id_, rev_motor_type);
            pid_controller_ = std::make_unique<rev::SparkMaxPIDController>(can_spark_max_->GetPIDController());
        }

        read_state_mutex_ = std::make_unique<std::mutex>();
        read_thread_state_ = std::make_unique<hardware_interface::SparkMaxHWState>(can_id_, motor_type);
        if constexpr (!SIMFLAG)
        {
            read_thread_ = std::make_unique<std::jthread>(&SparkMaxDevice::read_thread, this,
                                                          std::make_unique<Tracer>("read_" + name_ + " " + name_space),
                                                          read_hz_);
        }
    }
}

template <bool SIMFLAG>
SparkMaxDevice<SIMFLAG>::~SparkMaxDevice(void) = default;

template <bool SIMFLAG>
void SparkMaxDevice<SIMFLAG>::registerInterfaces(hardware_interface::SparkMaxStateInterface &state_interface,
                                                 hardware_interface::SparkMaxCommandInterface &command_interface,
                                                 hardware_interface::RemoteSparkMaxStateInterface &remote_state_interface) const
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for SparkMax : " << name_ << " at CAN id " << can_id_);

    hardware_interface::SparkMaxStateHandle state_handle(name_, state_.get());
    state_interface.registerHandle(state_handle);

    hardware_interface::SparkMaxCommandHandle command_handle(state_handle, command_.get());
    command_interface.registerHandle(command_handle);

    if (!local_update_)
    {
        hardware_interface::SparkMaxWritableStateHandle remote_handle(name_, state_.get());
        remote_state_interface.registerHandle(remote_handle);
    }
}

template <bool SIMFLAG>
void SparkMaxDevice<SIMFLAG>::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (local_hardware_)
    {
        std::unique_lock l(*read_state_mutex_, std::try_to_lock);
        if (!l.owns_lock())
        {
            return;
        }

        // Copy config items from spark max state to spark_max_read_thread_state
        // This makes sure config items set by controllers is
        // eventually reflected in the state unique to the
        // spark_max_read_thread code
        read_thread_state_->setForwardLimitSwitchPolarity(state_->getForwardLimitSwitchPolarity());
        read_thread_state_->setReverseLimitSwitchPolarity(state_->getReverseLimitSwitchPolarity());
        read_thread_state_->setEncoderType(state_->getEncoderType());
        read_thread_state_->setEncoderTicksPerRotation(state_->getEncoderTicksPerRotation());

        state_->setSetPoint(read_thread_state_->getSetPoint());
        state_->setPosition(read_thread_state_->getPosition());
        state_->setVelocity(read_thread_state_->getVelocity());
        state_->setForwardLimitSwitch(read_thread_state_->getForwardLimitSwitch());
        state_->setReverseLimitSwitch(read_thread_state_->getReverseLimitSwitch());
        state_->setFaults(read_thread_state_->getFaults());
        state_->setStickyFaults(read_thread_state_->getStickyFaults());
        state_->setBusVoltage(read_thread_state_->getBusVoltage());
        state_->setAppliedOutput(read_thread_state_->getAppliedOutput());
        state_->setOutputCurrent(read_thread_state_->getOutputCurrent());
        state_->setMotorTemperature(read_thread_state_->getMotorTemperature());
    }
}

template <bool SIMFLAG>
void SparkMaxDevice<SIMFLAG>::write(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (!local_hardware_)
    {
        return;
    }

    if (bool inverted; command_->changedInverted(inverted))
    {
        can_spark_max_->SetInverted(inverted);
        ROS_INFO_STREAM("Set spark max " << name_ << " invert = " << inverted);
        state_->setInverted(inverted);
    }

    const auto spark_max_mode = command_->getPIDFReferenceCtrl(command_->getPIDFReferenceSlot());
    const bool closed_loop_mode = (spark_max_mode != hardware_interface::kDutyCycle);
    if (closed_loop_mode)
    {
        size_t slot;
        const bool slot_changed = command_->changedPIDFReferenceSlot(slot);

        double p_gain;
        double i_gain;
        double d_gain;
        double f_gain;
        double i_zone;
        double d_filter;
        if (command_->changedPIDFConstants(slot, p_gain, i_gain, d_gain, f_gain, i_zone, d_filter))
        {
            bool rc;

            rc  = safeSparkMaxCall(pid_controller_->SetP(p_gain, slot), "SetP");
            rc &= safeSparkMaxCall(pid_controller_->SetI(i_gain, slot), "SetI");
            rc &= safeSparkMaxCall(pid_controller_->SetD(d_gain, slot), "SetD");
            rc &= safeSparkMaxCall(pid_controller_->SetFF(f_gain, slot), "SetFF");
            rc &= safeSparkMaxCall(pid_controller_->SetIZone(i_zone, slot), "SetIZone");
            rc &= safeSparkMaxCall(pid_controller_->SetDFilter(d_filter, slot), "SetDFilter");
            if (rc)
            {
                ROS_INFO_STREAM("Updated Spark Max" << name_ << " PIDF slot " << slot << " gains");
                state_->setPGain(slot, p_gain);
                state_->setIGain(slot, i_gain);
                state_->setDGain(slot, d_gain);
                state_->setFGain(slot, f_gain);
                state_->setIZone(slot, i_zone);
                state_->setDFilter(slot, d_filter);
            }
            else
            {
                command_->resetPIDFConstants(slot);
            }
        }

        double pid_output_min;
        double pid_output_max;
        if (command_->changedPIDOutputRange(slot, pid_output_min, pid_output_max))
        {
            if (safeSparkMaxCall(pid_controller_->SetOutputRange(pid_output_min, pid_output_max, slot), "SetOutputRange"))
            {
                ROS_INFO_STREAM("Updated Spark Max" << name_ << " PIDF slot " << slot << " output range");
                state_->setPIDFOutputMin(slot, pid_output_min);
                state_->setPIDFOutputMax(slot, pid_output_max);
            }
            else
            {
                command_->resetPIDOutputRange(slot);
            }
        }

        double pidf_reference_value;
        hardware_interface::ControlType pidf_reference_ctrl;
        double pidf_arb_feed_forward;
        hardware_interface::ArbFFUnits pidf_arb_feed_forward_units;

        rev::ControlType rev_reference_ctrl;
        rev::CANPIDController::ArbFFUnits rev_arb_feed_forward_units;

        const bool reference_changed = command_->changedPIDFReference(slot, pidf_reference_value, pidf_reference_ctrl, pidf_arb_feed_forward, pidf_arb_feed_forward_units);
        if ((slot_changed || reference_changed))
        {
            if (convertControlType(pidf_reference_ctrl, rev_reference_ctrl) &&
                convertArbFFUnits(pidf_arb_feed_forward_units, rev_arb_feed_forward_units) &&
                safeSparkMaxCall(pid_controller_->SetReference(pidf_reference_value, rev_reference_ctrl, slot, pidf_arb_feed_forward, rev_arb_feed_forward_units), "SetReference"))
            {
                ROS_INFO_STREAM("Updated Spark Max" << name_ << " PIDF slot " << slot << " refrence");

                state_->setPIDFReferenceOutput(slot, pidf_reference_value);
                state_->setPIDFReferenceCtrl(slot, pidf_reference_ctrl);
                state_->setPIDFArbFeedForward(slot, pidf_arb_feed_forward);
                state_->setPIDFArbFeedForwardUnits(slot, pidf_arb_feed_forward_units);
                state_->setPIDFReferenceSlot(slot);
            }
            else
            {
                command_->resetPIDReference(slot);
                command_->resetPIDFReferenceSlot();
            }
        }
    }

    bool limit_switch_enabled;
    hardware_interface::LimitSwitchPolarity limit_switch_polarity;
    rev::CANDigitalInput::LimitSwitchPolarity rev_limit_switch_polarity;
    if (command_->changedForwardLimitSwitch(limit_switch_polarity, limit_switch_enabled))
    {
        if (convertLimitSwitchPolarity(limit_switch_polarity, rev_limit_switch_polarity) &&
            safeSparkMaxCall(can_spark_max_->GetForwardLimitSwitch(rev_limit_switch_polarity).EnableLimitSwitch(limit_switch_enabled), "GetForwardLimitSwitch"))
        {
            ROS_INFO_STREAM("Updated Spark Max" << name_ << " forward limit switch");
            state_->setForwardLimitSwitchEnabled(limit_switch_enabled);
            state_->setForwardLimitSwitchPolarity(limit_switch_polarity);
        }
        else
        {
            command_->resetForwardLimitSwitch();
        }
    }
    if (command_->changedReverseLimitSwitch(limit_switch_polarity, limit_switch_enabled))
    {
        if (convertLimitSwitchPolarity(limit_switch_polarity, rev_limit_switch_polarity) &&
            safeSparkMaxCall(can_spark_max_->GetReverseLimitSwitch(rev_limit_switch_polarity).EnableLimitSwitch(limit_switch_enabled), "GetReverseLimitSwitch"))
        {
            ROS_INFO_STREAM("Updated Spark Max" << name_ << " reverse limit switch");
            state_->setReverseLimitSwitchEnabled(limit_switch_enabled);
            state_->setReverseLimitSwitchPolarity(limit_switch_polarity);
        }
        else
        {
            command_->resetReverseLimitSwitch();
        }
    }

    if (unsigned int current_limit; command_->changedCurrentLimitOne(current_limit))
    {
        if (safeSparkMaxCall(can_spark_max_->SetSmartCurrentLimit(current_limit), "SetSmartCurrentLimit(1)"))
        {
            ROS_INFO_STREAM("Updated Spark Max" << name_ << " current limit (1 arg)");
            state_->setCurrentLimit(current_limit);
        }
        else
        {
            command_->resetCurrentLimitOne();
        }
    }

    unsigned int current_limit_stall;
    unsigned int current_limit_free;
    unsigned int current_limit_rpm;
    if (command_->changedCurrentLimit(current_limit_stall, current_limit_free, current_limit_rpm))
    {
        if (safeSparkMaxCall(can_spark_max_->SetSmartCurrentLimit(current_limit_stall, current_limit_free, current_limit_rpm), "SetSmartCurrentLimit(3)"))
        {
            ROS_INFO_STREAM("Updated Spark Max" << name_ << " current limit (3 arg)");
            state_->setCurrentLimitStall(current_limit_stall);
            state_->setCurrentLimitFree(current_limit_free);
            state_->setCurrentLimitRPM(current_limit_rpm);
        }
        else
        {
            command_->resetCurrentLimit();
        }
    }

    double secondary_current_limit;
    unsigned int secondary_current_limit_cycles;
    if (command_->changedSecondaryCurrentLimits(secondary_current_limit, secondary_current_limit_cycles))
    {
        if (safeSparkMaxCall(can_spark_max_->SetSecondaryCurrentLimit(secondary_current_limit, secondary_current_limit_cycles), "SetSecondaryCurrentLimit()"))
        {
            ROS_INFO_STREAM("Updated Spark Max" << name_ << " secondary current limit");
            state_->setSecondaryCurrentLimit(secondary_current_limit);
            state_->setSecondaryCurrentLimitCycles(secondary_current_limit_cycles);
        }
        else
        {
            command_->resetSecondaryCurrentLimits();
        }
    }

    hardware_interface::IdleMode idle_mode;
    rev::CANSparkMax::IdleMode rev_idle_mode;
    if (command_->changedIdleMode(idle_mode))
    {
        if (convertIdleMode(idle_mode, rev_idle_mode) &&
            safeSparkMaxCall(can_spark_max_->SetIdleMode(rev_idle_mode), "SetIdleMode"))
        {
            ROS_INFO_STREAM("Updated Spark Max" << name_ << " idle mode");
            state_->setIdleMode(idle_mode);
        }
        else
        {
            command_->resetIdleMode();
        }
    }

    bool voltage_compensation_enable;
    double voltage_compensation_nominal_voltage;
    if (command_->changedVoltageCompensation(voltage_compensation_enable, voltage_compensation_nominal_voltage))
    {
        bool rc = false;

        if (voltage_compensation_enable)
        {
            rc = safeSparkMaxCall(can_spark_max_->EnableVoltageCompensation(voltage_compensation_nominal_voltage), "EnableVoltageCompensation");
        }
        else
        {
            rc = safeSparkMaxCall(can_spark_max_->DisableVoltageCompensation(), "DisableVoltageCompensation");
        }

        if (rc)
        {
            ROS_INFO_STREAM("Updated Spark Max" << name_ << " voltage compensation");
            state_->setVoltageCompensationEnable(voltage_compensation_enable);
            state_->setVoltageCompensationNominalVoltage(voltage_compensation_nominal_voltage);
        }
        else
        {
            command_->resetVoltageCompensation();
        }
    }

    if (double open_loop_ramp_rate; command_->changedOpenLoopRampRate(open_loop_ramp_rate))
    {
        if (safeSparkMaxCall(can_spark_max_->SetOpenLoopRampRate(open_loop_ramp_rate), "SetOpenLoopRampRate"))
        {
            ROS_INFO_STREAM("Updated Spark Max" << name_ << " open loop ramp rate");
            state_->setOpenLoopRampRate(open_loop_ramp_rate);
        }
        else
        {
            command_->resetOpenLoopRampRate();
        }
    }

    if (double closed_loop_ramp_rate; command_->changedClosedLoopRampRate(closed_loop_ramp_rate))
    {
        if (safeSparkMaxCall(can_spark_max_->SetClosedLoopRampRate(closed_loop_ramp_rate), "SetClosedLoopRampRate"))
        {
            ROS_INFO_STREAM("Updated Spark Max" << name_ << " closed loop ramp rate");
            state_->setClosedLoopRampRate(closed_loop_ramp_rate);
        }
        else
        {
            command_->resetClosedLoopRampRate();
        }
    }

    hardware_interface::ExternalFollower follower_type;
    rev::CANSparkMax::ExternalFollower rev_follower_type;
    int follower_id;
    bool follower_invert;
    if (command_->changedFollower(follower_type, follower_id, follower_invert))
    {
        if (convertExternalFollower(follower_type, rev_follower_type) &&
            safeSparkMaxCall(can_spark_max_->Follow(rev_follower_type, follower_id, follower_invert), "Follow"))
        {
            ROS_INFO_STREAM("Updated Spark Max" << name_ << " follow");
            state_->setFollowerType(follower_type);
            state_->setFollowerID(follower_id);
            state_->setFollowerInvert(follower_invert);
        }
        else
        {
            command_->resetFollower();
        }
    }

    bool forward_softlimit_enable;
    double forward_softlimit;
    if (command_->changedForwardSoftlimit(forward_softlimit_enable, forward_softlimit))
    {
        if (safeSparkMaxCall(can_spark_max_->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, forward_softlimit), " SetSoftLimit(kForward)") &&
            safeSparkMaxCall(can_spark_max_->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, forward_softlimit_enable), " EnableSoftLimit(kForward)"))
        {
            ROS_INFO_STREAM("Updated Spark Max" << name_ << " forward softlimit");
            state_->setForwardSoftlimitEnable(forward_softlimit_enable);
            state_->setForwardSoftlimit(forward_softlimit);
        }
        else
        {
            command_->resetForwardSoftlimit();
        }
    }

    bool reverse_softlimit_enable;
    double reverse_softlimit;
    if (command_->changedReverseSoftlimit(reverse_softlimit_enable, reverse_softlimit))
    {
        if (safeSparkMaxCall(can_spark_max_->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, reverse_softlimit), " SetSoftLimit(kReverse)") &&
            safeSparkMaxCall(can_spark_max_->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, reverse_softlimit_enable), " EnableSoftLimit(kReverse)"))
        {
            ROS_INFO_STREAM("Updated Spark Max" << name_ << " reverse softlimit");
            state_->setReverseSoftlimitEnable(reverse_softlimit_enable);
            state_->setReverseSoftlimit(reverse_softlimit);
        }
        else
        {
            command_->resetReverseSoftlimit();
        }
    }

    if (double set_point; command_->changedSetPoint(set_point))
    {
        can_spark_max_->Set(set_point);
        state_->setSetPoint(set_point);
    }
}

template <bool SIMFLAG>
void SparkMaxDevice<SIMFLAG>::read_thread(std::unique_ptr<Tracer> tracer,
                                          double poll_frequency)
{
#ifdef __linux__
	std::stringstream thread_name{"spkmax_rd_"};
	thread_name << read_thread_state_->getDeviceId();
	if (pthread_setname_np(pthread_self(), thread_name.str().c_str()))
	{
		ROS_ERROR_STREAM("Error setting thread name for spark_max_read " << errno);
	}
#endif
	ros::Duration(3.12 + read_thread_state_->getDeviceId() * .04).sleep(); // Sleep for a few seconds to let CAN start up
	ros::Rate rate(poll_frequency); // TODO : configure me from a file or
						 // be smart enough to run at the rate of the fastest status update?

	while(ros::ok())
	{
		tracer->start("spark_max read main_loop");

#if 0
		hardware_interface::TalonMode talon_mode;
		hardware_interface::FeedbackDevice encoder_feedback;
		int encoder_ticks_per_rotation;
		double conversion_factor;

		// Update local status with relevant global config
		// values set by write(). This way, items configured
		// by controllers will be reflected in the state here
		// used when reading from talons.
		// Realistically they won't change much (except maybe mode)
		// but unless it causes performance problems reading them
		// each time through the loop is easier than waiting until
		// they've been correctly set by write() before using them
		// here.
		// Note that this isn't a complete list - only the values
		// used by the read thread are copied over.  Update
		// as needed when more are read
		{
			std::lock_guard<std::mutex> l(*mutex);
			if (!state->getEnableReadThread())
				return;
			talon_mode = state->getTalonMode();
			encoder_feedback = state->getEncoderFeedback();
			encoder_ticks_per_rotation = state->getEncoderTicksPerRotation();
			conversion_factor = state->getConversionFactor();
		}
#endif


		// TODO :
		// create a CANEncoder / CANAnalog object
		// Update it when config items change it
		// Figure out conversion factors

		rev::CANDigitalInput::LimitSwitchPolarity forward_limit_switch_polarity;
		rev::CANDigitalInput::LimitSwitchPolarity reverse_limit_switch_polarity;
		rev::CANEncoder::EncoderType encoder_type;
		unsigned int encoder_ticks_per_rotation;
		{
			std::lock_guard<std::mutex> l(*read_state_mutex_);
			convertLimitSwitchPolarity(read_thread_state_->getForwardLimitSwitchPolarity(), forward_limit_switch_polarity);
			convertLimitSwitchPolarity(read_thread_state_->getReverseLimitSwitchPolarity(), reverse_limit_switch_polarity);
			convertEncoderType(read_thread_state_->getEncoderType(), encoder_type);
			encoder_ticks_per_rotation = read_thread_state_->getEncoderTicksPerRotation();
		}

		if (can_spark_max_->IsFollower())
			return;

		//const double radians_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, hardware_interface::TalonMode_Position) * conversion_factor;
		//const double radians_per_second_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, hardware_interface::TalonMode_Velocity) * conversion_factor;
		//
		constexpr double radians_scale = 1;
		constexpr double radians_per_second_scale = 1;

		const double   set_point            = can_spark_max_->Get();
		auto           encoder              = can_spark_max_->GetEncoder(encoder_type, encoder_ticks_per_rotation);
		const double   position             = encoder.GetPosition() * radians_scale;
		const double   velocity             = encoder.GetVelocity() * radians_per_second_scale;
		const bool     forward_limit_switch = can_spark_max_->GetForwardLimitSwitch(forward_limit_switch_polarity).Get();
		const bool     reverse_limit_switch = can_spark_max_->GetReverseLimitSwitch(reverse_limit_switch_polarity).Get();
		const uint16_t faults               = can_spark_max_->GetFaults();
		const uint16_t sticky_faults        = can_spark_max_->GetStickyFaults();
		const double   bus_voltage          = can_spark_max_->GetBusVoltage();
		const double   applied_output       = can_spark_max_->GetAppliedOutput();
		const double   output_current       = can_spark_max_->GetOutputCurrent();
		const double   motor_temperature    = can_spark_max_->GetMotorTemperature();

		// Actually update the SparkMaxHWState shared between
		// this thread and read()
		// Do this all at once so the code minimizes the amount
		// of time with mutex locked
		{
			// Lock the state entry to make sure writes
			// are atomic - reads won't grab data in
			// the middle of a write
			std::lock_guard l(*read_state_mutex_);
			read_thread_state_->setSetPoint(set_point);
			read_thread_state_->setPosition(position);
			read_thread_state_->setVelocity(velocity);
			read_thread_state_->setForwardLimitSwitch(forward_limit_switch);
			read_thread_state_->setReverseLimitSwitch(reverse_limit_switch);
			read_thread_state_->setFaults(faults);
			read_thread_state_->setStickyFaults(sticky_faults);
			read_thread_state_->setBusVoltage(bus_voltage);
			read_thread_state_->setAppliedOutput(applied_output);
			read_thread_state_->setOutputCurrent(output_current);
			read_thread_state_->setMotorTemperature(motor_temperature);
		}
		tracer->report(60);
		rate.sleep();
	}
}

static std::optional<std::string> revErrorCodeToString(const rev::REVLibError can_error)
{
	switch(can_error)
	{
		case rev::REVLibError::kOk:
            return std::nullopt;
        case rev::REVLibError::kError:
			return "kError";
		case rev::REVLibError::kTimeout:
			return "kTimeout";
		case rev::REVLibError::kNotImplemented:
			return "kNotImplemented";
		case rev::REVLibError::kHALError:
			return "kHALError";
		case rev::REVLibError::kCantFindFirmware:
			return "kCantFindFirmware";
		case rev::REVLibError::kFirmwareTooOld:
			return "kFirmwareTooOld";
		case rev::REVLibError::kFirmwareTooNew:
			return "kFirmwareTooNew";
		case rev::REVLibError::kParamInvalidID:
			return "kParamInvalidID";
		case rev::REVLibError::kParamMismatchType:
			return "kParamMismatchType";
		case rev::REVLibError::kParamAccessMode:
			return "kParamAccessMode";
		case rev::REVLibError::kParamInvalid:
			return "kParamInvalid";
		case rev::REVLibError::kParamNotImplementedDeprecated:
			return "kParamNotImplementedDeprecated";
		case rev::REVLibError::kFollowConfigMismatch:
			return "kFollowConfigMismatch";
		case rev::REVLibError::kInvalid:
			return "kInvalid";
		case rev::REVLibError::kSetpointOutOfRange:
			return "kSetpointOutOfRange";
		case rev::REVLibError::kUnknown:
			return "kUnknown";
		case rev::REVLibError::kCANDisconnected:
			return "kCANDisconnected";
		case rev::REVLibError::kDuplicateCANId:
			return "kDuplicateCANId";
		case rev::REVLibError::kInvalidCANId:
			return "kInvalidCANId";
		case rev::REVLibError::kSparkMaxDataPortAlreadyConfiguredDifferently:
			return "kSparkMaxDataPortAlreadyConfiguredDifferently";

        default:
        {
            std::stringstream s;
            s << "Unknown Spark Max error " << static_cast<int>(can_error);
            return s.str();
        }
    }
}

template <bool SIMFLAG>
bool SparkMaxDevice<SIMFLAG>::safeCall(const rev::REVLibError error_code, const std::string &method_name)
{
    const auto error_string = revErrorCodeToString(error_code);
    if (!error_string)
    {
        can_error_count_ = 0;
        can_error_sent_ = false;
        return true;
    }

	ROS_ERROR_STREAM("Error : Spark Max call CANid = " << can_id_ << " calling " << method_name << " : " << *error_string);
	can_error_count_++;
	if ((can_error_count_ > 1000) && !can_error_sent_)
	{
		HAL_SendError(true, -1, false, "safeSparkMaxCall - too many CAN bus errors!", "", "", true);
		can_error_sent_ = true;
	}

    return false;
}

static bool convertMotorType(const hardware_interface::MotorType input,
                             rev::CANSparkMaxLowLevel::MotorType &output)
{
    switch (input)
    {
    case hardware_interface::kBrushed:
        output = rev::CANSparkMaxLowLevel::MotorType::kBrushed;
        break;
    case hardware_interface::kBrushless:
        output = rev::CANSparkMaxLowLevel::MotorType::kBrushless;
        break;
    default:
        ROS_ERROR("Invalid input in convertRevMotorType");
        return false;
    }
    return true;
}
static bool convertLimitSwitchPolarity(const hardware_interface::LimitSwitchPolarity input,
                                       rev::CANDigitalInput::LimitSwitchPolarity &output)
{
    switch (input)
    {
    case hardware_interface::kNormallyOpen:
        output = rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen;
        break;
    case hardware_interface::kNormallyClosed:
        output = rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed;
        break;
    default:
        ROS_ERROR("Invalid input in convertRevLimitSwitchPolarity");
        return false;
    }
    return true;
}

static bool convertEncoderType(const hardware_interface::SensorType input,
                               rev::CANEncoder::EncoderType &output)
{
    switch (input)
    {
    case hardware_interface::kNoSensor:
        output = rev::CANEncoder::EncoderType::kNoSensor;
        break;
    case hardware_interface::kHallSensor:
        output = rev::CANEncoder::EncoderType::kHallSensor;
        break;
    case hardware_interface::kQuadrature:
        output = rev::CANEncoder::EncoderType::kQuadrature;
        break;
    default:
        ROS_ERROR("Invalid input in convertRevEncoderType");
        return false;
    }
    return true;
}

static bool convertControlType(const hardware_interface::ControlType input,
                               rev::ControlType &output)
{
    switch (input)
    {
    case hardware_interface::kDutyCycle:
        output = rev::ControlType::kDutyCycle;
        break;
    case hardware_interface::kVelocity:
        output = rev::ControlType::kVelocity;
        break;
    case hardware_interface::kVoltage:
        output = rev::ControlType::kVoltage;
        break;
    case hardware_interface::kPosition:
        output = rev::ControlType::kPosition;
        break;
    case hardware_interface::kSmartMotion:
        output = rev::ControlType::kSmartMotion;
        break;
    case hardware_interface::kCurrent:
        output = rev::ControlType::kCurrent;
        break;
    case hardware_interface::kSmartVelocity:
        output = rev::ControlType::kSmartVelocity;
        break;

    default:
        ROS_ERROR("Invalid input in convertRevControlType");
        return false;
    }
    return true;
}

static bool convertArbFFUnits(const hardware_interface::ArbFFUnits input,
                              rev::CANPIDController::ArbFFUnits &output)
{
    switch (input)
    {
    case hardware_interface::ArbFFUnits::kVoltage:
        output = rev::CANPIDController::ArbFFUnits::kVoltage;
        break;
    case hardware_interface::ArbFFUnits::kPercentOut:
        output = rev::CANPIDController::ArbFFUnits::kPercentOut;
        break;

    default:
        ROS_ERROR("Invalid input in convertRevControlType");
        return false;
    }
    return true;
}

static bool convertIdleMode(const hardware_interface::IdleMode input,
                            rev::CANSparkMax::IdleMode &output)
{
    switch (input)
    {
    case hardware_interface::IdleMode::kCoast:
        output = rev::CANSparkMax::IdleMode::kCoast;
        break;
    case hardware_interface::IdleMode::kBrake:
        output = rev::CANSparkMax::IdleMode::kBrake;
        break;

    default:
        ROS_ERROR("Invalid input in convertRevIdleMode");
        return false;
    }
    return true;
}

static bool convertExternalFollower(const hardware_interface::ExternalFollower input,
                                    rev::CANSparkMax::ExternalFollower &output)
{
    switch (input)
    {
    case hardware_interface::ExternalFollower::kFollowerDisabled:
        output = rev::CANSparkMax::kFollowerDisabled;
        break;
    case hardware_interface::ExternalFollower::kFollowerSparkMax:
        output = rev::CANSparkMax::kFollowerSparkMax;
        break;
    case hardware_interface::ExternalFollower::kFollowerPhoenix:
        output = rev::CANSparkMax::kFollowerPhoenix;
        break;

    default:
        ROS_ERROR("Invalid input in convertRevExternalFollower");
        return false;
    }
    return true;
}
