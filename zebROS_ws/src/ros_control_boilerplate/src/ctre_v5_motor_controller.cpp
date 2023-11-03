#include <thread>

#include <ros/node_handle.h>

#include <ctre/phoenix/motorcontrol/IMotorController.h>
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h"
#include "ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h"

#include "ctre_interfaces/talon_command_interface.h"
#include "ros_control_boilerplate/ctre_v5_motor_controller.h"
#include "ros_control_boilerplate/get_conversion_factor.h"
#include "ros_control_boilerplate/tracer.h"

static bool convertControlMode(const hardware_interface::TalonMode input_mode,
                               ctre::phoenix::motorcontrol::ControlMode &output_mode);
static bool convertDemand1Type(const hardware_interface::DemandType input,
                               ctre::phoenix::motorcontrol::DemandType &output);
static bool convertNeutralMode(const hardware_interface::NeutralMode input_mode,
                               ctre::phoenix::motorcontrol::NeutralMode &output_mode);
static bool convertFeedbackDevice(const hardware_interface::FeedbackDevice input_fd,
                                  ctre::phoenix::motorcontrol::FeedbackDevice &output_fd);
static bool convertRemoteFeedbackDevice(const hardware_interface::RemoteFeedbackDevice input_fd,
                                        ctre::phoenix::motorcontrol::RemoteFeedbackDevice &output_fd);
static bool convertRemoteSensorSource(const hardware_interface::RemoteSensorSource input_rss,
                                      ctre::phoenix::motorcontrol::RemoteSensorSource &output_rss);
static bool convertLimitSwitchSource(const hardware_interface::LimitSwitchSource input_ls,
                                     ctre::phoenix::motorcontrol::LimitSwitchSource &output_ls);
static bool convertRemoteLimitSwitchSource(const hardware_interface::RemoteLimitSwitchSource input_ls,
                                           ctre::phoenix::motorcontrol::RemoteLimitSwitchSource &output_ls);
static bool convertLimitSwitchNormal(const hardware_interface::LimitSwitchNormal input_ls,
                                     ctre::phoenix::motorcontrol::LimitSwitchNormal &output_ls);
static bool convertVelocityMeasurementPeriod(const hardware_interface::VelocityMeasurementPeriod input_v_m_p,
                                             ctre::phoenix::motorcontrol::VelocityMeasPeriod &output_v_m_period);
static bool convertStatusFrame(const hardware_interface::StatusFrame input,
                               ctre::phoenix::motorcontrol::StatusFrameEnhanced &output);
static bool convertControlFrame(const hardware_interface::ControlFrame input,
                                ctre::phoenix::motorcontrol::ControlFrame &output);
static bool convertMotorCommutation(const hardware_interface::MotorCommutation input,
                                    ctre::phoenix::motorcontrol::MotorCommutation &output);
static bool convertAbsoluteSensorRange(const hardware_interface::AbsoluteSensorRange input,
                                       ctre::phoenix::sensors::AbsoluteSensorRange &output);
static bool convertSensorInitializationStrategy(const hardware_interface::SensorInitializationStrategy input,
                                                ctre::phoenix::sensors::SensorInitializationStrategy &output);

CTREV5MotorController::CTREV5MotorController(const std::string &name_space,
                                             const int joint_index,
                                             const std::string &joint_name,
                                             const std::string &joint_type,
                                             const int can_id,
                                             const std::string &can_bus,
                                             const bool local,
                                             const double read_hz_)
    : CTREV5Device{name_space, "CTREV5MotorController", joint_name, can_id}
    , local_{local}
    , ctre_mc_{nullptr}
    , talon_{nullptr}
    , talon_fx_{nullptr}
    , talon_srx_{nullptr}
    , victor_spx_{nullptr}
    , state_{std::make_unique<hardware_interface::TalonHWState>(can_id)}
    , command_{std::make_unique<hardware_interface::TalonHWCommand>()}
    , read_thread_state_{nullptr}
    , read_state_mutex_{nullptr}
    , read_thread_{nullptr}
{
    ROS_INFO_STREAM_NAMED("frc_robot_interface",
                        "Loading joint " << joint_index << "=" << joint_name <<
                        (local_ ? "local" : "remote") << 
                        " as CTREV5 " << joint_type << " id = " << can_id << " on bus " << can_bus);

    if (local_)
    {
        if (joint_type == "can_talon_fx")
        {
            ctre_mc_ = std::make_shared<ctre::phoenix::motorcontrol::can::WPI_TalonFX>(can_id, can_bus);
        }
        else if (joint_type == "can_talon_srx")
        {
            ctre_mc_ = std::make_shared<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(can_id);
        }
        else if (joint_type == "can_victor_spx")
        {
            ctre_mc_ = std::make_shared<ctre::phoenix::motorcontrol::can::WPI_VictorSPX>(can_id);
        }
        // Set up various pointers to the class hierarchy for motor controllers
        // These will be nullptr if the allocated ctre_mc_ object doesn't support
        // the features of a specific controller type.
        // If non-null, these are used to access the method calls for those particular features
        talon_ = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::IMotorControllerEnhanced>(ctre_mc_);
        talon_fx_ = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::can::WPI_TalonFX>(ctre_mc_);
        talon_srx_ = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(ctre_mc_);
        victor_spx_ = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::IMotorController>(ctre_mc_);

        read_thread_state_ = std::make_unique<hardware_interface::TalonHWState>(can_id);
        read_state_mutex_ = std::make_unique<std::mutex>();
        read_thread_ = std::make_unique<std::jthread>(&CTREV5MotorController::read_thread, this,
                                                      std::make_unique<Tracer>("ctre_mc_read_" + joint_name + " " + name_space),
                                                      read_hz_);

        // Clear this out so we only get resets that occur after the controllers have been initialized
        ctre_mc_->HasResetOccurred();
    }
}

CTREV5MotorController::~CTREV5MotorController(void) = default;

void CTREV5MotorController::registerInterfaces(hardware_interface::TalonStateInterface &state_interface,
                                               hardware_interface::TalonCommandInterface &command_interface,
                                               hardware_interface::RemoteTalonStateInterface &remote_state_interface) const
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for CTREV5 : " << getName() << " at CAN id " << getId());

    hardware_interface::TalonStateHandle state_handle(getName(), state_.get());
    state_interface.registerHandle(state_handle);

    hardware_interface::TalonCommandHandle command_handle(state_handle, command_.get());
    command_interface.registerHandle(command_handle);

    if (!local_)
    {
        hardware_interface::TalonWritableStateHandle remote_handle(getName(), state_.get());
        remote_state_interface.registerHandle(remote_handle);
    }
}

void CTREV5MotorController::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (local_)
    {
        std::unique_lock l(*read_state_mutex_, std::try_to_lock);
        if (!l.owns_lock())
        {
            return;
        }

        // Copy config items from talon state to talon_read_thread_state
        // This makes sure config items set by controllers is
        // eventually reflected in the state unique to the
        // talon_read_thread code
        read_thread_state_->setTalonMode(state_->getTalonMode());
        read_thread_state_->setEncoderFeedback(state_->getEncoderFeedback());
        read_thread_state_->setEncoderTicksPerRotation(state_->getEncoderTicksPerRotation());
        read_thread_state_->setConversionFactor(state_->getConversionFactor());
        read_thread_state_->setEnableReadThread(state_->getEnableReadThread());
        // There looks like a bug in sim which requires us to read these
        // more slowly.  Pass the previously-read value in to use as
        // a default for iterations where the value isn't read
        //read_thread_state_->setBusVoltage(state_->getBusVoltage());
        //read_thread_state_->setTemperature(state_->getTemperature());

        // Copy talon state values read in the read thread into the
        // talon state shared globally with the rest of the hardware
        // interface code
        state_->setPosition(read_thread_state_->getPosition());
        state_->setSpeed(read_thread_state_->getSpeed());
        state_->setOutputCurrent(read_thread_state_->getOutputCurrent());
        state_->setStatorCurrent(read_thread_state_->getStatorCurrent());
        state_->setSupplyCurrent(read_thread_state_->getSupplyCurrent());
        state_->setBusVoltage(read_thread_state_->getBusVoltage());
        state_->setMotorOutputPercent(read_thread_state_->getMotorOutputPercent());
        state_->setOutputVoltage(read_thread_state_->getOutputVoltage());
        state_->setTemperature(read_thread_state_->getTemperature());
        state_->setClosedLoopError(read_thread_state_->getClosedLoopError());
        state_->setIntegralAccumulator(read_thread_state_->getIntegralAccumulator());
        state_->setErrorDerivative(read_thread_state_->getErrorDerivative());
        state_->setClosedLoopTarget(read_thread_state_->getClosedLoopTarget());
        state_->setActiveTrajectoryPosition(read_thread_state_->getActiveTrajectoryPosition());
        state_->setActiveTrajectoryVelocity(read_thread_state_->getActiveTrajectoryVelocity());
        state_->setActiveTrajectoryHeading(read_thread_state_->getActiveTrajectoryHeading());
        state_->setMotionProfileTopLevelBufferCount(read_thread_state_->getMotionProfileTopLevelBufferCount());
        state_->setMotionProfileStatus(read_thread_state_->getMotionProfileStatus());
        state_->setFaults(read_thread_state_->getFaults());
        state_->setForwardLimitSwitch(read_thread_state_->getForwardLimitSwitch());
        state_->setReverseLimitSwitch(read_thread_state_->getReverseLimitSwitch());
        state_->setForwardSoftlimitHit(read_thread_state_->getForwardSoftlimitHit());
        state_->setReverseSoftlimitHit(read_thread_state_->getReverseSoftlimitHit());
        state_->setStickyFaults(read_thread_state_->getStickyFaults());
        state_->setFirmwareVersion(read_thread_state_->getFirmwareVersion());
        state_->setPTerm(read_thread_state_->getPTerm());
        state_->setITerm(read_thread_state_->getITerm());
        state_->setDTerm(read_thread_state_->getDTerm());
        state_->setFTerm(read_thread_state_->getFTerm());
    }
}

void CTREV5MotorController::write(const ros::Time &/*time*/, const ros::Duration &/*period*/, const bool robot_enabled, const bool prev_robot_enabled)
{
    if (!local_)
    {
        return;
    }

    std::unique_lock l{*command_, std::try_to_lock};
    if (!l.owns_lock())
    {
        return;
    }

    // TODO : skip over most or all of this if the talon is in follower mode
    //        Only do the Set() call and then never do anything else?

    // If the motor controller has been reset since the last write()
    // call, reset all of the flags indicating that commands have been
    // written to the controllers. This will force the config data
    // to be re-written by the rest of the write() function
    if (victor_spx_->HasResetOccurred())
    {
        ROS_WARN_STREAM("Detected reset on CTRE mc " << getName());
        for (size_t i = 0; i < hardware_interface::TALON_PIDF_SLOTS; i++)
        {
            command_->resetPIDF(i);
        }
        command_->resetAuxPidPolarity();
        command_->resetIntegralAccumulator();
        command_->resetMode();
        command_->resetDemand1();
        command_->resetPidfSlot();
        command_->resetEncoderFeedback();
        // This should be deprecated anyway -
        // for now, comment it out to prevent it from overwriting
        // the main encoder feedback source above
        // command_->resetRemoteEncoderFeedback();
        command_->resetRemoteFeedbackFilters();
        command_->resetSensorTerms();
        command_->resetOutputShaping();
        command_->resetVoltageCompensation();
        command_->resetVelocityMeasurement();
        //command_->resetSensorPosition();
        command_->resetLimitSwitchesSource();
        command_->resetRemoteLimitSwitchesSource();
        command_->resetSoftLimit();
        command_->resetCurrentLimit();
        command_->resetMotionCruise();
        for (int i = hardware_interface::Status_1_General; i < hardware_interface::Status_Last; i++)
        {
            command_->resetStatusFramePeriod(static_cast<hardware_interface::StatusFrame>(i));
        }
        for (int i = hardware_interface::Control_3_General; i < hardware_interface::Control_Last; i++)
        {
            command_->resetControlFramePeriod(static_cast<hardware_interface::ControlFrame>(i));
        }
        command_->resetMotionProfileTrajectoryPeriod();
        command_->resetSupplyCurrentLimit();
        command_->resetStatorCurrentLimit();
        command_->resetMotorCommutation();
        command_->resetAbsoluteSensorRange();
        command_->resetSensorInitializationStrategy();
        command_->resetClearPositionOnLimitF();
        command_->resetClearPositionOnLimitR();
    }

    if (bool enable_read_thread; command_->enableReadThreadChanged(enable_read_thread))
    {
        state_->setEnableReadThread(enable_read_thread);
    }

    hardware_interface::FeedbackDevice internal_feedback_device = hardware_interface::FeedbackDevice_Uninitialized;
    double feedback_coefficient;

    ctre::phoenix::motorcontrol::FeedbackDevice talon_feedback_device;
    if (command_->encoderFeedbackChanged(internal_feedback_device, feedback_coefficient) &&
        convertFeedbackDevice(internal_feedback_device, talon_feedback_device))
    {
        // Check for errors on Talon writes. If it fails, used the reset() call to
        // set the changed var for the config items to true. This will trigger a re-try
        // the next time through the loop.
        bool rc = true;
        // Only actually set this on the hardware for Talon devices. But set it in
        // talon_states for both types of motor controllers. This allows the conversion
        // functions to work properly?
        if (talon_)
        {
            rc = safeConfigCall(talon_->ConfigSelectedFeedbackSensor(talon_feedback_device, pidIdx, configTimeoutMs),"ConfigSelectedFeedbackSensor");
            if (rc)
            {
                rc = safeConfigCall(talon_->ConfigSelectedFeedbackCoefficient(feedback_coefficient, pidIdx, configTimeoutMs),"ConfigSelectedFeedbackCoefficient");
            }
        }
        if (rc)
        {
            ROS_INFO_STREAM("Updated joint " << getName() << " feedback");
            state_->setEncoderFeedback(internal_feedback_device);
            state_->setFeedbackCoefficient(feedback_coefficient);
        }
        else
        {
            ROS_WARN_STREAM("Failed to update joint " << getName() << " feedback");
            command_->resetEncoderFeedback();
            return;
        }
    }

    ctre::phoenix::motorcontrol::RemoteFeedbackDevice talon_remote_feedback_device;
    hardware_interface::RemoteFeedbackDevice internal_remote_feedback_device;
    if (command_->remoteEncoderFeedbackChanged(internal_remote_feedback_device) &&
        convertRemoteFeedbackDevice(internal_remote_feedback_device, talon_remote_feedback_device))
    {
        // Check for errors on Talon writes. If it fails, used the reset() call to
        // set the changed var for the config items to true. This will trigger a re-try
        // the next time through the loop.
        if (safeConfigCall(victor_spx_->ConfigSelectedFeedbackSensor(talon_remote_feedback_device, pidIdx, configTimeoutMs), "ConfigSelectedFeedbackSensor (Remote)"))
        {
            ROS_INFO_STREAM("Updated joint " << getName() << " remote feedback sensor");
            state_->setRemoteEncoderFeedback(internal_remote_feedback_device);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update joint " << getName() << " remote feedback sensor");
            command_->resetRemoteEncoderFeedback();
            return;
        }
    }

    std::array<int, 2>                                             remote_feedback_device_ids;
    std::array<hardware_interface::RemoteSensorSource, 2>          internal_remote_feedback_filters;
    std::array<ctre::phoenix::motorcontrol::RemoteSensorSource, 2> victor_remote_feedback_filters;
    if (command_->remoteFeedbackFiltersChanged(remote_feedback_device_ids, internal_remote_feedback_filters) &&
        convertRemoteSensorSource(internal_remote_feedback_filters[0], victor_remote_feedback_filters[0]) &&
        convertRemoteSensorSource(internal_remote_feedback_filters[1], victor_remote_feedback_filters[1]))
    {
        bool rc = safeConfigCall(victor_spx_->ConfigRemoteFeedbackFilter(remote_feedback_device_ids[0], victor_remote_feedback_filters[0], 0, configTimeoutMs), "ConfigRemoteFeedbackFilter (0)");
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigRemoteFeedbackFilter(remote_feedback_device_ids[1], victor_remote_feedback_filters[1], 1, configTimeoutMs), "ConfigRemoteFeedbackFilter (1)");
        }
        if (rc)
        {
            ROS_INFO_STREAM("Updated joint " << getName() << " remote feedback filters" <<
                    " id[0] = " << remote_feedback_device_ids[0] <<
                    " id[1] = " << remote_feedback_device_ids[1] <<
                    " filter[0] = " << internal_remote_feedback_filters[0] <<
                    " filter[1] = " << internal_remote_feedback_filters[1]);
            state_->setRemoteFeedbackDeviceIds(remote_feedback_device_ids);
            state_->setRemoteFeedbackFilters(internal_remote_feedback_filters);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update joint " << getName() << " remote feedback filters");
            command_->resetRemoteFeedbackFilters();
            return;
        }
    }

    std::array<hardware_interface::FeedbackDevice, hardware_interface::SensorTerm_Last> internal_sensor_terms;
    std::array<ctre::phoenix::motorcontrol::FeedbackDevice, hardware_interface::SensorTerm_Last> victor_sensor_terms;
    if (command_->sensorTermsChanged(internal_sensor_terms) &&
        convertFeedbackDevice(internal_sensor_terms[0], victor_sensor_terms[0]) &&
        convertFeedbackDevice(internal_sensor_terms[1], victor_sensor_terms[1]) &&
        convertFeedbackDevice(internal_sensor_terms[2], victor_sensor_terms[2]) &&
        convertFeedbackDevice(internal_sensor_terms[3], victor_sensor_terms[3]))
    {
        // Check for errors on Talon writes. If it fails, used the reset() call to
        // set the changed var for the config items to true. This will trigger a re-try
        // the next time through the loop.
        bool rc = safeConfigCall(victor_spx_->ConfigSensorTerm(ctre::phoenix::motorcontrol::SensorTerm::SensorTerm_Sum0, victor_sensor_terms[0], configTimeoutMs),"ConfigSensorTerm Sum0");
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigSensorTerm(ctre::phoenix::motorcontrol::SensorTerm::SensorTerm_Sum1, victor_sensor_terms[1], configTimeoutMs),"ConfigSensorTerm Sum1");
        }
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigSensorTerm(ctre::phoenix::motorcontrol::SensorTerm::SensorTerm_Diff0, victor_sensor_terms[2], configTimeoutMs),"ConfigSensorTerm Diff0");
        }
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigSensorTerm(ctre::phoenix::motorcontrol::SensorTerm::SensorTerm_Diff1, victor_sensor_terms[3], configTimeoutMs),"ConfigSensorTerm Diff1");
        }
        if (rc)
        {
            ROS_INFO_STREAM("Updated joint " << getName() << " sensor terms");
            state_->setSensorTerms(internal_sensor_terms);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update joint " << getName() << " sensor terms");
            command_->resetSensorTerms();
            return;
        }
    }

    // Get mode that is about to be commanded
    const hardware_interface::TalonMode talon_mode = command_->getMode();
    const int encoder_ticks_per_rotation = command_->getEncoderTicksPerRotation();
    state_->setEncoderTicksPerRotation(encoder_ticks_per_rotation);

    const double conversion_factor = command_->getConversionFactor();
    // No point doing changed() since it's quicker just to just copy a double
    // rather than query a bool and do it conditionally
    state_->setConversionFactor(conversion_factor);

    const double radians_scale = getConversionFactor(encoder_ticks_per_rotation, internal_feedback_device, hardware_interface::TalonMode_Position) * conversion_factor;
    const double radians_per_second_scale = getConversionFactor(encoder_ticks_per_rotation, internal_feedback_device, hardware_interface::TalonMode_Velocity) * conversion_factor;
    const double closed_loop_scale = getConversionFactor(encoder_ticks_per_rotation, internal_feedback_device, talon_mode) * conversion_factor;

    int slot;
    const bool slot_changed = command_->slotChanged(slot);

    double pval;
    double ival;
    double dval;
    double fval;
    double iz;
    double allowable_closed_loop_error;
    double max_integral_accumulator;
    double closed_loop_peak_output;
    int    closed_loop_period;

    if (command_->pidfChanged(pval, ival, dval, fval, iz, allowable_closed_loop_error, max_integral_accumulator, closed_loop_peak_output, closed_loop_period, slot))
    {
        bool rc = safeConfigCall(victor_spx_->Config_kP(slot, pval, configTimeoutMs), "Config_kP");
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->Config_kI(slot, ival, configTimeoutMs), "Config_kI");
        }
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->Config_kD(slot, dval, configTimeoutMs), "Config_kD");
        }
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->Config_kF(slot, fval, configTimeoutMs), "Config_kF");
        }
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->Config_IntegralZone(slot, iz / closed_loop_scale, configTimeoutMs), "Config_IntegralZone");
        }
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigAllowableClosedloopError(slot, allowable_closed_loop_error / closed_loop_scale, configTimeoutMs), "ConfigAllowableClosedloopError");
        }
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigMaxIntegralAccumulator(slot, max_integral_accumulator, configTimeoutMs), "ConfigMaxIntegralAccumulator");
        }
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigClosedLoopPeakOutput(slot, closed_loop_peak_output, configTimeoutMs), "ConfigClosedLoopPeakOutput");
        }
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigClosedLoopPeriod(slot, closed_loop_period, configTimeoutMs), "ConfigClosedLoopPeriod");
        }

        if (rc)
        {
            ROS_INFO_STREAM("Updated joint " << getName() << " PIDF slot " << slot << " config values");
            state_->setPidfP(pval, slot);
            state_->setPidfI(ival, slot);
            state_->setPidfD(dval, slot);
            state_->setPidfF(fval, slot);
            state_->setPidfIzone(iz, slot);
            state_->setAllowableClosedLoopError(allowable_closed_loop_error, slot);
            state_->setMaxIntegralAccumulator(max_integral_accumulator, slot);
            state_->setClosedLoopPeakOutput(closed_loop_peak_output, slot);
            state_->setClosedLoopPeriod(closed_loop_period, slot);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update joint " << getName() << " PIDF slot " << slot << " config values");
            command_->resetPIDF(slot);
            return;
        }
    }

    bool aux_pid_polarity;
    if (command_->auxPidPolarityChanged(aux_pid_polarity))
    {
        if (safeConfigCall(victor_spx_->ConfigAuxPIDPolarity(aux_pid_polarity, configTimeoutMs), "ConfigAuxPIDPolarity"))
        {
            ROS_INFO_STREAM("Updated joint " << getName() <<
                    " AUX PIDF polarity to " << aux_pid_polarity << std::endl);
            state_->setAuxPidPolarity(aux_pid_polarity);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update joint " << getName() << " AUX PIDF polarity");
            command_->resetAuxPidPolarity();
            return;
        }
    }

    if (slot_changed)
    {
        if (safeConfigCall(victor_spx_->SelectProfileSlot(slot, pidIdx), "SelectProfileSlot"))
        {
            ROS_INFO_STREAM("Updated joint " << getName() <<
                    " PIDF slot to " << slot << std::endl);
            state_->setSlot(slot);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update joint " << getName() << " PIDF slot");
            command_->resetPidfSlot();
            return;
        }
    }

    bool invert;
    bool sensor_phase;
    if (command_->invertChanged(invert, sensor_phase))
    {
        ROS_INFO_STREAM("Updated joint " << getName() <<
                " invert = " << invert << " phase = " << sensor_phase);
        // TODO : can these calls fail? If so, what to do if they do?
        victor_spx_->SetInverted(invert);
        victor_spx_->SetSensorPhase(sensor_phase);
        state_->setInvert(invert);
        state_->setSensorPhase(sensor_phase);
    }

    hardware_interface::NeutralMode neutral_mode;
    ctre::phoenix::motorcontrol::NeutralMode ctre_neutral_mode;
    if (command_->neutralModeChanged(neutral_mode) &&
        convertNeutralMode(neutral_mode, ctre_neutral_mode))
    {

        ROS_INFO_STREAM("Updated joint " << getName() << " neutral mode");
        // TODO : can this call fail? If so, what to do if they do?
        victor_spx_->SetNeutralMode(ctre_neutral_mode);
        state_->setNeutralMode(neutral_mode);
    }

    double iaccum;
    if (command_->integralAccumulatorChanged(iaccum))
    {
        //The units on this aren't really right?
        if (safeConfigCall(victor_spx_->SetIntegralAccumulator(iaccum / closed_loop_scale, pidIdx, configTimeoutMs), "SetIntegralAccumulator"))
        {
            ROS_INFO_STREAM("Updated joint " << getName() << " integral accumulator");
            // Do not set talon state - this changes
            // dynamically so read it in read() above instead
        }
        else
        {
            ROS_INFO_STREAM("Failed to update joint " << getName() << " integral accumulator");
            command_->resetIntegralAccumulator();
            return;
        }
    }

    double closed_loop_ramp;
    double open_loop_ramp;
    double peak_output_forward;
    double peak_output_reverse;
    double nominal_output_forward;
    double nominal_output_reverse;
    double neutral_deadband;
    if (command_->outputShapingChanged(closed_loop_ramp,
                                       open_loop_ramp,
                                       peak_output_forward,
                                       peak_output_reverse,
                                       nominal_output_forward,
                                       nominal_output_reverse,
                                       neutral_deadband))
    {
        bool rc = safeConfigCall(victor_spx_->ConfigOpenloopRamp(open_loop_ramp, configTimeoutMs),"ConfigOpenloopRamp");
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigClosedloopRamp(closed_loop_ramp, configTimeoutMs),"ConfigClosedloopRamp");
        }
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigPeakOutputForward(peak_output_forward, configTimeoutMs),"ConfigPeakOutputForward");          // 100
        }
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigPeakOutputReverse(peak_output_reverse, configTimeoutMs),"ConfigPeakOutputReverse");          // -100
        }
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigNominalOutputForward(nominal_output_forward, configTimeoutMs),"ConfigNominalOutputForward"); // 0
        }
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigNominalOutputReverse(nominal_output_reverse, configTimeoutMs),"ConfigNominalOutputReverse"); // 0
        }
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigNeutralDeadband(neutral_deadband, configTimeoutMs),"ConfigNeutralDeadband");                 // 0
        }

        if (rc)
        {
            state_->setOpenloopRamp(open_loop_ramp);
            state_->setClosedloopRamp(closed_loop_ramp);
            state_->setPeakOutputForward(peak_output_forward);
            state_->setPeakOutputReverse(peak_output_reverse);
            state_->setNominalOutputForward(nominal_output_forward);
            state_->setNominalOutputReverse(nominal_output_reverse);
            state_->setNeutralDeadband(neutral_deadband);
            ROS_INFO_STREAM("Updated joint " << getName() << " output shaping" <<
                    " closed_loop_ramp = " << closed_loop_ramp <<
                    " open_loop_ramp = " << open_loop_ramp <<
                    " peak_output_forward = " << peak_output_forward <<
                    " peak_output_reverse = " << peak_output_reverse <<
                    " nominal_output_forward = " << nominal_output_forward <<
                    " nominal_output_reverse = " << nominal_output_reverse <<
                    " neutral_deadband = " << neutral_deadband);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update joint " << getName() << " output shaping");
            command_->resetOutputShaping();
            return;
        }
    }

    double v_c_saturation;
    int v_measurement_filter;
    bool v_c_enable;
    if (command_->voltageCompensationChanged(v_c_saturation,
        v_measurement_filter,
        v_c_enable))
    {
        bool rc = safeConfigCall(victor_spx_->ConfigVoltageCompSaturation(v_c_saturation, configTimeoutMs),"ConfigVoltageCompSaturation");
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigVoltageMeasurementFilter(v_measurement_filter, configTimeoutMs),"ConfigVoltageMeasurementFilter");
        }

        if (rc)
        {
            // Only enable once settings are correctly written to the Talon
            victor_spx_->EnableVoltageCompensation(v_c_enable);

            ROS_INFO_STREAM("Updated joint " << getName() << " voltage compensation");

            state_->setVoltageCompensationSaturation(v_c_saturation);
            state_->setVoltageMeasurementFilter(v_measurement_filter);
            state_->setVoltageCompensationEnable(v_c_enable);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update joint " << getName() << " voltage compensation");
            command_->resetVoltageCompensation();
            return;
        }
    }

    if (talon_)
    {
        hardware_interface::VelocityMeasurementPeriod internal_v_m_period;
        ctre::phoenix::motorcontrol::VelocityMeasPeriod phoenix_v_m_period;
        int v_m_window;

        if (command_->velocityMeasurementChanged(internal_v_m_period, v_m_window) &&
            convertVelocityMeasurementPeriod(internal_v_m_period, phoenix_v_m_period))
        {
            bool rc = safeConfigCall(talon_->ConfigVelocityMeasurementPeriod(phoenix_v_m_period, configTimeoutMs),"ConfigVelocityMeasurementPeriod");
            if (rc)
            {
                rc = safeConfigCall(talon_->ConfigVelocityMeasurementWindow(v_m_window, configTimeoutMs),"ConfigVelocityMeasurementWindow");
            }

            if (rc)
            {
                ROS_INFO_STREAM("Updated joint " << getName() << " velocity measurement period / window");
                state_->setVelocityMeasurementPeriod(internal_v_m_period);
                state_->setVelocityMeasurementWindow(v_m_window);
            }
            else
            {
                ROS_INFO_STREAM("Failed to update joint " << getName() << " velocity measurement period / window");
                command_->resetVelocityMeasurement();
                return;
            }
        }
    }

    double sensor_position;
    if (command_->sensorPositionChanged(sensor_position))
    {
        if (safeConfigCall(victor_spx_->SetSelectedSensorPosition(sensor_position / radians_scale, pidIdx, configTimeoutMs),
                    "SetSelectedSensorPosition"))
        {
            // TODO note we commented this out
            //ROS_INFO_STREAM("Updated joint " << getName() << " selected sensor position to " << sensor_position / radians_scale);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update joint " << getName() << " selected sensor position to " << sensor_position / radians_scale);
            command_->resetSensorPosition();
            return;
        }
    }

    if (talon_)
    {
        hardware_interface::LimitSwitchSource internal_local_forward_source;
        hardware_interface::LimitSwitchNormal internal_local_forward_normal;
        hardware_interface::LimitSwitchSource internal_local_reverse_source;
        hardware_interface::LimitSwitchNormal internal_local_reverse_normal;
        ctre::phoenix::motorcontrol::LimitSwitchSource talon_local_forward_source;
        ctre::phoenix::motorcontrol::LimitSwitchNormal talon_local_forward_normal;
        ctre::phoenix::motorcontrol::LimitSwitchSource talon_local_reverse_source;
        ctre::phoenix::motorcontrol::LimitSwitchNormal talon_local_reverse_normal;
        if (command_->limitSwitchesSourceChanged(internal_local_forward_source, internal_local_forward_normal,
                                                 internal_local_reverse_source, internal_local_reverse_normal) &&
            convertLimitSwitchSource(internal_local_forward_source, talon_local_forward_source) &&
            convertLimitSwitchNormal(internal_local_forward_normal, talon_local_forward_normal) &&
            convertLimitSwitchSource(internal_local_reverse_source, talon_local_reverse_source) &&
            convertLimitSwitchNormal(internal_local_reverse_normal, talon_local_reverse_normal))
        {
            bool rc = safeConfigCall(talon_->ConfigForwardLimitSwitchSource(talon_local_forward_source, talon_local_forward_normal, configTimeoutMs),"ConfigForwardLimitSwitchSource");
            if (rc)
            {
                rc = safeConfigCall(talon_->ConfigReverseLimitSwitchSource(talon_local_reverse_source, talon_local_reverse_normal, configTimeoutMs),"ConfigReverseLimitSwitchSource");
            }

            if (rc)
            {
                ROS_INFO_STREAM("Updated joint " << getName()
                        << " limit switches "
                        << talon_local_forward_source << " " << talon_local_forward_normal << " "
                        << talon_local_reverse_source << " " << talon_local_reverse_normal);
                state_->setForwardLimitSwitchSource(internal_local_forward_source, internal_local_forward_normal);
                state_->setReverseLimitSwitchSource(internal_local_reverse_source, internal_local_reverse_normal);
            }
            else
            {
                ROS_INFO_STREAM("Failed to update joint " << getName() << " limit switches");
                command_->resetLimitSwitchesSource();
                return;
            }
        }
    }
    hardware_interface::RemoteLimitSwitchSource internal_remote_forward_source;
    hardware_interface::LimitSwitchNormal internal_remote_forward_normal;
    unsigned int remote_forward_id;
    hardware_interface::RemoteLimitSwitchSource internal_remote_reverse_source;
    hardware_interface::LimitSwitchNormal internal_remote_reverse_normal;
    unsigned int remote_reverse_id;
    ctre::phoenix::motorcontrol::RemoteLimitSwitchSource talon_remote_forward_source;
    ctre::phoenix::motorcontrol::LimitSwitchNormal talon_remote_forward_normal;
    ctre::phoenix::motorcontrol::RemoteLimitSwitchSource talon_remote_reverse_source;
    ctre::phoenix::motorcontrol::LimitSwitchNormal talon_remote_reverse_normal;
    if (command_->remoteLimitSwitchesSourceChanged(internal_remote_forward_source, internal_remote_forward_normal, remote_forward_id,
                                                   internal_remote_reverse_source, internal_remote_reverse_normal, remote_reverse_id) &&
        convertRemoteLimitSwitchSource(internal_remote_forward_source, talon_remote_forward_source) &&
        convertLimitSwitchNormal(internal_remote_forward_normal, talon_remote_forward_normal) &&
        convertRemoteLimitSwitchSource(internal_remote_reverse_source, talon_remote_reverse_source) &&
        convertLimitSwitchNormal(internal_remote_reverse_normal, talon_remote_reverse_normal))
    {
        bool rc = safeConfigCall(victor_spx_->ConfigForwardLimitSwitchSource(talon_remote_forward_source, talon_remote_forward_normal, remote_forward_id, configTimeoutMs),"ConfigForwardLimitSwitchSource(Remote)");
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigReverseLimitSwitchSource(talon_remote_reverse_source, talon_remote_reverse_normal, remote_reverse_id, configTimeoutMs),"ConfigReverseLimitSwitchSource(Remote)");
        }

        if (rc)
        {
            ROS_INFO_STREAM("Updated joint " << getName()
                    << " remote limit switches "
                    << talon_remote_forward_source << " " << talon_remote_forward_normal << " " << remote_forward_id << " "
                    << talon_remote_reverse_source << " " << talon_remote_reverse_normal << " " << remote_reverse_id);
            state_->setRemoteForwardLimitSwitchSource(internal_remote_forward_source, internal_remote_forward_normal, remote_forward_id);
            state_->setRemoteReverseLimitSwitchSource(internal_remote_reverse_source, internal_remote_reverse_normal, remote_reverse_id);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update joint " << getName() << " remote limit switches");
            command_->resetRemoteLimitSwitchesSource();
            return;
        }
    }

    bool clear_position_on_limit_f;
    if (command_->clearPositionOnLimitFChanged(clear_position_on_limit_f))
    {
        if (safeConfigCall(victor_spx_->ConfigClearPositionOnLimitF(clear_position_on_limit_f, configTimeoutMs), "ConfigClearPositionOnLimitF"))
        {
            ROS_INFO_STREAM("Updated joint " << getName()
                    << " clear position on limit F = " << clear_position_on_limit_f);
            state_->setClearPositionOnLimitF(clear_position_on_limit_f);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update joint " << getName()
                    << " clear position on limit F");
            command_->resetClearPositionOnLimitF();
            return;
        }

    }

    bool clear_position_on_limit_r;
    if (command_->clearPositionOnLimitRChanged(clear_position_on_limit_r))
    {
        if (safeConfigCall(victor_spx_->ConfigClearPositionOnLimitR(clear_position_on_limit_f, configTimeoutMs), "ConfigClearPositionOnLimitR"))
        {
            ROS_INFO_STREAM("Updated joint " << getName()
                    << " clear position on limit R = " << clear_position_on_limit_r);
            state_->setClearPositionOnLimitR(clear_position_on_limit_r);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update joint " << getName()
                    << " clear position on limit R");
            command_->resetClearPositionOnLimitR();
            return;
        }

    }

    double softlimit_forward_threshold;
    bool softlimit_forward_enable;
    double softlimit_reverse_threshold;
    bool softlimit_reverse_enable;
    bool softlimit_override_enable;
    if (command_->softLimitChanged(softlimit_forward_threshold,
                            softlimit_forward_enable,
                            softlimit_reverse_threshold,
                            softlimit_reverse_enable,
                            softlimit_override_enable))
    {
        const double softlimit_forward_threshold_NU = softlimit_forward_threshold / radians_scale; //native units
        const double softlimit_reverse_threshold_NU = softlimit_reverse_threshold / radians_scale;
        bool rc = safeConfigCall(victor_spx_->ConfigForwardSoftLimitThreshold(softlimit_forward_threshold_NU, configTimeoutMs),"ConfigForwardSoftLimitThreshold");
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigForwardSoftLimitEnable(softlimit_forward_enable, configTimeoutMs),"ConfigForwardSoftLimitEnable");
        }
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigReverseSoftLimitThreshold(softlimit_reverse_threshold_NU, configTimeoutMs),"ConfigReverseSoftLimitThreshold");
        }
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigReverseSoftLimitEnable(softlimit_reverse_enable, configTimeoutMs),"ConfigReverseSoftLimitEnable");
        }

        if (rc)
        {
            // Only set override enable if all config calls succeed
            victor_spx_->OverrideSoftLimitsEnable(softlimit_override_enable);
            state_->setOverrideSoftLimitsEnable(softlimit_override_enable);
            state_->setForwardSoftLimitThreshold(softlimit_forward_threshold);
            state_->setForwardSoftLimitEnable(softlimit_forward_enable);
            state_->setReverseSoftLimitThreshold(softlimit_reverse_threshold);
            state_->setReverseSoftLimitEnable(softlimit_reverse_enable);
            ROS_INFO_STREAM("Updated joint " << getName() << " soft limits " <<
                    std::endl << "\tforward enable=" << softlimit_forward_enable << " forward threshold=" << softlimit_forward_threshold <<
                    std::endl << "\treverse enable=" << softlimit_reverse_enable << " reverse threshold=" << softlimit_reverse_threshold <<
                    std::endl << "\toverride_enable=" << softlimit_override_enable);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update joint " << getName() << " soft limits");
            command_->resetSoftLimit();
            return;
        }
    }

    if (talon_srx_)
    {
        int peak_amps;
        int peak_msec;
        int continuous_amps;
        bool enable;
        if (command_->currentLimitChanged(peak_amps, peak_msec, continuous_amps, enable))
        {
            bool rc = safeConfigCall(talon_srx_->ConfigPeakCurrentLimit(peak_amps, configTimeoutMs),"ConfigPeakCurrentLimit");
            if (rc)
            {
                rc = safeConfigCall(talon_srx_->ConfigPeakCurrentDuration(peak_msec, configTimeoutMs),"ConfigPeakCurrentDuration");
            }
            if (rc)
            {
                rc = safeConfigCall(talon_srx_->ConfigContinuousCurrentLimit(continuous_amps, configTimeoutMs),"ConfigContinuousCurrentLimit");
            }
            if (rc)
            {
                // Only enable current limit if all config calls succeed
                talon_srx_->EnableCurrentLimit(enable);
                ROS_INFO_STREAM("Updated joint " << getName() << " peak current");
                state_->setPeakCurrentLimit(peak_amps);
                state_->setPeakCurrentDuration(peak_msec);
                state_->setContinuousCurrentLimit(continuous_amps);
                state_->setCurrentLimitEnable(enable);
            }
            else
            {
                ROS_INFO_STREAM("Failed to update joint " << getName() << " peak current");
                command_->resetCurrentLimit();
                return;
            }
        }
    }

    if (talon_fx_)
    {
        double limit;
        double trigger_threshold_current;
        double trigger_threshold_time;
        bool   limit_enable;
        if (command_->supplyCurrentLimitChanged(limit, trigger_threshold_current, trigger_threshold_time, limit_enable))
        {
            if (safeConfigCall(talon_fx_->ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(limit_enable, limit, trigger_threshold_current, trigger_threshold_time), configTimeoutMs), "ConfigSupplyCurrentLimit"))
            {
                ROS_INFO_STREAM("Updated joint " << getName() << " supply current limit");
                state_->setSupplyCurrentLimit(limit);
                state_->setSupplyCurrentLimitEnable(limit_enable);
                state_->setSupplyCurrentTriggerThresholdCurrent(trigger_threshold_current);
                state_->setSupplyCurrentTriggerThresholdTime(trigger_threshold_time);
            }
            else
            {
                ROS_INFO_STREAM("Failed to update joint " << getName() << " supply current limit");
                command_->resetSupplyCurrentLimit();
                return;
            }
        }
        if (command_->statorCurrentLimitChanged(limit, trigger_threshold_current, trigger_threshold_time, limit_enable))
        {
            if (safeConfigCall(talon_fx_->ConfigStatorCurrentLimit(ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration(limit_enable, limit, trigger_threshold_current, trigger_threshold_time), configTimeoutMs), "ConfigStatorCurrentLimit"))
            {
                ROS_INFO_STREAM("Updated joint " << getName() << " stator current limit");
                state_->setStatorCurrentLimit(limit);
                state_->setStatorCurrentLimitEnable(limit_enable);
                state_->setStatorCurrentTriggerThresholdCurrent(trigger_threshold_current);
                state_->setStatorCurrentTriggerThresholdTime(trigger_threshold_time);
            }
            else
            {
                ROS_INFO_STREAM("Failed to update joint " << getName() << " stator current limit");
                command_->resetStatorCurrentLimit();
                return;
            }
        }

        hardware_interface::MotorCommutation motor_commutation;
        ctre::phoenix::motorcontrol::MotorCommutation motor_commutation_ctre;
        if (command_->motorCommutationChanged(motor_commutation) &&
            convertMotorCommutation(motor_commutation, motor_commutation_ctre))
        {
            if (safeConfigCall(talon_fx_->ConfigMotorCommutation(motor_commutation_ctre, configTimeoutMs), "ConfigMotorCommutation"))
            {
                ROS_INFO_STREAM("Updated joint " << getName() << " motor commutation");
                state_->setMotorCommutation(motor_commutation);
            }
            else
            {
                ROS_INFO_STREAM("Failed to update joint " << getName() << " motor commutation");
                command_->resetMotorCommutation();
                return;
            }
        }

        hardware_interface::AbsoluteSensorRange absolute_sensor_range;
        ctre::phoenix::sensors::AbsoluteSensorRange absolute_sensor_range_ctre;
        if (command_->absoluteSensorRangeChanged(absolute_sensor_range) &&
            convertAbsoluteSensorRange(absolute_sensor_range, absolute_sensor_range_ctre))
        {
            if (safeConfigCall(talon_fx_->ConfigIntegratedSensorAbsoluteRange(absolute_sensor_range_ctre, configTimeoutMs), "ConfigIntegratedSensorAbsoluteRange"))
            {
                ROS_INFO_STREAM("Updated joint " << getName() << " absolute sensor range");
                state_->setAbsoluteSensorRange(absolute_sensor_range);
            }
            else
            {
                ROS_INFO_STREAM("Failed to update joint " << getName() << " absolute sensor range");
                command_->resetAbsoluteSensorRange();
                return;
            }
        }

        hardware_interface::SensorInitializationStrategy sensor_initialization_strategy;
        ctre::phoenix::sensors::SensorInitializationStrategy sensor_initialization_strategy_ctre;
        if (command_->sensorInitializationStrategyChanged(sensor_initialization_strategy) &&
            convertSensorInitializationStrategy(sensor_initialization_strategy, sensor_initialization_strategy_ctre))
        {
            if (safeConfigCall(talon_fx_->ConfigIntegratedSensorInitializationStrategy(sensor_initialization_strategy_ctre, configTimeoutMs), "ConfigIntegratedSensorInitializationStrategy"))
            {
                ROS_INFO_STREAM("Updated joint " << getName() << " absolute sensor range");
                state_->setSensorInitializationStrategy(sensor_initialization_strategy);
            }
            else
            {
                ROS_INFO_STREAM("Failed to update joint " << getName() << " absolute sensor range");
                command_->resetSensorInitializationStrategy();
                return;
            }
        }
    }

    if (talon_)
    {
        // TODO : fix for Victor non-enhanced status frames
        for (int i = hardware_interface::Status_1_General; (i < hardware_interface::Status_Last); i++)
        {
            uint8_t status_frame_period;
            const auto status_frame = static_cast<hardware_interface::StatusFrame>(i);
            if (command_->statusFramePeriodChanged(status_frame, status_frame_period) && (status_frame_period != 0))
            {
                ctre::phoenix::motorcontrol::StatusFrameEnhanced status_frame_enhanced;
                if (convertStatusFrame(status_frame, status_frame_enhanced))
                {
                    if (safeConfigCall(talon_->SetStatusFramePeriod(status_frame_enhanced, status_frame_period), "SetStatusFramePeriod"))
                    {
                        state_->setStatusFramePeriod(status_frame, status_frame_period);
                        ROS_INFO_STREAM("Updated joint " << getName() << " status_frame " << i << "=" << static_cast<int>(status_frame_period) << "mSec");
                    }
                    else
                    {
                        ROS_INFO_STREAM("Failed to update joint " << getName() << " status_frame " << i);
                        command_->resetStatusFramePeriod(status_frame);
                        return;
                    }
                }
            }
        }
    }

    for (int i = hardware_interface::Control_3_General; (i < hardware_interface::Control_Last); i++)
    {
        uint8_t control_frame_period;
        const auto control_frame = static_cast<hardware_interface::ControlFrame>(i);
        if (command_->controlFramePeriodChanged(control_frame, control_frame_period) && (control_frame_period != 0))
        {
            ctre::phoenix::motorcontrol::ControlFrame control_frame_phoenix;
            if (convertControlFrame(control_frame, control_frame_phoenix))
            {
                if (safeConfigCall(victor_spx_->SetControlFramePeriod(control_frame_phoenix, control_frame_period), "SetControlFramePeriod"))
                {
                    state_->setControlFramePeriod(control_frame, control_frame_period);
                    ROS_INFO_STREAM("Updated joint " << getName() << " control_frame " << i << "=" << static_cast<int>(control_frame_period) << "mSec");
                }
                else
                {
                    ROS_INFO_STREAM("Failed to update joint " << getName() << " control_frame " << i);
                    command_->resetControlFramePeriod(control_frame);
                    return;
                }
            }
        }
    }

    double motion_cruise_velocity;
    double motion_acceleration;
    int motion_s_curve_strength;
    if (command_->motionCruiseChanged(motion_cruise_velocity, motion_acceleration, motion_s_curve_strength))
    {
        //converted from rad/sec to native units
        bool rc = safeConfigCall(victor_spx_->ConfigMotionCruiseVelocity(motion_cruise_velocity / radians_per_second_scale, configTimeoutMs),"ConfigMotionCruiseVelocity(");
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigMotionAcceleration(motion_acceleration / radians_per_second_scale, configTimeoutMs),"ConfigMotionAcceleration(");
        }
        if (rc)
        {
            rc = safeConfigCall(victor_spx_->ConfigMotionSCurveStrength(motion_s_curve_strength, configTimeoutMs), "ConfigMotionSCurveStrength");
        }

        if (rc)
        {
            ROS_INFO_STREAM("Updated joint " << getName() << " cruise velocity / acceleration");
            state_->setMotionCruiseVelocity(motion_cruise_velocity);
            state_->setMotionAcceleration(motion_acceleration);
            state_->setMotionSCurveStrength(motion_s_curve_strength);
        }
        else
        {
            ROS_INFO_STREAM("Failed to update joint " << getName() << " cruise velocity / acceleration");
            command_->resetMotionCruise();
            return;
        }
    }
    if (int motion_profile_trajectory_period; command_->motionProfileTrajectoryPeriodChanged(motion_profile_trajectory_period))
    {
        if (safeConfigCall(victor_spx_->ConfigMotionProfileTrajectoryPeriod(motion_profile_trajectory_period, configTimeoutMs),"ConfigMotionProfileTrajectoryPeriod"))
        {
            state_->setMotionProfileTrajectoryPeriod(motion_profile_trajectory_period);
            ROS_INFO_STREAM("Updated joint " << getName() << " motion profile trajectory period");
        }
        else
        {
            ROS_INFO_STREAM("Failed to update joint " << getName() << " motion profile trajectory period");
            command_->resetMotionProfileTrajectoryPeriod();
            return;
        }
    }

    if (command_->clearMotionProfileTrajectoriesChanged())
    {
        if (safeConfigCall(victor_spx_->ClearMotionProfileTrajectories(), "ClearMotionProfileTrajectories"))
        {
            ROS_INFO_STREAM("Cleared joint " << getName() << " motion profile trajectories");
        }
        else
        {
            ROS_INFO_STREAM("Failed to clear joint " << getName() << " motion profile trajectories");
            command_->setClearMotionProfileTrajectories();
            return;
        }
    }

    if (command_->clearMotionProfileHasUnderrunChanged())
    {
        if (safeConfigCall(victor_spx_->ClearMotionProfileHasUnderrun(configTimeoutMs),"ClearMotionProfileHasUnderrun"))
        {
            ROS_INFO_STREAM("Cleared joint " << getName() << " motion profile underrun changed");
        }
        else
        {
            ROS_INFO_STREAM("Failed to Clear joint " << getName() << " motion profile underrun changed");
            command_->setClearMotionProfileHasUnderrun();
            return;
        }
    }

    // TODO : check that Talon motion buffer is not full
    // before writing, communicate how many have been written
    // - and thus should be cleared - from the talon_command
    // list of requesstate_->

    // TODO : rewrite this using BufferedTrajectoryPointStream
    if (std::vector<hardware_interface::TrajectoryPoint> trajectory_points; command_->motionProfileTrajectoriesChanged(trajectory_points))
    {
        //int i = 0;
        for (auto it = trajectory_points.cbegin(); it != trajectory_points.cend(); ++it)
        {
            ctre::phoenix::motion::TrajectoryPoint pt;
            pt.position = it->position / radians_scale;
            pt.velocity = it->velocity / radians_per_second_scale;
            pt.headingDeg = it->headingRad * 180. / M_PI;
            pt.arbFeedFwd = it->arbFeedFwd;
            pt.auxiliaryPos = it->auxiliaryPos; // TODO : unit conversion?
            pt.auxiliaryVel = it->auxiliaryVel; // TODO : unit conversion?
            pt.auxiliaryArbFeedFwd = it->auxiliaryArbFeedFwd; // TODO : unit conversion?
            pt.profileSlotSelect0 = it->profileSlotSelect0;
            pt.profileSlotSelect1 = it->profileSlotSelect1;
            pt.isLastPoint = it->isLastPoint;
            pt.zeroPos = it->zeroPos;
            pt.timeDur = it->timeDur;
            pt.useAuxPID = it->useAuxPID;
            //ROS_INFO_STREAM("id: " << joint_id << " pos: " << pt.position << " i: " << i++);
        }
        ROS_INFO_STREAM("Added joint " << getName() << " motion profile trajectories");
    }

    // Set new motor setpoint if either the mode or the setpoint has been changed
    if (robot_enabled)
    {
        double command;
        hardware_interface::TalonMode in_mode;
        hardware_interface::DemandType demand1_type_internal;
        double demand1_value;

        const bool b1 = command_->modeChanged(in_mode);
        const bool b2 = command_->commandChanged(command);
        const bool b3 = command_->demand1Changed(demand1_type_internal, demand1_value);

        // ROS_INFO_STREAM(getName()_ << "b1 = " << b1 << " b2 = " << b2 << " b3 = " << b3);
        if (b1 || b2 || b3)
        {
            ctre::phoenix::motorcontrol::ControlMode out_mode;
            ctre::phoenix::motorcontrol::DemandType demand1_type_phoenix;
            if (convertControlMode(in_mode, out_mode) &&
                convertDemand1Type(demand1_type_internal, demand1_type_phoenix))
            {
                state_->setSetpoint(command); // set the state before converting it to native units
                switch (out_mode)
                {
                    case ctre::phoenix::motorcontrol::ControlMode::Velocity:
                        command /= radians_per_second_scale;
                        break;
                    case ctre::phoenix::motorcontrol::ControlMode::Position:
                        command /= radians_scale;
                        break;
                    case ctre::phoenix::motorcontrol::ControlMode::MotionMagic:
                        command /= radians_scale;
                        break;
                    default:
                        break;
                }

#if 0
                ROS_INFO_STREAM("called Set(4) on " << getName() <<
                        " out_mode = " << static_cast<int>(out_mode) << " command = " << command <<
                        " demand1_type_phoenix = " << static_cast<int>(demand1_type_phoenix) <<
                        " demand1_value = " << demand1_value);
#endif
                state_->setTalonMode(in_mode);
                state_->setDemand1Type(demand1_type_internal);
                state_->setDemand1Value(demand1_value);

                victor_spx_->Set(out_mode, command, demand1_type_phoenix, demand1_value);
            }
            else
            {
                ROS_ERROR_STREAM("Couldn't convert to the talon enum type");
            }
        }
    }
    else
    {
        // Update talon state with requested setpoints for
        // debugging. Don't actually write them to the physical
        // Talons until the robot is re-enabled, though.
        state_->setSetpoint(command_->get());
        state_->setDemand1Type(command_->getDemand1Type());
        state_->setDemand1Value(command_->getDemand1Value());
        if (prev_robot_enabled)
        {
            // On the switch from robot enabled to robot disabled, set Talons to ControlMode::Disabled
            // call resetMode() to queue up a change back to the correct mode / setpoint
            // when the robot switches from disabled back to enabled
            command_->resetMode();    // also forces re-write of setpoint
            command_->resetDemand1(); // make sure demand1 type/value is also written on re-enable
            victor_spx_->Set(ctre::phoenix::motorcontrol::ControlMode::Disabled, 0);
            state_->setTalonMode(hardware_interface::TalonMode_Disabled);
            ROS_INFO_STREAM("Robot disabled - called Set(Disabled) on " << getName());
        }
    }

    if (command_->clearStickyFaultsChanged())
    {
        if (safeConfigCall(victor_spx_->ClearStickyFaults(configTimeoutMs), "ClearStickyFaults"))
        {
            ROS_INFO_STREAM("Cleared joint " << getName() << " sticky_faults");
        }
        else
        {
            ROS_INFO_STREAM("Failed to clear joint " << getName() << " sticky_faults");
            command_->setClearStickyFaults();
            return;
        }
    }
}

void CTREV5MotorController::read_thread(std::unique_ptr<Tracer> tracer,
                                        const double poll_frequency)
{
#ifdef __linux__
	std::stringstream thread_name;
	// Use abbreviations since pthread_setname will fail if name is >= 16 characters
	thread_name << "ctre_mc_rd_" << read_thread_state_->getCANID();
	if (pthread_setname_np(pthread_self(), thread_name.str().c_str()))
	{
		ROS_ERROR_STREAM("Error setting thread name " << thread_name.str() << " " << errno);
	}
#endif
	ros::Duration(2.75 + read_thread_state_->getCANID() * 0.05).sleep(); // Sleep for a few seconds to let CAN start up
	ROS_INFO_STREAM("Starting ctre_mc " << read_thread_state_->getCANID() << " thread at " << ros::Time::now());

	for (ros::Rate r(poll_frequency); ros::ok(); r.sleep())
	{
		tracer->start("ctre v5 mc read main_loop");

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
			std::lock_guard l(*read_state_mutex_);
			if (!read_thread_state_->getEnableReadThread())
            {
                return;
            }
            talon_mode = read_thread_state_->getTalonMode();
			encoder_feedback = read_thread_state_->getEncoderFeedback();
			encoder_ticks_per_rotation = read_thread_state_->getEncoderTicksPerRotation();
			conversion_factor = read_thread_state_->getConversionFactor();
		}

		// TODO : in main read() loop copy status from talon being followed
		// into follower talon state?
		if (talon_mode == hardware_interface::TalonMode_Follower)
        {
			return;
        }

		const double radians_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, hardware_interface::TalonMode_Position) * conversion_factor;
		const double radians_per_second_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, hardware_interface::TalonMode_Velocity) * conversion_factor;

		const double motor_output_percent = victor_spx_->GetMotorOutputPercent();
		safeCall(victor_spx_->GetLastError(), "GetMotorOutputPercent");

		ctre::phoenix::motorcontrol::Faults faults;
		safeCall(victor_spx_->GetFaults(faults), "GetFaults");

		// applied control mode - cached
		// soft limit and limit switch override - cached

		const double position = victor_spx_->GetSelectedSensorPosition(pidIdx) * radians_scale;
		safeCall(victor_spx_->GetLastError(), "GetSelectedSensorPosition");

		const double velocity = victor_spx_->GetSelectedSensorVelocity(pidIdx) * radians_per_second_scale;
		safeCall(victor_spx_->GetLastError(), "GetSelectedSensorVelocity");

		double output_current = -1;
		if (talon_)
		{
			output_current = talon_->GetOutputCurrent();
			safeCall(victor_spx_->GetLastError(), "GetOutputCurrent");
		}
		double stator_current = -1;
		double supply_current = -1;
		if (talon_srx_)
		{
			stator_current = talon_srx_->GetStatorCurrent();
			safeCall(victor_spx_->GetLastError(), "GetStatorCurrent");
			supply_current = talon_srx_->GetSupplyCurrent();
			safeCall(victor_spx_->GetLastError(), "GetSupplyCurrent");
		}
		else if (talon_fx_)
		{
			stator_current = talon_fx_->GetStatorCurrent();
			safeCall(victor_spx_->GetLastError(), "GetStatorCurrent");
			supply_current = talon_fx_->GetSupplyCurrent();
			safeCall(victor_spx_->GetLastError(), "GetSupplyCurrent");
		}

		ctre::phoenix::motorcontrol::StickyFaults sticky_faults;
		safeCall(victor_spx_->GetStickyFaults(sticky_faults), "GetStickyFault");

		const double bus_voltage = victor_spx_->GetBusVoltage();
		safeCall(victor_spx_->GetLastError(), "GetBusVoltage");

		const double temperature = victor_spx_->GetTemperature(); //returns in Celsius
		safeCall(victor_spx_->GetLastError(), "GetTemperature");

		const double output_voltage = victor_spx_->GetMotorOutputVoltage();
		safeCall(victor_spx_->GetLastError(), "GetMotorOutputVoltage");

		double closed_loop_error = 0;
		double integral_accumulator = 0;
		double error_derivative = 0;
		double closed_loop_target = 0;

		if ((talon_mode == hardware_interface::TalonMode_Position) ||
			(talon_mode == hardware_interface::TalonMode_Velocity) ||
			(talon_mode == hardware_interface::TalonMode_Current ) ||
			(talon_mode == hardware_interface::TalonMode_MotionProfile) ||
			(talon_mode == hardware_interface::TalonMode_MotionMagic)   ||
			(talon_mode == hardware_interface::TalonMode_MotionProfileArc))
		{
			const double closed_loop_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, talon_mode) * conversion_factor;

			closed_loop_error = victor_spx_->GetClosedLoopError(pidIdx) * closed_loop_scale;
			safeCall(victor_spx_->GetLastError(), "GetClosedLoopError");

			integral_accumulator = victor_spx_->GetIntegralAccumulator(pidIdx);
			safeCall(victor_spx_->GetLastError(), "GetIntegralAccumulator");

			error_derivative = victor_spx_->GetErrorDerivative(pidIdx);
			safeCall(victor_spx_->GetLastError(), "GetErrorDerivative");

			closed_loop_target = victor_spx_->GetClosedLoopTarget(pidIdx) * closed_loop_scale;
			safeCall(victor_spx_->GetLastError(), "GetClosedLoopTarget");
		}

		// Targets Status 10 - 160 mSec default
		double active_trajectory_position = 0.0;
		double active_trajectory_velocity = 0.0;
		double active_trajectory_heading = 0.0;
		if ((talon_mode == hardware_interface::TalonMode_MotionProfile) ||
			(talon_mode == hardware_interface::TalonMode_MotionMagic)   ||
			(talon_mode == hardware_interface::TalonMode_MotionProfileArc))
		{
			active_trajectory_position = victor_spx_->GetActiveTrajectoryPosition() * radians_scale;
			safeCall(victor_spx_->GetLastError(), "GetActiveTrajectoryPosition");

			active_trajectory_velocity = victor_spx_->GetActiveTrajectoryVelocity() * radians_per_second_scale;
			safeCall(victor_spx_->GetLastError(), "GetActiveTrajectoryVelocity");

			if (talon_mode == hardware_interface::TalonMode_MotionProfileArc)
			{
				active_trajectory_heading = victor_spx_->GetActiveTrajectoryPosition(1) * 2. * M_PI / 360.; //returns in degrees
				safeCall(victor_spx_->GetLastError(), "GetActiveTrajectoryHeading");
			}
		}

		int mp_top_level_buffer_count = 0;
		hardware_interface::MotionProfileStatus internal_status;
		if (talon_mode == hardware_interface::TalonMode_MotionProfile)
		{
			mp_top_level_buffer_count = victor_spx_->GetMotionProfileTopLevelBufferCount();
			ctre::phoenix::motion::MotionProfileStatus talon_status;
			safeCall(victor_spx_->GetMotionProfileStatus(talon_status), "GetMotionProfileStatus");

			internal_status.topBufferRem = talon_status.topBufferRem;
			internal_status.topBufferCnt = talon_status.topBufferCnt;
			internal_status.btmBufferCnt = talon_status.btmBufferCnt;
			internal_status.hasUnderrun = talon_status.hasUnderrun;
			internal_status.isUnderrun = talon_status.isUnderrun;
			internal_status.activePointValid = talon_status.activePointValid;
			internal_status.isLast = talon_status.isLast;
			internal_status.profileSlotSelect0 = talon_status.profileSlotSelect0;
			internal_status.profileSlotSelect1 = talon_status.profileSlotSelect1;
			internal_status.outputEnable = static_cast<hardware_interface::SetValueMotionProfile>(talon_status.outputEnable);
			internal_status.timeDurMs = talon_status.timeDurMs;
		}

		bool forward_limit_switch = false;
		bool reverse_limit_switch = false;
		if (talon_srx_)
		{
			auto sensor_collection = talon_srx_->GetSensorCollection();

			forward_limit_switch = sensor_collection.IsFwdLimitSwitchClosed();
			reverse_limit_switch = sensor_collection.IsRevLimitSwitchClosed();
		}
		else if (talon_fx_)
		{
			auto sensor_collection = talon_fx_->GetSensorCollection();

			forward_limit_switch = sensor_collection.IsFwdLimitSwitchClosed();
			reverse_limit_switch = sensor_collection.IsRevLimitSwitchClosed();
		}

		const int firmware_version = victor_spx_->GetFirmwareVersion();

		// Actually update the TalonHWState shared between
		// this thread and read()
		// Do this all at once so the code minimizes the amount
		// of time with mutex locked
		{
			// Lock the state entry to make sure writes
			// are atomic - reads won't grab data in
			// the middle of a write
			std::lock_guard l(*read_state_mutex_);

			if (talon_mode == hardware_interface::TalonMode_MotionProfile)
			{
				read_thread_state_->setMotionProfileStatus(internal_status);
				read_thread_state_->setMotionProfileTopLevelBufferCount(mp_top_level_buffer_count);
			}

			read_thread_state_->setMotorOutputPercent(motor_output_percent);
			read_thread_state_->setFaults(faults.ToBitfield());

			read_thread_state_->setForwardSoftlimitHit(faults.ForwardSoftLimit);
			read_thread_state_->setReverseSoftlimitHit(faults.ReverseSoftLimit);

			read_thread_state_->setPosition(position);
			read_thread_state_->setSpeed(velocity);
			if (talon_)
			{
				read_thread_state_->setOutputCurrent(output_current);
			}
			if (talon_srx_ || talon_fx_)
			{
				read_thread_state_->setStatorCurrent(stator_current);
				read_thread_state_->setSupplyCurrent(supply_current);
			}
			read_thread_state_->setStickyFaults(sticky_faults.ToBitfield());

			read_thread_state_->setBusVoltage(bus_voltage);
			read_thread_state_->setTemperature(temperature);
			read_thread_state_->setOutputVoltage(output_voltage);

			if ((talon_mode == hardware_interface::TalonMode_Position) ||
				(talon_mode == hardware_interface::TalonMode_Velocity) ||
				(talon_mode == hardware_interface::TalonMode_Current ) ||
				(talon_mode == hardware_interface::TalonMode_MotionProfile) ||
				(talon_mode == hardware_interface::TalonMode_MotionMagic)   ||
				(talon_mode == hardware_interface::TalonMode_MotionProfileArc))
			{
				read_thread_state_->setClosedLoopError(closed_loop_error);
				read_thread_state_->setIntegralAccumulator(integral_accumulator);
				read_thread_state_->setErrorDerivative(error_derivative);
				// Reverse engineer the individual P,I,D,F components used
				// to generate closed-loop control signals to the motor
				// This is just for debugging PIDF tuning
				const auto closed_loop_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, talon_mode) * conversion_factor;
				const auto pidf_slot = read_thread_state_->getSlot();
				const auto kp = read_thread_state_->getPidfP(pidf_slot);
				const auto ki = read_thread_state_->getPidfI(pidf_slot);
				const auto kd = read_thread_state_->getPidfD(pidf_slot);
				const auto native_closed_loop_error = closed_loop_error / closed_loop_scale;
				read_thread_state_->setPTerm(native_closed_loop_error * kp);
				read_thread_state_->setITerm(integral_accumulator * ki);
				read_thread_state_->setDTerm(error_derivative * kd);
				read_thread_state_->setClosedLoopTarget(closed_loop_target);

				const double kf = read_thread_state_->getPidfF(pidf_slot);
				read_thread_state_->setFTerm(closed_loop_target / closed_loop_scale * kf);
			}

			if ((talon_mode == hardware_interface::TalonMode_MotionProfile) ||
				(talon_mode == hardware_interface::TalonMode_MotionMagic)   ||
				(talon_mode == hardware_interface::TalonMode_MotionProfileArc))
			{
				read_thread_state_->setActiveTrajectoryPosition(active_trajectory_position);
				read_thread_state_->setActiveTrajectoryVelocity(active_trajectory_velocity);
				if (talon_mode == hardware_interface::TalonMode_MotionProfileArc)
				{
					read_thread_state_->setActiveTrajectoryHeading(active_trajectory_heading);
				}
			}

			if (talon_)
			{
				read_thread_state_->setForwardLimitSwitch(forward_limit_switch);
				read_thread_state_->setReverseLimitSwitch(reverse_limit_switch);
			}

			read_thread_state_->setFirmwareVersion(firmware_version);
		}
		tracer->report(60);
	}
}

// These always come from the main write thread
void CTREV5MotorController::setSimCollection(int position,
                                             int velocity,
                                             int delta_position) const
{
	if (talon_fx_)
    {
		setSimCollectionTalonFX(position, velocity, delta_position);
    }
	else if (talon_srx_)
    {
		setSimCollectionTalonSRX(position, velocity, delta_position);
    }
}

void CTREV5MotorController::setSimCollectionTalonFX(int position,
                                                    int velocity,
                                                    int delta_position) const
{
	auto &collection = talon_fx_->GetSimCollection();
	if (delta_position)
		collection.AddIntegratedSensorPosition(delta_position);
	else
		collection.SetIntegratedSensorRawPosition(position);
	collection.SetIntegratedSensorVelocity(velocity);
	collection.SetBusVoltage(12.5);
}

void CTREV5MotorController::setSimCollectionTalonSRX(int position,
                                                     int velocity,
                                                     int delta_position) const
{
	auto &collection = talon_srx_->GetSimCollection();
	if (delta_position)
		collection.AddQuadraturePosition(delta_position);
	else
		collection.SetQuadratureRawPosition(position);
	collection.SetQuadratureVelocity(velocity);
	collection.SetBusVoltage(12.5);
}
void CTREV5MotorController::updateSimValues(const ros::Time &/*time*/, const ros::Duration &period) const
{
    if (!talon_srx_ && !talon_fx_)
    {
        return;
    }
    hardware_interface::FeedbackDevice encoder_feedback = state_->getEncoderFeedback();
    const int encoder_ticks_per_rotation = state_->getEncoderTicksPerRotation();
    const double conversion_factor = state_->getConversionFactor();
    const double radians_scale = getConversionFactor(encoder_ticks_per_rotation,
                                                     encoder_feedback,
                                                     hardware_interface::TalonMode_Position) *
                                 conversion_factor;

    const double radians_per_second_scale = getConversionFactor(encoder_ticks_per_rotation,
                                                                encoder_feedback,
                                                                hardware_interface::TalonMode_Velocity) *
                                            conversion_factor;

    // Get current mode of the motor controller.
    const auto mode = state_->getTalonMode();

    // Set the encoder position based on the current motor mode.
    // auto &sim_motor = ctre_mc->GetSimCollection();
    if (mode == hardware_interface::TalonMode_Position)
    {
        // Set encoder position to set point.
        // Set velocity to 0.
        setSimCollection(static_cast<int>(state_->getSetpoint() / radians_scale), 0);
    }
    else if (mode == hardware_interface::TalonMode_Velocity)
    {
        // Set velocity to set point
        // Set encoder position to current position + velocity * dt
        setSimCollection(0,
                         static_cast<int>(state_->getSetpoint() / radians_per_second_scale),
                         static_cast<int>(state_->getSpeed() * radians_per_second_scale * period.toSec()));
    }
    else if (mode == hardware_interface::TalonMode_MotionMagic)
    {
        // TODO : maybe apply this to velocity and position PID modes above,
        // waiting on a motor where we actually need this function to test with
        const double invert = state_->getInvert() ? -1.0 : 1.0;
        // Do some ~magic~ to figure out position/velocity.
        setSimCollection(static_cast<int>(invert * state_->getActiveTrajectoryPosition() / radians_scale),
                         static_cast<int>(invert * state_->getActiveTrajectoryVelocity() / radians_per_second_scale));
    }
}

bool CTREV5MotorController::setSimLimitSwitches(const bool forward_limit, const bool reverse_limit) const
{
    if (!local_)
    {
        ROS_ERROR_STREAM("Couldn't set sim limit switches for CTRE V5 MC " << getName() << " : device not local to this hardware interface");
        return false;
    }
    if (talon_srx_)
    {
        auto &collection = talon_srx_->GetSimCollection();
        collection.SetLimitFwd(forward_limit);
        collection.SetLimitRev(reverse_limit);
        return true;
    }
    if (talon_fx_)
    {
        auto &collection = talon_fx_->GetSimCollection();
        collection.SetLimitFwd(forward_limit);
        collection.SetLimitRev(reverse_limit);
        return true;
    }
    ROS_ERROR_STREAM("Couldn't set sim limit switches for CTRE V5 MC " << getName() << " : device not an FX or SRX");
    return false;
}

bool CTREV5MotorController::setSimCurrent(const double stator_current, const double supply_current) const
{
    if (!local_)
    {
        ROS_ERROR_STREAM("Couldn't set sim current for CTRE V5 MC " << getName() << " : device not local to this hardware interface");
        return false;
    }
    if (talon_srx_)
    {
        auto &collection = talon_srx_->GetSimCollection();
        collection.SetStatorCurrent(stator_current);
        collection.SetSupplyCurrent(supply_current);
        return true;
    }
    if (talon_fx_)
    {
        auto &collection = talon_fx_->GetSimCollection();
        collection.SetStatorCurrent(stator_current);
        collection.SetSupplyCurrent(supply_current);
        return true;
    }
    ROS_ERROR_STREAM("Couldn't set sim current for CTRE V5 MC " << getName() << " : device not an FX or SRX");
    return false;
}

static bool convertControlMode(const hardware_interface::TalonMode input_mode,
                               ctre::phoenix::motorcontrol::ControlMode &output_mode)
{
    switch (input_mode)
    {
    case hardware_interface::TalonMode_PercentOutput:
        output_mode = ctre::phoenix::motorcontrol::ControlMode::PercentOutput;
        break;
    case hardware_interface::TalonMode_Position: // CloseLoop
        output_mode = ctre::phoenix::motorcontrol::ControlMode::Position;
        break;
    case hardware_interface::TalonMode_Velocity: // CloseLoop
        output_mode = ctre::phoenix::motorcontrol::ControlMode::Velocity;
        break;
    case hardware_interface::TalonMode_Current: // CloseLoop
        output_mode = ctre::phoenix::motorcontrol::ControlMode::Current;
        break;
    case hardware_interface::TalonMode_Follower:
        output_mode = ctre::phoenix::motorcontrol::ControlMode::Follower;
        break;
    case hardware_interface::TalonMode_MotionProfile:
        output_mode = ctre::phoenix::motorcontrol::ControlMode::MotionProfile;
        break;
    case hardware_interface::TalonMode_MotionMagic:
        output_mode = ctre::phoenix::motorcontrol::ControlMode::MotionMagic;
        break;
    case hardware_interface::TalonMode_MotionProfileArc:
        output_mode = ctre::phoenix::motorcontrol::ControlMode::MotionProfileArc;
        break;
    case hardware_interface::TalonMode_Disabled:
        output_mode = ctre::phoenix::motorcontrol::ControlMode::Disabled;
        break;
    default:
        output_mode = ctre::phoenix::motorcontrol::ControlMode::Disabled;
        ROS_WARN("Unknown mode seen in HW interface");
        return false;
    }
    return true;
}

static bool convertDemand1Type(const hardware_interface::DemandType input,
                               ctre::phoenix::motorcontrol::DemandType &output)
{
    switch (input)
    {
    case hardware_interface::DemandType::DemandType_Neutral:
        output = ctre::phoenix::motorcontrol::DemandType::DemandType_Neutral;
        break;
    case hardware_interface::DemandType::DemandType_AuxPID:
        output = ctre::phoenix::motorcontrol::DemandType::DemandType_AuxPID;
        break;
    case hardware_interface::DemandType::DemandType_ArbitraryFeedForward:
        output = ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward;
        break;
    default:
        output = ctre::phoenix::motorcontrol::DemandType::DemandType_Neutral;
        ROS_WARN("Unknown demand1 type seen in HW interface");
        return false;
    }
    return true;
}

static bool convertNeutralMode(const hardware_interface::NeutralMode input_mode,
                               ctre::phoenix::motorcontrol::NeutralMode &output_mode)
{
    switch (input_mode)
    {
    case hardware_interface::NeutralMode_EEPROM_Setting:
        output_mode = ctre::phoenix::motorcontrol::EEPROMSetting;
        break;
    case hardware_interface::NeutralMode_Coast:
        output_mode = ctre::phoenix::motorcontrol::Coast;
        break;
    case hardware_interface::NeutralMode_Brake:
        output_mode = ctre::phoenix::motorcontrol::Brake;
        break;
    default:
        output_mode = ctre::phoenix::motorcontrol::EEPROMSetting;
        ROS_WARN("Unknown neutral mode seen in HW interface");
        return false;
    }

    return true;
}

static bool convertFeedbackDevice(const hardware_interface::FeedbackDevice input_fd,
                                  ctre::phoenix::motorcontrol::FeedbackDevice &output_fd)
{
    switch (input_fd)
    {
    case hardware_interface::FeedbackDevice_QuadEncoder:
        output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder;
        break;
    case hardware_interface::FeedbackDevice_IntegratedSensor:
        output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
        break;
    case hardware_interface::FeedbackDevice_Analog:
        output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::Analog;
        break;
    case hardware_interface::FeedbackDevice_Tachometer:
        output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::Tachometer;
        break;
    case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
        output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::PulseWidthEncodedPosition;
        break;
    case hardware_interface::FeedbackDevice_SensorSum:
        output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::SensorSum;
        break;
    case hardware_interface::FeedbackDevice_SensorDifference:
        output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::SensorDifference;
        break;
    case hardware_interface::FeedbackDevice_RemoteSensor0:
        output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
        break;
    case hardware_interface::FeedbackDevice_RemoteSensor1:
        output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor1;
        break;
    case hardware_interface::FeedbackDevice_None:
        output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::None;
        break;
    case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
        output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::SoftwareEmulatedSensor;
        break;
    default:
        ROS_WARN("Unknown feedback device seen in HW interface");
        return false;
    }
    return true;
}

static bool convertRemoteFeedbackDevice(const hardware_interface::RemoteFeedbackDevice input_fd,
                                        ctre::phoenix::motorcontrol::RemoteFeedbackDevice &output_fd)
{
    switch (input_fd)
    {
    case hardware_interface::RemoteFeedbackDevice_SensorSum:
        output_fd = ctre::phoenix::motorcontrol::RemoteFeedbackDevice::SensorSum;
        break;
    case hardware_interface::RemoteFeedbackDevice_SensorDifference:
        output_fd = ctre::phoenix::motorcontrol::RemoteFeedbackDevice::SensorDifference;
        break;
    case hardware_interface::RemoteFeedbackDevice_RemoteSensor0:
        output_fd = ctre::phoenix::motorcontrol::RemoteFeedbackDevice::RemoteSensor0;
        break;
    case hardware_interface::RemoteFeedbackDevice_RemoteSensor1:
        output_fd = ctre::phoenix::motorcontrol::RemoteFeedbackDevice::RemoteSensor1;
        break;
    case hardware_interface::RemoteFeedbackDevice_None:
        output_fd = ctre::phoenix::motorcontrol::RemoteFeedbackDevice::None;
        break;
    case hardware_interface::RemoteFeedbackDevice_SoftwareEmulatedSensor:
        output_fd = ctre::phoenix::motorcontrol::RemoteFeedbackDevice::SoftwareEmulatedSensor;
        break;
    default:
        ROS_WARN("Unknown remote feedback device seen in HW interface");
        return false;
    }

    return true;
}

static bool convertRemoteSensorSource(const hardware_interface::RemoteSensorSource input_rss,
                                      ctre::phoenix::motorcontrol::RemoteSensorSource &output_rss)
{
    switch (input_rss)
    {
    case hardware_interface::RemoteSensorSource_Off:
        output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off;
        break;
    case hardware_interface::RemoteSensorSource_TalonSRX_SelectedSensor:
        output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_TalonSRX_SelectedSensor;
        break;
    case hardware_interface::RemoteSensorSource_Pigeon_Yaw:
        output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw;
        break;
    case hardware_interface::RemoteSensorSource_Pigeon_Pitch:
        output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Pigeon_Pitch;
        break;
    case hardware_interface::RemoteSensorSource_Pigeon_Roll:
        output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Pigeon_Roll;
        break;
    case hardware_interface::RemoteSensorSource_CANifier_Quadrature:
        output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANifier_Quadrature;
        break;
    case hardware_interface::RemoteSensorSource_CANifier_PWMInput0:
        output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput0;
        break;
    case hardware_interface::RemoteSensorSource_CANifier_PWMInput1:
        output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput1;
        break;
    case hardware_interface::RemoteSensorSource_CANifier_PWMInput2:
        output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput2;
        break;
    case hardware_interface::RemoteSensorSource_CANifier_PWMInput3:
        output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput3;
        break;
    case hardware_interface::RemoteSensorSource_GadgeteerPigeon_Yaw:
        output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Yaw;
        break;
    case hardware_interface::RemoteSensorSource_GadgeteerPigeon_Pitch:
        output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Pitch;
        break;
    case hardware_interface::RemoteSensorSource_GadgeteerPigeon_Roll:
        output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Roll;
        break;
    case hardware_interface::RemoteSensorSource_CANCoder:
        output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
        break;

    default:
        ROS_WARN("Unknown remote sensor source seen in HW interface");
        return false;
    }

    return true;
}

static bool convertLimitSwitchSource(const hardware_interface::LimitSwitchSource input_ls,
                                     ctre::phoenix::motorcontrol::LimitSwitchSource &output_ls)
{
    switch (input_ls)
    {
    case hardware_interface::LimitSwitchSource_FeedbackConnector:
        output_ls = ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector;
        break;
    case hardware_interface::LimitSwitchSource_RemoteTalonSRX:
        output_ls = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
        break;
    case hardware_interface::LimitSwitchSource_RemoteCANifier:
        output_ls = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteCANifier;
        break;
    case hardware_interface::LimitSwitchSource_Deactivated:
        output_ls = ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
        break;
    default:
        ROS_WARN("Unknown limit switch source seen in HW interface");
        return false;
    }
    return true;
}

static bool convertRemoteLimitSwitchSource(const hardware_interface::RemoteLimitSwitchSource input_ls,
                                           ctre::phoenix::motorcontrol::RemoteLimitSwitchSource &output_ls)
{
    switch (input_ls)
    {
    case hardware_interface::RemoteLimitSwitchSource_RemoteTalonSRX:
        output_ls = ctre::phoenix::motorcontrol::RemoteLimitSwitchSource_RemoteTalonSRX;
        break;
    case hardware_interface::RemoteLimitSwitchSource_RemoteCANifier:
        output_ls = ctre::phoenix::motorcontrol::RemoteLimitSwitchSource_RemoteCANifier;
        break;
    case hardware_interface::RemoteLimitSwitchSource_Deactivated:
        output_ls = ctre::phoenix::motorcontrol::RemoteLimitSwitchSource_Deactivated;
        break;
    default:
        ROS_WARN("Unknown remote limit switch source seen in HW interface");
        return false;
    }
    return true;
}

static bool convertLimitSwitchNormal(const hardware_interface::LimitSwitchNormal input_ls,
                                     ctre::phoenix::motorcontrol::LimitSwitchNormal &output_ls)
{
    switch (input_ls)
    {
    case hardware_interface::LimitSwitchNormal_NormallyOpen:
        output_ls = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
        break;
    case hardware_interface::LimitSwitchNormal_NormallyClosed:
        output_ls = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
        break;
    case hardware_interface::LimitSwitchNormal_Disabled:
        output_ls = ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
        break;
    default:
        ROS_WARN("Unknown limit switch normal seen in HW interface");
        return false;
    }
    return true;
}

static bool convertVelocityMeasurementPeriod(const hardware_interface::VelocityMeasurementPeriod input_v_m_p,
                                             ctre::phoenix::motorcontrol::VelocityMeasPeriod &output_v_m_period)
{
    switch (input_v_m_p)
    {
    case hardware_interface::VelocityMeasurementPeriod::Period_1Ms:
        output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_1Ms;
        break;
    case hardware_interface::VelocityMeasurementPeriod::Period_2Ms:
        output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_2Ms;
        break;
    case hardware_interface::VelocityMeasurementPeriod::Period_5Ms:
        output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_5Ms;
        break;
    case hardware_interface::VelocityMeasurementPeriod::Period_10Ms:
        output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_10Ms;
        break;
    case hardware_interface::VelocityMeasurementPeriod::Period_20Ms:
        output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_20Ms;
        break;
    case hardware_interface::VelocityMeasurementPeriod::Period_25Ms:
        output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_25Ms;
        break;
    case hardware_interface::VelocityMeasurementPeriod::Period_50Ms:
        output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_50Ms;
        break;
    case hardware_interface::VelocityMeasurementPeriod::Period_100Ms:
        output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_100Ms;
        break;
    default:
        ROS_WARN("Unknown velocity measurement period seen in HW interface");
        return false;
    }
    return true;
}

static bool convertStatusFrame(const hardware_interface::StatusFrame input,
                               ctre::phoenix::motorcontrol::StatusFrameEnhanced &output)
{
    switch (input)
    {
    case hardware_interface::Status_1_General:
        output = ctre::phoenix::motorcontrol::Status_1_General;
        break;
    case hardware_interface::Status_2_Feedback0:
        output = ctre::phoenix::motorcontrol::Status_2_Feedback0;
        break;
    case hardware_interface::Status_3_Quadrature:
        output = ctre::phoenix::motorcontrol::Status_3_Quadrature;
        break;
    case hardware_interface::Status_4_AinTempVbat:
        output = ctre::phoenix::motorcontrol::Status_4_AinTempVbat;
        break;
    case hardware_interface::Status_6_Misc:
        output = ctre::phoenix::motorcontrol::Status_6_Misc;
        break;
    case hardware_interface::Status_7_CommStatus:
        output = ctre::phoenix::motorcontrol::Status_7_CommStatus;
        break;
    case hardware_interface::Status_8_PulseWidth:
        output = ctre::phoenix::motorcontrol::Status_8_PulseWidth;
        break;
    case hardware_interface::Status_9_MotProfBuffer:
        output = ctre::phoenix::motorcontrol::Status_9_MotProfBuffer;
        break;
    case hardware_interface::Status_10_MotionMagic:
        output = ctre::phoenix::motorcontrol::Status_10_MotionMagic;
        break;
    case hardware_interface::Status_11_UartGadgeteer:
        output = ctre::phoenix::motorcontrol::Status_11_UartGadgeteer;
        break;
    case hardware_interface::Status_12_Feedback1:
        output = ctre::phoenix::motorcontrol::Status_12_Feedback1;
        break;
    case hardware_interface::Status_13_Base_PIDF0:
        output = ctre::phoenix::motorcontrol::Status_13_Base_PIDF0;
        break;
    case hardware_interface::Status_14_Turn_PIDF1:
        output = ctre::phoenix::motorcontrol::Status_14_Turn_PIDF1;
        break;
    case hardware_interface::Status_15_FirmwareApiStatus:
        output = ctre::phoenix::motorcontrol::Status_15_FirmareApiStatus;
        break;
    case hardware_interface::Status_17_Targets1:
        output = ctre::phoenix::motorcontrol::Status_17_Targets1;
        break;
    case hardware_interface::Status_Brushless_Current:
        output = ctre::phoenix::motorcontrol::Status_Brushless_Current;
        break;
    default:
        ROS_ERROR("Invalid input in convertStatusFrame");
        return false;
    }
    return true;
}

static bool convertControlFrame(const hardware_interface::ControlFrame input,
                                ctre::phoenix::motorcontrol::ControlFrame &output)
{
    switch (input)
    {
    case hardware_interface::Control_3_General:
        output = ctre::phoenix::motorcontrol::Control_3_General;
        break;
    case hardware_interface::Control_4_Advanced:
        output = ctre::phoenix::motorcontrol::Control_4_Advanced;
        break;
#if 0 // There's no SetControlFramePeriod which takes an enhanced ControlFrame, so this is out for now
		case hardware_interface::Control_5_FeedbackOutputOverride:
			output = ctre::phoenix::motorcontrol::Control_5_FeedbackOutputOverride_;
			break;
#endif
    case hardware_interface::Control_6_MotProfAddTrajPoint:
        output = ctre::phoenix::motorcontrol::Control_6_MotProfAddTrajPoint;
        break;
    default:
        ROS_ERROR("Invalid input in convertControlFrame");
        return false;
    }
    return true;
}

static bool convertMotorCommutation(const hardware_interface::MotorCommutation input,
                                    ctre::phoenix::motorcontrol::MotorCommutation &output)
{
    switch (input)
    {
    case hardware_interface::MotorCommutation::Trapezoidal:
        output = ctre::phoenix::motorcontrol::MotorCommutation::Trapezoidal;
        break;
    default:
        ROS_ERROR("Invalid input in convertMotorCommutation");
        return false;
    }
    return true;
}

static bool convertAbsoluteSensorRange(const hardware_interface::AbsoluteSensorRange input,
                                       ctre::phoenix::sensors::AbsoluteSensorRange &output)
{
    switch (input)
    {
    case hardware_interface::Unsigned_0_to_360:
        output = ctre::phoenix::sensors::Unsigned_0_to_360;
        break;
    case hardware_interface::Signed_PlusMinus180:
        output = ctre::phoenix::sensors::Signed_PlusMinus180;
        break;
    default:
        ROS_ERROR("Invalid input in convertAbsoluteSensorRange");
        return false;
    }
    return true;
}

static bool convertSensorInitializationStrategy(const hardware_interface::SensorInitializationStrategy input,
                                                ctre::phoenix::sensors::SensorInitializationStrategy &output)
{
    switch (input)
    {
    case hardware_interface::BootToZero:
        output = ctre::phoenix::sensors::BootToZero;
        break;
    case hardware_interface::BootToAbsolutePosition:
        output = ctre::phoenix::sensors::BootToAbsolutePosition;
        break;
    default:
        ROS_ERROR("Invalid input in convertSensorInitializationStrategy");
        return false;
    }
    return true;
}