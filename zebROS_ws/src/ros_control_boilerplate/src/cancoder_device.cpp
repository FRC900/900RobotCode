#include "ros/ros.h"

#include "ctre/phoenix6/core/CoreCANcoder.hpp"

#include "ctre_interfaces/cancoder_command_interface.h"
#include "ctre_interfaces/cancoder_sim_command_interface.h" // TODO - split sim into derived class so this isn't included in HW code?
#include "ros_control_boilerplate/cancoder_device.h"
#include "ros_control_boilerplate/tracer.h"

static bool convertMagnetHealth(const ctre::phoenix6::signals::MagnetHealthValue &input,
                                hardware_interface::cancoder::MagnetHealth &output);
static bool convertMagnetHealth(const hardware_interface::cancoder::MagnetHealth input,
                                ctre::phoenix6::signals::MagnetHealthValue &output);
static bool convertSensorDirection(hardware_interface::cancoder::SensorDirection input,
                                   ctre::phoenix6::signals::SensorDirectionValue &output);
static bool convertAbsoluteSensorRange(hardware_interface::cancoder::AbsoluteSensorRange input,
                                       ctre::phoenix6::signals::AbsoluteSensorRangeValue &output);

template <bool SIM>
CANCoderDevice<SIM>::CANCoderDevice(const std::string &name_space,
                                    const int joint_index,
                                    const std::string &joint_name,
                                    const int can_id,
                                    const std::string &can_bus,
                                    const bool local_hardware,
                                    const bool local_update,
                                    const double read_hz_)
    : CTREV6Device("CANcoder", joint_name, can_id)
    , local_hardware_{local_hardware}
    , local_update_{local_update}
    , state_{std::make_unique<hardware_interface::cancoder::CANCoderHWState>(can_id)}
{
    ROS_INFO_STREAM_NAMED("frc_robot_interface",
                        "Loading joint " << joint_index << "=" << joint_name <<
                        (local_update_ ? " local" : " remote") << " update, " <<
                        (local_hardware_ ? "local" : "remote") << " hardware" <<
                        " as CANCoder " << can_id << " on bus " << can_bus);

    if (local_hardware_)
    {
        cancoder_ = std::make_unique<ctre::phoenix6::hardware::core::CoreCANcoder>(getId(), can_bus);
        setParentDevice(cancoder_.get());
        read_thread_state_ = std::make_unique<hardware_interface::cancoder::CANCoderHWState>(can_id);
        read_state_mutex_ = std::make_unique<std::mutex>();
        read_thread_ = std::make_unique<std::jthread>(&CANCoderDevice<SIM>::read_thread, this,
                                                      std::make_unique<Tracer>("cancoder_read_" + joint_name + " " + name_space),
                                                      read_hz_);
    }
}

template <bool SIM>
CANCoderDevice<SIM>::~CANCoderDevice(void) = default;

template <bool SIM>
void CANCoderDevice<SIM>::registerInterfaces(hardware_interface::cancoder::CANCoderStateInterface &state_interface,
                                             hardware_interface::cancoder::CANCoderCommandInterface &command_interface,
                                             hardware_interface::cancoder::RemoteCANcoderStateInterface &remote_state_interface,
                                             hardware_interface::cancoder::CANCoderSimCommandInterface &sim_command_interface) const
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for CANCoder : " << getName() << " at CAN id " << getId());

    hardware_interface::cancoder::CANCoderStateHandle state_handle(getName(), state_.get());
    state_interface.registerHandle(state_handle);

    hardware_interface::cancoder::CANCoderCommandHandle command_handle(state_handle, command_.get());
    command_interface.registerHandle(command_handle);

    if (!local_update_)
    {
        hardware_interface::cancoder::CANCoderWritableStateHandle remote_handle(getName(), state_.get());
        remote_state_interface.registerHandle(remote_handle);
    }

    if constexpr (SIM)
    {
        hardware_interface::cancoder::CANCoderSimCommandHandle command_sim_handle(state_handle, sim_fields_.command_sim_.get());
        sim_command_interface.registerHandle(command_sim_handle);
    }
}

template <bool SIM>
void CANCoderDevice<SIM>::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (local_update_)
    {
        std::unique_lock l(*read_state_mutex_, std::try_to_lock);
        if (l.owns_lock())
        {
            // These are used to convert position and velocity units - make sure the
            // read thread's local copy of state is kept up to date
            read_thread_state_->setConversionFactor(state_->getConversionFactor());
            read_thread_state_->setEnableReadThread(state_->getEnableReadThread());

            state_->setVersionMajor(read_thread_state_->getVersionMajor());
            state_->setVersionMinor(read_thread_state_->getVersionMinor());
            state_->setVersionBugfix(read_thread_state_->getVersionBugfix());
            state_->setVersionBuild(read_thread_state_->getVersionBuild());

			state_->setVelocity(read_thread_state_->getVelocity());
			state_->setPosition(read_thread_state_->getPosition());
			state_->setAbsolutePosition(read_thread_state_->getAbsolutePosition());
            state_->setUnfilteredVelocity(read_thread_state_->getUnfilteredVelocity());
			state_->setPositionSinceBoot(read_thread_state_->getPositionSinceBoot());
			state_->setSupplyVoltage(read_thread_state_->getSupplyVoltage());
			state_->setMagnetHealth(read_thread_state_->getMagnetHealth());

            state_->setFaultHardware(read_thread_state_->getFaultHardware());
            state_->setFaultUndervoltage(read_thread_state_->getFaultUndervoltage());
            state_->setFaultBootDuringEnable(read_thread_state_->getFaultBootDuringEnable());
            state_->setFaultUnlicensedFeatureInUse(read_thread_state_->getFaultUnlicensedFeatureInUse());
            state_->setFaultBadMagnet(read_thread_state_->getFaultBadMagnet());

            state_->setStickyFaultHardware(read_thread_state_->getStickyFaultHardware());
            state_->setStickyFaultUndervoltage(read_thread_state_->getStickyFaultUndervoltage());
            state_->setStickyFaultBootDuringEnable(read_thread_state_->getStickyFaultBootDuringEnable());
            state_->setStickyFaultUnlicensedFeatureInUse(read_thread_state_->getStickyFaultUnlicensedFeatureInUse());
            state_->setStickyFaultBadMagnet(read_thread_state_->getStickyFaultBadMagnet());
        }
    }
}

template <bool SIM>
void CANCoderDevice<SIM>::write(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (!local_hardware_)
    {
        return;
    }

    if (cancoder_->HasResetOccurred())
    {
        command_->resetMagnetSensorConfigs();
        ROS_WARN_STREAM("Reset detected on CANCoder " << getName());
    }
    state_->setConversionFactor(command_->getConversionFactor());

    hardware_interface::cancoder::SensorDirection sensor_direction;
    double offset_radians;
    hardware_interface::cancoder::AbsoluteSensorRange absolute_sensor_range;

    if (command_->magnetSensorConfigsChanged(sensor_direction,
                                             offset_radians,
                                             absolute_sensor_range))
    {
        ctre::phoenix6::configs::MagnetSensorConfigs magnet_sensor_configs;
        if (convertSensorDirection(sensor_direction, magnet_sensor_configs.SensorDirection) &&
            convertAbsoluteSensorRange(absolute_sensor_range, magnet_sensor_configs.AbsoluteSensorRange))
        {
            magnet_sensor_configs.MagnetOffset = units::turn_t{units::radian_t{offset_radians}}.value();
            if (safeCall(cancoder_->GetConfigurator().Apply(magnet_sensor_configs), "GetConfigurator().Apply(magnet_sensor_configs)"))
            {
                ROS_INFO_STREAM("Updated CANcoder id << " << getId() << " = " << getName() << "magnetSensorConfigs " << magnet_sensor_configs);
                state_->setSensorDirection(sensor_direction);
                state_->setMagnetOffset(offset_radians);
                state_->setAbsoluteSensorRange(absolute_sensor_range);
            }
            else
            {

                ROS_INFO_STREAM("Failed to update CANcoder id " << getId() << " = " << getName() << " MagnetSensorConfigs " << magnet_sensor_configs);
                command_->resetMagnetSensorConfigs();
                return;
            }
        }
    }

    if (double position; command_->setPositionChanged(position))
    {
        if (safeCall(cancoder_->SetPosition(units::radian_t{position / state_->getConversionFactor()}), "cancoder_->SetPosition"))
        {
            ROS_INFO_STREAM("CANcoder id = " << getId() << " = " << getName() << " : Set position to " << position);
            // Don't set state - it will be updated in next read() loop
        }
        else
        {
            command_->resetSetPosition();
            return;
        }
    }

    if (command_->clearStickyFaultsChanged())
    {
        if (safeCall(cancoder_->ClearStickyFaults(), "cancoder_->ClearStickyFaults"))
        {
            ROS_INFO_STREAM("CANcoder id = " << getId() << " = " << getName() << " : Sticky faults cleared");
        }
        else
        {
            command_->setClearStickyFaults();
            return;
        }
    }

    state_->setEnableReadThread(command_->getEnableReadThread());
}

// Set of macros for common code used to read 
// all signals with error checking.
// MAKE_SIGNAL is just an easy way to create the signal object
// that's used for subsequent reads
#define MAKE_SIGNAL(var, function) \
auto var##_signal = function; \
signals.push_back(&var##_signal);

// SAFE_READ takes the signal short name, passes the 
// previously-created signal name to the base-class
// safeRead, and checks the returned std::optional
// var. If the var is not valid, don't run any
// additional code in the loop.
#define SAFE_READ(var, function) \
const auto var = safeRead(var##_signal, #function); \
if (!var) { tracer->stop() ; continue; }

template <bool SIM>
void CANCoderDevice<SIM>::read_thread(std::unique_ptr<Tracer> tracer,
                                      const double poll_frequency)
{
#ifdef __linux__
    std::stringstream thread_name{"cancdr_rd_"};
    thread_name << getId();
    if (pthread_setname_np(pthread_self(), thread_name.str().c_str()))
    {
		ROS_ERROR_STREAM("Error setting thread name " << thread_name.str() << " " << errno);
	}
#endif
	ros::Duration(2.452 + read_thread_state_->getDeviceNumber() * 0.07).sleep(); // Sleep for a few seconds to let CAN start up
    ROS_INFO_STREAM("Starting CANCoder read thread for " << getName() << " id = " << getId());

    std::vector<ctre::phoenix6::BaseStatusSignal *> signals;

    MAKE_SIGNAL(version_major, cancoder_->GetVersionMajor())
    MAKE_SIGNAL(version_minor, cancoder_->GetVersionMinor())
    MAKE_SIGNAL(version_bugfix, cancoder_->GetVersionBugfix())
    MAKE_SIGNAL(version_build, cancoder_->GetVersionBuild())

    MAKE_SIGNAL(velocity, cancoder_->GetVelocity())
    MAKE_SIGNAL(position, cancoder_->GetPosition())
    MAKE_SIGNAL(absolute_position, cancoder_->GetAbsolutePosition())
    MAKE_SIGNAL(unfilitered_vecloity, cancoder_->GetUnfilteredVelocity())
    MAKE_SIGNAL(position_since_boot, cancoder_->GetPositionSinceBoot())
    MAKE_SIGNAL(supply_voltage, cancoder_->GetSupplyVoltage())
    MAKE_SIGNAL(magnet_health, cancoder_->GetMagnetHealth())

    MAKE_SIGNAL(fault_hardware, cancoder_->GetFault_Hardware())
    MAKE_SIGNAL(fault_undervoltage, cancoder_->GetFault_Undervoltage())
    MAKE_SIGNAL(fault_bootduringenable, cancoder_->GetFault_BootDuringEnable())
    MAKE_SIGNAL(fault_unlicensed_feature_in_use, cancoder_->GetFault_UnlicensedFeatureInUse())
    MAKE_SIGNAL(fault_bad_magnet, cancoder_->GetFault_BadMagnet())

    MAKE_SIGNAL(sticky_fault_hardware, cancoder_->GetStickyFault_Hardware())
    MAKE_SIGNAL(sticky_fault_undervoltage, cancoder_->GetStickyFault_Undervoltage())
    MAKE_SIGNAL(sticky_fault_bootduringenable, cancoder_->GetStickyFault_BootDuringEnable())
    MAKE_SIGNAL(sticky_fault_unlicensed_feature_in_use, cancoder_->GetStickyFault_UnlicensedFeatureInUse())
    MAKE_SIGNAL(sticky_fault_bad_magnet, cancoder_->GetStickyFault_BadMagnet())
	for (ros::Rate r(poll_frequency); ros::ok(); r.sleep())
	{
		tracer->start("cancoder read main_loop");

		double conversion_factor;

		// Update local status with relevant global config
		// values set by write(). This way, items configured
		// by controllers will be reflected in the read_thread_state_ here
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
			std::scoped_lock l(*read_state_mutex_);
            if (!read_thread_state_->getEnableReadThread())
            {
                return;
            }
			conversion_factor = read_thread_state_->getConversionFactor();
		}

        ctre::phoenix6::BaseStatusSignal::WaitForAll(units::second_t{0}, signals);

        SAFE_READ(version_major, cancoder_->GetVersionMajor())
        SAFE_READ(version_minor, cancoder_->GetVersionMinor())
        SAFE_READ(version_bugfix, cancoder_->GetVersionBugfix())
        SAFE_READ(version_build, cancoder_->GetVersionBuild())

        SAFE_READ(velocity, cancoder_->GetVelocity())
        SAFE_READ(position, cancoder_->GetPosition())
        SAFE_READ(absolute_position, cancoder_->GetAbsolutePosition())
        SAFE_READ(unfilitered_vecloity, cancoder_->GetUnfilteredVelocity())
        SAFE_READ(position_since_boot, cancoder_->GetPositionSinceBoot())
        SAFE_READ(supply_voltage, cancoder_->GetSupplyVoltage())
        SAFE_READ(magnet_health, cancoder_->GetMagnetHealth())

        SAFE_READ(fault_hardware, cancoder_->GetFault_Hardware())
        SAFE_READ(fault_undervoltage, cancoder_->GetFault_Undervoltage())
        SAFE_READ(fault_bootduringenable, cancoder_->GetFault_BootDuringEnable())
        SAFE_READ(fault_unlicensed_feature_in_use, cancoder_->GetFault_UnlicensedFeatureInUse())
        SAFE_READ(fault_bad_magnet, cancoder_->GetFault_BadMagnet())

        SAFE_READ(sticky_fault_hardware, cancoder_->GetStickyFault_Hardware())
        SAFE_READ(sticky_fault_undervoltage, cancoder_->GetStickyFault_Undervoltage())
        SAFE_READ(sticky_fault_bootduringenable, cancoder_->GetStickyFault_BootDuringEnable())
        SAFE_READ(sticky_fault_unlicensed_feature_in_use, cancoder_->GetStickyFault_UnlicensedFeatureInUse())
        SAFE_READ(sticky_fault_bad_magnet, cancoder_->GetStickyFault_BadMagnet())

		// Actually update the CANCoderHWread_thread_state_ shared between
		// this thread and read()
		// Do this all at once so the code minimizes the amount
		// of time with mutex locked
		{
			// Lock the read_thread_state_ entry to make sure writes
			// are atomic - reads won't grab data in
			// the middle of a write
			std::scoped_lock l(*read_state_mutex_);
            read_thread_state_->setVersionMajor(*version_major);
            read_thread_state_->setVersionMinor(*version_minor);
            read_thread_state_->setVersionBugfix(*version_bugfix);
            read_thread_state_->setVersionBuild(*version_build);

			read_thread_state_->setVelocity(units::radians_per_second_t{*velocity}.value() * conversion_factor);
			read_thread_state_->setPosition(units::radian_t{*position}.value() * conversion_factor);
			read_thread_state_->setAbsolutePosition(units::radian_t{*absolute_position}.value() * conversion_factor);
            read_thread_state_->setUnfilteredVelocity(units::radians_per_second_t{*unfilitered_vecloity}.value() * conversion_factor);
			read_thread_state_->setPositionSinceBoot(units::radian_t{*position_since_boot}.value() * conversion_factor);
			read_thread_state_->setSupplyVoltage(units::volt_t{*supply_voltage}.value());
            if (hardware_interface::cancoder::MagnetHealth hwi_magnet_health;
                convertMagnetHealth(*magnet_health, hwi_magnet_health))
            {
                read_thread_state_->setMagnetHealth(hwi_magnet_health);
            }

            read_thread_state_->setFaultHardware(*fault_hardware);
            read_thread_state_->setFaultUndervoltage(*fault_undervoltage);
            read_thread_state_->setFaultBootDuringEnable(*fault_bootduringenable);
            read_thread_state_->setFaultUnlicensedFeatureInUse(*fault_unlicensed_feature_in_use);
            read_thread_state_->setFaultBadMagnet(*fault_bad_magnet);

            read_thread_state_->setStickyFaultHardware(*sticky_fault_hardware);
            read_thread_state_->setStickyFaultUndervoltage(*sticky_fault_undervoltage);
            read_thread_state_->setStickyFaultBootDuringEnable(*sticky_fault_bootduringenable);
            read_thread_state_->setStickyFaultUnlicensedFeatureInUse(*sticky_fault_unlicensed_feature_in_use);
            read_thread_state_->setStickyFaultBadMagnet(*sticky_fault_bad_magnet);
		}
		tracer->report(60);
	}
}

static bool convertMagnetHealth(const ctre::phoenix6::signals::MagnetHealthValue &input,
                                hardware_interface::cancoder::MagnetHealth &output)
{
    switch (input.value)
    {
    case ctre::phoenix6::signals::MagnetHealthValue::Magnet_Invalid:
        output = hardware_interface::cancoder::MagnetHealth::Invalid;
        break;
    case ctre::phoenix6::signals::MagnetHealthValue::Magnet_Red:
        output = hardware_interface::cancoder::MagnetHealth::Red;
        break;
    case ctre::phoenix6::signals::MagnetHealthValue::Magnet_Orange:
        output = hardware_interface::cancoder::MagnetHealth::Orange;
        break;
    case ctre::phoenix6::signals::MagnetHealthValue::Magnet_Green:
        output = hardware_interface::cancoder::MagnetHealth::Green;
        break;
    default:
        ROS_ERROR("Invalid input in convertMagentHealth");
        return false;
    }
    return true;
}

static bool convertMagnetHealth(const hardware_interface::cancoder::MagnetHealth input,
                                ctre::phoenix6::signals::MagnetHealthValue &output)
{
    switch (input)
    {
    case hardware_interface::cancoder::MagnetHealth::Invalid:
        output.value = ctre::phoenix6::signals::MagnetHealthValue::Magnet_Invalid;
        break;
    case hardware_interface::cancoder::MagnetHealth::Red:
        output.value = ctre::phoenix6::signals::MagnetHealthValue::Magnet_Red;
        break;
    case hardware_interface::cancoder::MagnetHealth::Orange:
        output.value = ctre::phoenix6::signals::MagnetHealthValue::Magnet_Orange;
        break;
    case hardware_interface::cancoder::MagnetHealth::Green:
        output.value = ctre::phoenix6::signals::MagnetHealthValue::Magnet_Green;
        break;
    default:
        ROS_ERROR("Invalid input in convertMagnetHealth");
        return false;
    }
    return true;
}
static bool convertSensorDirection(hardware_interface::cancoder::SensorDirection input,
                                   ctre::phoenix6::signals::SensorDirectionValue &output)
{
    switch (input)
    {
    case hardware_interface::cancoder::SensorDirection::CounterClockwise_Positive:
        output = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
        break;
    case hardware_interface::cancoder::SensorDirection::Clockwise_Positive:
        output = ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;
        break;
    default:
        ROS_ERROR("Invalid input in convertSensorDirection");
        return false;
    }
    return true;
}
static bool convertAbsoluteSensorRange(hardware_interface::cancoder::AbsoluteSensorRange input,
                                       ctre::phoenix6::signals::AbsoluteSensorRangeValue &output)
{
    switch (input)
    {
    case hardware_interface::cancoder::AbsoluteSensorRange::Unsigned_0To1:
        output = ctre::phoenix6::signals::AbsoluteSensorRangeValue::Unsigned_0To1;
        break;
    case hardware_interface::cancoder::AbsoluteSensorRange::Signed_PlusMinusHalf:
        output = ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf;
        break;
    default:
        ROS_ERROR("Invalid input in convertAbsoluteSensorRange");
        return false;
    }
    return true;
}

template <bool SIM>
void CANCoderDevice<SIM>::simWrite(const ros::Time &time, Tracer &tracer)
{
    if constexpr (SIM)
    {
        tracer.start_unique("cancoder_sim_read");
        if (!local_hardware_)
        {
            return;
        }

        auto &sim_collection = cancoder_->GetSimState();

        if (double supply_voltage; sim_fields_.command_sim_->supplyVoltageChanged(supply_voltage))
        {
            if (safeCall(sim_collection.SetSupplyVoltage(units::volt_t{supply_voltage}), "cancoder sim->SetSupplyVoltage"))
            {
                ROS_INFO_STREAM("CANcoder id = " << getId() << " = " << getName() << " : Set supply voltage to " << supply_voltage);
                // Don't set state - it will be updated in next read() loop
            }
            else
            {
                sim_fields_.command_sim_->resetSupplyVoltage();
                return;
            }
        }
        if (double raw_position; sim_fields_.command_sim_->rawPositionChanged(raw_position))
        {
            ROS_INFO_STREAM("CANCoderDevice::simWrite raw_position: " << raw_position);
            if (safeCall(sim_collection.SetRawPosition(units::radian_t{raw_position}), "cancoder sim->SetRawPosition"))
            {
                ROS_INFO_STREAM("CANcoder id = " << getId() << " = " << getName() << " : Set raw position to " << raw_position);
                // Don't set state - it will be updated in next read() loop
            }
            else
            {
                sim_fields_.command_sim_->resetRawPosition();
                return;
            }
        }
        if (double add_position; sim_fields_.command_sim_->addPositionChanged(add_position))
        {
            if (safeCall(sim_collection.AddPosition(units::radian_t{add_position}), "cancoder sim->AddPosition"))
            {
                ROS_INFO_STREAM("CANcoder id = " << getId() << " = " << getName() << " : Adding position " << add_position);
                // Don't set state - it will be updated in next read() loop
            }
            else
            {
                sim_fields_.command_sim_->resetAddPosition();
                return;
            }
        }
        if (double velocity; sim_fields_.command_sim_->velocityChanged(velocity))
        {
            if (safeCall(sim_collection.SetVelocity(units::radians_per_second_t{velocity}), "cancoder sim->SetVelocity"))
            {
                ROS_INFO_STREAM("CANcoder id = " << getId() << " = " << getName() << " : Set velocity to " << velocity);
                // Don't set state - it will be updated in next read() loop
            }
            else
            {
                sim_fields_.command_sim_->resetVelocity();
                return;
            }
        }
        if (hardware_interface::cancoder::MagnetHealth magnet_health_hwi; sim_fields_.command_sim_->magnetHealthChanged(magnet_health_hwi))
        {
            ctre::phoenix6::signals::MagnetHealthValue magnet_health_ctre;
            if (convertMagnetHealth(magnet_health_hwi, magnet_health_ctre))
            {
                if (safeCall(sim_collection.SetMagnetHealth(magnet_health_ctre), "cancoder sim->SetMagnetHealth"))
                {
                    // Don't set state - it will be updated in next read() loop
                }
                else
                {
                    sim_fields_.command_sim_->resetMagnetHealth();
                    return;
                }
            }
        }
    }
}
