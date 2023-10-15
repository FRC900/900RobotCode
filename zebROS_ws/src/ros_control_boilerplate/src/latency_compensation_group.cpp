#include <chrono>

#include "ros/ros.h"

#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/core/CoreCANcoder.hpp"
#include "ctre/phoenix6/core/CorePigeon2.hpp"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"

#include "ctre_interfaces/latency_compensation_state_interface.h"
#include "ros_control_boilerplate/latency_compensation_group.h"
#include "ros_control_boilerplate/read_config_utils.h"
#include "ros_control_boilerplate/tracer.h"


// Need a map which handles several different types of particular
// instances of the LatencyCompensationGroupEntry type.
// Create base class, store pointers to that in the map
// Then use dynamic polymorphism to access the derived
// class's overrides of the get functions to access
// the member values using the correct type info
class LatencyCompensationGroupEntryBase
{
public:
    LatencyCompensationGroupEntryBase() = default;
    LatencyCompensationGroupEntryBase(const LatencyCompensationGroupEntryBase &) = delete;
    LatencyCompensationGroupEntryBase(LatencyCompensationGroupEntryBase &&other) noexcept = delete;
    virtual ~LatencyCompensationGroupEntryBase() = default;

    LatencyCompensationGroupEntryBase &operator=(const LatencyCompensationGroupEntryBase &) = delete;
    LatencyCompensationGroupEntryBase &operator=(LatencyCompensationGroupEntryBase &&) noexcept = delete;
    virtual ros::Time getTimestamp(void) const = 0;
    virtual double getValue(void) const = 0; 
    virtual double getSlope(void) const = 0;
};

// Each entry in a group stores the value of a signal as well as the slope
// (derivative w.r.t. time) of that signal at the same time.  All entries
// in a group have their data synchronized, so that they are read at the 
// same time. 
// Using the value + dt * slope gives the latency compensated value
template <typename VALUE_SIGNAL_TYPE, typename SLOPE_SIGNAL_TYPE>
class LatencyCompensationGroupEntry : public LatencyCompensationGroupEntryBase
{
public:
    LatencyCompensationGroupEntry(ctre::phoenix6::StatusSignal<VALUE_SIGNAL_TYPE> *value_signal,
                                  ctre::phoenix6::StatusSignal<SLOPE_SIGNAL_TYPE> *slope_signal)
        : value_signal_{value_signal}
        , slope_signal_{slope_signal}
    {
    }
    LatencyCompensationGroupEntry(const LatencyCompensationGroupEntry &) = delete;
    LatencyCompensationGroupEntry(LatencyCompensationGroupEntry &&other) noexcept = delete;
    virtual ~LatencyCompensationGroupEntry() = default;

    LatencyCompensationGroupEntry &operator=(const LatencyCompensationGroupEntry &) = delete;
    LatencyCompensationGroupEntry &operator=(LatencyCompensationGroupEntry &&) noexcept = delete;

    ros::Time getTimestamp(void) const override
    {
        return ros::Time(value_signal_->GetTimestamp().GetTime().value());
    }
    double getValue(void) const override
    {
        return units::radian_t{value_signal_->GetValue()}.value();
    }
    double getSlope(void) const override
    {
        return units::radians_per_second_t{slope_signal_->GetValue()}.value();
    }

private:
    ctre::phoenix6::StatusSignal<VALUE_SIGNAL_TYPE> *value_signal_;
    ctre::phoenix6::StatusSignal<SLOPE_SIGNAL_TYPE> *slope_signal_;
};

LatencyCompensationGroup::LatencyCompensationGroup(const XmlRpc::XmlRpcValue &entry_array,
                                                   const std::string &name,
                                                   const double update_frequency,
                                                   const std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> &devices)
    : name_{name}
    , state_{std::make_unique<hardware_interface::latency_compensation::CTRELatencyCompensationState>(name)}
    , read_thread_state_{std::make_unique<hardware_interface::latency_compensation::CTRELatencyCompensationState>(name)}
{
	for (int i = 0; i < entry_array.size(); i++)
    {
        const XmlRpc::XmlRpcValue &entry = entry_array[i];

        std::string entry_name;
        std::string entry_type;
        readStringRequired(entry, "name", entry_name);
        readStringRequired(entry, "type", entry_type, entry_name);

        auto check_for_correct_pointer_entry = [&]<typename T>()
        {
            const auto map_entries = devices.equal_range(entry_name);
            for (auto map_entry = map_entries.first; map_entry != map_entries.second; ++map_entry)
            {
                const auto pointer = dynamic_cast<T *>(map_entry->second);
                if (pointer)
                {
                    return pointer;
                }
            }
            return static_cast<T *>(nullptr);
        };

        if (entry_type == "talonfx")
        {
            const auto talon_fx_ptr = check_for_correct_pointer_entry.operator()<ctre::phoenix6::hardware::core::CoreTalonFX>();
            if (talon_fx_ptr)
            {
                ROS_WARN_STREAM("Got device ID " << talon_fx_ptr->GetDeviceID() << " for entry " << entry_name);
                create_group_entry(entry_name, &talon_fx_ptr->GetPosition(), &talon_fx_ptr->GetVelocity());
            }
            else
            {
                throw std::runtime_error("Latency compensation group " + name + " : could not find TalonFXPro joint " + entry_name);
            }
        }
        else if (entry_type == "cancoder")
        {
            const auto cancoder_ptr = check_for_correct_pointer_entry.operator()<ctre::phoenix6::hardware::core::CoreCANcoder>();
            if (cancoder_ptr)
            {
                create_group_entry(entry_name, &cancoder_ptr->GetPosition(), &cancoder_ptr->GetVelocity());
            }
            else
            {
                throw std::runtime_error("Latency compensation group " + name + " : could not find CANcoder joint " + entry_name);
            }
        }
        else if (entry_type == "pigeon2_roll")
        {
            const auto pigeon2_ptr = check_for_correct_pointer_entry.operator()<ctre::phoenix6::hardware::core::CorePigeon2>();
            if (pigeon2_ptr)
            {
                create_group_entry(entry_name, &pigeon2_ptr->GetRoll(), &pigeon2_ptr->GetAngularVelocityX());
            }
            else
            {
                throw std::runtime_error("Latency compensation group " + name + " : could not find Pigeon2 joint " + entry_name);
            }
        }
        else if (entry_type == "pigeon2_pitch")
        {
            const auto pigeon2_ptr = check_for_correct_pointer_entry.operator()<ctre::phoenix6::hardware::core::CorePigeon2>();
            if (pigeon2_ptr)
            {
                create_group_entry(entry_name, &pigeon2_ptr->GetPitch(), &pigeon2_ptr->GetAngularVelocityY());
            }
            else
            {
                throw std::runtime_error("Latency compensation group " + name + " : could not find Pigeon2 joint " + entry_name);
            }
        }
        else if (entry_type == "pigeon2_yaw")
        {
            const auto pigeon2_ptr = check_for_correct_pointer_entry.operator()<ctre::phoenix6::hardware::core::CorePigeon2>();
            if (pigeon2_ptr)
            {
                create_group_entry(entry_name, &pigeon2_ptr->GetYaw(), &pigeon2_ptr->GetAngularVelocityZ());
            }
            else
            {
                throw std::runtime_error("Latency compensation group " + name + " : could not find Pigeon2 joint " + entry_name);
            }
        }
    }
    for (auto &signal : signals_)
    {
        ROS_WARN_STREAM("Setting update frequency to " << update_frequency << " for signal " << signal->GetName());
        signal->SetUpdateFrequency(units::frequency::hertz_t{update_frequency});
    }
    read_thread_ = std::thread(&LatencyCompensationGroup::read_thread, this);
}

LatencyCompensationGroup::~LatencyCompensationGroup()
{
    if (read_thread_.joinable())
    {
        read_thread_.join();
    }
}

void LatencyCompensationGroup::registerInterfaces(hardware_interface::latency_compensation::CTRELatencyCompensationStateInterface &state_interface)
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for LatencyInterface : " << name_);
    hardware_interface::latency_compensation::CTRELatencyCompensationStateHandle state_handle(name_, state_.get());
    state_interface.registerHandle(state_handle);
}

template <typename VALUE_SIGNAL_TYPE, typename SLOPE_SIGNAL_TYPE>
void LatencyCompensationGroup::create_group_entry(const std::string &name,
                                                  ctre::phoenix6::StatusSignal<VALUE_SIGNAL_TYPE> *value_signal,
                                                  ctre::phoenix6::StatusSignal<SLOPE_SIGNAL_TYPE> *slope_signal)
{
    signals_.push_back(value_signal);
    signals_.push_back(slope_signal);
    entries_.emplace(name,
                     std::make_unique<LatencyCompensationGroupEntry<VALUE_SIGNAL_TYPE, SLOPE_SIGNAL_TYPE>>(value_signal, slope_signal));
    state_->addEntry(name);
    read_thread_state_->addEntry(name);
    ROS_INFO_STREAM("Created entry in latency group " << name_ << " for entry " << name);
}

void LatencyCompensationGroup::read(void)
{
    std::unique_lock<std::mutex> l(read_state_mutex_, std::try_to_lock);
    if (l.owns_lock())
    {
        // Copy from the most recent value read from hardware
        // into the state buffer accessable by the rest of the code
        for (const auto &entry : entries_)
        {
            ros::Time timestamp;
            double value;
            double slope;
            read_thread_state_->getEntry(entry.first, timestamp, value, slope);
            state_->setEntry(entry.first, timestamp, value, slope);
        }
    }
}

void LatencyCompensationGroup::read_thread()
{
    ros::Duration(2.63).sleep();
    Tracer tracer("latency compensation " + name_);
    ROS_INFO_STREAM("Starting latency compensation read thread for " << name_ << " at " << ros::Time::now());
    ROS_INFO_STREAM("CTRE / steady clock time = " << ctre:: phoenix6::GetCurrentTimeSeconds());
    while (ros::ok())
    {
        tracer.start("WaitForAll");
        auto status = ctre::phoenix6::BaseStatusSignal::WaitForAll(units::time::second_t{0.01}, signals_);
        tracer.start_unique("Update state");
        if (status.IsOK())
        {
            // Redo this offset from steady-clock time to wall clock time
            // each iteration in case system time changes.
            const ros::Duration time_offset{ros::Time::now().toSec() - ctre::phoenix6::GetCurrentTimeSeconds()};
            std::scoped_lock l{read_state_mutex_};

            for (const auto &entry : entries_)
            {
                read_thread_state_->setEntry(entry.first,
                                             entry.second->getTimestamp() + time_offset,
                                             entry.second->getValue(),
                                             entry.second->getSlope());
            }
        }
        else
        {
            ROS_ERROR_STREAM("waitForAll failed for latency compensation group " << name_ << " : " << status.GetName());
        }
        tracer.report(60);
    }
}