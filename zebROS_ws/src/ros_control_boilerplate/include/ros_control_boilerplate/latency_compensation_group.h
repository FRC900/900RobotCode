// This is a device which creates a set of CTREv6 sensor signals
// For each signal, both the value and slope are stored
// (for example, both position and velocity) along with
// the timestamp of the most recent sensor reading.  Reads
// of all sensors in a group are synchronized.
// Using the timestamp plus slope, a linear approximation
// can be applied to compensate for latency between
// reading and using the signals.
#ifndef LATENCY_COMPENSATION_GROUP_INC__
#define LATENCY_COMPENSATION_GROUP_INC__

#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "xmlrpcpp/XmlRpcValue.h"

namespace ctre::phoenix6
{
    class BaseStatusSignal;
    template <typename T> class StatusSignal;
    namespace hardware
    {
        class ParentDevice;
    }
}
namespace hardware_interface::latency_compensation
{
    class CTRELatencyCompensationState;
    class CTRELatencyCompensationStateInterface;
}

class LatencyCompensationGroupEntryBase;

class LatencyCompensationGroup
{
public:
    LatencyCompensationGroup(const XmlRpc::XmlRpcValue &entry_array,
                             const std::string &name,
                             const double update_frequency,
                             const std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> &devices);
    LatencyCompensationGroup(const LatencyCompensationGroup &) = delete;
    LatencyCompensationGroup(LatencyCompensationGroup &&other) noexcept = delete;
    virtual ~LatencyCompensationGroup();

    LatencyCompensationGroup &operator=(const LatencyCompensationGroup &) = delete;
    LatencyCompensationGroup &operator=(LatencyCompensationGroup &&) noexcept = delete;

    void registerInterfaces(hardware_interface::latency_compensation::CTRELatencyCompensationStateInterface &state_interface) const;
    void read(void);

private:
    template <typename VALUE_SIGNAL_TYPE, typename SLOPE_SIGNAL_TYPE>
    void create_group_entry(const std::string &name,
                            ctre::phoenix6::StatusSignal<VALUE_SIGNAL_TYPE> *value_signal,
                            ctre::phoenix6::StatusSignal<SLOPE_SIGNAL_TYPE> *slope_signal);
    void read_thread();
    const std::string name_;
    std::map<const std::string, std::unique_ptr<LatencyCompensationGroupEntryBase>> entries_;
    std::vector<ctre::phoenix6::BaseStatusSignal *> signals_;

    std::unique_ptr<hardware_interface::latency_compensation::CTRELatencyCompensationState> state_;

    std::mutex read_state_mutex_;
    std::unique_ptr<hardware_interface::latency_compensation::CTRELatencyCompensationState> read_thread_state_;
    std::jthread read_thread_;
};

#endif
