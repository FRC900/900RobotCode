#ifndef LATENCY_COMPENSATION_STATE_INTERFACE_INC__
#define LATENCY_COMPENSATION_STATE_INTERFACE_INC__

#include <map>
#include <memory>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include "state_handle/state_handle.h"

namespace hardware_interface::latency_compensation
{

class CTRELatencyCompensationEntry;

class CTRELatencyCompensationState
{
public:
    CTRELatencyCompensationState(const std::string &name);
    CTRELatencyCompensationState(const CTRELatencyCompensationState &) = delete;
    CTRELatencyCompensationState(CTRELatencyCompensationState &&) noexcept = delete;
    ~CTRELatencyCompensationState();
    CTRELatencyCompensationState &operator=(const CTRELatencyCompensationState &) = delete;
    CTRELatencyCompensationState &operator=(CTRELatencyCompensationState &&) = delete;

    void addEntry(const std::string &name);
    void setEntry(const std::string &name, const ros::Time &timestamp, const double value, const double slope);
    void getEntry(const std::string &name, ros::Time &timestamp, double &value, double &slope) const;

    std::vector<std::string> getEntryNames(void) const;

    double getLatencyCompensatedValue(const std::string &name, const ros::Time &timestamp) const;

private:
    std::string name_;
    std::map<std::string, std::unique_ptr<CTRELatencyCompensationEntry>> entries_;
};

// Glue code to let this be registered in the list of
// hardware resources on the robot.  Since state is
// read-only, allow multiple controllers to register it.
using CTRELatencyCompensationStateHandle = StateHandle<const CTRELatencyCompensationState>;
class CTRELatencyCompensationStateInterface : public HardwareResourceManager<CTRELatencyCompensationStateHandle> {};
}

#endif