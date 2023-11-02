#ifndef INC_LATENCY_COMPENSATION_GROUPS_H_INC__
#define INC_LATENCY_COMPENSATION_GROUPS_H_INC__

#include "ros_control_boilerplate/devices.h"

class LatencyCompensationGroup;
namespace hardware_interface::latency_compensation
{
    class CTRELatencyCompensationStateInterface;
}

namespace ctre::phoenix6::hardware
{
    class ParentDevice;
}

class LatencyCompensationGroups : public Devices
{
public:
    LatencyCompensationGroups(ros::NodeHandle &root_nh,
                              const std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> &devices);
    LatencyCompensationGroups(const LatencyCompensationGroups &) = delete;
    LatencyCompensationGroups(LatencyCompensationGroups &&) noexcept = delete;
    ~LatencyCompensationGroups() override;

    LatencyCompensationGroups &operator=(const LatencyCompensationGroups &) = delete;
    LatencyCompensationGroups &operator=(LatencyCompensationGroups &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

private:
    std::vector<std::unique_ptr<LatencyCompensationGroup>> devices_;
    std::unique_ptr<hardware_interface::latency_compensation::CTRELatencyCompensationStateInterface> state_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

#endif