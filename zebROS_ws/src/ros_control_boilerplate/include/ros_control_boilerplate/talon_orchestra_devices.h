#ifndef TALON_ORCHESTRA_DEVICES_H_INC__
#define TALON_ORCHESTRA_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"

template <bool SIMFLAG> class TalonOrchestraDevice;
namespace hardware_interface
{
    class OrchestraStateInterface;
    class OrchestraCommandInterface;
}

namespace ctre::phoenix6::hardware
{
    class ParentDevice;
}

template <bool SIMFLAG>
class TalonOrchestraDevices : public Devices
{
public:
    explicit TalonOrchestraDevices(ros::NodeHandle &root_nh);
    TalonOrchestraDevices(const TalonOrchestraDevices &) = delete;
    TalonOrchestraDevices(TalonOrchestraDevices &&) noexcept = delete;
    ~TalonOrchestraDevices() override;

    TalonOrchestraDevices &operator=(const TalonOrchestraDevices &) = delete;
    TalonOrchestraDevices &operator=(TalonOrchestraDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;
    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

    void setTalonFXData(const std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> &talonfxs);

private:
    std::vector<std::unique_ptr<TalonOrchestraDevice<SIMFLAG>>> devices_;
    std::unique_ptr<hardware_interface::OrchestraStateInterface> state_interface_;
    std::unique_ptr<hardware_interface::OrchestraCommandInterface> command_interface_;
    hardware_interface::InterfaceManager interface_manager_;

    std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> talonfxs_;
};

#endif