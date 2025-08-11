#ifndef TALON_ORCHESTRA_DEVICE_INC__
#define TALON_ORCHESTRA_DEVICE_INC__

#include <map>
#include <memory>
#include <string>
#include "ros_control_boilerplate/ctre_v6_device.h"

namespace ctre::phoenix6
{
    class Orchestra;
    namespace hardware
    {
        class ParentDevice;
    }
}
namespace hardware_interface
{
    class OrchestraState;
    class OrchestraCommand;
    class OrchestraStateInterface;
    class OrchestraCommandInterface;
}
class Tracer;

template <bool SIMFLAG>
class TalonOrchestraDevice : public CTREV6Device
{
public:
    TalonOrchestraDevice(const std::string &name_space,
                         const int joint_index,
                         const std::string &joint_name);
    TalonOrchestraDevice(const TalonOrchestraDevice &) = delete;
    TalonOrchestraDevice(TalonOrchestraDevice &&other) noexcept = delete;
    ~TalonOrchestraDevice() override;

    TalonOrchestraDevice &operator=(const TalonOrchestraDevice &) = delete;
    TalonOrchestraDevice &operator=(TalonOrchestraDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::OrchestraStateInterface &state_interface,
                            hardware_interface::OrchestraCommandInterface &command_interface) const;

    void read(const ros::Time &/*time*/, const ros::Duration &/*period*/);
    void write(const ros::Time &/*time*/, const ros::Duration &/*period*/,
               const std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> &talonfxs);

private:
    std::unique_ptr<ctre::phoenix6::Orchestra> orchestra_;

    std::unique_ptr<hardware_interface::OrchestraState> state_;
    std::unique_ptr<hardware_interface::OrchestraCommand> command_;
};

#endif