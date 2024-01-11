#ifndef MATCH_DATA_DEVICES_H_INC__
#define MATCH_DATA_DEVICES_H_INC__

#include <mutex>
#include <optional>

#include "ros_control_boilerplate/devices.h"

#include "hal/DriverStationTypes.h"

class MatchDataDevice;
namespace hardware_interface::match_data
{
    class MatchStateInterface;
    class RemoteMatchStateInterface;
}
struct HAL_ControlWord;

class MatchDataDevice;
class SimMatchDataDevice;

template <bool SIM>
class MatchDataDevices : public Devices
{
public:
    explicit MatchDataDevices(ros::NodeHandle &root_nh);
    MatchDataDevices(const MatchDataDevices &) = delete;
    MatchDataDevices(MatchDataDevices &&) noexcept = delete;
    virtual ~MatchDataDevices();

    MatchDataDevices &operator=(const MatchDataDevices &) = delete;
    MatchDataDevices &operator=(MatchDataDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

    void simInit(ros::NodeHandle &nh) override;

    std::optional<bool> isEnabled(void) const;
    bool getControlWord(HAL_ControlWord &cw) const;

private:
    using DEVICE_TYPE = std::conditional_t<SIM, SimMatchDataDevice, MatchDataDevice>;
    std::vector<std::unique_ptr<DEVICE_TYPE>> devices_;
    std::unique_ptr<hardware_interface::match_data::MatchStateInterface> state_interface_;
    std::unique_ptr<hardware_interface::match_data::RemoteMatchStateInterface> remote_state_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

#endif