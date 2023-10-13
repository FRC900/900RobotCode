#ifndef MATCH_DATA_DEVICE_INC__
#define MATCH_DATA_DEVICE_INC__

#include <optional>
#include <string>

#include <ros/node_handle.h>

namespace frc { class MatchData; }
namespace hardware_interface
{
    class MatchHWState;
    class MatchStateInterface;
    class RemoteMatchStateInterface;
}
class PeriodicIntervalCounter;

class MatchDataDevice
{
public:
    MatchDataDevice(ros::NodeHandle &nh);
    MatchDataDevice(const MatchDataDevice &) = delete;
    MatchDataDevice(MatchDataDevice &&other) noexcept = delete;
    virtual ~MatchDataDevice();

    MatchDataDevice &operator=(const MatchDataDevice &) = delete;
    MatchDataDevice &operator=(MatchDataDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::MatchStateInterface &state_interface,
                            hardware_interface::RemoteMatchStateInterface &remote_state_interface);
    virtual void read(const ros::Time& /*time*/, const ros::Duration& period);

    virtual void simInit(ros::NodeHandle nh) {};

    virtual std::optional<bool> isEnabled(void) const;
    virtual bool getControlWord(HAL_ControlWord &cw) const;

protected:
    bool getLocal(void) const { return local_; }
private:
    bool local_{true};
    std::unique_ptr<hardware_interface::MatchHWState> state_;
    std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
};

#endif