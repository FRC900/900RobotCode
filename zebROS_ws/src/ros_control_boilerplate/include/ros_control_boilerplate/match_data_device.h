#ifndef MATCH_DATA_DEVICE_INC__
#define MATCH_DATA_DEVICE_INC__

#include <optional>
#include <string>

#include <ros/node_handle.h>

namespace hardware_interface::match_data
{
    class MatchHWState;
    class MatchStateInterface;
    class RemoteMatchStateInterface;
}
class PeriodicIntervalCounter;

class MatchDataDevice
{
public:
    explicit MatchDataDevice(const ros::NodeHandle &nh);
    MatchDataDevice(const MatchDataDevice &) = delete;
    MatchDataDevice(MatchDataDevice &&other) noexcept = delete;
    virtual ~MatchDataDevice();

    MatchDataDevice &operator=(const MatchDataDevice &) = delete;
    MatchDataDevice &operator=(MatchDataDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::match_data::MatchStateInterface &state_interface,
                            hardware_interface::match_data::RemoteMatchStateInterface &remote_state_interface) const;
    virtual void read(const ros::Time& /*time*/, const ros::Duration& period);

    virtual void simInit(ros::NodeHandle &nh) {};

    virtual std::optional<bool> isEnabled(void) const;
    virtual bool getControlWord(HAL_ControlWord &cw) const;

protected:
    bool getLocal(void) const { return local_; }
private:
    bool local_{true};
    std::unique_ptr<hardware_interface::match_data::MatchHWState> state_;
    std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
};

#endif