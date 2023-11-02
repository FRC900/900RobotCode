#ifndef SPARKMAX_DEVICES_H_INC__
#define SPARKMAX_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"

template <bool SIMFLAG> class SparkMaxDevice;
namespace hardware_interface
{
    class SparkMaxStateInterface;
    class SparkMaxCommandInterface;
    class RemoteSparkMaxStateInterface;
}

template <bool SIMFLAG>
class SparkMaxDevices : public Devices
{
public:
    explicit SparkMaxDevices(ros::NodeHandle &root_nh);
    SparkMaxDevices(const SparkMaxDevices &) = delete;
    SparkMaxDevices(SparkMaxDevices &&) noexcept = delete;
    virtual ~SparkMaxDevices();

    SparkMaxDevices &operator=(const SparkMaxDevices &) = delete;
    SparkMaxDevices &operator=(SparkMaxDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;
    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

private:
    double read_hz_{100};
    std::vector<std::unique_ptr<SparkMaxDevice<SIMFLAG>>> devices_;
    std::unique_ptr<hardware_interface::SparkMaxStateInterface> spark_max_state_interface_;
    std::unique_ptr<hardware_interface::SparkMaxCommandInterface> spark_max_command_interface_;
    std::unique_ptr<hardware_interface::RemoteSparkMaxStateInterface> remote_spark_max_state_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

using HWSparkMaxDevices = SparkMaxDevices<false>;
using SimSparkMaxDevices = SparkMaxDevices<true>;

#endif