#ifndef INC_TALONFXPRO_DEVICES_H_INC__
#define INC_TALONFXPRO_DEVICES_H_INC__
#include <map>

#include "ros_control_boilerplate/devices.h"
#include "ros_control_boilerplate/set_limit_switch.h"
#include "ros_control_boilerplate/set_current.h"

class TalonFXProDevice;
namespace hardware_interface::talonfxpro
{
    class TalonFXProStateInterface;
    class TalonFXProCommandInterface;
}

namespace ctre::phoenix6::hardware
{
    class ParentDevice;
}

class TalonFXProDevices : public Devices
{

public:
    TalonFXProDevices(ros::NodeHandle &root_nh);
    TalonFXProDevices(const TalonFXProDevices &) = delete;
    TalonFXProDevices(TalonFXProDevices &&) noexcept = delete;
    virtual ~TalonFXProDevices();

    TalonFXProDevices &operator=(const TalonFXProDevices &) = delete;
    TalonFXProDevices &operator=(TalonFXProDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;
    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

    void simInit(ros::NodeHandle nh) override;
    // simRead hooks up to CTRE simulation code and updates motor state each control cycle
    void simRead(const ros::Time &time, const ros::Duration &period, Tracer &tracer) override;

    void appendDeviceMap(std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> &device_map) const;

private:
    std::vector<std::unique_ptr<TalonFXProDevice>> devices_;
    std::unique_ptr<hardware_interface::talonfxpro::TalonFXProStateInterface> state_interface_;
    std::unique_ptr<hardware_interface::talonfxpro::TalonFXProCommandInterface> command_interface_;
    hardware_interface::InterfaceManager interface_manager_;
    double read_hz_{100.};

    bool setlimit(ros_control_boilerplate::set_limit_switch::Request &req,
                  ros_control_boilerplate::set_limit_switch::Response &res);
    bool setcurrent(ros_control_boilerplate::set_current::Request &req,
                    ros_control_boilerplate::set_current::Response &res);

    ros::ServiceServer sim_limit_switch_srv_;
    ros::ServiceServer sim_current_srv_;

    bool prev_robot_enabled_{true};
};

#endif