#ifndef INC_SIMULATOR_DEVICES_H_INC__
#define INC_SIMULATOR_DEVICES_H_INC__

#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "ros_control_boilerplate/devices.h"
#include "simulator_interface/simulator_base.h"
#include "pluginlib/class_loader.h"
#include "ros_control_boilerplate/tracer.h"

class SimulatorDevice; // forward declaration so we can use the class (I think?)
namespace hardware_interface::talonfxpro
{
    class TalonFXProHWState;
    class TalonFXProStateInterface;
}

namespace ctre::phoenix6::hardware
{
    class ParentDevice;
}

class SimulatorDevices : public Devices
{
public:
    SimulatorDevices(ros::NodeHandle &root_nh,
                              const std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> &devices);
    SimulatorDevices(const SimulatorDevices &) = delete;
    SimulatorDevices(SimulatorDevices &&) noexcept = delete;
    ~SimulatorDevices() override;

    SimulatorDevices &operator=(const SimulatorDevices &) = delete;
    SimulatorDevices &operator=(SimulatorDevices &&) noexcept = delete;

    // these run in this order
    void simInit(ros::NodeHandle &nh) override;
    void simPostRead(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

private:
    std::vector<std::unique_ptr<SimulatorDevice>> devices_;
    std::unique_ptr<hardware_interface::talonfxpro::TalonFXProStateInterface> state_interface_;
    hardware_interface::InterfaceManager interface_manager_;

    std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> ctre_devices_;

    std::unique_ptr<pluginlib::ClassLoader<simulator_base::Simulator>> loader_;
    std::map<std::string, boost::shared_ptr<simulator_base::Simulator>> simulators_;
    std::map<std::string, std::string> simulator_types_;

    std::vector<std::string> controlled_joints_;
};

#endif