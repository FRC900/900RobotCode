#ifndef SIMULATOR_DEVICE_INC__
#define SIMULATOR_DEVICE_INC__

#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "xmlrpcpp/XmlRpcValue.h"
#include "simulator_interface/simulator_base.h"
#include "pluginlib/class_loader.h"
#include "ros_control_boilerplate/tracer.h"


namespace hardware_interface::talonfxpro
{
    class TalonFXProHWState;
}

class SimulatorDevice
{
public:
    SimulatorDevice(const std::string &name, const XmlRpc::XmlRpcValue &joints, const boost::shared_ptr<simulator_base::Simulator> &simulator, const std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> &devices);
    SimulatorDevice(const SimulatorDevice &) = delete;
    SimulatorDevice(SimulatorDevice &&other) noexcept = delete;
    virtual ~SimulatorDevice();

    SimulatorDevice &operator=(const SimulatorDevice &) = delete;
    SimulatorDevice &operator=(SimulatorDevice &&) noexcept = delete;

    void simInit(ros::NodeHandle &nh);
    void simStep(const ros::Time& time, const ros::Duration& period, hardware_interface::talonfxpro::TalonFXProSimCommandInterface *sim_talonfxpro_if, Tracer &tracer);
    
    std::string simulator_name_;

private:

    std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> ctre_devices_;
    boost::shared_ptr<simulator_base::Simulator> simulator_;

    std::vector<std::string> joints_;
    std::vector<std::string> names_;
    std::unique_ptr<hardware_interface::talonfxpro::TalonFXProStateInterface> state_interface_;
};

#endif
