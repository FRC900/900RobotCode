#ifndef INC_TALONFXPRO_DEVICES_H_INC__
#define INC_TALONFXPRO_DEVICES_H_INC__
#include <map>

#include "ros_control_boilerplate/devices.h"
#include "ros_control_boilerplate/set_limit_switch.h"
#include "ros_control_boilerplate/set_current.h"

class SimTalonFXProDevice;
class TalonFXProDevice;
namespace hardware_interface::talonfxpro
{
    class TalonFXProStateInterface;
    class TalonFXProCommandInterface;
    class TalonFXProSimCommandInterface;
}

namespace ctre::phoenix6::hardware
{
    class ParentDevice;
}

template <bool SIM>
class TalonFXProDevices : public Devices
{
public:
    explicit TalonFXProDevices(ros::NodeHandle &root_nh);
    TalonFXProDevices(const TalonFXProDevices &) = delete;
    TalonFXProDevices(TalonFXProDevices &&) noexcept = delete;
    ~TalonFXProDevices() override;

    TalonFXProDevices &operator=(const TalonFXProDevices &) = delete;
    TalonFXProDevices &operator=(TalonFXProDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;
    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

    void appendDeviceMap(std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> &device_map) const;

    // Sim-only functions below
    void simInit(ros::NodeHandle &nh) override;

    void simPreRead(const ros::Time &time, const ros::Duration &period, Tracer &tracer) override;
    // simPostRead is responsible for writing changes to the actual CTRE simulation code
    void simPostRead(const ros::Time &time, const ros::Duration &period, Tracer &tracer) override;

    bool gazeboSimInit(const ros::NodeHandle &/*nh*/, boost::shared_ptr<gazebo::physics::Model> parent_model) override;

private:
    using DEVICE_TYPE = std::conditional_t<SIM, SimTalonFXProDevice, TalonFXProDevice>;
    std::vector<std::unique_ptr<DEVICE_TYPE>> devices_;
    std::unique_ptr<hardware_interface::talonfxpro::TalonFXProStateInterface> state_interface_;
    std::unique_ptr<hardware_interface::talonfxpro::TalonFXProCommandInterface> command_interface_;
    hardware_interface::InterfaceManager interface_manager_;
    double read_hz_{100.};

    // Set this to true so the first time through ::write there will
    // be a forced true->false transition. This will be used to force the
    // devices into a disabled state.
    bool prev_robot_enabled_{true};
    
    // Sim-only functions below
    bool setlimit(ros_control_boilerplate::set_limit_switch::Request &req,
                  ros_control_boilerplate::set_limit_switch::Response &res);
    bool setcurrent(ros_control_boilerplate::set_current::Request &req,
                    ros_control_boilerplate::set_current::Response &res);

    struct HwFields {};
    struct SimFields
    {
        ros::ServiceServer sim_limit_switch_srv_;
        ros::ServiceServer sim_current_srv_;
        std::unique_ptr<hardware_interface::talonfxpro::TalonFXProSimCommandInterface> sim_command_interface_{std::make_unique<hardware_interface::talonfxpro::TalonFXProSimCommandInterface>()};
    };
    std::conditional_t<SIM, SimFields, HwFields> sim_fields_{};
};

#endif