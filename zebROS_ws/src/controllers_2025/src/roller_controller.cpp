#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <talon_controllers/talonfxpro_controller_interface.h>
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include "controllers_2025_msgs/RollerSrv.h"

#include "ddynamic_reconfigure/ddynamic_reconfigure.h"
namespace roller_controller_2025
{

template <typename T>
bool readIntoScalar(ros::NodeHandle &n, const std::string &name, std::atomic<T> &scalar)
{
    if (T val; n.getParam(name, val))
    {
        scalar = val;
        return true;
    }
    return false;
}

//this is the actual controller, so it stores all of the update() functions and the actual handle from the joint interface
class RollerController_2025 : public controller_interface::Controller<hardware_interface::talonfxpro::TalonFXProCommandInterface>
{
public:
    bool init(hardware_interface::talonfxpro::TalonFXProCommandInterface *talon_command_iface,
              ros::NodeHandle & /*root_nh*/,
              ros::NodeHandle &controller_nh) override
    {
        ROS_INFO_STREAM("2025_roller_controller: init");

        if (!readIntoScalar(controller_nh, "roller_stopped_position", roller_stopped_position_))
        {
            ROS_WARN("2025_roller_controller: Could not find roller_stopped_position");
        }

        // get config values for the roller talon
        XmlRpc::XmlRpcValue roller_params;
        if (!controller_nh.getParam("roller_joint", roller_params))
        {
            ROS_ERROR("Could not find roller_joint");
            return false;
        }

        if (!roller_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, roller_params))
        {
            ROS_ERROR("Cannot initialize roller_joint!");
            return false;
        }

        bool dynamic_reconfigure = true;
        controller_nh.param("dynamic_reconfigure", dynamic_reconfigure, dynamic_reconfigure);

        if (dynamic_reconfigure)
        {
            ddr_ = std::make_unique<ddynamic_reconfigure::DDynamicReconfigure>(controller_nh);

            ddr_->registerVariable<double>("roller_stopped_position",
                                           [this]() { return roller_stopped_position_.load(); },
                                           [this](double d) { roller_stopped_position_.store(d); },
                                           "Position to hold when roller is stopped",
                                           -100., 100.);

            ddr_->publishServicesTopics();
        }

        roller_service_ = controller_nh.advertiseService("roller_service", &RollerController_2025::cmd_service, this);
        ROS_INFO_STREAM("2025_roller_controller: init successful");
        return true;
    }

    void starting(const ros::Time &time) override
    {
        // Force the roller to re-zero and hold position on the first update call
        roller_joint_.setControlMode(hardware_interface::talonfxpro::TalonMode::VoltageOut);
        voltage_command_ = 0;
        ROS_INFO_STREAM("2025_roller_controller: starting");
    }

    void update(const ros::Time &time, const ros::Duration & /*duration*/) override
    {
        if (voltage_command_ == 0.)
        {
            // When switching from running to stopped, set the current position
            // as the new zero, then command the motor to hold that position.
            if (roller_joint_.getControlMode() != hardware_interface::talonfxpro::TalonMode::PositionVoltage)
            {
                roller_joint_.setRotorPosition(0);
            }
            roller_joint_.setControlMode(hardware_interface::talonfxpro::TalonMode::PositionVoltage);
            roller_joint_.setControlPosition(0);
        }
        else
        {
            // Otherwise, pass through the voltage command.
            roller_joint_.setControlMode(hardware_interface::talonfxpro::TalonMode::VoltageOut);
            roller_joint_.setControlOutput(voltage_command_);
        }
    }

    void stopping(const ros::Time & /*time*/) override
    {
    }

private:
    // Command Service Function
    bool cmd_service(controllers_2025_msgs::RollerSrv::Request &req,
                     controllers_2025_msgs::RollerSrv::Response & /*response*/)
    {
        if (this->isRunning())
        {
            voltage_command_ = req.voltage;
            ROS_INFO_STREAM("writing " << std::to_string(req.voltage) << " to roller controller");
        }
        else
        {
            ROS_ERROR_STREAM("Can't accept new commands. RollerController_2025 is not running.");
            return false;
        }
        return true;
    }

    talonfxpro_controllers::TalonFXProControllerInterface roller_joint_;

    std::atomic<double> voltage_command_;
    ros::ServiceServer roller_service_; // service for receiving commands

    std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;
    std::atomic<double> roller_stopped_position_{0.0};

}; // class

}//namespace

PLUGINLIB_EXPORT_CLASS(roller_controller_2025::RollerController_2025, controller_interface::ControllerBase)