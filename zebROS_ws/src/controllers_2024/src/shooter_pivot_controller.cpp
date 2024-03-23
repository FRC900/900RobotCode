#include <ros/ros.h>
#include <controller_interface/multi_interface_controller.h>
#include <talon_controllers/talon_controller_interface.h> // "
#include <talon_controllers/talonfxpro_controller_interface.h> // "
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include "controllers_2024_msgs/ShooterPivotSrv.h"
#include "controllers_2024/interpolating_map.h"
#include "ddynamic_reconfigure/ddynamic_reconfigure.h"
#include <std_srvs/Empty.h>

namespace shooter_pivot_controller_2024
{
template<typename T>
static bool readIntoScalar(ros::NodeHandle &n, const std::string &name, std::atomic<T> &scalar)
{
    T val;
    if (n.getParam(name, val))
    {
        scalar = val;
        return true;
    }
    return false;
}

static double readFloatParam(const XmlRpc::XmlRpcValue &param)
{
	if (!param.valid())
    {
		throw std::runtime_error("2024_shooter_pivot_controller - readFloatParam : param was not a valid type");
    }
	if (param.getType() == XmlRpc::XmlRpcValue::TypeDouble)
	{
		return static_cast<double>(param);
	}
	if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
	{
		return static_cast<int>(param);
	}
    throw std::runtime_error("2024_shooter_pivot_controller - readFloatParam : A non-double value was read for param");
}

class ShooterPivotController_2024 : public controller_interface::MultiInterfaceController <hardware_interface::talonfxpro::TalonFXProCommandInterface>
{
    public:
        bool init(hardware_interface::RobotHW *hw,
                  ros::NodeHandle & /*root_nh*/,
                  ros::NodeHandle &controller_nh) override
        {

            // create the interface used to initialize the talon joint
            auto * const command_iface = hw->get<hardware_interface::talonfxpro::TalonFXProCommandInterface>();
            if (!readIntoScalar(controller_nh, "max_angle", max_angle_))
            {
                ROS_WARN("Could not find max_angle");
                return false;
            }

            if (!readIntoScalar(controller_nh, "min_angle", min_angle_))
            {
                ROS_WARN("Could not find min_angle");
                return false;
            }

            if (!readIntoScalar(controller_nh, "motion_magic_velocity", motion_magic_velocity_))
            {
                ROS_ERROR("Could not find motion_magic_velocity_");
                return false;
            }

            if (!readIntoScalar(controller_nh, "motion_magic_acceleration", motion_magic_acceleration_))
            {
                ROS_ERROR("Could not find motion_magic_acceleration_");
                return false;
            }

            if (!readIntoScalar(controller_nh, "motion_magic_jerk", motion_magic_jerk_))
            {
                ROS_ERROR("Could not find motion_magic_jerk_");
                return false;
            }

            XmlRpc::XmlRpcValue feed_forward_map_xml_;
            if (!controller_nh.getParam("feed_forward_map", feed_forward_map_xml_))
            {
                ROS_WARN_STREAM("2024_shooter_pivot_controller : COULD NOT FIND FEED FORWARD MAP");
                return false;
            }

            for (size_t i = 0; i < (unsigned)feed_forward_map_xml_.size(); i++)
            {
                auto s = feed_forward_map_xml_[i];
                angle_to_feed_forward_.insert(readFloatParam(s[0]), readFloatParam(s[1]));
                ROS_INFO_STREAM("2024_shooter_pivot_controller : Inserted " << s[0] << " " << s[1] << " " << s[2]);
            }

            if (controller_nh.hasParam("switch_control_slot")) {
                if (!(controller_nh.hasParam("slot_switchover_lower") && controller_nh.hasParam("slot_switchover_upper"))) {
                    ROS_WARN_STREAM("2024_shooter_pivot_controller: told to switch slots but not told threshold");
                } else {
                    if (!controller_nh.getParam("switch_control_slot", switch_control_slot_))
                    {
                        ROS_WARN_STREAM("2024_shooter_pivot_controller : could not find switch control slot even though it exists??");
                        return false;
                    }
                    if (!controller_nh.getParam("slot_switchover_lower", slot_switchover_lower_))
                    {
                        ROS_WARN_STREAM("2024_shooter_pivot_controller : could not find slot_switchover_lower even though it exists??");
                        return false;
                    }
                    if (!controller_nh.getParam("slot_switchover_upper", slot_switchover_upper_))
                    {
                        ROS_WARN_STREAM("2024_shooter_pivot_controller : could not find slot_switchover_upper even though it exists??");
                        return false;
                    }
                }
            }

            // get config values for the shooter_pivot talon
            XmlRpc::XmlRpcValue shooter_pivot_params;
            if (!controller_nh.getParam("shooter_pivot_joint", shooter_pivot_params))
            {
                ROS_ERROR("Could not find shooter_pivot_joint");
                return false;
            }

            // initialize the shooter_pivot joint
            if (!shooter_pivot_joint_.initWithNode(command_iface, nullptr, controller_nh, shooter_pivot_params))
            {
                ROS_ERROR("Cannot initialize shooter_pivot joint!");
                return false;
            }
            bool dynamic_reconfigure = true;
            controller_nh.param("dynamic_reconfigure", dynamic_reconfigure, dynamic_reconfigure);

            if (dynamic_reconfigure)
            {
                ddr_ = std::make_unique<ddynamic_reconfigure::DDynamicReconfigure>(controller_nh);

                ddr_->registerVariable<double>(
                    "max_angle",
                    [this]()
                    { return max_angle_.load(); },
                    [this](double b)
                    { max_angle_.store(b); },
                    "Max angle",
                    0.0, 6.28);

                ddr_->registerVariable<double>(
                    "min_angle",
                    [this]()
                    { return min_angle_.load(); },
                    [this](double b)
                    { min_angle_.store(b); },
                    "Min angle", 0.0, 3.14);

                ddr_->registerVariable<double>(
                    "motion_magic_velocity",
                    [this]()
                    { return motion_magic_velocity_.load(); },
                    [this](double b)
                    { motion_magic_velocity_.store(b); },
                    "Motion Magic Velocity",
                    0.0, 20.0);

                ddr_->registerVariable<double>(
                    "motion_magic_acceleration",
                    [this]()
                    { return motion_magic_acceleration_.load(); },
                    [this](double b)
                    { motion_magic_acceleration_.store(b); },
                    "Motion Magic Acceleration",
                    0.0, 200.0);

                ddr_->registerVariable<double>(
                    "motion_magic_jerk",
                    [this]()
                    { return motion_magic_jerk_.load(); },
                    [this](int b)
                    { motion_magic_jerk_.store(b); },
                    "Motion Magic Jerk",
                    0, 500);

                ddr_->publishServicesTopics();
            }

            shooter_pivot_service_ = controller_nh.advertiseService("shooter_pivot_service", &ShooterPivotController_2024::cmdService, this);

            return true;
        }

        void starting(const ros::Time &time) override
        {
            position_command_ = 0.0; // 0 is when we are fully retracted
        }

        void update(const ros::Time &time, const ros::Duration & /*duration*/) override
        {
            shooter_pivot_joint_.setControlMode(hardware_interface::talonfxpro::TalonMode::MotionMagicVoltage);
            if (shooter_pivot_joint_.getControlMode() == hardware_interface::talonfxpro::TalonMode::Disabled)
            {
                if (time_at_disable_ == ros::Time::MAX)
                {
                    time_at_disable_ = time;
                }
                else if ((time - time_at_disable_).toSec() > 0.5)
                {
                    position_command_ = shooter_pivot_joint_.getPosition();
                }
            }
            else
            {
                time_at_disable_ = ros::Time::MAX;
            }
            shooter_pivot_joint_.setControlPosition(position_command_);

            /*
            
            void setDemand1Type(const hardware_interface::DemandType demand_type) const;
            void setDemand1Value(const double demand_value);
            void setCommand(const double command);
            void setMotionCruiseVelocity(const double motion_cruise_velocity);
            void setMotionAcceleration(const double motion_acceleration);
            void setMotionSCurveStrength(const double motion_magic_jerk);
            void setPIDFSlot(const int slot);
            void setSelectedSensorPosition(const double sensor_position);
            void setNeutralMode(const hardware_interface::NeutralMode neutral_mode);

            */

            // if we're not zeroing, add an arbitrary feed forward to hold the shooter_pivot up
            shooter_pivot_joint_.setMotionMagicAcceleration(motion_magic_acceleration_);
            shooter_pivot_joint_.setMotionMagicCruiseVelocity(motion_magic_velocity_);
            shooter_pivot_joint_.setMotionMagicJerk(motion_magic_jerk_);
            if (switch_control_slot_) {
                if (shooter_pivot_joint_.getPosition() > slot_switchover_lower_ && shooter_pivot_joint_.getPosition() < slot_switchover_upper_) {
                    shooter_pivot_joint_.setControlSlot(1);
                } else {
                    shooter_pivot_joint_.setControlSlot(0);
                }
            } else {
                shooter_pivot_joint_.setControlSlot(0);
            }

            shooter_pivot_joint_.setControlFeedforward(angle_to_feed_forward_[shooter_pivot_joint_.getPosition()]);
        }

        void stopping(const ros::Time & /*time*/) override
        {
        }

    private:
        ros::Time last_time_down_;

        talonfxpro_controllers::TalonFXProControllerInterface shooter_pivot_joint_;

        std::atomic<double> position_command_; //this is the buffer for percent output commands to be published
        ros::ServiceServer shooter_pivot_service_; //service for receiving commands

        wpi::interpolating_map<double, double> angle_to_feed_forward_;

        std::atomic<double> max_angle_;
        std::atomic<double> min_angle_;

        std::atomic<double> motion_magic_velocity_;
        std::atomic<double> motion_magic_acceleration_;
        std::atomic<double> motion_magic_jerk_;

        std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;

        ros::Time time_at_disable_;

        bool switch_control_slot_ = false;
        double slot_switchover_lower_;
        double slot_switchover_upper_;

        bool cmdService(controllers_2024_msgs::ShooterPivotSrv::Request &req,
                        controllers_2024_msgs::ShooterPivotSrv::Response &response)
        {
            if (req.angle > max_angle_)
            {
                ROS_ERROR_STREAM("ShooterPivot controller: requested angle too high : " << req.angle << ".");
                return false;
            }
            if (req.angle < min_angle_)
            {
                ROS_ERROR_STREAM("ShooterPivot controller: requested angle too low : " << req.angle << ".");
                return false;
            }

            if (this->isRunning())
            {
                // adjust talon mode, arb feed forward, and PID slot appropriately
                position_command_ = req.angle;
                ROS_INFO_STREAM("writing " << std::to_string(req.angle) << " radians to shooter_pivot_controller");
            }
            else
            {
                ROS_ERROR_STREAM("Can't accept new commands. ShooterPivotController_2024 is not running.");
                return false;
            }
            return true;
        }

}; //class

} // namespace shooter_pivot_controller_2024

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(shooter_pivot_controller_2024::ShooterPivotController_2024, controller_interface::ControllerBase)
