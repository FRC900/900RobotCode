#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <talon_controllers/talonfxpro_controller_interface.h> // "
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include "controllers_2024_msgs/ShooterPivotSrv.h"
#include "controllers_2024/interpolating_map.h"
#include "ddynamic_reconfigure/ddynamic_reconfigure.h"

namespace shooter_pivot_controller_2024
{
template<typename T>
static bool readIntoScalar(ros::NodeHandle &n, const std::string &name, std::atomic<T> &scalar)
{
    if (T val; n.getParam(name, val))
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

class ShooterPivotController_2024 : public controller_interface::Controller<hardware_interface::talonfxpro::TalonFXProCommandInterface>
{
    public:
        bool init(hardware_interface::talonfxpro::TalonFXProCommandInterface *hw,
                  ros::NodeHandle & /*root_nh*/,
                  ros::NodeHandle &controller_nh) override
        {
            // create the interface used to initialize the talon joint
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

            if (!readIntoScalar(controller_nh, "ki", ki_))
            {
                ROS_ERROR("Could not find ki_");
                return false;
            }

            if (!readIntoScalar(controller_nh, "izone", izone_))
            {
                ROS_ERROR("Could not find izone_");
                return false;
            }

            XmlRpc::XmlRpcValue feed_forward_map_xml_;
            if (!controller_nh.getParam("feed_forward_map", feed_forward_map_xml_))
            {
                ROS_WARN_STREAM("2024_shooter_pivot_controller : COULD NOT FIND FEED FORWARD MAP");
                return false;
            }

            for (int i = 0; i < feed_forward_map_xml_.size(); i++)
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
            if (!shooter_pivot_joint_.initWithNode(hw, nullptr, controller_nh, shooter_pivot_params))
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
                    [this](const double d)
                    { max_angle_.store(d); },
                    "Max angle",
                    0.0, 6.28);

                ddr_->registerVariable<double>(
                    "min_angle",
                    [this]()
                    { return min_angle_.load(); },
                    [this](const double d)
                    { min_angle_.store(d); },
                    "Min angle", 0.0, 3.14);

                ddr_->registerVariable<double>(
                    "motion_magic_velocity",
                    [this]()
                    { return motion_magic_velocity_.load(); },
                    [this](const double d)
                    { motion_magic_velocity_.store(d); },
                    "Motion Magic Velocity",
                    0.0, 20.0);

                ddr_->registerVariable<double>(
                    "motion_magic_acceleration",
                    [this]()
                    { return motion_magic_acceleration_.load(); },
                    [this](const double d)
                    { motion_magic_acceleration_.store(d); },
                    "Motion Magic Acceleration",
                    0.0, 200.0);

                ddr_->registerVariable<double>(
                    "motion_magic_jerk",
                    [this]()
                    { return motion_magic_jerk_.load(); },
                    [this](const double d)
                    { motion_magic_jerk_.store(d); },
                    "Motion Magic Jerk",
                    0, 5000.0);

                ddr_->registerVariable<double>(
                    "kI",
                    [this]()
                    { return ki_.load(); },
                    [this](const double d)
                    { ki_.store(d); },
                    "kI",
                    0, 5.);

                ddr_->registerVariable<double>(
                    "IZone",
                    [this]()
                    { return izone_.load(); },
                    [this](const double d)
                    { izone_.store(d); },
                    "IZone",
                    0, 6.28);

                // ddr_->registerVariable<double>(
                //     "IZoneMin",
                //     [this]()
                //     { return izone_min_.load(); },
                //     [this](const double d)
                //     { izone_min_.store(d); },
                //     "IZoneMin",
                //     0, 6.28);

                // ddr_->registerVariable<double>(
                //     "Stopped FF Multiplier",
                //     [this]()
                //     { return stopped_ff_multiplier_.load(); },
                //     [this](const double d)
                //     { stopped_ff_multiplier_.store(d); },
                //     "Stopped FF Multiplier",
                //     0, 1);

                ddr_->publishServicesTopics();
            }

            shooter_pivot_service_ = controller_nh.advertiseService("shooter_pivot_service", &ShooterPivotController_2024::cmdService, this);

            return true;
        }

        void starting(const ros::Time &time) override
        {
            position_command_ = min_angle_.load(); // Default to all the way down
            iaccum_ = 0.0;
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
                iaccum_ = 0.0;
            }
            else
            {
                time_at_disable_ = ros::Time::MAX;
                const double error = position_command_ - shooter_pivot_joint_.getPosition();
                const double abs_error = fabs(error);
                if (abs_error <= 0.005) {
                    iaccum_ = 0.0;
                } else if (abs_error <= izone_) { // Accumulate error if we're
                    iaccum_ += error;      // near the setpoint
                } else {
                    iaccum_ = 0.0;
                }
            }
            shooter_pivot_joint_.setControlPosition(position_command_);

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

            // add an arbitrary feed forward to hold the shooter_pivot up against gravity
            // Also add in the integral term
            shooter_pivot_joint_.setControlFeedforward(angle_to_feed_forward_[shooter_pivot_joint_.getPosition()] + ki_ * iaccum_);
        }

    private:
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

        ros::Time time_at_disable_{};

        bool switch_control_slot_ = false;
        double slot_switchover_lower_;
        double slot_switchover_upper_;

        double iaccum_{0};
        std::atomic<double> izone_;
        std::atomic<double> izone_min_{0.01};
        std::atomic<double> ki_;

        std::atomic<double> stopped_ff_multiplier_{0.75};

        bool cmdService(controllers_2024_msgs::ShooterPivotSrv::Request &req,
                        controllers_2024_msgs::ShooterPivotSrv::Response &/*response*/)
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
