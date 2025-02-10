#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <talon_controllers/talonfxpro_controller_interface.h>
#include <ctre_interfaces/talon_state_interface.h>
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include "controllers_2025_msgs/ElevatorSrv.h"

#include "ddynamic_reconfigure/ddynamic_reconfigure.h"
namespace elevator_controller_2025
{

template<typename T>
bool readIntoScalar(ros::NodeHandle &n, const std::string &name, std::atomic<T> &scalar){
	if (T val; n.getParam(name, val)){
        scalar = val;
        return true;
    }
    return false;
}

double readFloatParam(const XmlRpc::XmlRpcValue &param)
{
	if (!param.valid())
    {
        ROS_ERROR("2025_elevator_controller: readFloatParam : param was not a valid type");
		throw std::runtime_error("2025_elevator_controller: readFloatParam : param was not a valid type");
    }
	if (param.getType() == XmlRpc::XmlRpcValue::TypeDouble)
	{
		return static_cast<double>(param);
	}
	if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
	{
		return static_cast<int>(param);
	}
    ROS_ERROR("2025_elevator_controller: readFloatParam : A non-double value was read for param");
    throw std::runtime_error("2025_elevator_controller: readFloatParam : A non-double value was read for param");
}

//this is the actual controller, so it stores all of the  update() functions and the actual handle from the joint interface
class ElevatorController_2025 : public controller_interface::Controller<hardware_interface::talonfxpro::TalonFXProCommandInterface>
{
public:
    bool init(hardware_interface::talonfxpro::TalonFXProCommandInterface *talon_command_iface,
              ros::NodeHandle & /*root_nh*/,
              ros::NodeHandle &controller_nh) override
    {
        ROS_INFO_STREAM("2025_elevator_controller: init");

        // Key: min height to activate (e.g. 0 for stage 0), value: feed forward in volts
        // e.g. 
        /* - [0.0, 0.1]
           - [0.5, 0.2]
        for second stage at 0.5 meters and ff = 0.1 V for first stage, 0.2 V for second*/
        XmlRpc::XmlRpcValue feed_forward_map_xml_;
        if (!controller_nh.getParam("feed_forward_map", feed_forward_map_xml_))
        {
            ROS_WARN_STREAM("2025_elevator_controller : COULD NOT FIND FEED FORWARD MAP");
            return false;
        }

        for (int i = 0; i < feed_forward_map_xml_.size(); i++)
        {
            const auto s = feed_forward_map_xml_[i];
            feed_forward_map_.emplace_back(readFloatParam(s[0]), readFloatParam(s[1]));
            ROS_INFO_STREAM("2025_elevator_controller: Inserted " << s[0] << " " << s[1]);
        }

        // there has to be some cool function combinator to do first < first
        // sorts by highest to lowest height to activate (reverse order), so we can iterate through this until the first value is less than or equal to current pos
        std::sort(feed_forward_map_.begin(), feed_forward_map_.end(), [](const FeedForwardMapEntry &a, const FeedForwardMapEntry &b) { return a.position_ > b.position_; });

        if (!readIntoScalar(controller_nh, "elevator_zeroing_percent_output", elevator_zeroing_percent_output_))
        {
            ROS_ERROR("Could not find elevator_zeroing_percent_output");
            return false;
        }

        if (!readIntoScalar(controller_nh, "elevator_zeroing_timeout", elevator_zeroing_timeout_))
        {
            ROS_ERROR("Could not find elevator_zeroing_timeout");
            return false;
        }
    
        if (!readIntoScalar(controller_nh, "current_limit_for_zero", current_limit_for_zero_))
        {
            ROS_ERROR("Could not find current_limit_for_zero_");
            return false;
        }

        // get config values for the elevator talon
        XmlRpc::XmlRpcValue elevator_params;
        if (!controller_nh.getParam("elevator_joint", elevator_params))
        {
            ROS_ERROR("Could not find elevator_joint");
            return false;
        }

        if (!controller_nh.getParam("max_height_val", MAX_HEIGHT_VAL))
        {
            ROS_ERROR("Could not find max_height_val");
            return false;
        }

        if (!elevator_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, elevator_params))
        {
            ROS_ERROR("Cannot initialize elevator joint!");
            return false;
        }

        bool dynamic_reconfigure = true;
        controller_nh.param("dynamic_reconfigure", dynamic_reconfigure, dynamic_reconfigure);

        if (dynamic_reconfigure)
        {
            ddr_ = std::make_unique<ddynamic_reconfigure::DDynamicReconfigure>(controller_nh);

            ddr_->registerVariable<double>(
                "elevator_zeroing_percent_output",
                [this]()
                { return elevator_zeroing_percent_output_.load(); },
                [this](double b)
                { elevator_zeroing_percent_output_.store(b); },
                "Elevator Zeroing Percent Output",
                -0.2, 0.0);

            ddr_->registerVariable<double>(
                "current_limit_for_zero",
                [this]()
                { return current_limit_for_zero_.load(); },
                [this](double b)
                { current_limit_for_zero_.store(b); },
                "Current threshold for zeroing the elevator",
                50, 0.0);

            ddr_->registerVariable<double>(
                "elevator_zeroing_timeout",
                [this]()
                { return elevator_zeroing_timeout_.load(); },
                [this](double b)
                { elevator_zeroing_timeout_.store(b); },
                "Elevator Zeroing Timeout",
                0.0, 15.0);

            ddr_->publishServicesTopics();
        }

        elevator_service_ = controller_nh.advertiseService("elevator_service", &ElevatorController_2025::cmd_service, this);
        ROS_INFO_STREAM("2025_elevator_controller: init successful");
        return true;
    }

    void starting(const ros::Time &time) override
    {
        ROS_INFO_STREAM("2025_elevator_controller: starting");
        zeroed_ = false;
        last_zeroed_ = false;
        position_command_ = 0;
    }

    void update(const ros::Time &time, const ros::Duration & /*duration*/) override
    {
        // If we hit the limit switch, (re)zero the position.
        if (elevator_joint_.getStatorCurrent() > current_limit_for_zero_)
        {
            ROS_INFO_THROTTLE(2, "ElevatorController_2025 : hit current spike");
            if (!last_zeroed_)
            {
                zeroed_ = true;
                // last_zeroed_ = true;
                elevator_joint_.setRotorPosition(0);
                elevator_joint_.setControlFeedforward(find_feedforward());
            }
        }
        else
        {
            last_zeroed_ = false;
        }

        if (zeroed_) // run normally, seeking to various positions
        {
            elevator_joint_.setControlMode(hardware_interface::talonfxpro::TalonMode::MotionMagicExpoVoltage);
            if (elevator_joint_.getControlMode() == hardware_interface::talonfxpro::TalonMode::Disabled)
            {
                position_command_ = elevator_joint_.getPosition();
            }
            elevator_joint_.setControlOutput(0);
            elevator_joint_.setControlPosition(position_command_);
            elevator_joint_.setControlVelocity(0);
            elevator_joint_.setControlAcceleration(0);
            elevator_joint_.setControlSlot(0);
            
            // Set arbitrary feed forward based on height (as elevator is raised higher, it's lifting more stages --> more mass)
            elevator_joint_.setControlFeedforward(find_feedforward());
        }
        else
        {
            elevator_joint_.setControlMode(hardware_interface::talonfxpro::TalonMode::DutyCycleOut);
            if ((time - last_time_down_).toSec() < elevator_zeroing_timeout_)
            {
                // Not yet zeroed. Run the elevator down slowly until the limit switch is set.
                ROS_INFO_STREAM_THROTTLE(0.1, "Zeroing elevator with percent output: "
                                                   << elevator_zeroing_percent_output_);
                elevator_joint_.setCommand(elevator_zeroing_percent_output_);
            }
            else
            {
                // Stop moving to prevent motor from burning out
                ROS_INFO_STREAM_THROTTLE(1, "Elevator timed out");
                elevator_joint_.setCommand(0);
            }

            // If not zeroed but enabled, check if the arm is moving down
            if ((elevator_joint_.getControlMode() == hardware_interface::talonfxpro::TalonMode::Disabled) ||
                (elevator_joint_.getVelocity() < 0)) // TODO : param
            {
                // If moving down, or disabled and thus not expected to move down, reset the timer
                last_time_down_ = time;
            }
        }
    }

    void stopping(const ros::Time & /*time*/) override
    {
    }

private:
    // Command Service Function
    bool cmd_service(controllers_2025_msgs::ElevatorSrv::Request &req,
                    controllers_2025_msgs::ElevatorSrv::Response & /*response*/)
    {
        if (req.position > MAX_HEIGHT_VAL) // TODO : get real measurement, make a param
        {
            ROS_ERROR_STREAM("Elevator controller: req.position too large : " << req.position);
            return false;
        }
        if (this->isRunning())
        {
            // adjust talon mode, arb feed forward, and PID slot appropriately

            position_command_ = req.position;
            ROS_INFO_STREAM("writing " << std::to_string(req.position) << " to elevator controller");
        }
        else
        {
            ROS_ERROR_STREAM("Can't accept new commands. ElevatorController_2025 is not running.");
            return false;
        }
        return true;
    }

    // list MUST be sorted highest height to lowest height
    // (...binary search is faster, which is what C++'s upper_bound does, but whatever, we only have a few stages)
    double find_feedforward() {
        for (const auto &ff : feed_forward_map_) {
            if (ff.position_ <= elevator_joint_.getPosition()) {
                return ff.feed_forward_;
            }
        }
        ROS_ERROR("2025_elevator_controller: no feed forward low enough? returning zero");
        return 0.0;
    }

    ros::Time last_time_down_;

    talonfxpro_controllers::TalonFXProControllerInterface elevator_joint_;

    std::atomic<double> position_command_;
    ros::ServiceServer elevator_service_; // service for receiving commands

    bool zeroed_;
    bool last_zeroed_;

    double MAX_HEIGHT_VAL{1.2};

    struct FeedForwardMapEntry
    {
        double position_;
        double feed_forward_;
        FeedForwardMapEntry(const double position, const double feed_forward)
        : position_{position}
        , feed_forward_{feed_forward}
        {
        }
    };
    std::vector<FeedForwardMapEntry> feed_forward_map_;

    std::atomic<double> elevator_zeroing_percent_output_;
    std::atomic<double> elevator_zeroing_timeout_;
    std::atomic<double> current_limit_for_zero_;

    std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;

}; // class

}//namespace

PLUGINLIB_EXPORT_CLASS(elevator_controller_2025::ElevatorController_2025, controller_interface::ControllerBase)