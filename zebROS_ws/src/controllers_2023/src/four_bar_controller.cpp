#include <ros/ros.h>
#include <controller_interface/multi_interface_controller.h>
#include <talon_controllers/talon_controller_interface.h> // "
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include "controllers_2023_msgs/FourBarSrv.h"

#include "ddynamic_reconfigure/ddynamic_reconfigure.h"

namespace four_bar_controller_2023
{

//this is the actual controller, so it stores all of the  update() functions and the actual handle from the joint interface
//if it was only one type, controller_interface::Controller<TalonCommandInterface> here
class FourBarController_2023 : public controller_interface::MultiInterfaceController<hardware_interface::TalonCommandInterface>
{
    public:
        FourBarController_2023()
        {
        }

        //should this be hardware_interface::TalonCommandInterface instead? What's the reason to import RobotHW then get CommandInterface from that instead of just importing TalonCommandIface?
        //answer to my question: the TalonCommandInterface is passed in if it's not a multiInterfaceController, and just one kind of joint is made!
        virtual bool init(hardware_interface::RobotHW *hw,
                          ros::NodeHandle             &root_nh,
                          ros::NodeHandle             &controller_nh) override;
        virtual void starting(const ros::Time &time) override;
        virtual void update(const ros::Time &time, const ros::Duration &period) override;
        virtual void stopping(const ros::Time &time) override;


    private:
        ros::Time last_time_down_;
        talon_controllers::TalonControllerInterface four_bar_joint_; //interface for the talon joint

        std::atomic<double> position_command_; //this is the buffer for percent output commands to be published
        ros::ServiceServer four_bar_service_; //service for receiving commands

        bool zeroed_;
        bool last_zeroed_;

        std::atomic<double> max_angle_;
        std::atomic<double> min_angle_;

        std::atomic<double> arb_feed_forward_angle;
        std::atomic<double> straight_up_angle;
        std::atomic<double> four_bar_zeroing_percent_output;
        std::atomic<double> four_bar_zeroing_timeout;
        std::atomic<double> motion_magic_velocity;
        std::atomic<double> motion_magic_acceleration;
        std::atomic<int> motion_s_curve_strength;

        std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;

        bool cmdService(controllers_2023_msgs::FourBarSrv::Request &req,
                        controllers_2023_msgs::FourBarSrv::Response &res);

}; //class

template<typename T>
bool readIntoScalar(ros::NodeHandle &n, const std::string &name, std::atomic<T> &scalar)
{
    T val;
    if (n.getParam(name, val))
    {
        scalar = val;
        return true;
    }
    return false;
}

bool FourBarController_2023::init(hardware_interface::RobotHW *hw,
                                  ros::NodeHandle             &/*root_nh*/,
                                  ros::NodeHandle             &controller_nh)
{
    
    //create the interface used to initialize the talon joint
    hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();

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

    if (!readIntoScalar(controller_nh, "arb_feed_forward_angle", arb_feed_forward_angle))
    {
        ROS_ERROR("Could not find arb_feed_forward_angle");
        return false;
    }

    if (!readIntoScalar(controller_nh, "straight_up_angle", straight_up_angle))
    {
        ROS_ERROR("Could not find straight_up_angle");
        return false;
    }

    if (!readIntoScalar(controller_nh, "four_bar_zeroing_percent_output", four_bar_zeroing_percent_output))
    {
        ROS_ERROR("Could not find four_bar_zeroing_percent_output");
        return false;
    }

    if (!readIntoScalar(controller_nh, "four_bar_zeroing_timeout", four_bar_zeroing_timeout))
    {
        ROS_ERROR("Could not find four_bar_zeroing_timeout");
        return false;
    }

    if (!readIntoScalar(controller_nh, "motion_magic_velocity", motion_magic_velocity))
    {
        ROS_ERROR("Could not find motion_magic_velocity");
        return false;
    }

    if (!readIntoScalar(controller_nh, "motion_magic_acceleration", motion_magic_acceleration))
    {
        ROS_ERROR("Could not find motion_magic_acceleration");
        return false;
    }

    if (!readIntoScalar(controller_nh, "motion_s_curve_strength", motion_s_curve_strength))
    {
        ROS_ERROR("Could not find motion_s_curve_strength");
        return false;
    }

    //get config values for the four_bar talon
    XmlRpc::XmlRpcValue four_bar_params;
    if (!controller_nh.getParam("four_bar_joint", four_bar_params))
    {
        ROS_ERROR("Could not find four_bar_joint");
        return false;
    }

    //initialize the four_bar joint
    if (!four_bar_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, four_bar_params))
    {
        ROS_ERROR("Cannot initialize four_bar joint!");
    }
    bool dynamic_reconfigure = true;
    controller_nh.param("dynamic_reconfigure", dynamic_reconfigure, dynamic_reconfigure);

    if (dynamic_reconfigure)
    {
        ddr_ = std::make_unique<ddynamic_reconfigure::DDynamicReconfigure>(controller_nh);

        ddr_->registerVariable<double>
            ("max_angle",
            [this]() { return max_angle_.load(); },
            [this](double b) { max_angle_.store(b); },
            "Max angle",
            0.0, 6.28);

        ddr_->registerVariable<double>
            ("min_angle",
            [this]() { return min_angle_.load(); },
            [this](double b) { min_angle_.store(b); },
            "Min angle", 0.0, 3.14);

        ddr_->registerVariable<double>
            ("arb_feed_forward_angle",
            [this]() { return arb_feed_forward_angle.load(); },
            [this](double b) { arb_feed_forward_angle.store(b); },
            "Arb feedforward angle. calculation: cos(angle) * this",
             -1.0, 1.0);

        ddr_->registerVariable<double>
            ("straight_up_angle",
            [this]() { return straight_up_angle.load(); },
            [this](double b) { straight_up_angle.store(b); },
            "Angle which makes the four bar straight up",
             0.0, 3.14159265);

        ddr_->registerVariable<double>
            ("four_bar_zeroing_percent_output",
            [this]() { return four_bar_zeroing_percent_output.load(); },
            [this](double b) { four_bar_zeroing_percent_output.store(b); },
            "FourBar Zeroing Percent Output",
             -1.0, 1.0);

        ddr_->registerVariable<double>
            ("four_bar_zeroing_timeout",
            [this]() { return four_bar_zeroing_timeout.load(); },
            [this](double b) { four_bar_zeroing_timeout.store(b); },
            "FourBar Zeroing Timeout",
            0.0, 15.0);
        
        ddr_->registerVariable<double>
            ("motion_magic_velocity",
            [this]() { return motion_magic_velocity.load(); },
            [this](double b) { motion_magic_velocity.store(b); },
            "Motion Magic Velocity",
            0.0, 20.0);

        ddr_->registerVariable<double>
            ("motion_magic_acceleration",
            [this]() { return motion_magic_acceleration.load(); },
            [this](double b) { motion_magic_acceleration.store(b); },
            "Motion Magic Acceleration",
            0.0, 200.0);
        
        ddr_->registerVariable<int>
            ("motion_s_curve_strength",
            [this]() { return motion_s_curve_strength.load(); },
            [this](int b) { motion_s_curve_strength.store(b); },
            "S Curve Strength",
             0, 8);

        ddr_->publishServicesTopics();
    }

    four_bar_service_ = controller_nh.advertiseService("four_bar_service", &FourBarController_2023::cmdService, this);

    return true;
}

void FourBarController_2023::starting(const ros::Time &time)
{
    zeroed_ = false;
    last_zeroed_ = false;
    position_command_ = 0.0; // 0 is when we are fully retracted
}

void FourBarController_2023::update(const ros::Time &time, const ros::Duration &/*duration*/)
{
    // If we hit the limit switch, (re)zero the position.
    if (four_bar_joint_.getReverseLimitSwitch())
    {
        ROS_INFO_THROTTLE(2, "FourBarController_2023 : hit limit switch");
        if (!last_zeroed_)
        {
            zeroed_ = true;
            last_zeroed_ = true;
            four_bar_joint_.setSelectedSensorPosition(0); // relative to min position
            four_bar_joint_.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
            four_bar_joint_.setDemand1Value(sin(four_bar_joint_.getPosition() - straight_up_angle) * arb_feed_forward_angle);
        }
    }
    else
    {
        last_zeroed_ = false;
    }
    
    if (zeroed_) // run normally, seeking to various positions
    {
        four_bar_joint_.setMode(hardware_interface::TalonMode_MotionMagic);
        if (four_bar_joint_.getMode() == hardware_interface::TalonMode_Disabled)
        {
            position_command_ = four_bar_joint_.getPosition();
        }
        four_bar_joint_.setCommand(position_command_);

        //if we're not zeroing, add an arbitrary feed forward to hold the four_bar up
        four_bar_joint_.setMotionAcceleration(motion_magic_acceleration);
        four_bar_joint_.setMotionCruiseVelocity(motion_magic_velocity);
        four_bar_joint_.setPIDFSlot(0);

        four_bar_joint_.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
        four_bar_joint_.setDemand1Value(sin(four_bar_joint_.getPosition() - straight_up_angle) * arb_feed_forward_angle);
    }
    else
    {
        four_bar_joint_.setMode(hardware_interface::TalonMode_PercentOutput);
        if ((time - last_time_down_).toSec() < four_bar_zeroing_timeout)
        {
            // Not yet zeroed. Run the four_bar down slowly until the limit switch is set.
            ROS_INFO_STREAM_THROTTLE(0.25, "Zeroing four_bar with percent output: "
                                     << four_bar_zeroing_percent_output);
            four_bar_joint_.setCommand(four_bar_zeroing_percent_output);
        }
        //check stream
        else
        {
            // Stop moving to prevent motor from burning out
            ROS_INFO_STREAM_THROTTLE(1.00, "FourBar timed out");
            four_bar_joint_.setCommand(0);
        }

        // If not zeroed but enabled, check if the arm is moving down
        if ((four_bar_joint_.getMode() == hardware_interface::TalonMode_Disabled) ||
                (four_bar_joint_.getSpeed() < 0)) // TODO : param
        {
            // If moving down, or disabled and thus not expected to move down, reset the timer
            last_time_down_ = time;
        }
    }
}

void FourBarController_2023::stopping(const ros::Time &/*time*/)
{
}

bool FourBarController_2023::cmdService(controllers_2023_msgs::FourBarSrv::Request  &req,
                                        controllers_2023_msgs::FourBarSrv::Response &/*response*/)
{
    if (req.angle > max_angle_)
    {
        ROS_ERROR_STREAM("FourBar controller: requested angle too high : " << req.angle << ".");
        return false;
    }
    if (req.angle < min_angle_)
    {
        ROS_ERROR_STREAM("FourBar controller: requested angle too low : " << req.angle << ".");
        return false;
    }

    if (isRunning())
    {
        //adjust talon mode, arb feed forward, and PID slot appropriately
        position_command_ = req.angle;
        ROS_INFO_STREAM("writing " << std::to_string(req.angle) << " radians to four_bar_controller");
    }
    else
    {
        ROS_ERROR_STREAM("Can't accept new commands. FourBarController_2023 is not running.");
        return false;
    }
    return true;
}

}

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(four_bar_controller_2023::FourBarController_2023, controller_interface::ControllerBase)