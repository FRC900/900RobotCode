#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/multi_interface_controller.h>
#include <talon_controllers/talonfxpro_controller_interface.h> // "
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include "controllers_2024_msgs/Climb.h"
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>

namespace climb_controller
{

struct ClimbCommand
{
    ClimbCommand()
    {
        position = 0.0;
        velocity = 0.0;
        acceleration = 0.0;
        use_percent_output = true;
    }

    ClimbCommand(double position, double velocity, double acceleration, bool use_percent_output = false)
    {
        this->position = position;
        this->velocity = velocity;
        this->acceleration = acceleration;
        this->use_percent_output = use_percent_output;
    }
    double position;
    double velocity;
    double acceleration;
    bool use_percent_output; // if true, velocity is a percent output
    // double jerk_;
    // double snap_;
    // double crackle_;
    // double pop_;
};

class ClimbController : public controller_interface::MultiInterfaceController<hardware_interface::talonfxpro::TalonFXProCommandInterface>
{
    public:
        ClimbController()
        {
        }

        virtual bool init(hardware_interface::RobotHW *hw,
                          ros::NodeHandle             &root_nh,
                          ros::NodeHandle             &controller_nh) override;
        virtual void starting(const ros::Time &time) override;
        virtual void update(const ros::Time &time, const ros::Duration &period) override;
        virtual void stopping(const ros::Time &time) override;

        bool cmdService(controllers_2024_msgs::Climb::Request &req,
                        controllers_2024_msgs::Climb::Response &res);
        bool zeroService(std_srvs::Trigger::Request  &req,
                         std_srvs::Trigger::Response &res);

    private:
        talonfxpro_controllers::TalonFXProControllerInterface climb_joint_; // interface for the talon joint

        realtime_tools::RealtimeBuffer<ClimbCommand> command_buffer_; // this is the buffer for commands to be published
        ros::ServiceServer climb_service_; //service for receiving commands
        ros::ServiceServer climb_zeroing_service_; //service for zeroing

        ros::Publisher zeroed_publisher_;
        bool zeroed_;
        bool last_zeroed_;
        bool do_zero_ = true;

        double zeroing_percent_output_;
        double zeroing_timeout_;

        double current_threshold_;
        double current_iterations_ = 0;
        double max_current_iterations_;

        ros::Time last_time_down_;
};
}

// #define SENSE_CURRENT // comment out to disable current sensing

namespace climb_controller
{
bool ClimbController::init(hardware_interface::RobotHW *hw,
                           ros::NodeHandle             &/*root_nh*/,
                           ros::NodeHandle             &controller_nh)
{
    // create the interface used to initialize the talon joint
    hardware_interface::talonfxpro::TalonFXProCommandInterface *const talon_command_iface = hw->get<hardware_interface::talonfxpro::TalonFXProCommandInterface>();

    if (!controller_nh.getParam("zeroing_percent_output", zeroing_percent_output_))
    {
        ROS_ERROR("climb_controller : could not find zeroing_percent_output");
        return false;
    }

    if (!controller_nh.getParam("zeroing_timeout", zeroing_timeout_))
    {
        ROS_ERROR("climb_controller : could not find zeroing_timeout");
        return false;
    }

    if (!controller_nh.getParam("current_threshold", current_threshold_))
    {
        ROS_ERROR("dynamic_arm_controller : could not find current_threshold");
        return false;
    }

    if (!controller_nh.getParam("max_current_iterations", max_current_iterations_)) // 1 iteration = 10ms
    {
        ROS_ERROR("dynamic_arm_controller : could not find max_current_iterations");
        return false;
    }

    // get config values for the dynamic arm talons
    XmlRpc::XmlRpcValue climb_params;
    if (!controller_nh.getParam("joint", climb_params))
    {
        ROS_ERROR("climb_controller : could not find joint");
        return false;
    }

    //initialize the climb joint
    if (!climb_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, climb_params))
    {
        ROS_ERROR("climb_controller : could not initialize joint!");
        return false;
    }

    climb_service_ = controller_nh.advertiseService("command", &ClimbController::cmdService, this);
    climb_zeroing_service_ = controller_nh.advertiseService("zero", &ClimbController::zeroService, this);
    zeroed_publisher_ = controller_nh.advertise<std_msgs::Bool>("is_zeroed", 100);

    return true;
}

void ClimbController::starting(const ros::Time &time)
{
    do_zero_ = true;
    zeroed_ = false;
    last_zeroed_  = false;
    last_time_down_ = ros::Time::now();
    command_buffer_.writeFromNonRT(ClimbCommand());
    current_iterations_ = 0;
}

void ClimbController::update(const ros::Time &time, const ros::Duration &/*duration*/)
{
    bool current_is_too_high = false;
#ifdef SENSE_CURRENT
    if (climb_joint_.getTorqueCurrent() >= current_threshold_)
    {
        current_iterations_++;
        if (current_iterations_ >= max_current_iterations_)
        {
            ROS_WARN_STREAM_THROTTLE(0.5, "climb_controller : motor is above current limit. stopping.");
            current_is_too_high = true;
        }
    }
    else
    {
        current_iterations_ = 0;
    }
#endif
    ClimbCommand setpoint = *(command_buffer_.readFromRT());

    // If we hit the limit switch, (re)zero the position.
    if (climb_joint_.getReverseLimit() || current_is_too_high)
    {
        ROS_INFO_STREAM_THROTTLE(2, "climb_controller : " << (!current_is_too_high ? "hit bottom limit switch" : "current too high"));
        zeroed_ = true;
        if (!last_zeroed_)
        {
            zeroed_ = true;
            last_zeroed_ = true;
            climb_joint_.setRotorPosition(0);
            if (climb_joint_.getControlMode() == hardware_interface::talonfxpro::TalonMode::DutyCycleOut)
            {
                setpoint.velocity = 0.0; // stop motor
            }
            // climb_joint_.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
            // climb_joint_.setDemand1Value(config_.arb_feed_forward_up_low);
        }
    }
    else
    {
        last_zeroed_ = false;
    }

    std_msgs::Bool zeroed;
    zeroed.data = zeroed_;
    zeroed_publisher_.publish(zeroed);

    if (zeroed_) // run normally, seeking to various positions
    {

        if (setpoint.use_percent_output)
        {
            climb_joint_.setControlMode(hardware_interface::talonfxpro::TalonMode::DutyCycleOut);
            climb_joint_.setControlOutput(setpoint.velocity);
        }
        else
        {
            climb_joint_.setControlMode(hardware_interface::talonfxpro::TalonMode::MotionMagicVoltage);
            climb_joint_.setControlPosition(setpoint.position);

            climb_joint_.setMotionMagicCruiseVelocity(setpoint.velocity);
            climb_joint_.setMotionMagicAcceleration(setpoint.acceleration);
        }

    }
    else if (do_zero_)
    {
        climb_joint_.setControlMode(hardware_interface::talonfxpro::TalonMode::DutyCycleOut);
        if ((ros::Time::now() - last_time_down_).toSec() < zeroing_timeout_)
        {
            // Not yet zeroed. Run the climb down slowly until the limit switch is set.
            ROS_INFO_STREAM_THROTTLE(0.25, "climb_controller : zeroing with percent output: "
                                     << zeroing_percent_output_);
            climb_joint_.setControlOutput(zeroing_percent_output_);
        }
        else
        {
            // Stop moving to prevent motor from burning out
            ROS_INFO_STREAM_THROTTLE(0.25, "climb_controller : timed out");
            climb_joint_.setControlOutput(0);
        }

        // If not zeroed but enabled, check if the arm is moving down
        if ((climb_joint_.getControlMode() == hardware_interface::talonfxpro::TalonMode::Disabled) ||
                (climb_joint_.getVelocity() < 0))
        {
            // If moving down, or disabled and thus not expected to move down, reset the timer
            last_time_down_ = ros::Time::now();
        }
    }
}

void ClimbController::stopping(const ros::Time &/*time*/)
{
}

//Command Service Function
bool ClimbController::cmdService(controllers_2024_msgs::Climb::Request  &req,
                                 controllers_2024_msgs::Climb::Response &/*response*/)
{
    if (isRunning())
    {
        if (!zeroed_)
        {
            ROS_ERROR_STREAM("climb_controller : this command WILL NOT BE RUN until the arm is zeroed");
            ROS_INFO_STREAM("climb_controller : If you want to zero now, call the /frcrobot_jetson/climb_controller/zero service");
        }
        command_buffer_.writeFromNonRT(ClimbCommand(req.position, req.velocity, req.acceleration, req.use_percent_output));
    }
    else
    {
        ROS_ERROR_STREAM("climb_controller : can't accept new commands. climb controller is not running.");
        return false;
    }
    return true;
}

//Command Service Function
bool ClimbController::zeroService(std_srvs::Trigger::Request  &req,
                                  std_srvs::Trigger::Response &/*response*/)
{
    zeroed_ = false;
    last_zeroed_ = false;
    do_zero_ = true;
    std_msgs::Bool zeroed;
    zeroed.data = zeroed_;
    zeroed_publisher_.publish(zeroed);
    last_time_down_ = ros::Time::now();
    return true;
}

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(climb_controller::ClimbController, controller_interface::ControllerBase)
