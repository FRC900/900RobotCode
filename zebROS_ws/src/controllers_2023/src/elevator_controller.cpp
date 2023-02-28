#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/multi_interface_controller.h>
#include <talon_controllers/talon_controller_interface.h>
#include <ctre_interfaces/talon_state_interface.h>
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include "controllers_2023_msgs/ElevatorSrv.h"

#include "ddynamic_reconfigure/ddynamic_reconfigure.h"
double MAX_HEIGHT_VAL = 1.2;
namespace elevator_controller_2023
{

double getFourBarLength(double angle, double min_extension, double parallel_bar_length, double diagonal_bar_length, double intake_length)
{
    double minAngle = acos((min_extension - intake_length - parallel_bar_length) / diagonal_bar_length);
    // if below, angle = minAngle + acos((x - intake_length_ - parallel_bar_length_) / diagonal_bar_length_). if above, angle = minAngle - acos((x - intake_length_ - parallel_bar_length_) / diagonal_bar_length_).
    if (angle > minAngle) {
        return cos(angle - minAngle) * diagonal_bar_length + intake_length + parallel_bar_length;
    } else {
        // (minAngle - (minAngle - angle)) = minAngle - minAngle + angle = angle
        return cos(minAngle - angle) * diagonal_bar_length + intake_length + parallel_bar_length;
    }
}

//this is the actual controller, so it stores all of the  update() functions and the actual handle from the joint interface
//if it was only one type, controller_interface::Controller<TalonCommandInterface> here
class ElevatorController_2023 : public controller_interface::MultiInterfaceController<hardware_interface::TalonCommandInterface, hardware_interface::TalonStateInterface>

{
    public:
        ElevatorController_2023()
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

        bool cmdService(controllers_2023_msgs::ElevatorSrv::Request &req,
                        controllers_2023_msgs::ElevatorSrv::Response &res);

    private:
        ros::Time last_time_down_;
        talon_controllers::TalonControllerInterface elevator_joint_; //interface for the talon joint
        hardware_interface::TalonStateHandle fourbar_joint_; //interface for the fourbar joint, used to calculate feed forward

        std::atomic<double> position_command_;
        ros::ServiceServer elevator_service_; //service for receiving commands

        bool zeroed_;
        bool last_zeroed_;

        std::atomic<double> arb_feed_forward_high;
        std::atomic<double> arb_feed_forward_low;
        std::atomic<double> arb_feed_forward_angle;
        // feed forward calculation: low_or_high_ff + maximum - |sin(four bar angular position)|*ff_angle
        std::atomic<double> elevator_zeroing_percent_output;
        std::atomic<double> elevator_zeroing_timeout;
        std::atomic<double> stage_2_height;
        std::atomic<double> motion_magic_velocity_fast;
        std::atomic<double> motion_magic_acceleration_fast;
        std::atomic<int> motion_s_curve_strength;

        std::atomic<double> max_extension_;
        std::atomic<double> min_extension_;
        std::atomic<double> parallel_bar_length_;
        std::atomic<double> diagonal_bar_length_;
        std::atomic<double> intake_length_;

        std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;

}; //class

template<typename T>
bool readIntoScalar(ros::NodeHandle &n, const std::string &name, std::atomic<T> &scalar){
    T val;
	if (n.getParam(name, val)){
        scalar = val;
        return true;
    }
    return false;
}

bool ElevatorController_2023::init(hardware_interface::RobotHW *hw,
                                   ros::NodeHandle             &/*root_nh*/,
                                   ros::NodeHandle             &controller_nh)

{

    ROS_INFO_STREAM("INIT CALLED FOR ELEVATOR CONTROLLER============");

    //create the interface used to initialize the talon joint
    hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();
    hardware_interface::TalonStateInterface *const talon_state_iface = hw->get<hardware_interface::TalonStateInterface>();

    //hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>()

    if (!readIntoScalar(controller_nh, "four_bar_parallel_bar_length", parallel_bar_length_))
    {
        ROS_ERROR("Could not find four_bar_parallel_bar_length");
        return false;
    }

    if (!readIntoScalar(controller_nh, "four_bar_diagonal_bar_length", diagonal_bar_length_))
    {
        ROS_ERROR("Could not find four_bar_diagonal_bar_length");
        return false;
    }

    if (!readIntoScalar(controller_nh, "four_bar_intake_length", intake_length_))
    {
        ROS_ERROR("Could not find four_bar_intake_length");
        return false;
    }

    if (!readIntoScalar(controller_nh, "four_bar_min_extension", min_extension_))
    {
        ROS_WARN("Could not find four_bar_min_extension");
        return false;
    }

    if (!readIntoScalar(controller_nh, "four_bar_max_extension", max_extension_))
    {
        ROS_WARN("Could not find four_bar_max_extension");
        return false;
    }

    if (!readIntoScalar(controller_nh, "arb_feed_forward_low", arb_feed_forward_low))
    {
        ROS_ERROR("Could not find arb_feed_forward_low");
        return false;
    }

    if (!readIntoScalar(controller_nh, "arb_feed_forward_high", arb_feed_forward_high))
    {
        ROS_ERROR("Could not find arb_feed_forward_hgih");
        return false;
    }

    if (!readIntoScalar(controller_nh, "arb_feed_forward_angle", arb_feed_forward_angle))
    {
        ROS_ERROR("Could not find arb_feed_forward_angle");
        return false;
    }

    if (!readIntoScalar(controller_nh, "elevator_zeroing_percent_output", elevator_zeroing_percent_output))
    {
        ROS_ERROR("Could not find elevator_zeroing_percent_output");
        return false;
    }

    if (!readIntoScalar(controller_nh, "elevator_zeroing_timeout", elevator_zeroing_timeout))
    {
        ROS_ERROR("Could not find elevator_zeroing_timeout");
        return false;
    }

    if (!readIntoScalar(controller_nh, "stage_2_height", stage_2_height))
    {
        ROS_ERROR("Could not find stage_2_height");
        return false;
    }

    if (!readIntoScalar(controller_nh, "motion_magic_velocity", motion_magic_velocity_fast))
    {
        ROS_ERROR("Could not find motion_magic_velocity");
        return false;
    }

    if (!readIntoScalar(controller_nh, "motion_magic_acceleration", motion_magic_acceleration_fast))
    {
        ROS_ERROR("Could not find motion_magic_acceleration");
        return false;
    }

    if (!readIntoScalar(controller_nh, "motion_s_curve_strength", motion_s_curve_strength))
	{
		ROS_ERROR("Could not find motion_s_curve_strength");
		return false;
	}

    //get config values for the elevator talon
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

    std::string fourbar_name;
    if (!controller_nh.getParam("fourbar_joint", fourbar_name))
    {
        ROS_ERROR("Could not find fourbar_joint");
        return false;
    }

    fourbar_joint_ = talon_state_iface->getHandle(fourbar_name);

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

        ddr_->registerVariable<double>
            ("arb_feed_forward_high",
            [this]() { return arb_feed_forward_high.load(); },
            [this](double b) { arb_feed_forward_high.store(b); },
            "Arb feedforward high",
            0.0, 0.5);

        ddr_->registerVariable<double>
            ("max_extension",
            [this]() { return max_extension_.load(); },
            [this](double b) { max_extension_.store(b); },
            "Max extension",
            0.0, 1.0);

        ddr_->registerVariable<double>
            ("min_extension",
            [this]() { return min_extension_.load(); },
            [this](double b) { min_extension_.store(b); },
            "Min extension",
            0.0, 1.0);

        ddr_->registerVariable<double>
            ("parallel_bar_length",
            [this]() { return parallel_bar_length_.load(); },
            [this](double b) { parallel_bar_length_.store(b); },
            "Parallel bar length",
            0.0, 1.0);

        ddr_->registerVariable<double>
            ("diagonal_bar_length",
            [this]() { return diagonal_bar_length_.load(); },
            [this](double b) { diagonal_bar_length_.store(b); },
            "Diagonal bar length",
            0.0, 1.0);

        ddr_->registerVariable<double>
            ("intake_length",
            [this]() { return intake_length_.load(); },
            [this](double b) { intake_length_.store(b); },
            "Intake/static attachment length",
             0.0, 1.0);

        ddr_->registerVariable<double>
            ("arb_feed_forward_low",
            [this]() { return arb_feed_forward_low.load(); },
            [this](double b) { arb_feed_forward_low.store(b); },
            "Arb feedforward low",
             0.0, 1.0);

        ddr_->registerVariable<double>
            ("arb_feed_forward_angle",
            [this]() { return arb_feed_forward_angle.load(); },
            [this](double b) { arb_feed_forward_angle.store(b); },
            "Arb feedforward angle. calculation: arb_ff_low_or_high + ff_max - |sin(four bar angle)| * this",
             -1.0, 1.0);

        ddr_->registerVariable<double>
            ("elevator_zeroing_percent_output",
            [this]() { return elevator_zeroing_percent_output.load(); },
            [this](double b) { elevator_zeroing_percent_output.store(b); },
            "Elevator Zeroing Percent Output",
            -1.0, 0.0);
        
        ddr_->registerVariable<double>
            ("elevator_zeroing_timeout",
            [this]() { return elevator_zeroing_timeout.load(); },
            [this](double b) { elevator_zeroing_timeout.store(b); },
            "Elevator Zeroing Timeout",
            0.0, 15.0);
        
        ddr_->registerVariable<double>
            ("stage_2_height",
            [this]() { return stage_2_height.load(); },
            [this](double b) { stage_2_height.store(b); },
            "Stage 2 Height",
            0.0, 2.0);

        ddr_->registerVariable<double>
            ("motion_magic_velocity_fast",
            [this]() { return motion_magic_velocity_fast.load(); },
            [this](double b) { motion_magic_velocity_fast.store(b); },
            "fast Motion Magic Velocity",
            0.0, 10); 
        
        ddr_->registerVariable<double>
            ("motion_magic_acceleration_fast",
            [this]() { return motion_magic_acceleration_fast.load(); },
            [this](double b) { motion_magic_acceleration_fast.store(b); },
            "Fast Motion Magic Acceleration",
            0.0, 20.0);
        
        ddr_->registerVariable<int>
            ("motion_s_curve_strength",
            [this]() { return motion_s_curve_strength.load(); },
            [this](int b) { motion_s_curve_strength.store(b); },
            "S Curve Strength",
             0, 8);
    
        ddr_->publishServicesTopics();
    }

    elevator_service_ = controller_nh.advertiseService("elevator_service", &ElevatorController_2023::cmdService, this);
    ROS_INFO_STREAM("===========ELEVATOR INIT RETURNS TRUE================");
    return true;
}

void ElevatorController_2023::starting(const ros::Time &time)
{
    ROS_INFO_STREAM("CALLED STARTING ELEVATOR==============");
    zeroed_ = false;
    last_zeroed_ = false;
    position_command_ = 0;
}

void ElevatorController_2023::update(const ros::Time &time, const ros::Duration &/*duration*/)
{
    double fourbar_ff = ((getFourBarLength(fourbar_joint_->getPosition(), min_extension_, parallel_bar_length_, diagonal_bar_length_, intake_length_) - min_extension_) / (max_extension_ - min_extension_)) * arb_feed_forward_angle;
    // If we hit the limit switch, (re)zero the position.
    if (elevator_joint_.getReverseLimitSwitch())
    {
        ROS_INFO_THROTTLE(2, "ElevatorController_2023 : hit limit switch");
        if (!last_zeroed_)
        {
            zeroed_ = true;
            last_zeroed_ = true;
            elevator_joint_.setSelectedSensorPosition(0);
            elevator_joint_.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
            elevator_joint_.setDemand1Value(arb_feed_forward_low + fourbar_ff);
        }
    }
    else
    {
        last_zeroed_ = false;
    }

    if (zeroed_) // run normally, seeking to various positions
    {
        elevator_joint_.setMode(hardware_interface::TalonMode_MotionMagic);
        if (elevator_joint_.getMode() == hardware_interface::TalonMode_Disabled)
        {
            position_command_ = elevator_joint_.getPosition();
        }
        elevator_joint_.setCommand(position_command_);

        //if we're not climbing, add an arbitrary feed forward to hold the elevator up

        elevator_joint_.setMotionAcceleration(motion_magic_acceleration_fast);
        elevator_joint_.setMotionCruiseVelocity(motion_magic_velocity_fast);
        elevator_joint_.setPIDFSlot(0);
        // Add arbitrary feed forward for upwards motion
        // We could have arb ff for both up and down, but seems
        // easier (and good enough) to tune PID for down motion
		// and add an arb FF correction for u
        if (elevator_joint_.getPosition() >= stage_2_height)
        {
            elevator_joint_.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
            elevator_joint_.setDemand1Value(arb_feed_forward_high + fourbar_ff);
        }
        else
        {
            elevator_joint_.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
            elevator_joint_.setDemand1Value(arb_feed_forward_low + fourbar_ff);
        }
    }
    else
    {
        elevator_joint_.setMode(hardware_interface::TalonMode_PercentOutput);
        if ((time - last_time_down_).toSec() < elevator_zeroing_timeout)
        {
            // Not yet zeroed. Run the elevator down slowly until the limit switch is set.
            ROS_INFO_STREAM_THROTTLE(0.25, "Zeroing elevator with percent output: "
                                     << elevator_zeroing_percent_output);
            elevator_joint_.setCommand(elevator_zeroing_percent_output);
        }
        else
        {
            // Stop moving to prevent motor from burning out
            ROS_INFO_STREAM_THROTTLE(1, "Elevator timed out");
            elevator_joint_.setCommand(0);
        }

        // If not zeroed but enabled, check if the arm is moving down
        if ((elevator_joint_.getMode() == hardware_interface::TalonMode_Disabled) ||
                (elevator_joint_.getSpeed() < 0)) // TODO : param
        {
            // If moving down, or disabled and thus not expected to move down, reset the timer
            last_time_down_ = time;
        }
    }
}

void ElevatorController_2023::stopping(const ros::Time &/*time*/)
{
}

//Command Service Function
bool ElevatorController_2023::cmdService(controllers_2023_msgs::ElevatorSrv::Request  &req,
        controllers_2023_msgs::ElevatorSrv::Response &/*response*/)
{
    if (req.position > MAX_HEIGHT_VAL) // TODO : get real measurement, make a param
    {
        ROS_ERROR_STREAM("Elevator controller: req.position too large : " << req.position);
        return false;
    }
    if (isRunning())
    {
        //adjust talon mode, arb feed forward, and PID slot appropriately

        position_command_ = req.position;
        ROS_INFO_STREAM("writing " << std::to_string(req.position) << " to elevator controller");
    }
    else
    {
        ROS_ERROR_STREAM("Can't accept new commands. ElevatorController_2023 is not running.");
        return false;
    }
    return true;
}

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(elevator_controller_2023::ElevatorController_2023, controller_interface::ControllerBase)
