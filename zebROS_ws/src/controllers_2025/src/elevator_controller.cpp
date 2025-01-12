#include <ros/ros.h>
#include <controller_interface/multi_interface_controller.h>
#include <talon_controllers/talon_controller_interface.h>
#include <talon_controllers/talonfxpro_controller_interface.h>
#include <ctre_interfaces/talon_state_interface.h>
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include "controllers_2025_msgs/ElevatorSrv.h"

#include "ddynamic_reconfigure/ddynamic_reconfigure.h"
namespace elevator_controller_2025
{

template<typename T>
bool readIntoScalar(ros::NodeHandle &n, const std::string &name, std::atomic<T> &scalar){
    T val;
	if (n.getParam(name, val)){
        scalar = val;
        return true;
    }
    return false;
}

//this is the actual controller, so it stores all of the  update() functions and the actual handle from the joint interface
//if it was only one type, controller_interface::Controller<TalonCommandInterface> here
template <typename COMMAND_INTERFACE_TYPE>
class ElevatorController_2025 : public controller_interface::MultiInterfaceController<COMMAND_INTERFACE_TYPE, typename std::conditional_t<std::is_same_v<COMMAND_INTERFACE_TYPE, hardware_interface::TalonCommandInterface>, hardware_interface::TalonStateInterface, hardware_interface::talonfxpro::TalonFXProStateInterface>>
{
public:
    bool init(hardware_interface::RobotHW *hw,
              ros::NodeHandle & /*root_nh*/,
              ros::NodeHandle &controller_nh) override
    {
        ROS_INFO_STREAM("INIT CALLED FOR ELEVATOR CONTROLLER============");

        // create the interface used to initialize the talon joint
        auto *const talon_command_iface = hw->get<COMMAND_INTERFACE_TYPE>();
        auto *const talon_state_iface = hw->get<typename std::conditional_t<std::is_same_v<COMMAND_INTERFACE_TYPE, hardware_interface::TalonCommandInterface>, hardware_interface::TalonStateInterface, hardware_interface::talonfxpro::TalonFXProStateInterface>>();

        // hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>()


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
                "arb_feed_forward_high",
                [this]()
                { return arb_feed_forward_high.load(); },
                [this](double b)
                { arb_feed_forward_high.store(b); },
                "Arb feedforward high",
                0.0, 0.5);

            ddr_->registerVariable<double>(
                "max_extension",
                [this]()
                { return max_extension_.load(); },
                [this](double b)
                { max_extension_.store(b); },
                "Max extension",
                0.0, 1.0);

            ddr_->registerVariable<double>(
                "min_extension",
                [this]()
                { return min_extension_.load(); },
                [this](double b)
                { min_extension_.store(b); },
                "Min extension",
                0.0, 1.0);

            ddr_->registerVariable<double>(
                "arb_feed_forward_low",
                [this]()
                { return arb_feed_forward_low.load(); },
                [this](double b)
                { arb_feed_forward_low.store(b); },
                "Arb feedforward low",
                0.0, 1.0);

            ddr_->registerVariable<double>(
                "arb_feed_forward_angle",
                [this]()
                { return arb_feed_forward_angle.load(); },
                [this](double b)
                { arb_feed_forward_angle.store(b); },
                "Arb feedforward angle. calculation: arb_ff_low_or_high + ff_max - |sin(four bar angle)| * this",
                -1.0, 1.0);

            ddr_->registerVariable<double>(
                "elevator_zeroing_percent_output",
                [this]()
                { return elevator_zeroing_percent_output.load(); },
                [this](double b)
                { elevator_zeroing_percent_output.store(b); },
                "Elevator Zeroing Percent Output",
                -1.0, 0.0);

            ddr_->registerVariable<double>(
                "elevator_zeroing_timeout",
                [this]()
                { return elevator_zeroing_timeout.load(); },
                [this](double b)
                { elevator_zeroing_timeout.store(b); },
                "Elevator Zeroing Timeout",
                0.0, 15.0);

            ddr_->registerVariable<double>(
                "stage_2_height",
                [this]()
                { return stage_2_height.load(); },
                [this](double b)
                { stage_2_height.store(b); },
                "Stage 2 Height",
                0.0, 2.0);

            ddr_->registerVariable<double>(
                "motion_magic_velocity_fast",
                [this]()
                { return motion_magic_velocity_fast.load(); },
                [this](double b)
                { motion_magic_velocity_fast.store(b); },
                "fast Motion Magic Velocity",
                0.0, 10);

            ddr_->registerVariable<double>(
                "motion_magic_acceleration_fast",
                [this]()
                { return motion_magic_acceleration_fast.load(); },
                [this](double b)
                { motion_magic_acceleration_fast.store(b); },
                "Fast Motion Magic Acceleration",
                0.0, 50.0);

            ddr_->registerVariable<double>(
                "motion_s_curve_strength",
                [this]()
                { return motion_s_curve_strength.load(); },
                [this](int b)
                { motion_s_curve_strength.store(b); },
                "S Curve Strength",
                0, 500);

            ddr_->publishServicesTopics();
        }

        elevator_service_ = controller_nh.advertiseService("elevator_service", &ElevatorController_2025::cmdService, this);
        ROS_INFO_STREAM("===========ELEVATOR INIT RETURNS TRUE================");
        return true;
    }

    void starting(const ros::Time &time) override
    {
        ROS_INFO_STREAM("CALLED STARTING ELEVATOR==============");
        zeroed_ = false;
        last_zeroed_ = false;
        position_command_ = 0;
    }

    void update(const ros::Time &time, const ros::Duration & /*duration*/) override
    {
        // If we hit the limit switch, (re)zero the position.
        if (elevator_joint_.getReverseLimitSwitch())
        {
            ROS_INFO_THROTTLE(2, "ElevatorController_2025 : hit limit switch");
            if (!last_zeroed_)
            {
                zeroed_ = true;
                // last_zeroed_ = true;
                elevator_joint_.setSelectedSensorPosition(0);
                elevator_joint_.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
                elevator_joint_.setDemand1Value(arb_feed_forward_low);
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

            // if we're not climbing, add an arbitrary feed forward to hold the elevator up

            elevator_joint_.setMotionAcceleration(motion_magic_acceleration_fast);
            elevator_joint_.setMotionCruiseVelocity(motion_magic_velocity_fast);
            elevator_joint_.setMotionSCurveStrength(motion_s_curve_strength);
            elevator_joint_.setPIDFSlot(0);
            // Add arbitrary feed forward for upwards motion
            // We could have arb ff for both up and down, but seems
            // easier (and good enough) to tune PID for down motion
            // and add an arb FF correction for u
            if (elevator_joint_.getPosition() >= stage_2_height)
            {
                elevator_joint_.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
                elevator_joint_.setDemand1Value(arb_feed_forward_high);
            }
            else
            {
                elevator_joint_.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
                elevator_joint_.setDemand1Value(arb_feed_forward_low);
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

    void stopping(const ros::Time & /*time*/) override
    {
    }

private:
    // Command Service Function
    bool cmdService(controllers_2025_msgs::ElevatorSrv::Request &req,
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
    ros::Time last_time_down_;

    // This generates (at compile time) the typename for the joint interface depending on the COMMAND_INTERFACE_TYPE defined for this particular joint.
    // For the TalonCommandInterface, the joint type is TalonControllerInterface.
    // For TalonFXProCommandInterface, use the corresponding TalonFXProControllerInterface instead
    typename std::conditional_t<std::is_same_v<COMMAND_INTERFACE_TYPE, hardware_interface::TalonCommandInterface>, talon_controllers::TalonControllerInterface, talonfxpro_controllers::TalonFXProControllerInterface> elevator_joint_;

    std::atomic<double> position_command_;
    ros::ServiceServer elevator_service_; // service for receiving commands

    bool zeroed_;
    bool last_zeroed_;

    double MAX_HEIGHT_VAL{1.2};

    std::atomic<double> arb_feed_forward_high;
    std::atomic<double> arb_feed_forward_low;
    std::atomic<double> arb_feed_forward_angle;
    // feed forward calculation: low_or_high_ff + maximum - |sin(four bar angular position)|*ff_angle
    std::atomic<double> elevator_zeroing_percent_output;
    std::atomic<double> elevator_zeroing_timeout;
    std::atomic<double> stage_2_height;
    std::atomic<double> motion_magic_velocity_fast;
    std::atomic<double> motion_magic_acceleration_fast;
    std::atomic<double> motion_s_curve_strength;

    std::atomic<double> max_extension_;
    std::atomic<double> min_extension_;

    std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;

}; // class

}//namespace

// Generate the code for each type of the templated controller
// should be phoneix5 version
//template class elevator_controller_2025::ElevatorController_2025<hardware_interface::TalonCommandInterface>;
template class elevator_controller_2025::ElevatorController_2025<hardware_interface::talonfxpro::TalonFXProCommandInterface>;
//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
// should be phoneix5 version 
//PLUGINLIB_EXPORT_CLASS(elevator_controller_2025::ElevatorController_2025<hardware_interface::TalonCommandInterface>, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(elevator_controller_2025::ElevatorController_2025<hardware_interface::talonfxpro::TalonFXProCommandInterface>, controller_interface::ControllerBase)