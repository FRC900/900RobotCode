//HPP CONTENTS INTO CPP
//added _2023 to the two lines above ^

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/multi_interface_controller.h>
#include <talon_controllers/talon_controller_interface.h> // "
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include "controllers_2023_msgs/FourBarSrv.h"
#include "controllers_2023_msgs/FourBarState.h"
#include <realtime_tools/realtime_publisher.h>

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
        std::unique_ptr<realtime_tools::RealtimePublisher<controllers_2023_msgs::FourBarState>> realtime_pub_;

        bool zeroed_;
        bool last_zeroed_;
        double last_angle_;
        //double last_setpoint_;
        hardware_interface::TalonMode last_mode_;

        std::atomic<double> max_extension_;
        std::atomic<double> min_extension_;
        std::atomic<double> parallel_bar_length_;
        std::atomic<double> diagonal_bar_length_;
        std::atomic<double> intake_length_;

        std::atomic<double> arb_feed_forward_maximum;
        std::atomic<double> arb_feed_forward_angle;
        // feed forward calculation: maximum - |sin(angular position)|*ff_angle
        std::atomic<double> four_bar_zeroing_percent_output;
        std::atomic<double> four_bar_zeroing_timeout;
        std::atomic<double> motion_magic_velocity;
        std::atomic<double> motion_magic_acceleration;
        std::atomic<int> motion_s_curve_strength;

        ddynamic_reconfigure::DDynamicReconfigure ddr_;

        bool cmdService(controllers_2023_msgs::FourBarSrv::Request &req,
                        controllers_2023_msgs::FourBarSrv::Response &res);

        double angleFromX(double x, bool below) const
        {
            // motor reads clockwise as positive, but angles are counterclockwise.
            // so, this angle needs to be negative.
            double xAngle;
            if ((x - intake_length_ - parallel_bar_length_) >= diagonal_bar_length_)
            {
                xAngle = 0;
                ROS_WARN_STREAM("Desired x was >= maximum x, setting to maximum angle");
            }
            else
            {
                xAngle = acos((x - intake_length_ - parallel_bar_length_) / diagonal_bar_length_);
            }

            double angle = acos((min_extension_ - intake_length_ - parallel_bar_length_) / diagonal_bar_length_) - xAngle;
            return below ? acos((min_extension_ - intake_length_ - parallel_bar_length_) / diagonal_bar_length_) + xAngle : angle;
        }

        controllers_2023_msgs::FourBarSrv::Request stateFromAngle(double angle) const
        {
            double minAngle = acos((min_extension_ - intake_length_ - parallel_bar_length_) / diagonal_bar_length_);
            // if below, angle = minAngle + acos((x - intake_length_ - parallel_bar_length_) / diagonal_bar_length_). if above, angle = minAngle - acos((x - intake_length_ - parallel_bar_length_) / diagonal_bar_length_).
            if (angle > minAngle) {
                controllers_2023_msgs::FourBarSrv::Request toReturn;
                toReturn.position = cos(angle - minAngle) * diagonal_bar_length_ + intake_length_ + parallel_bar_length_;
                toReturn.below = true;
                return toReturn;
            } else {
                // (minAngle - (minAngle - angle)) = minAngle - minAngle + angle = angle
                controllers_2023_msgs::FourBarSrv::Request toReturn;
                toReturn.position = cos(minAngle - angle) * diagonal_bar_length_ + intake_length_ + parallel_bar_length_;
                toReturn.below = false;
                return toReturn;
            }
        }

        void stateMsg(double current_angle, double set_angle, controllers_2023_msgs::FourBarState &state) {
            // float64 set_position # x distance outward (negative is backwards and may be invalid)
            // bool set_below # if four bar can flip down, this being set to true makes it do that
            // float64 current_position # x distance outward (negative is backwards and may be invalid)
            // bool current_below # if four bar can flip down, this being set to true makes it do that
            auto current_state = stateFromAngle(current_angle);
            auto set_state = stateFromAngle(set_angle);
            state.set_position = set_state.position;
            state.set_below = set_state.below;
            state.current_position = current_state.position;
            state.current_below = current_state.below;
        }
}; //class

// Set the conversion_factor so that 1 rad = 1 turn of the 4bar

//END OF HPP CONTENTS
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

    if (!readIntoScalar(controller_nh, "parallel_bar_length", parallel_bar_length_))
    {
        ROS_ERROR("Could not find parallel_bar_length");
        return false;
    }

    if (!readIntoScalar(controller_nh, "diagonal_bar_length", diagonal_bar_length_))
    {
        ROS_ERROR("Could not find diagonal_bar_length");
        return false;
    }

    if (!readIntoScalar(controller_nh, "intake_length", intake_length_))
    {
        ROS_ERROR("Could not find intake_length");
        return false;
    }

    // set defaults
    double math_max_extension_ = parallel_bar_length_ + diagonal_bar_length_ + intake_length_;
    double math_min_extension_ = -diagonal_bar_length_ + parallel_bar_length_ + intake_length_;

    ROS_INFO_STREAM("math. max: " << math_max_extension_ << ", " << "min: " << math_min_extension_);

    if (!readIntoScalar(controller_nh, "max_extension", max_extension_))
    {
        ROS_WARN("Could not find max_extension, using default");
    }

    if (!readIntoScalar(controller_nh, "min_extension", min_extension_))
    {
        ROS_WARN("Could not find min_extension, using default");
    }

    if (max_extension_ > math_max_extension_)
    {
        ROS_WARN_STREAM("max_extension_ > math_max_extension_, setting to math_max_extension_: " << math_max_extension_);
        max_extension_ = math_max_extension_;
        // if we set max_extension_ higher than math_max_extension_, we will get NaN because that physically won't work
    }
    if (min_extension_ < math_min_extension_)
    {
        ROS_WARN_STREAM("min_extension_ < math_min_extension_, setting to math_max_extension_: " << math_min_extension_);
        min_extension_ = math_min_extension_;
    }

    if (!readIntoScalar(controller_nh, "arb_feed_forward_maximum", arb_feed_forward_maximum))
    {
        ROS_ERROR("Could not find arb_feed_forward_maximum");
        return false;
    }

    if (!readIntoScalar(controller_nh, "arb_feed_forward_angle", arb_feed_forward_angle))
    {
        ROS_ERROR("Could not find arb_feed_forward_angle");
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

    ddr_.registerVariable<double>
    ("max_extension",
     [this]()
    {
        return max_extension_.load();
    },
    [this, math_max_extension_](double b)
    {
        if (b > math_max_extension_)
        {
            max_extension_.store(b);
        }
        else
        {
            ROS_WARN_STREAM("ddr: max_extension_ set to greater than the theoretical maximum. setting to calculated maximum instead.");
        }
    },
    "Max extension");

    ddr_.registerVariable<double>
    ("min_extension",
     [this]()
    {
        return min_extension_.load();
    },
    [this, math_min_extension_](double b)
    {
        if (b < math_min_extension_)
        {
            min_extension_.store(b);
        }
        else
        {
            ROS_WARN_STREAM("ddr: min_extension_ set to less than the theoretical minimum. setting to calculated minimum instead.");
        }
    },
    "Min extension");

    ddr_.registerVariable<double>
    ("parallel_bar_length",
     [this]()
    {
        return parallel_bar_length_.load();
    },
    [this](double b)
    {
        parallel_bar_length_.store(b);
    },
    "Parallel bar length");

    ddr_.registerVariable<double>
    ("diagonal_bar_length",
     [this]()
    {
        return diagonal_bar_length_.load();
    },
    [this](double b)
    {
        diagonal_bar_length_.store(b);
    },
    "Diagonal bar length");

    ddr_.registerVariable<double>
    ("intake_length",
     [this]()
    {
        return intake_length_.load();
    },
    [this](double b)
    {
        intake_length_.store(b);
    },
    "Intake/static attachment length");

    ddr_.registerVariable<double>
    ("arb_feed_forward_maximum",
     [this]()
    {
        return arb_feed_forward_maximum.load();
    },
    [this](double b)
    {
        arb_feed_forward_maximum.store(b);
    },
    "Arb feedforward maximum (maximum horizontal length)");

    ddr_.registerVariable<double>
    ("arb_feed_forward_angle",
     [this]()
    {
        return arb_feed_forward_angle.load();
    },
    [this](double b)
    {
        arb_feed_forward_angle.store(b);
    },
    "Arb feedforward angle. calculation: ff_max - |sin(angle)| * this");

    ddr_.registerVariable<double>
    ("four_bar_zeroing_percent_output",
     [this]()
    {
        return four_bar_zeroing_percent_output.load();
    },
    [this](double b)
    {
        four_bar_zeroing_percent_output.store(b);
    },
    "FourBar Zeroing Percent Output");
    ddr_.registerVariable<double>
    ("four_bar_zeroing_timeout",
     [this]()
    {
        return four_bar_zeroing_timeout.load();
    },
    [this](double b)
    {
        four_bar_zeroing_timeout.store(b);
    },
    "FourBar Zeroing Timeout");
    ddr_.registerVariable<double>
    ("motion_magic_velocity",
     [this]()
    {
        return motion_magic_velocity.load();
    },
    [this](double b)
    {
        motion_magic_velocity.store(b);
    },
    "Motion Magic Velocity");
    ddr_.registerVariable<double>
    ("motion_magic_acceleration",
     [this]()
    {
        return motion_magic_acceleration.load();
    },
    [this](double b)
    {
        motion_magic_acceleration.store(b);
    },
    "Motion Magic Acceleration");
    ddr_.registerVariable<int>
    ("motion_s_curve_strength",
     [this]()
    {
        return motion_s_curve_strength.load();
    },
    [this](int b)
    {
        motion_s_curve_strength.store(b);
    },
    "S Curve Strength");

    ddr_.publishServicesTopics();

    four_bar_service_ = controller_nh.advertiseService("four_bar_service", &FourBarController_2023::cmdService, this);
    realtime_pub_.reset(new realtime_tools::RealtimePublisher<controllers_2023_msgs::FourBarState>(controller_nh, "state", 1));

    return true;
}

void FourBarController_2023::starting(const ros::Time &time)
{
    zeroed_ = false;
    last_zeroed_  = false;
    last_mode_ = hardware_interface::TalonMode_Disabled;
    last_angle_ = -1; // give nonsense position to force update on first time through update()
    position_command_ = angleFromX(min_extension_, false);
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
            four_bar_joint_.setDemand1Value(arb_feed_forward_maximum - arb_feed_forward_angle);
        }
    }
    else
    {
        last_zeroed_ = false;
    }

    if (zeroed_) // run normally, seeking to various positions
    {
        four_bar_joint_.setMode(hardware_interface::TalonMode_MotionMagic);
        if (four_bar_joint_.getMode() == hardware_interface::TalonMode_Disabled && last_mode_ == hardware_interface::TalonMode_Disabled)
        {
            position_command_ = four_bar_joint_.getPosition();
        }
        four_bar_joint_.setCommand(position_command_);

        //if we're not climbing, add an arbitrary feed forward to hold the four_bar up

        four_bar_joint_.setMotionAcceleration(motion_magic_acceleration);
        four_bar_joint_.setMotionCruiseVelocity(motion_magic_velocity);
        four_bar_joint_.setPIDFSlot(0);

        four_bar_joint_.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
        four_bar_joint_.setDemand1Value(arb_feed_forward_maximum - fabs(sin(four_bar_joint_.getPosition())) * arb_feed_forward_angle);
    }
    else
    {

        four_bar_joint_.setMode(hardware_interface::TalonMode_PercentOutput);
        if ((ros::Time::now() - last_time_down_).toSec() < four_bar_zeroing_timeout)
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
            ROS_INFO_STREAM_THROTTLE(0.25, "FourBar timed out");
            four_bar_joint_.setCommand(0);
        }

        // If not zeroed but enabled, check if the arm is moving down
        if ((four_bar_joint_.getMode() == hardware_interface::TalonMode_Disabled) ||
                (four_bar_joint_.getSpeed() < 0)) // TODO : param
        {
            // If moving down, or disabled and thus not expected to move down, reset the timer
            last_time_down_ = ros::Time::now();
        }
    }
    last_angle_ = four_bar_joint_.getPosition();
    last_mode_ = four_bar_joint_.getMode();

    if (realtime_pub_->trylock()) {
        stateMsg(last_angle_, position_command_, realtime_pub_->msg_);
        realtime_pub_->unlockAndPublish();
    }
}

void FourBarController_2023::stopping(const ros::Time &/*time*/)
{
}

bool FourBarController_2023::cmdService(controllers_2023_msgs::FourBarSrv::Request  &req,
                                        controllers_2023_msgs::FourBarSrv::Response &/*response*/)
{
    if (req.position > max_extension_)
    {
        ROS_ERROR_STREAM("FourBar controller: req.position too forward : " << req.position << ". Stop violating physics!");
        return false;
    }
    if (req.position < min_extension_)
    {
        ROS_ERROR_STREAM("FourBar controller: req.position too backward : " << req.position << ". Stop violating physics!");
        return false;
    }

    if (isRunning())
    {
        //adjust talon mode, arb feed forward, and PID slot appropriately
        double calcAngle = angleFromX(req.position, req.below);
        if (!std::isfinite(calcAngle))
        {
            ROS_ERROR_STREAM("FourBar controller: req.position resulted in a NaN angle! The robot can't derive meaning of not a number!");
            return false;
        }
        position_command_ = calcAngle;
        ROS_INFO_STREAM("writing " << std::to_string(calcAngle) << " radians to four_bar_controller");
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