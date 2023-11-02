#include <pluginlib/class_list_macros.h> //to compile as a controller
#include <angles/angles.h>

#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <controller_interface/multi_interface_controller.h>
#include <talon_controllers/talon_controller_interface.h>
#include <ctre_interfaces/cancoder_command_interface.h>
#include "controllers_2023_msgs/TurretSrv.h"
#include <std_msgs/Float64.h>

namespace turret_controller
{

//this is the controller class, used to make a controller
class TurretController : public controller_interface::MultiInterfaceController<hardware_interface::cancoder::CANCoderCommandInterface, hardware_interface::TalonCommandInterface>
{
    public:
        TurretController() = default;

        bool init(hardware_interface::RobotHW *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override {
            
            //get interface
            hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();
            hardware_interface::cancoder::CANCoderCommandInterface *const cancoder_joint_iface = hw->get<hardware_interface::cancoder::CANCoderCommandInterface>();

            //Initialize piston joints
            // has type of CANCoderStateInterface, check if this is correct
            cancoder_joint_ = cancoder_joint_iface->getHandle("cancoder_for_turret_zero"); //joint name comes from ros_control_boilerplate/config/[insert_year]_compbot_base_jetson.yaml

            inital_cancoder_angle_ = cancoder_joint_.state()->getPosition(); // take the initial angle of the cancoder

            // initialize motor joint using those config values
            if (!turret_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, XmlRpc::XmlRpcValue("turret_leader"))) {
                ROS_ERROR("Cannot initialize turret_joint!");
                return false;
            }

            //ros::NodeHandle n_shooter_params(controller_nh, "shooter_params");
            if (!controller_nh.getParam("cancoder_offset_from_0", cancoder_offset_from_0_))
            {
                ROS_ERROR("Could not find cancoder_offset_from_0");
                return false;
            }

            if (!controller_nh.getParam("lower_angle_bound", lower_angle_bound_))
            {
                ROS_ERROR("Could not find lower_angle_bound");
                return false;
            }

            if (!controller_nh.getParam("upper_angle_bound", upper_angle_bound_))
            {
                ROS_ERROR("Could not find upper_angle_bound");
                return false;
            }

            turret_service_ = controller_nh.advertiseService("turret_command", &TurretController::cmdService, this);
            turret_relative_angle_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64>(controller_nh, "turret_angle", 4));

            return true;
                                
        }

        void starting(const ros::Time &time) override {
            position_command_ = getTurretRelativeAngle();
            ROS_WARN_STREAM("Setting turret on init to " << position_command_);
        }


        double getTurretRelativeAngle() const {
            double intial_angle = inital_cancoder_angle_ + cancoder_offset_from_0_;
            double current_angle = turret_joint_.getPosition();
            return intial_angle + current_angle; // should be in radians here
        }

        void update(const ros::Time & time, const ros::Duration& period) override {
            ROS_INFO_STREAM_THROTTLE(0.5, "Moving turret to " << position_command_);
            if (turret_joint_.getMode() == hardware_interface::TalonMode_Disabled)
            {
                if (turret_joint_.getPosition() != position_command_) {
                    ROS_ERROR_STREAM("We are disabled :(");
                }
                position_command_ = turret_joint_.getPosition();
            }

            turret_joint_.setCommand(position_command_);
            if(turret_relative_angle_pub_->trylock())
            {
                turret_relative_angle_pub_->msg_.data = getTurretRelativeAngle();
                turret_relative_angle_pub_->unlockAndPublish();
            }
        }

        void stopping(const ros::Time &time) override {

        }

        bool cmdService(controllers_2023_msgs::TurretSrv::Request &req,
                        controllers_2023_msgs::TurretSrv::Response &res) {

            if (isRunning()) {
                double result;
                double current_turret_angle = getTurretRelativeAngle();

                bool angle_result = angles::shortest_angular_distance_with_limits(current_turret_angle, req.angle, lower_angle_bound_, upper_angle_bound_, result);

                if (!angle_result) {
                    ROS_ERROR_STREAM("Angle is outside the bounds - requested angle " << req.angle);
                    return true;
                }
                ROS_INFO_STREAM("TurretController: Received new command: " << req.angle);
                ROS_INFO_STREAM_THROTTLE(0.01, "Current Turret Angle: " << current_turret_angle << " Position Command: " << position_command_ << " Result: " << result);
                position_command_ = current_turret_angle + result;
            }
            else {
                ROS_ERROR_STREAM("Can't accept new commands. TurretController is not running.");
                return false;
            }
            return true;
        }

    private:
        talon_controllers::TalonMotionMagicCloseLoopControllerInterface turret_joint_;
        hardware_interface::cancoder::CANCoderCommandHandle cancoder_joint_;

        ros::ServiceServer turret_service_;
        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>> turret_relative_angle_pub_; // so other code can see the angle of the turret without also doing the conversion 

        //double encoder_ticks_to_turret_rel_;
        double cancoder_offset_from_0_;
        double inital_cancoder_angle_;
        double lower_angle_bound_;
        double upper_angle_bound_;
        std_msgs::Float64 turret_angle_msg_;
        ros::Time last_command_time_;
        std::atomic<double> position_command_; //this is the buffer for percent output commands to be published


}; //class

} //namespace

// DO NOT FORGET
PLUGINLIB_EXPORT_CLASS(turret_controller::TurretController, controller_interface::ControllerBase)

