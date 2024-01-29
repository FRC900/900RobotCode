#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include <hardware_interface/joint_command_interface.h>
#include "remote_joint_interface/remote_joint_interface.h"
#include "ros_control_boilerplate/pwm_devices.h"
#include "ros_control_boilerplate/pwm_device.h"
#include "ros_control_boilerplate/read_config_utils.h"

PWMDevices::PWMDevices(ros::NodeHandle &root_nh)
    : state_interface_{std::make_unique<hardware_interface::JointStateInterface>()}
    , command_interface_{std::make_unique<hardware_interface::JointCommandInterface>()}
    , position_joint_interface_{std::make_unique<hardware_interface::PositionJointInterface>()}
    , remote_joint_interface_{std::make_unique<hardware_interface::RemoteJointInterface>()}
{
    // TODO : this shouldn't be hard-coded?
	ros::NodeHandle nh(root_nh, "hardware_interface");
    XmlRpc::XmlRpcValue joint_param_list;
    if (!nh.getParam("joints", joint_param_list))
    {
        throw std::runtime_error("No joints were specified->");
    }
	for (int i = 0; i < joint_param_list.size(); i++)
	{
 		const XmlRpc::XmlRpcValue &joint_params = joint_param_list[i];
        std::string joint_name;
        std::string joint_type;
        readStringRequired(joint_params, "name", joint_name);
        readStringRequired(joint_params, "type", joint_type, joint_name);

        if (joint_type == "pwm")
        {
            bool local = true;
            const bool saw_local_keyword = readBooleanOptional(joint_params, "local", local, joint_name);
            bool local_update;
            bool local_hardware;
            int pwm_channel = 0;
            // Sensible defaults from allwpilib/hal/src/main/native/athena/PWM.cpp
            double output_max = 2000.;
            double deadband_max = 1501.;
            double center = 1500.;
            double deadband_min = 1499.;
            double output_min = 100.;
            int period_multiplier = 1;
            readJointLocalParams(joint_params, joint_name, local, saw_local_keyword, local_update, local_hardware);
            const bool has_pwm_channel = readIntOptional(joint_params, "pwm_channel", pwm_channel, joint_name);
            if (!local && has_pwm_channel)
            {
				throw std::runtime_error("A PWM pwm_channel was specified with local_hardware == false for joint " + joint_name);
            }
			if (local)
			{
				if (!has_pwm_channel)
                {
					throw std::runtime_error("A PWM pwm_channel was not specified for joint " + joint_name);
                }

                /*
                auto it = std::find(digital_output_pwm_channels_.cbegin(), digital_output_pwm_channels_.cend(), digital_output_pwm_channel);
                if (it != digital_output_pwm_channels_.cend())
                {
                    ROS_WARN_STREAM("A duplicate digital output pwm_channel was specified for joint " << joint_name);
                }
                */
            }

            if (readDoubleOptional(joint_params, "output_max", output_max, joint_name) && !local)
            {
                throw std::runtime_error("A PWM output_max was specified for non-local hardware for joint " + joint_name);
            }
            if (readDoubleOptional(joint_params, "deadband_max", deadband_max, joint_name) && !local)
            {
                throw std::runtime_error("A PWM deadband_max was specified for non-local hardware for joint " + joint_name);
            }
            if (readDoubleOptional(joint_params, "output_min", output_min, joint_name) && !local)
            {
                throw std::runtime_error("A PWM output_min was specified for non-local hardware for joint " + joint_name);
            }
            if (readDoubleOptional(joint_params, "deadband_min", deadband_min, joint_name) && !local)
            {
                throw std::runtime_error("A PWM deadband_min was specified for non-local hardware for joint " + joint_name);
            }
            if (readDoubleOptional(joint_params, "center", center, joint_name) && !local)
            {
                throw std::runtime_error("A PWM center was specified for non-local hardware for joint " + joint_name);
            }
            if (readIntOptional(joint_params, "period_multiplier", period_multiplier, joint_name) && !local)
            {
                throw std::runtime_error("A PWM period_multiplier was specified for non-local hardware for joint " + joint_name);
            }

			bool invert = false;
            if (readBooleanOptional(joint_params, "invert", invert, joint_name) && !local)
			{
                throw std::runtime_error("A PWM joint invert was specified for non-local hardware for joint " + joint_name);
            }

            devices_.emplace_back(std::make_unique<PWMDevice>(i,
                                                              joint_name,
                                                              pwm_channel,
                                                              output_max,
                                                              deadband_max,
                                                              center,
                                                              output_min,
                                                              deadband_min,
                                                              period_multiplier,
                                                              invert,
                                                              local_update,
                                                              local_hardware));
        }
    }
}

PWMDevices::~PWMDevices() = default;

hardware_interface::InterfaceManager *PWMDevices::registerInterface()
{
    for (const auto &d : devices_)
    {
        d->registerInterfaces(*state_interface_, *command_interface_, *position_joint_interface_, *remote_joint_interface_);
    }
    interface_manager_.registerInterface(state_interface_.get());
    interface_manager_.registerInterface(command_interface_.get());
    interface_manager_.registerInterface(position_joint_interface_.get());
    interface_manager_.registerInterface(remote_joint_interface_.get());
    return &interface_manager_;
}

void PWMDevices::write(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("PWMs");
    for (const auto &d : devices_)
    {
        d->write(time, period);
    }
}