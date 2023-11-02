#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "spark_max_interface/spark_max_command_interface.h"
#include "ros_control_boilerplate/sparkmax_devices.h"
#include "ros_control_boilerplate/sparkmax_device.h"
#include "ros_control_boilerplate/read_config_utils.h"

template<bool SIMFLAG>
SparkMaxDevices<SIMFLAG>::SparkMaxDevices(ros::NodeHandle &root_nh)
    : spark_max_state_interface_{std::make_unique<hardware_interface::SparkMaxStateInterface>()}
    , spark_max_command_interface_{std::make_unique<hardware_interface::SparkMaxCommandInterface>()}
    , remote_spark_max_state_interface_{std::make_unique<hardware_interface::RemoteSparkMaxStateInterface>()}
{
    ros::NodeHandle param_nh(root_nh, "generic_hw_control_loop"); // TODO : this shouldn't be hard-coded?
    if(!param_nh.param("spark_max_read_hz", read_hz_, read_hz_)) 
    {
		ROS_WARN("Failed to read spark_max_read_hz in frc_robot_interface");
	}
	ros::NodeHandle nh(root_nh, "hardware_interface"); // TODO : this shouldn't be hard-coded?
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

        if (joint_type == "spark_max")
        {
            bool local = true;
            const bool saw_local_keyword = readBooleanOptional(joint_params, "local", local, joint_name);
            bool local_update;
            bool local_hardware;
            int can_id = 0;
            readJointLocalParams(joint_params, joint_name, local, saw_local_keyword, local_update, local_hardware);
            const bool has_can_id = readIntOptional(joint_params, "can_id", can_id, joint_name);
            if (!local && has_can_id)
            {
				throw std::runtime_error("A SparkMax can_id was specified with local_hardware == false for joint " + joint_name);
            }
            if (local && !has_can_id)
            {
                throw std::runtime_error("A SparkMax can_id was not specified for joint " + joint_name);
            }
            std::string motor_type_string;
            hardware_interface::MotorType motor_type{hardware_interface::kBrushed};
            const bool has_motor_type = readStringOptional(joint_params, "motor_type", motor_type_string, joint_name);
            if (!local && has_motor_type)
            {
				throw std::runtime_error("A SparkMax motor_type was specified with local_hardware == false for joint " + joint_name);
            }
            if (local)
            {
                if (!has_motor_type)
                {
					throw std::runtime_error("A SparkMax motor_type was not specified for joint " + joint_name);
                }
                if (motor_type_string == "brushed")
                {
                    motor_type = hardware_interface::kBrushed;
                }
                else if (motor_type_string == "brushless")
                {
                    motor_type = hardware_interface::kBrushless;
                }
                else
                {
                    throw std::runtime_error("Motor_type not valid : expecting \"brushed\" or \"brushless\"");
                }
            }

            devices_.emplace_back(std::make_unique<SparkMaxDevice<SIMFLAG>>(root_nh.getNamespace(), i, joint_name, can_id, motor_type, local_update, local_hardware, read_hz_));
        }
    }
}

template <bool SIMFLAG>
SparkMaxDevices<SIMFLAG>::~SparkMaxDevices() = default;

template <bool SIMFLAG>
hardware_interface::InterfaceManager *SparkMaxDevices<SIMFLAG>::registerInterface()
{
    for (const auto &d : devices_)
    {
        d->registerInterfaces(*spark_max_state_interface_, *spark_max_command_interface_, *remote_spark_max_state_interface_);
    }
    interface_manager_.registerInterface(spark_max_state_interface_.get());
    interface_manager_.registerInterface(spark_max_command_interface_.get());
    interface_manager_.registerInterface(remote_spark_max_state_interface_.get());
    return &interface_manager_;
}

template <bool SIMFLAG>
void SparkMaxDevices<SIMFLAG>::read(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("SparkMaxs");
    for (const auto &d : devices_)
    {
        d->read(time, period);
    }
}

template <bool SIMFLAG>
void SparkMaxDevices<SIMFLAG>::write(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("SparkMaxs");
    for (const auto &d : devices_)
    {
        d->write(time, period);
    }
}