#include <ros_control_boilerplate/frc_robot_interface.h>

namespace ros_control_boilerplate
{
void FRCRobotInterface::readJointLocalParams(const XmlRpc::XmlRpcValue &joint_params,
											 const bool local,
											 const bool saw_local_keyword,
											 bool &local_update,
											 bool &local_hardware)
{
	local_update = local;
	if (joint_params.hasMember("local_update"))
	{
		if (saw_local_keyword)
			throw std::runtime_error("local can't be combined with local_update");
		const XmlRpc::XmlRpcValue &xml_joint_local_update = joint_params["local_update"];
		if (!xml_joint_local_update.valid() ||
			xml_joint_local_update.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
			throw std::runtime_error("An invalid joint local_update was specified (expecting a boolean).");
		local_update = xml_joint_local_update;
	}
	local_hardware = local;
	if (joint_params.hasMember("local_hardware"))
	{
		if (saw_local_keyword)
			throw std::runtime_error("local can't be combined with local_hardware");
		const XmlRpc::XmlRpcValue &xml_joint_local_hardware = joint_params["local_hardware"];
		if (!xml_joint_local_hardware.valid() ||
			xml_joint_local_hardware.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
			throw std::runtime_error("An invalid joint local_hardware was specified (expecting a boolean).");
		local_hardware = xml_joint_local_hardware;
	}
}

int FRCRobotInterface::readIntParam(const XmlRpc::XmlRpcValue &joint_params,
		bool local_hardware,
		const char *key,
		const std::string &joint_name)
{
	const bool has_key = joint_params.hasMember(key);
	if (!local_hardware && has_key)
		throw std::runtime_error(std::string("A ") + key +
				" was specified for non-local hardware for joint " + joint_name);
	int ret_val = 0;
	if (local_hardware)
	{
		if (!has_key)
			throw std::runtime_error(std::string("A ") + key + " was not specified for joint " + joint_name);
		const XmlRpc::XmlRpcValue &param_val = joint_params[key];
		if (!param_val.valid() || (param_val.getType() != XmlRpc::XmlRpcValue::TypeInt))
			throw std::runtime_error(std::string("An invalid ") + key +
					" was specified (expecting an int) for joint " + joint_name);
		ret_val = param_val;
	}
	return ret_val;
}

frc::PneumaticsModuleType FRCRobotInterface::readSolenoidModuleType(const XmlRpc::XmlRpcValue &joint_params,
		bool local_hardware,
		const std::string &joint_name)
{
	const bool has_module_type = joint_params.hasMember("module_type");
	if (!local_hardware && has_module_type)
		throw std::runtime_error("A module_type was specified for non-local hardware for joint " + joint_name);
	frc::PneumaticsModuleType solenoid_module_type;
	if (local_hardware)
	{
		if (!has_module_type)
			throw std::runtime_error("A module_type was not specified for joint " + joint_name);
		XmlRpc::XmlRpcValue &xml_solenoid_module_type = joint_params["module_type"];
		if (!xml_solenoid_module_type.valid() ||
				xml_solenoid_module_type.getType() != XmlRpc::XmlRpcValue::TypeString)
			throw std::runtime_error("An invalid module_type was specified (expecting a string) for joint " + joint_name);
		std::string solenoid_module_string = xml_solenoid_module_type;
		if (solenoid_module_string == "ctrepcm")
			solenoid_module_type = frc::PneumaticsModuleType::CTREPCM;
		else if (solenoid_module_string == "revph")
			solenoid_module_type = frc::PneumaticsModuleType::REVPH;
		else
			throw std::runtime_error("Unknown module_type for " + joint_name + ", expecting \"ctrepcm\" or \"revph\"");
	}
	return solenoid_module_type;
}

void FRCRobotInterface::readConfig(ros::NodeHandle rpnh)
{
	// Read a list of joint information from ROS parameters.  Each entry in the list
	// specifies a name for the joint and a hardware ID corresponding
	// to that value.  Joint types and locations are specified (by name)
	// in a URDF file loaded along with the controller.
	XmlRpc::XmlRpcValue joint_param_list;
	if (!rpnh.getParam("joints", joint_param_list))
		throw std::runtime_error("No joints were specified.");
	for (int i = 0; i < joint_param_list.size(); i++)
	{
		XmlRpc::XmlRpcValue &joint_params = joint_param_list[i];
		if (!joint_params.hasMember("name"))
			throw std::runtime_error("A joint name was not specified");
		XmlRpc::XmlRpcValue &xml_joint_name = joint_params["name"];
		if (!xml_joint_name.valid() ||
			xml_joint_name.getType() != XmlRpc::XmlRpcValue::TypeString)
			throw std::runtime_error("An invalid joint name was specified (expecting a string)");
		const std::string joint_name = xml_joint_name;

		if (!joint_params.hasMember("type"))
			throw std::runtime_error("A joint type was not specified for joint " + joint_name);
		XmlRpc::XmlRpcValue &xml_joint_type = joint_params["type"];
		if (!xml_joint_type.valid() ||
			xml_joint_type.getType() != XmlRpc::XmlRpcValue::TypeString)
			throw std::runtime_error("An invalid joint type was specified (expecting a string) for joint " + joint_name);
		const std::string joint_type = xml_joint_type;

		bool saw_local_keyword = false;
		bool local = true;
		bool local_update;
		bool local_hardware;
		if (joint_params.hasMember("local"))
		{
			XmlRpc::XmlRpcValue &xml_joint_local = joint_params["local"];
			if (!xml_joint_local.valid() ||
				xml_joint_local.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
				throw std::runtime_error("An invalid joint local was specified (expecting a boolean) for joint " + joint_name);
			local = xml_joint_local;
			saw_local_keyword = true;
		}

		if ((joint_type == "can_talon_srx") || (joint_type == "can_victor_spx") || (joint_type == "can_talon_fx"))
		{
			readJointLocalParams(joint_params, local, saw_local_keyword, local_update, local_hardware);

			const bool has_can_id = joint_params.hasMember("can_id");
			if (!local_hardware && has_can_id)
				throw std::runtime_error("A CAN Talon SRX / Victor SPX / Talon FX can_id was specified with local_hardware == false for joint " + joint_name);

			int can_id = 0;
			if (local_hardware)
			{
				if (!has_can_id)
					throw std::runtime_error("A CAN Talon SRX / Victor SPX / TalonFX can_id was not specified");
				XmlRpc::XmlRpcValue &xml_can_id = joint_params["can_id"];
				if (!xml_can_id.valid() ||
						xml_can_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid joint can_id was specified (expecting an int) for joint " + joint_name);
				can_id = xml_can_id;
				auto it = std::find(can_ctre_mc_can_ids_.cbegin(), can_ctre_mc_can_ids_.cend(), can_id);
				if (it != can_ctre_mc_can_ids_.cend())
					throw std::runtime_error("A duplicate can_id was specified for joint " + joint_name);
			}
			can_ctre_mc_names_.push_back(joint_name);
			can_ctre_mc_can_ids_.push_back(can_id);
			can_ctre_mc_local_updates_.push_back(local_update);
			can_ctre_mc_local_hardwares_.push_back(local_hardware);
			can_ctre_mc_is_talon_srx_.push_back(joint_type == "can_talon_srx");
			can_ctre_mc_is_talon_fx_.push_back(joint_type == "can_talon_fx");
		}
		else if (joint_type == "canifier")
		{
			readJointLocalParams(joint_params, local, saw_local_keyword, local_update, local_hardware);

			const bool has_can_id = joint_params.hasMember("can_id");
			if (!local_hardware && has_can_id)
				throw std::runtime_error("A CANifier can_id was specified with local_hardware == false for joint " + joint_name);

			int can_id = 0;
			if (local_hardware)
			{
				if (!has_can_id)
					throw std::runtime_error("A CANifier can_id was not specified");
				XmlRpc::XmlRpcValue &xml_can_id = joint_params["can_id"];
				if (!xml_can_id.valid() ||
						xml_can_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid joint can_id was specified (expecting an int) for joint " + joint_name);
				can_id = xml_can_id;
				auto it = std::find(canifier_can_ids_.cbegin(), canifier_can_ids_.cend(), can_id);
				if (it != canifier_can_ids_.cend())
					throw std::runtime_error("A duplicate can_id was specified for joint " + joint_name);
			}
			canifier_names_.push_back(joint_name);
			canifier_can_ids_.push_back(can_id);
			canifier_local_updates_.push_back(local_update);
			canifier_local_hardwares_.push_back(local_hardware);
		}
		else if (joint_type == "cancoder")
		{
			readJointLocalParams(joint_params, local, saw_local_keyword, local_update, local_hardware);

			const bool has_can_id = joint_params.hasMember("can_id");
			if (!local_hardware && has_can_id)
				throw std::runtime_error("A CANCoder can_id was specified with local_hardware == false for joint " + joint_name);

			int can_id = 0;
			if (local_hardware)
			{
				if (!has_can_id)
					throw std::runtime_error("A CANCoder can_id was not specified");
				XmlRpc::XmlRpcValue &xml_can_id = joint_params["can_id"];
				if (!xml_can_id.valid() ||
						xml_can_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid joint can_id was specified (expecting an int) for joint " + joint_name);
				can_id = xml_can_id;
				auto it = std::find(cancoder_can_ids_.cbegin(), cancoder_can_ids_.cend(), can_id);
				if (it != cancoder_can_ids_.cend())
					throw std::runtime_error("A duplicate can_id was specified for joint " + joint_name);
			}
			cancoder_names_.push_back(joint_name);
			cancoder_can_ids_.push_back(can_id);
			cancoder_local_updates_.push_back(local_update);
			cancoder_local_hardwares_.push_back(local_hardware);
		}
		else if (joint_type == "can_spark_max")
		{
			if (!joint_params.hasMember("can_id"))
				throw std::runtime_error("A CAN Spark Max can_id was not specified");
			XmlRpc::XmlRpcValue &xml_can_id = joint_params["can_id"];
			if (!xml_can_id.valid() ||
				xml_can_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
				throw std::runtime_error("An invalid joint can_id was specified (expecting an int).");
			const int can_id = xml_can_id;

			readJointLocalParams(joint_params, local, saw_local_keyword, local_update, local_hardware);

			if (!joint_params.hasMember("motor_type"))
				throw std::runtime_error("A CAN Spark Max motor_type was not specified");
			XmlRpc::XmlRpcValue &xml_motor_type = joint_params["motor_type"];
			if (!xml_motor_type.valid() ||
					xml_motor_type.getType() != XmlRpc::XmlRpcValue::TypeString)
				throw std::runtime_error("An invalid motor_type was specified (expecting a string)");
			const std::string motor_type = xml_motor_type;
			if (motor_type == "brushed")
				spark_max_motor_types_.push_back(hardware_interface::kBrushed);
			else if (motor_type == "brushless")
				spark_max_motor_types_.push_back(hardware_interface::kBrushless);
			else
				throw std::runtime_error("Motor_type not valid : expecting \"brushed\" or \"brushless\"");

			spark_max_names_.push_back(joint_name);
			spark_max_can_ids_.push_back(can_id);
			spark_max_local_updates_.push_back(local_update);
			spark_max_local_hardwares_.push_back(local_hardware);
		}
		else if (joint_type == "nidec_brushless")
		{
			readJointLocalParams(joint_params, local, saw_local_keyword, local_update, local_hardware);

			const bool has_pwm_channel = joint_params.hasMember("pwm_channel");

			if (!local_hardware && has_pwm_channel)
				throw std::runtime_error("A Nidec Brushless pwm_channel was specified with local_hardware == false for joint " + joint_name);
			int pwm_channel = 0;
			if (local_hardware)
			{
				if (!has_pwm_channel)
					throw std::runtime_error("A Nidec Brushless pwm_channel was not specified");
				XmlRpc::XmlRpcValue &xml_pwm_channel = joint_params["pwm_channel"];
				if (!xml_pwm_channel.valid() ||
						xml_pwm_channel.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid joint pwm_channel was specified (expecting an int) for joint " + joint_name);
				pwm_channel = xml_pwm_channel;
			}

			const bool has_dio_channel = joint_params.hasMember("dio_channel");
			if (!local_hardware && has_dio_channel)
				throw std::runtime_error("A Nidec Brushless dio_channel was specified with local_hardware == false for joint " + joint_name);
			int dio_channel = 0;
			if (local_hardware)
			{
				if (!has_dio_channel)
					throw std::runtime_error("A Nidec Brushless dio_channel was not specified for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_dio_channel = joint_params["dio_channel"];
				if (!xml_dio_channel.valid() ||
						xml_dio_channel.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid joint dio_channel was specified (expecting an int) for joint " + joint_name);
				dio_channel = xml_dio_channel;

				for (size_t j = 0; j < nidec_brushless_pwm_channels_.size(); j++)
					if ((nidec_brushless_pwm_channels_[j] = pwm_channel) &&
						(nidec_brushless_dio_channels_[j] = dio_channel) )
						throw std::runtime_error("Duplicate PWM & DIO Channels for Nidec Brushless joint " + joint_name);
			}

			bool invert = false;
			if (joint_params.hasMember("invert"))
			{
				if (!local_hardware)
					throw std::runtime_error("A Nidec Brushless joint invert was specified for non-local hardware for joint " + joint_name);

				XmlRpc::XmlRpcValue &xml_invert = joint_params["invert"];
				if (!xml_invert.valid() ||
					xml_invert.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
					throw std::runtime_error("An invalid Nidec brushless joint invert was specified (expecting a boolean) for joint " + joint_name);
				invert = xml_invert;
			}

			nidec_brushless_names_.push_back(joint_name);
			nidec_brushless_pwm_channels_.push_back(pwm_channel);
			nidec_brushless_dio_channels_.push_back(dio_channel);
			nidec_brushless_inverts_.push_back(invert);
			nidec_brushless_local_updates_.push_back(local_update);
			nidec_brushless_local_hardwares_.push_back(local_hardware);
		}
		else if (joint_type == "digital_input")
		{
			const bool has_dio_channel = joint_params.hasMember("dio_channel");
			if (!local && has_dio_channel)
				throw std::runtime_error("A Digital Input dio_channel was specified with local_hardware == false for joint " + joint_name);
			int digital_input_dio_channel = 0;
			if (local)
			{
				if (!has_dio_channel)
					throw std::runtime_error("A Digital Input dio_channel was not specified for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_digital_input_dio_channel = joint_params["dio_channel"];
				if (!xml_digital_input_dio_channel.valid() ||
						xml_digital_input_dio_channel.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid joint dio_channel was specified (expecting an int) for joint " + joint_name);
				digital_input_dio_channel = xml_digital_input_dio_channel;

				auto it = std::find(digital_input_dio_channels_.cbegin(), digital_input_dio_channels_.cend(), digital_input_dio_channel);
				if (it != digital_input_dio_channels_.cend())
					ROS_WARN_STREAM("A duplicate digital input dio_channel was specified for joint " << joint_name);
			}

			bool invert = false;
			if (joint_params.hasMember("invert"))
			{
				if (!local)
					throw std::runtime_error("A Digital Input joint invert was specified for non-local hardware for joint " + joint_name);

				XmlRpc::XmlRpcValue &xml_invert = joint_params["invert"];
				if (!xml_invert.valid() ||
					xml_invert.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
					throw std::runtime_error("An invalid digital input joint invert was specified (expecting a boolean) for joint " + joint_name);
				invert = xml_invert;
			}

			digital_input_names_.push_back(joint_name);
			digital_input_dio_channels_.push_back(digital_input_dio_channel);
			digital_input_inverts_.push_back(invert);
			digital_input_locals_.push_back(local);
		}
		else if (joint_type == "digital_output")
		{
			readJointLocalParams(joint_params, local, saw_local_keyword, local_update, local_hardware);

			const bool has_dio_channel = joint_params.hasMember("dio_channel");
			if (!local_hardware && has_dio_channel)
				throw std::runtime_error("A Digital Output dio_channel was specified with local_hardware == false for joint " + joint_name);
			int digital_output_dio_channel = 0;
			if (local_hardware)
			{
				if (!has_dio_channel)
					throw std::runtime_error("A Digital Output dio_channel was not specified for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_digital_output_dio_channel = joint_params["dio_channel"];
				if (!xml_digital_output_dio_channel.valid() ||
					xml_digital_output_dio_channel.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid joint dio_channel was specified (expecting an int) for joint " + joint_name);
				digital_output_dio_channel = xml_digital_output_dio_channel;
					auto it = std::find(digital_output_dio_channels_.cbegin(), digital_output_dio_channels_.cend(), digital_output_dio_channel);
					if (it != digital_output_dio_channels_.cend())
						throw std::runtime_error("A duplicate digital output channel was specified for joint " + joint_name);
			}

			bool invert = false;
			if (joint_params.hasMember("invert"))
			{
				if (!local_hardware)
					throw std::runtime_error("A Digital Output joint invert was specified for non-local hardware for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_invert = joint_params["invert"];
				if (!xml_invert.valid() ||
					xml_invert.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
					throw std::runtime_error("An invalid digital output joint invert was specified (expecting a boolean) for joint " + joint_name);
				invert = xml_invert;
			}

			digital_output_names_.push_back(joint_name);
			digital_output_dio_channels_.push_back(digital_output_dio_channel);
			digital_output_inverts_.push_back(invert);
			digital_output_local_updates_.push_back(local_update);
			digital_output_local_hardwares_.push_back(local_hardware);
		}
		else if (joint_type == "pwm")
		{
			readJointLocalParams(joint_params, local, saw_local_keyword, local_update, local_hardware);

			const bool has_pwm_channel = joint_params.hasMember("pwm_channel");
			if (!local_hardware && has_pwm_channel)
				throw std::runtime_error("A PWM pwm_channel was specified for non-local hardware for joint " + joint_name);
			int pwm_pwm_channel = 0;
			if (local_hardware)
			{
				if (!has_pwm_channel)
					throw std::runtime_error("A PWM pwm_channel was not specified for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_pwm_pwm_channel = joint_params["pwm_channel"];
				if (!xml_pwm_pwm_channel.valid() ||
					xml_pwm_pwm_channel.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid joint pwm_channel was specified (expecting an int) for joint " + joint_name);
				pwm_pwm_channel = xml_pwm_pwm_channel;
				auto it = std::find(pwm_pwm_channels_.cbegin(), pwm_pwm_channels_.cend(), pwm_pwm_channel);
				if (it != pwm_pwm_channels_.cend())
					throw std::runtime_error("A duplicate pwm channel was specified for joint " + joint_name);
			}

			bool invert = false;
			if (joint_params.hasMember("invert"))
			{
				if (!local_hardware)
					throw std::runtime_error("A PWM joint invert was specified for non-local hardware for joint " + joint_name);

				XmlRpc::XmlRpcValue &xml_invert = joint_params["invert"];
				if (!xml_invert.valid() ||
					xml_invert.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
					throw std::runtime_error("An invalid pwm joint invert was specified (expecting a boolean) for joint " + joint_name);
				invert = xml_invert;
			}

			pwm_names_.push_back(joint_name);
			pwm_pwm_channels_.push_back(pwm_pwm_channel);
			pwm_inverts_.push_back(invert);
			pwm_local_updates_.push_back(local_update);
			pwm_local_hardwares_.push_back(local_hardware);
		}
		else if (joint_type == "solenoid")
		{
			readJointLocalParams(joint_params, local, saw_local_keyword, local_update, local_hardware);

			// These don't matter for remote hardware
			frc::PneumaticsModuleType solenoid_module_type = frc::PneumaticsModuleType::CTREPCM;
			int solenoid_channel = -1;
			int solenoid_module_id = -1;

			if (local_hardware)
			{
				solenoid_channel = readIntParam(joint_params, local_hardware, "channel", joint_name);
				solenoid_module_type = readSolenoidModuleType(joint_params, local_hardware, joint_name);
				solenoid_module_id = readIntParam(joint_params, local_hardware, "module_id", joint_name);
				for (size_t j = 0; j < solenoid_module_ids_.size(); j++)
					if ((solenoid_module_ids_[j] == solenoid_module_id) &&
						(solenoid_module_types_[j] == solenoid_module_type) &&
					    (solenoid_channels_[j] == solenoid_channel))
					throw std::runtime_error("Duplicate solenoid module_id & id was specified for joint " + joint_name +
							" (previously used in " + solenoid_names_[j] + ")");
				for (size_t j = 0; j < double_solenoid_module_ids_.size(); j++)
					if ((double_solenoid_module_ids_[j] == solenoid_module_id) &&
						(double_solenoid_module_types_[j] == solenoid_module_type) &&
					   ((double_solenoid_forward_channels_[j] == solenoid_channel) ||
						(double_solenoid_reverse_channels_[j] == solenoid_channel) ))
					throw std::runtime_error("Duplicate solenoid module & channel was reused in joint " + joint_name +
							" (previously used in " + double_solenoid_names_[j] + ")");
			}

			solenoid_names_.push_back(joint_name);
			solenoid_channels_.push_back(solenoid_channel);
			solenoid_module_types_.push_back(solenoid_module_type);
			solenoid_module_ids_.push_back(solenoid_module_id);
			solenoid_local_updates_.push_back(local_update);
			solenoid_local_hardwares_.push_back(local_hardware);
		}
		else if (joint_type == "double_solenoid")
		{
			readJointLocalParams(joint_params, local, saw_local_keyword, local_update, local_hardware);
			int double_solenoid_forward_channel = -1;
			int double_solenoid_reverse_channel = -1;
			frc::PneumaticsModuleType double_solenoid_module_type = frc::PneumaticsModuleType::CTREPCM;
			int double_solenoid_module_id = -1;

			if (local_hardware)
			{
				double_solenoid_forward_channel = readIntParam(joint_params, local_hardware, "forward_channel", joint_name);
				double_solenoid_reverse_channel = readIntParam(joint_params, local_hardware, "reverse_channel", joint_name);
				double_solenoid_module_type = readSolenoidModuleType(joint_params, local_hardware, joint_name);
				double_solenoid_module_id = readIntParam(joint_params, local_hardware, "module_id", joint_name);
				if (double_solenoid_forward_channel == double_solenoid_reverse_channel)
					throw std::runtime_error("Double solenoid " + joint_name +
							" delcared with the same forward and reverse channel");

				for (size_t j = 0; j < solenoid_module_ids_.size(); j++)
					if ((solenoid_module_ids_[j] == double_solenoid_module_id) &&
					    (solenoid_module_types_[j] == double_solenoid_module_type) &&
					    ((solenoid_channels_[j] == double_solenoid_forward_channel) ||
						 (solenoid_channels_[j] == double_solenoid_reverse_channel) ))
					throw std::runtime_error("Duplicate solenoid module_id & channel was specified for joint "
							+ joint_name + " (previously used in " + solenoid_names_[j] + ")");

				for (size_t j = 0; j < double_solenoid_module_ids_.size(); j++)
					if ((double_solenoid_module_ids_[j] == double_solenoid_module_id) &&
					     (double_solenoid_module_types_[j] == double_solenoid_module_type) &&
					   ((double_solenoid_forward_channels_[j] == double_solenoid_forward_channel) ||
					    (double_solenoid_forward_channels_[j] == double_solenoid_reverse_channel) ||
					    (double_solenoid_reverse_channels_[j] == double_solenoid_forward_channel) ||
					    (double_solenoid_reverse_channels_[j] == double_solenoid_reverse_channel) ))
					throw std::runtime_error("Duplicate solenoid module_id & channel was specified for joint "
							+ joint_name + " (previously used in " + double_solenoid_names_[j] + ")");
			}

			double_solenoid_names_.push_back(joint_name);
			double_solenoid_forward_channels_.push_back(double_solenoid_forward_channel);
			double_solenoid_reverse_channels_.push_back(double_solenoid_reverse_channel);
			double_solenoid_module_types_.push_back(double_solenoid_module_type);
			double_solenoid_module_ids_.push_back(double_solenoid_module_id);
			double_solenoid_local_updates_.push_back(local_update);
			double_solenoid_local_hardwares_.push_back(local_hardware);
		}
		else if (joint_type == "rumble")
		{
			readJointLocalParams(joint_params, local, saw_local_keyword, local_update, local_hardware);

			const bool has_rumble_port = joint_params.hasMember("rumble_port");
			if (local_hardware && !has_rumble_port)
				throw std::runtime_error("A rumble_port was specified for non-local hardware for joint " + joint_name);
			int rumble_port = 0;
			if (local_hardware)
			{
				if (!has_rumble_port)
					throw std::runtime_error("A rumble_port was not specified for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_rumble_port = joint_params["rumble_port"];
				if (!xml_rumble_port.valid() ||
						xml_rumble_port.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid joint rumble_port was specified (expecting an int) for joint " + joint_name);
				rumble_port = xml_rumble_port;

				auto it = std::find(rumble_ports_.cbegin(), rumble_ports_.cend(), rumble_port);
				if (it != rumble_ports_.cend())
					throw std::runtime_error("A duplicate rumble port was specified for joint " + joint_name);
			}

			rumble_names_.push_back(joint_name);
			rumble_ports_.push_back(rumble_port);
			rumble_local_updates_.push_back(local_update);
			rumble_local_hardwares_.push_back(local_hardware);
		}
		else if (joint_type == "navX")
		{
			// TODO : id might instead be a string - MXP, USB, etc
			// telling where the navX is attached?
			const bool has_id = joint_params.hasMember("id");
			if (!local && has_id)
				throw std::runtime_error("A navX id was specified for non-local hardware for joint " + joint_name);
			int navX_id = 0;
			if (local)
			{
				if (!has_id)
					throw std::runtime_error("A navX id was not specified for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_navX_id = joint_params["id"];
				if (!xml_navX_id.valid() ||
						xml_navX_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid joint id was specified (expecting an int) for joint " + joint_name);
				navX_id = xml_navX_id;
				auto it = std::find(navX_ids_.cbegin(), navX_ids_.cend(), navX_id);
				if (it != navX_ids_.cend())
					throw std::runtime_error("A duplicate navX_id was specified for joint " + joint_name);
			}

			const bool has_frame_id = joint_params.hasMember("id");
			if (!local && has_frame_id)
				throw std::runtime_error("A navX frame_id was specified for non-local hardware for joint " + joint_name);
			std::string frame_id;
			if (local)
			{
				if (!has_frame_id)
					throw std::runtime_error("A navX frame_id was not specified for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_joint_frame_id= joint_params["frame_id"];
				if (!xml_joint_frame_id.valid() ||
						xml_joint_frame_id.getType() != XmlRpc::XmlRpcValue::TypeString)
					throw std::runtime_error("An invalid navX frame_id was specified (expecting a string) for joint " + joint_name);
				frame_id = std::string(xml_joint_frame_id);
			}

			navX_names_.push_back(joint_name);
			navX_frame_ids_.push_back(frame_id);
			navX_ids_.push_back(navX_id);
			navX_locals_.push_back(local);
		}
		else if (joint_type == "analog_input")
		{
			const bool has_analog_channel = joint_params.hasMember("analog_channel");
			if (!local && has_analog_channel)
				throw std::runtime_error("A Analog input analog_channel was specified for non-local hardware for joint " + joint_name);
			int analog_input_analog_channel = 0;
			if (local)
			{
				if (!has_analog_channel)
					throw std::runtime_error("A Analog input analog_channel was not specified for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_analog_input_analog_channel = joint_params["analog_channel"];
				if (!xml_analog_input_analog_channel.valid() ||
					xml_analog_input_analog_channel.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid joint analog_channel was specified (expecting an int) for joint " + joint_name);
				analog_input_analog_channel = xml_analog_input_analog_channel;
				auto it = std::find(analog_input_analog_channels_.cbegin(), analog_input_analog_channels_.cend(), analog_input_analog_channel);
				if (it != analog_input_analog_channels_.cend())
					ROS_WARN_STREAM("A duplicate analog input channel was specified for joint " << joint_name);
			}

			double analog_input_a = 1;

			if (joint_params.hasMember("analog_a"))
			{
				if (!local)
					throw std::runtime_error("A Analog input analog_a was specified for non-local hardware for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_analog_input_a = joint_params["analog_a"];
				if (!xml_analog_input_a.valid() ||
					xml_analog_input_a.getType() != XmlRpc::XmlRpcValue::TypeDouble)
					throw std::runtime_error("An invalid joint a term was specified (expecting an double) for joint " + joint_name);
				analog_input_a = xml_analog_input_a;
			}

			double analog_input_b = 0;
			if (joint_params.hasMember("analog_b"))
			{
				if (!local)
					throw std::runtime_error("A Analog input analog_b was specified for non-local hardware for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_analog_input_b = joint_params["analog_b"];
				if (!xml_analog_input_b.valid() ||
					xml_analog_input_b.getType() != XmlRpc::XmlRpcValue::TypeDouble)
					throw std::runtime_error("An invalid joint b term was specified (expecting an double) for joint " + joint_name);
				analog_input_b = xml_analog_input_b;
			}

			analog_input_a_.push_back(analog_input_a);
			analog_input_b_.push_back(analog_input_b);
			analog_input_names_.push_back(joint_name);
			analog_input_analog_channels_.push_back(analog_input_analog_channel);
			analog_input_locals_.push_back(local);
		}
		else if (joint_type == "pcm")
		{
			readJointLocalParams(joint_params, local, saw_local_keyword, local_update, local_hardware);

			const bool has_pcm_id = joint_params.hasMember("pcm_id");
			if (!local_hardware && has_pcm_id)
				throw std::runtime_error("A PCM id was specified for non-local hardware for joint " + joint_name);
			int pcm_id = 0;
			if (local_hardware)
			{
				if (!has_pcm_id)
					throw std::runtime_error("A PCM id was not specified for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_pcm_id = joint_params["pcm_id"];
				if (!xml_pcm_id.valid() || xml_pcm_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An PCM id was specified (expecting an int) for joint " + joint_name);
				pcm_id = xml_pcm_id;
				if (std::find(pcm_ids_.cbegin(), pcm_ids_.cend(), pcm_id) != pcm_ids_.cend())
					throw std::runtime_error("A duplicate PCM id was specified for joint " + joint_name);
			}

			pcm_names_.push_back(joint_name);
			pcm_ids_.push_back(pcm_id);
			pcm_local_updates_.push_back(local_update);
			pcm_local_hardwares_.push_back(local_hardware);
		}
		else if (joint_type == "pdh")
		{
			readJointLocalParams(joint_params, local, saw_local_keyword, local_update, local_hardware);
			int32_t pdh_module = 0;
			if (joint_params.hasMember("module"))
			{
				if (!local)
					throw std::runtime_error("A PDH id was specified for non-local hardware for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_pdh_module = joint_params["module"];
				if (!xml_pdh_module.valid() ||
					 xml_pdh_module.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid PDH joint module id was specified (expecting an int) for joint " + joint_name);
				pdh_module = xml_pdh_module;
				auto it = std::find(pdh_modules_.cbegin(), pdh_modules_.cend(), pdh_module);
				if (it != pdh_modules_.cend())
					throw std::runtime_error("A duplicate PDH module was specified for joint " + joint_name);
			}

			pdh_names_.push_back(joint_name);
			pdh_local_updates_.push_back(local_update);
			pdh_local_hardwares_.push_back(local_hardware);
			pdh_modules_.push_back(pdh_module);
		}
		else if (joint_type == "pdp")
		{
			int32_t pdp_module = 0;
			if (joint_params.hasMember("module"))
			{
				if (!local)
					throw std::runtime_error("A PDP id was specified for non-local hardware for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_pdp_module = joint_params["module"];
				if (!xml_pdp_module.valid() ||
					 xml_pdp_module.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid PDP joint module id was specified (expecting an int) for joint " + joint_name);
				pdp_module = xml_pdp_module;
				auto it = std::find(pdp_modules_.cbegin(), pdp_modules_.cend(), pdp_module);
				if (it != pdp_modules_.cend())
					throw std::runtime_error("A duplicate PDP module was specified for joint " + joint_name);
			}

			pdp_names_.push_back(joint_name);
			pdp_locals_.push_back(local);
			pdp_modules_.push_back(pdp_module);
		}
		else if (joint_type == "ph")
		{
			readJointLocalParams(joint_params, local, saw_local_keyword, local_update, local_hardware);

			const bool has_ph_id = joint_params.hasMember("ph_id");
			if (!local_hardware && has_ph_id)
				throw std::runtime_error("A PH id was specified for non-local hardware for joint " + joint_name);
			int ph_id = 0;
			if (local_hardware)
			{
				if (!has_ph_id)
					throw std::runtime_error("A PH id was not specified for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_ph_id = joint_params["ph_id"];
				if (!xml_ph_id.valid() || xml_ph_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An PH id was specified (expecting an int) for joint " + joint_name);
				ph_id = xml_ph_id;
				if (std::find(ph_ids_.cbegin(), ph_ids_.cend(), ph_id) != ph_ids_.cend())
					throw std::runtime_error("A duplicate PH id was specified for joint " + joint_name);
			}

			ph_names_.push_back(joint_name);
			ph_ids_.push_back(ph_id);
			ph_local_updates_.push_back(local_update);
			ph_local_hardwares_.push_back(local_hardware);
		}
		else if (joint_type == "dummy")
		{
			dummy_joint_names_.push_back(joint_name);
			dummy_joint_locals_.push_back(local);
		}
		else if (joint_type == "ready")
		{
			ready_signal_names_.push_back(joint_name);
			ready_signal_locals_.push_back(local);
		}
		else if (joint_type == "joystick")
		{
			if (!local)
				throw std::runtime_error("A joystick ID was specified for non-local hardware for joint " + joint_name);
			const bool has_id = joint_params.hasMember("id");
			if (!has_id)
				throw std::runtime_error("A joystick ID was not specified for joint " + joint_name);
			XmlRpc::XmlRpcValue &xml_id = joint_params["id"];
			if (!xml_id.valid() ||
				xml_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
				throw std::runtime_error("An invalid joystick id was specified (expecting an int) for joint " + joint_name);

			int id = xml_id;
			auto it = std::find(joystick_ids_.cbegin(), joystick_ids_.cend(), id);
			if (it != joystick_ids_.cend())
				throw std::runtime_error("A duplicate joystick ID was specified for joint " + joint_name);
			joystick_names_.push_back(joint_name);
			joystick_ids_.push_back(id);
		}
		else if (joint_type == "as726x")
		{
			readJointLocalParams(joint_params, local, saw_local_keyword, local_update, local_hardware);

			const bool has_port = joint_params.hasMember("port");
			if (local_hardware && !has_port)
				throw std::runtime_error("A port was specified for non-local hardware for joint " + joint_name);
			std::string port_string;
			if (local_hardware)
			{
				if (!has_port)
					throw std::runtime_error("A port was not specified for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_port = joint_params["port"];
				if (!xml_port.valid() || xml_port.getType() != XmlRpc::XmlRpcValue::TypeString)
					throw std::runtime_error("An invalid joint port was specified (expecting a string) for joint " + joint_name);
				port_string = std::string(xml_port);
			}

			const bool has_address = joint_params.hasMember("address");
			if (!local_hardware && has_address)
				throw std::runtime_error("An address was specified for non-local hardware for joint " + joint_name);
			int address = 0;
			if (local_hardware)
			{
				if (!has_address)
					throw std::runtime_error("An as726x address was not specified for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_address = joint_params["address"];
				if (!xml_address.valid() || xml_address.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid address was specified (expecting an int) for joint " + joint_name);
				address = xml_address;
			}

			as726x_names_.push_back(joint_name);
			as726x_ports_.push_back(port_string);
			as726x_addresses_.push_back(address);
			as726x_local_updates_.push_back(local_update);
			as726x_local_hardwares_.push_back(local_hardware);
		}
		else if (joint_type == "orchestra")
		{
			talon_orchestra_names_.push_back(joint_name);

			const bool has_id = joint_params.hasMember("id");
			int orchestra_id = 0;
			if (!has_id)
				throw std::runtime_error("An orchestra id was not specified for joint " + joint_name);
			XmlRpc::XmlRpcValue &xml_orchestra_id = joint_params["id"];
			if (!xml_orchestra_id.valid() ||
					xml_orchestra_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
				throw std::runtime_error("An invalid joint orchestra id was specified (expecting an int) for joint " + joint_name);
			orchestra_id = xml_orchestra_id;

			talon_orchestra_ids_.push_back(orchestra_id);
		}
		else
		{
			std::stringstream s;
			s << "Unknown joint type " << joint_type << " specified for joint " + joint_name;
			throw std::runtime_error(s.str());
		}
	}
	run_hal_robot_ = rpnh.param<bool>("run_hal_robot", true);
	can_interface_ = rpnh.param<std::string>("can_interface", "can0");
}

} // namespace
