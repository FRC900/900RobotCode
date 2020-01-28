/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Original Author: Dave Coleman
   Desc:   Helper ros_control hardware interface that loads configurations
*/

//PURPOSE: Stuff used by to run both hw and sim interfaces

#include <ros_control_boilerplate/frc_robot_interface.h>
#include <limits>

namespace ros_control_boilerplate
{

void FRCRobotInterface::readJointLocalParams(XmlRpc::XmlRpcValue joint_params,
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
		XmlRpc::XmlRpcValue &xml_joint_local_update = joint_params["local_update"];
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
		XmlRpc::XmlRpcValue &xml_joint_local_hardware = joint_params["local_hardware"];
		if (!xml_joint_local_hardware.valid() ||
			xml_joint_local_hardware.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
			throw std::runtime_error("An invalid joint local_hardware was specified (expecting a boolean).");
		local_hardware = xml_joint_local_hardware;
	}
}

FRCRobotInterface::FRCRobotInterface(ros::NodeHandle &nh, urdf::Model *urdf_model) :
	  name_("generic_hw_interface")
	, nh_(nh)
	, num_can_ctre_mcs_(0)
	, num_nidec_brushlesses_(0)
	, num_digital_inputs_(0)
	, num_digital_outputs_(0)
	, num_pwms_(0)
	, num_solenoids_(0)
	, num_double_solenoids_(0)
	, num_compressors_(0)
	, num_rumbles_(0)
	, num_navX_(0)
	, num_analog_inputs_(0)
	, num_dummy_joints_(0)
    , num_ready_signals_(0)
	, robot_code_ready_(false)
{
	// Check if the URDF model needs to be loaded
	if (urdf_model == NULL)
		loadURDF(nh, "robot_description");
	else
		urdf_model_ = urdf_model;

	// Load rosparams
	ros::NodeHandle rpnh(nh_, "hardware_interface"); // TODO(davetcoleman): change the namespace to "frc_robot_interface" aka name_

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
				throw std::runtime_error("A CAN Talon SRX / Victor SPX can_id was specified with local_hardware == false for joint " + joint_name);

			int can_id = 0;
			if (local_hardware)
			{
				if (!has_can_id)
					throw std::runtime_error("A CAN Talon SRX / Victor SPX can_id was not specified");
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

			const bool has_id = joint_params.hasMember("id");
			if (!local_hardware && has_id)
				throw std::runtime_error("A solenoid id was specified for non-local hardware for joint " + joint_name);
			int solenoid_id = 0;
			if (local_hardware)
			{
				if (!has_id)
					throw std::runtime_error("A solenoid id was not specified");
				XmlRpc::XmlRpcValue &xml_solenoid_id = joint_params["id"];
				if (!xml_solenoid_id.valid() ||
					xml_solenoid_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid joint solenoid id was specified (expecting an int) for joint " + joint_name);
				solenoid_id = xml_solenoid_id;
			}

			const bool has_pcm = joint_params.hasMember("pcm");
			if (!local_hardware && has_pcm)
				throw std::runtime_error("A solenoid pcm was specified for non-local hardware for joint " + joint_name);
			int solenoid_pcm = 0;
			if (local_hardware)
			{
				if (!has_pcm)
					throw std::runtime_error("A solenoid pcm was not specified for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_solenoid_pcm = joint_params["pcm"];
				if (!xml_solenoid_pcm.valid() ||
						xml_solenoid_pcm.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid joint solenoid pcm was specified (expecting an int) for joint " + joint_name);
				solenoid_pcm = xml_solenoid_pcm;
				for (size_t j = 0; j < solenoid_pcms_.size(); j++)
					if ((solenoid_pcms_[j] == solenoid_pcm) &&
					    (solenoid_ids_[j] == solenoid_id))
					throw std::runtime_error("Duplicate solenoid pcm & id was specified for joint " + joint_name);
				for (size_t j = 0; j < double_solenoid_pcms_.size(); j++)
					if ((double_solenoid_pcms_[j] == solenoid_pcm) &&
					   ((double_solenoid_forward_ids_[j] == solenoid_id) ||
						(double_solenoid_reverse_ids_[j] == solenoid_id) ))
					throw std::runtime_error("Duplicate solenoid pcm & id was specified for joint " + joint_name);
			}

			solenoid_names_.push_back(joint_name);
			solenoid_ids_.push_back(solenoid_id);
			solenoid_pcms_.push_back(solenoid_pcm);
			solenoid_local_updates_.push_back(local_update);
			solenoid_local_hardwares_.push_back(local_hardware);
		}
		else if (joint_type == "double_solenoid")
		{
			readJointLocalParams(joint_params, local, saw_local_keyword, local_update, local_hardware);

			const bool has_forward_id = joint_params.hasMember("forward_id");
			if (!local_hardware && has_forward_id)
				throw std::runtime_error("A double solenoid forward_id was specified for non-local hardware for joint " + joint_name);
			int double_solenoid_forward_id = 0;
			if (local_hardware)
			{
				if (!has_forward_id)
					throw std::runtime_error("A double solenoid forward_id was not specified for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_double_solenoid_forward_id = joint_params["forward_id"];
				if (!xml_double_solenoid_forward_id.valid() ||
					xml_double_solenoid_forward_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid joint double solenoid forward_id was specified (expecting an int) for joint " + joint_name);
				double_solenoid_forward_id = xml_double_solenoid_forward_id;
			}

			const bool has_reverse_id = joint_params.hasMember("reverse_id");
			if (!local_hardware && has_reverse_id)
				throw std::runtime_error("A double solenoid reverse_id was specified for non-local hardware for joint " + joint_name);
					int double_solenoid_reverse_id = 0;
			if (local_hardware)
			{
				if (!has_reverse_id)
					throw std::runtime_error("A double solenoid reverse_id was not specified for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_double_solenoid_reverse_id = joint_params["reverse_id"];
				if (!xml_double_solenoid_reverse_id.valid() ||
					xml_double_solenoid_reverse_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid joint double solenoid reverse_id was specified (expecting an int) for joint " + joint_name);
				double_solenoid_reverse_id = xml_double_solenoid_reverse_id;
			}

			const bool has_pcm = joint_params.hasMember("pcm");
			if (!local_hardware && has_pcm)
				throw std::runtime_error("A double solenoid pcm was specified for non-local hardware for joint " + joint_name);
			int double_solenoid_pcm = 0;
			if (local_hardware)
			{
				if (!has_pcm)
					throw std::runtime_error("A double solenoid pcm was not specified for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_double_solenoid_pcm = joint_params["pcm"];
				if (!xml_double_solenoid_pcm.valid() ||
						xml_double_solenoid_pcm.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid joint double solenoid pcm was specified (expecting an int) for joint " + joint_name);
				double_solenoid_pcm = xml_double_solenoid_pcm;

				for (size_t j = 0; j < solenoid_pcms_.size(); j++)
					if ((solenoid_pcms_[j] == double_solenoid_pcm) &&
					    ((solenoid_ids_[j] == double_solenoid_forward_id) ||
						 (solenoid_ids_[j] == double_solenoid_reverse_id) ))
					throw std::runtime_error("Duplicate solenoid pcm & id was specified for joint " + joint_name);
				for (size_t j = 0; j < double_solenoid_pcms_.size(); j++)
					if ((double_solenoid_pcms_[j] == double_solenoid_pcm) &&
					   ((double_solenoid_forward_ids_[j] == double_solenoid_forward_id) ||
					    (double_solenoid_forward_ids_[j] == double_solenoid_reverse_id) ||
					    (double_solenoid_reverse_ids_[j] == double_solenoid_forward_id) ||
					    (double_solenoid_reverse_ids_[j] == double_solenoid_reverse_id) ))
					throw std::runtime_error("Duplicate solenoid pcm & id was specified for joint " + joint_name);
			}

			double_solenoid_names_.push_back(joint_name);
			double_solenoid_forward_ids_.push_back(double_solenoid_forward_id);
			double_solenoid_reverse_ids_.push_back(double_solenoid_reverse_id);
			double_solenoid_pcms_.push_back(double_solenoid_pcm);
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
		else if (joint_type == "compressor")
		{
			readJointLocalParams(joint_params, local, saw_local_keyword, local_update, local_hardware);

			const bool has_pcm_id = joint_params.hasMember("pcm_id");
			if (!local_hardware && has_pcm_id)
				throw std::runtime_error("A compressor pcm id was specified for non-local hardware for joint " + joint_name);
			int compressor_pcm_id = 0;
			if (local_hardware)
			{
				if (!has_pcm_id)
					throw std::runtime_error("A compressor pcm id was not specified for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_compressor_pcm_id = joint_params["pcm_id"];
				if (!xml_compressor_pcm_id.valid() ||
						xml_compressor_pcm_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid compressor joint pcm id was specified (expecting an int) for joint " + joint_name);
				compressor_pcm_id = xml_compressor_pcm_id;
				auto it = std::find(compressor_pcm_ids_.cbegin(), compressor_pcm_ids_.cend(), compressor_pcm_id);
				if (it != compressor_pcm_ids_.cend())
					throw std::runtime_error("A duplicate compressor CAN id was specified for joint " + joint_name);
			}

			compressor_names_.push_back(joint_name);
			compressor_pcm_ids_.push_back(compressor_pcm_id);
			compressor_local_updates_.push_back(local_update);
			compressor_local_hardwares_.push_back(local_hardware);
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
			const bool has_id = joint_params.hasMember("id");
			if (!local && has_id)
				throw std::runtime_error("A joystick ID was specified for non-local hardware for joint " + joint_name);
			int id = 0;
			if (local)
			{
				if (!has_id)
					throw std::runtime_error("A joystick ID was not specified for joint " + joint_name);
				XmlRpc::XmlRpcValue &xml_id = joint_params["id"];
				if (!xml_id.valid() ||
						xml_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid joystick id was specified (expecting an int) for joint " + joint_name);
				id = xml_id;
				auto it = std::find(joystick_ids_.cbegin(), joystick_ids_.cend(), id);
				if (it != joystick_ids_.cend())
					throw std::runtime_error("A duplicate joystick ID was specified for joint " + joint_name);
			}
			joystick_names_.push_back(joint_name);
			joystick_ids_.push_back(id);
			joystick_locals_.push_back(local);
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

void FRCRobotInterface::init()
{
	num_can_ctre_mcs_ = can_ctre_mc_names_.size();
	// Create vectors of the correct size for
	// talon HW state and commands
	talon_command_.resize(num_can_ctre_mcs_);

	// Loop through the list of joint names
	// specified as params for the hardware_interface.
	// For each of them, create a Talon object. This
	// object is used to send and recieve commands
	// and status to/from the physical Talon motor
	// controller on the robot.  Use this pointer
	// to initialize each Talon with various params
	// set for that motor controller in config files.
	for (size_t i = 0; i < num_can_ctre_mcs_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering Talon Interface for " << can_ctre_mc_names_[i] << " at hw ID " << can_ctre_mc_can_ids_[i]);

		// Create joint state interface
		// Also register as JointStateInterface so that legacy
		// ROS code which uses that object type can
		// access basic state info from the talon
		// Code which needs more specific status should
		// get a TalonStateHandle instead.
		talon_state_.push_back(hardware_interface::TalonHWState(can_ctre_mc_can_ids_[i]));
	}
	for (size_t i = 0; i < num_can_ctre_mcs_; i++)
	{
		// Create state interface for the given Talon
		// and point it to the data stored in the
		// corresponding talon_state array entry
		hardware_interface::TalonStateHandle tsh(can_ctre_mc_names_[i], &talon_state_[i]);
		talon_state_interface_.registerHandle(tsh);

		// Do the same for a command interface for
		// the same talon
		hardware_interface::TalonCommandHandle tch(tsh, &talon_command_[i]);
		talon_command_interface_.registerHandle(tch);
		if (!can_ctre_mc_local_updates_[i])
		{
			hardware_interface::TalonWritableStateHandle twsh(can_ctre_mc_names_[i], &talon_state_[i]); /// writing directly to state?
			talon_remote_state_interface_.registerHandle(twsh);
		}
		custom_profile_state_.push_back(CustomProfileState());
	}

	// Set vectors to correct size to hold data
	// for each of the brushless motors we're trying
	// to control
	num_nidec_brushlesses_ = nidec_brushless_names_.size();
	brushless_command_.resize(num_nidec_brushlesses_);
	brushless_vel_.resize(num_nidec_brushlesses_);
	for (size_t i = 0; i < num_nidec_brushlesses_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering interface for : " << nidec_brushless_names_[i] << " at PWM channel " << nidec_brushless_pwm_channels_[i] << " / DIO channel " << nidec_brushless_dio_channels_[i]);

		brushless_command_[i] = 0;

		// Create state interface for the given brushless motor
		// and point it to the data stored in the
		// corresponding brushless_state array entry
		hardware_interface::JointStateHandle jsh(nidec_brushless_names_[i], &brushless_vel_[i], &brushless_vel_[i], &brushless_vel_[i]);
		joint_state_interface_.registerHandle(jsh);

		// Do the same for a command interface for
		// the same brushless motor
		hardware_interface::JointHandle jh(jsh, &brushless_command_[i]);
		joint_velocity_interface_.registerHandle(jh);
		if (!nidec_brushless_local_updates_[i])
			joint_remote_interface_.registerHandle(jh);
	}

	num_digital_inputs_ = digital_input_names_.size();
	digital_input_state_.resize(num_digital_inputs_);
	for (size_t i = 0; i < num_digital_inputs_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering interface for : " << digital_input_names_[i] << " at DIO channel " << digital_input_dio_channels_[i] << " / invert " << digital_input_inverts_[i]);
		// Create state interface for the given digital input
		// and point it to the data stored in the
		// corresponding brushless_state array entry
		hardware_interface::JointStateHandle dish(digital_input_names_[i], &digital_input_state_[i], &digital_input_state_[i], &digital_input_state_[i]);
		joint_state_interface_.registerHandle(dish);
		if (!digital_input_locals_[i])
		{
			hardware_interface::JointHandle dih(dish, &digital_input_state_[i]); /// writing directly to state?
			joint_remote_interface_.registerHandle(dih);
		}
	}

	num_digital_outputs_ = digital_output_names_.size();
	digital_output_command_.resize(num_digital_outputs_);
	digital_output_state_.resize(num_digital_outputs_);
	for (size_t i = 0; i < num_digital_outputs_; i++)
	{
		digital_output_state_[i] = std::numeric_limits<double>::max();
		digital_output_command_[i] = 0;

		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering interface for : " << digital_output_names_[i] << " at DIO channel " << digital_output_dio_channels_[i] << " / invert " << digital_output_inverts_[i]);

		hardware_interface::JointStateHandle dosh(digital_output_names_[i], &digital_output_state_[i], &digital_output_state_[i], &digital_output_state_[i]);
		joint_state_interface_.registerHandle(dosh);

		// Do the same for a command interface for
		// the digital output
		hardware_interface::JointHandle doh(dosh, &digital_output_command_[i]);
		joint_position_interface_.registerHandle(doh);
		if (!digital_output_local_updates_[i])
			joint_remote_interface_.registerHandle(doh);
	}

	num_pwms_ = pwm_names_.size();
	pwm_state_.resize(num_pwms_);
	pwm_command_.resize(num_pwms_);
	for (size_t i = 0; i < num_pwms_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering interface for : " << pwm_names_[i] << " at PWM channel " << pwm_pwm_channels_[i] << " / invert " << pwm_inverts_[i]);
		pwm_state_[i] = std::numeric_limits<double>::max();
		pwm_command_[i] = 0;

		hardware_interface::JointStateHandle psh(pwm_names_[i], &pwm_state_[i], &pwm_state_[i], &pwm_state_[i]);
		joint_state_interface_.registerHandle(psh);

		hardware_interface::JointHandle ph(psh, &pwm_command_[i]);
		joint_velocity_interface_.registerHandle(ph);
		if (!pwm_local_updates_[i])
			joint_remote_interface_.registerHandle(ph);
	}
	num_solenoids_ = solenoid_names_.size();
	solenoid_state_.resize(num_solenoids_);
	solenoid_command_.resize(num_solenoids_);
	for (size_t i = 0; i < num_solenoids_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering interface for : " << solenoid_names_[i] << " at id " << solenoid_ids_[i]<< " at pcm " << solenoid_pcms_[i]);

		solenoid_state_[i] = std::numeric_limits<double>::max();
		solenoid_command_[i] = 0;

		hardware_interface::JointStateHandle ssh(solenoid_names_[i], &solenoid_state_[i], &solenoid_state_[i], &solenoid_state_[i]);
		joint_state_interface_.registerHandle(ssh);

		hardware_interface::JointHandle soh(ssh, &solenoid_command_[i]);
		joint_position_interface_.registerHandle(soh);
		if (!solenoid_local_updates_[i])
			joint_remote_interface_.registerHandle(soh);
	}

	num_double_solenoids_ = double_solenoid_names_.size();
	double_solenoid_state_.resize(num_double_solenoids_);
	double_solenoid_command_.resize(num_double_solenoids_);
	for (size_t i = 0; i < num_double_solenoids_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering interface for : " << double_solenoid_names_[i] << " at forward id " << double_solenoid_forward_ids_[i] << " at reverse id " << double_solenoid_reverse_ids_[i] << " at pcm " << double_solenoid_pcms_[i]);

		double_solenoid_state_[i] = std::numeric_limits<double>::max();
		double_solenoid_command_[i] = 0;

		hardware_interface::JointStateHandle dssh(double_solenoid_names_[i], &double_solenoid_state_[i], &double_solenoid_state_[i], &double_solenoid_state_[i]);
		joint_state_interface_.registerHandle(dssh);

		hardware_interface::JointHandle dsoh(dssh, &double_solenoid_command_[i]);
		joint_position_interface_.registerHandle(dsoh);
		if (!double_solenoid_local_updates_[i])
			joint_remote_interface_.registerHandle(dsoh);
	}
	num_rumbles_ = rumble_names_.size();
	rumble_state_.resize(num_rumbles_);
	rumble_command_.resize(num_rumbles_);
	for (size_t i = 0; i < num_rumbles_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering interface for : " << rumble_names_[i] << " at port " << rumble_ports_[i]);

		rumble_state_[i] = std::numeric_limits<double>::max();
		rumble_command_[i] = 0;
		hardware_interface::JointStateHandle rsh(rumble_names_[i], &rumble_state_[i], &rumble_state_[i], &rumble_state_[i]);
		joint_state_interface_.registerHandle(rsh);

		hardware_interface::JointHandle rh(rsh, &rumble_command_[i]);
		joint_position_interface_.registerHandle(rh);
		if (!rumble_local_updates_[i])
			joint_remote_interface_.registerHandle(rh);
	}

	num_as726xs_ = as726x_names_.size();
	as726x_command_.resize(num_as726xs_);
	for (size_t i = 0; i < num_as726xs_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering AS726x Interface for " << as726x_names_[i]
				<< " at port " << as726x_ports_[i]
				<< " at address " << as726x_addresses_[i]);
		as726x_state_.push_back(hardware_interface::as726x::AS726xState(as726x_ports_[i], as726x_addresses_[i]));
	}
	for (size_t i = 0; i < num_as726xs_; i++)
	{
		hardware_interface::as726x::AS726xStateHandle ash(as726x_names_[i], &as726x_state_[i]);
		as726x_state_interface_.registerHandle(ash);

		hardware_interface::as726x::AS726xCommandHandle aoh(ash, &as726x_command_[i]);
		as726x_command_interface_.registerHandle(aoh);
		if (!as726x_local_updates_[i])
		{
			hardware_interface::as726x::AS726xWritableStateHandle awsh(as726x_names_[i], &as726x_state_[i]); /// writing directly to state?
			as726x_remote_state_interface_.registerHandle(awsh);
		}
	}

	// Differentiate between navX and IMU here
	// We might want more than 1 type of IMU
	// at some point - eventually allow this by making IMU
	// data sized to hold results from all IMU
	// hardware rather than just navX size
	num_navX_ = navX_names_.size();
	imu_orientations_.resize(num_navX_);
	imu_orientation_covariances_.resize(num_navX_);
	imu_angular_velocities_.resize(num_navX_);
	imu_angular_velocity_covariances_.resize(num_navX_);
	imu_linear_accelerations_.resize(num_navX_);
	imu_linear_acceleration_covariances_.resize(num_navX_);

	for (size_t i = 0; i < num_navX_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering navX interface for : " << navX_names_[i] << " at id " << navX_ids_[i]);

		// Create state interface for the given IMU
		// and point it to the data stored in the
		// corresponding imu arrays
		hardware_interface::ImuSensorHandle::Data imu_data;
		imu_data.name = navX_names_[i];
		imu_data.frame_id = navX_frame_ids_[i];
		for (size_t j = 0; j < 3; j++)
		{
			imu_orientations_[i][j] = 0;
			imu_angular_velocities_[i][j] = 0;
			imu_linear_accelerations_[i][j] = 0;
		}
		imu_orientations_[i][3] = 1;
		imu_data.orientation = &imu_orientations_[i][0];
		imu_data.orientation_covariance = &imu_orientation_covariances_[i][0];
		imu_data.angular_velocity = &imu_angular_velocities_[i][0];
		imu_data.angular_velocity_covariance = &imu_angular_velocity_covariances_[i][0];
		imu_data.linear_acceleration = &imu_linear_accelerations_[i][0];
		imu_data.linear_acceleration_covariance = &imu_linear_acceleration_covariances_[i][0];

		hardware_interface::ImuSensorHandle imuh(imu_data);
		imu_interface_.registerHandle(imuh);

		if (!navX_locals_[i])
		{
			hardware_interface::ImuWritableSensorHandle ish(imu_data);
			imu_remote_interface_.registerHandle(ish);
		}
	}

	num_analog_inputs_ = analog_input_names_.size();
	analog_input_state_.resize(num_analog_inputs_);
	for (size_t i = 0; i < num_analog_inputs_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering interface for : " << analog_input_names_[i] << " at analog channel " << analog_input_analog_channels_[i]);
		// Create state interface for the given analog input
		// and point it to the data stored in the
		// corresponding brushless_state array entry
		hardware_interface::JointStateHandle aish(analog_input_names_[i], &analog_input_state_[i], &analog_input_state_[i], &analog_input_state_[i]);
		joint_state_interface_.registerHandle(aish);
		if (!analog_input_locals_[i])
		{
			hardware_interface::JointHandle aih(aish, &analog_input_state_[i]); /// writing directly to state?
			joint_remote_interface_.registerHandle(aih);
		}
	}
	num_compressors_ = compressor_names_.size();
	compressor_state_.resize(num_compressors_);
	compressor_command_.resize(num_compressors_);
	for (size_t i = 0; i < num_compressors_; i++)
		pcm_state_.push_back(hardware_interface::PCMState(compressor_pcm_ids_[i]));
	for (size_t i = 0; i < num_compressors_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering interface for compressor / PCM : " << compressor_names_[i] << " at pcm_id " << compressor_pcm_ids_[i]);

		// Default compressors to running
		compressor_command_[i] = 1;
		compressor_state_[i] = std::numeric_limits<double>::max();

		hardware_interface::JointStateHandle csh(compressor_names_[i], &compressor_state_[i], &compressor_state_[i], &compressor_state_[i]);
		joint_state_interface_.registerHandle(csh);

		hardware_interface::JointHandle cch(csh, &compressor_command_[i]);
		joint_position_interface_.registerHandle(cch);
		if (!compressor_local_updates_[i])
			joint_remote_interface_.registerHandle(cch);

		hardware_interface::PCMStateHandle pcmsh(compressor_names_[i], &pcm_state_[i]);
		pcm_state_interface_.registerHandle(pcmsh);
		if (!compressor_local_updates_[i])
		{
			hardware_interface::PCMWritableStateHandle rpcmsh(compressor_names_[i], &pcm_state_[i]);
			pcm_remote_state_interface_.registerHandle(rpcmsh);
		}
	}

	num_pdps_ = pdp_names_.size();
	pdp_state_.resize(num_pdps_);
	for (size_t i = 0; i < num_pdps_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering interface for PDP : " << pdp_names_[i]);

		hardware_interface::PDPStateHandle csh(pdp_names_[i], &pdp_state_[i]);
		pdp_state_interface_.registerHandle(csh);
		if (!pdp_locals_[i])
		{
			hardware_interface::PDPWritableStateHandle psh(pdp_names_[i], &pdp_state_[i]);
			pdp_remote_state_interface_.registerHandle(psh);
		}
	}

	num_dummy_joints_ = dummy_joint_names_.size();
	dummy_joint_position_.resize(num_dummy_joints_);
	dummy_joint_velocity_.resize(num_dummy_joints_);
	dummy_joint_effort_.resize(num_dummy_joints_);
	dummy_joint_command_.resize(num_dummy_joints_);
	for (size_t i = 0; i < num_dummy_joints_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering interface for dummy joint : " << dummy_joint_names_[i]);

		dummy_joint_command_[i] = 0;
		dummy_joint_position_[i] = 0;
		dummy_joint_velocity_[i] = 0;
		dummy_joint_effort_[i] = 0;

		hardware_interface::JointStateHandle dsh(dummy_joint_names_[i], &dummy_joint_position_[i],&dummy_joint_velocity_[i], &dummy_joint_effort_[i]);
		joint_state_interface_.registerHandle(dsh);

		hardware_interface::JointHandle dch(dsh, &dummy_joint_command_[i]);
		joint_command_interface_.registerHandle(dch);
		joint_position_interface_.registerHandle(dch);
		joint_velocity_interface_.registerHandle(dch);
		if (!dummy_joint_locals_[i])
			joint_remote_interface_.registerHandle(dch);
	}

	num_ready_signals_ = ready_signal_names_.size();
	robot_ready_signals_.resize(num_ready_signals_);
	for (size_t i = 0; i < num_ready_signals_; i++)
	{
		// Add a flag which indicates we should signal
		// the driver station that robot code is initialized
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering interface for ready signal : " << ready_signal_names_[i]);
		hardware_interface::JointStateHandle sh(ready_signal_names_[i], &robot_ready_signals_[i],&robot_ready_signals_[i],&robot_ready_signals_[i]);
		joint_state_interface_.registerHandle(sh);

		hardware_interface::JointHandle ch(sh, &robot_ready_signals_[i]);
		joint_command_interface_.registerHandle(ch);
		joint_position_interface_.registerHandle(ch);
		joint_velocity_interface_.registerHandle(ch);
		if (!ready_signal_locals_[i])
			joint_remote_interface_.registerHandle(ch);
	}

	// TODO : Think some more on how this will work.  Previous idea of making them
	// definable joints was good as well, but required some hard coding to
	// convert from name to an actual variable. This requires hard-coding here
	// but not in the read or write code.  Not sure which is better
	auto dummy_joints = getDummyJoints();
	for (auto d : dummy_joints)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering interface for DummyVar: " << d.name_);

		*d.address_ = 0;

		hardware_interface::JointStateHandle dsh(d.name_, d.address_, d.address_, d.address_);
		joint_state_interface_.registerHandle(dsh);

		hardware_interface::JointHandle dch(dsh, d.address_);
		joint_command_interface_.registerHandle(dch);
		joint_position_interface_.registerHandle(dch);
		joint_velocity_interface_.registerHandle(dch);
		if (!run_hal_robot_)
			joint_remote_interface_.registerHandle(dch);
	}
	if (run_hal_robot_)
	{
		hardware_interface::MatchStateHandle msh("match_name", &match_data_);
		match_state_interface_.registerHandle(msh);
	}
	else
	{
		hardware_interface::MatchStateWritableHandle msh("match_name", &match_data_);
		match_remote_state_interface_.registerHandle(msh);
	}

	// TODO : add joint interface for joysticks
	num_joysticks_ = joystick_names_.size();

	if (run_hal_robot_)
	{
		hardware_interface::RobotControllerStateHandle rcsh("robot_controller_name", &robot_controller_state_);
		robot_controller_state_interface_.registerHandle(rcsh);
	}

	// Publish various FRC-specific data using generic joint state for now
	// For simple things this might be OK, but for more complex state
	// (e.g. joystick) it probably makes more sense to write a
	// RealtimePublisher() for the data coming in from
	// the DS
	registerInterface(&talon_state_interface_);
	registerInterface(&talon_command_interface_);
	registerInterface(&joint_state_interface_);
	registerInterface(&joint_command_interface_);
	registerInterface(&joint_position_interface_);
	registerInterface(&joint_velocity_interface_);
	registerInterface(&joint_effort_interface_); // empty for now
	registerInterface(&imu_interface_);
	registerInterface(&pdp_state_interface_);
	registerInterface(&pcm_state_interface_);
	registerInterface(&robot_controller_state_interface_);
	registerInterface(&match_state_interface_);
	registerInterface(&as726x_state_interface_);
	registerInterface(&as726x_command_interface_);

	registerInterface(&joint_remote_interface_); // list of Joints defined as remote
	registerInterface(&talon_remote_state_interface_);
	registerInterface(&pdp_remote_state_interface_);
	registerInterface(&pcm_remote_state_interface_);
	registerInterface(&imu_remote_interface_);
	registerInterface(&match_remote_state_interface_);
	registerInterface(&as726x_remote_state_interface_);

	ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface Ready.");
}

// Using the mode, setpoint, etc generated from a given Talon's custom profile,
// update the talon command values for that Talon. This way the rest of the
// write() command will use those values to update hardware / sim for the
// motor controller
void FRCRobotInterface::custom_profile_set_talon(hardware_interface::TalonMode mode,
												 double setpoint, double fTerm,
												 int joint_id, int pidSlot, bool zeroPos)
{
	auto &tc = talon_command_[joint_id];
	if(zeroPos)
	{
		tc.setSelectedSensorPosition(0);
		ROS_INFO_STREAM("custom_profile_set_talon zeroing talon:" <<  joint_id);
	}
	ROS_INFO_STREAM("joint_id:" << joint_id << " mode:" << mode << " setpoint: " << setpoint << " fterm: " << fTerm << " slot: " << pidSlot);

	// Set talon mode based on profile type
	if(mode == hardware_interface::TalonMode_PercentOutput)
	{
		// Percent output doesn't use feedforward
		tc.setDemand1Type(hardware_interface::DemandType_Neutral);
	}
	else
	{
		tc.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
		tc.setDemand1Value(fTerm);
	}

	tc.setMode(mode);
	tc.set(setpoint);

	tc.setPidfSlot(pidSlot);
}

// Called once per talon in each write loop.  Used to generate
// commands for that talon if it is running in custom
// motion profile mode
// TODO : see if there's a way to only zero out position once,
// and then not send pt[0]'s zero pos command for points
// interpolated from it
void FRCRobotInterface::custom_profile_write(int joint_id)
{
	// Don't run if the talon isn't local
	if (!can_ctre_mc_local_hardwares_[joint_id])
	{
		return;
	}

	auto &tc = talon_command_[joint_id];

	if (tc.getCustomProfileDisable())
	{
		return;
	}

	auto &ts = talon_state_[joint_id];
	auto &cps = custom_profile_state_[joint_id];
	auto &ps = cps.status_;

	// Grab points to hit and times to hit them from the
	// talon command buffer
	auto &prof_pts = cps.saved_points_;
	auto &prof_times = cps.saved_times_;

	tc.getCustomProfilePointsTimesChanged(prof_pts, prof_times);

	// TODO : add check for talon mode == disabled,, run, etc.
	// if so clear out getCustomProfileRun(), run, etc.
	if (ts.getTalonMode() == hardware_interface::TalonMode_Disabled)
	{
		tc.setCustomProfileRun(false);
	}

	const bool run = tc.getCustomProfileRun();

	// Clear out the current slot when profile status
	// transitions from running to stopped
	// This should also catch the case where a profile was being run
	// when the robot was disabled, because we force custom profile
	// run to false on robot disable
	if(ps.running && !run)
	{
		std::vector<hardware_interface::CustomProfilePoint> empty_points;
		tc.overwriteCustomProfilePoints(empty_points, ps.slotRunning);
		//Right now we wipe everything if the profile is stopped
		//This could be changed to a pause type feature in which the first point has zeroPos set and the other
		//positions get shifted
		cps.points_run_ = 0;
	}

	// Reset start time to now when switching from non-running to running
	if((run && !ps.running) || !run)
	{
		cps.time_start_ = ros::Time::now().toSec();
	}
	const int slot = tc.getCustomProfileSlot();

	if(slot != ps.slotRunning && run && ps.running)
	{
		ROS_WARN("transitioned between two profile slots without any break between. Intended?");
		std::vector<hardware_interface::CustomProfilePoint> empty_points;
		tc.overwriteCustomProfilePoints(empty_points, ps.slotRunning);
		//Right now we wipe everything if the slots are flipped
		//Should try to be analagous to having a break between
		cps.points_run_= 0;
		cps.time_start_= ros::Time::now().toSec();
	}
	ps.slotRunning = slot;
	// Actully run profile code for this talon
	if(run)
	{
		if(prof_pts[slot].size() == 0)
		{
			ROS_ERROR_THROTTLE(1.0, "Tried to run custom profile with no points buffered");
			//Potentially add more things to do if this exception is caught
			//Like maybe set talon to neutral mode or something
			return;
		}

		//Find the point just greater than time since start
		size_t end;
		ps.outOfPoints = true;
		const double time_since_start = ros::Time::now().toSec() - cps.time_start_;
		for(end = std::max(cps.points_run_ - 1, 0); end < prof_pts[slot].size(); end++)
		{
			if(prof_times[slot][end] > time_since_start)
			{
				ps.outOfPoints = false;
				break;
			}
		}

		// Save the current point found to run to speed up the
		// search for it next time through the loop.
		if(ps.outOfPoints)
		{
			cps.points_run_ = prof_pts[slot].size();
		}
		else
		{
			cps.points_run_ = std::max(static_cast<int>(end) - 1, 0);
		}
#if 0
		ROS_INFO_STREAM(" cps.points_run_:" << cps.points_run_
				<< " time_since_start:" << time_since_start
				<< " end:" << end
				<< " ps.outOfPoints:" << ps.outOfPoints);
#endif
		if(ps.outOfPoints)
		{
			auto next_slot = tc.getCustomProfileNextSlot();
			auto back = prof_pts[slot].back();

			//If all points have been exhausted, just use the last point
			custom_profile_set_talon(back.mode, back.setpoint, back.fTerm, joint_id, back.pidSlot, back.zeroPos);
			if (next_slot.size() > 0)
			{
				tc.setCustomProfileSlot(next_slot[0]);
				next_slot.erase(next_slot.begin());
				tc.setCustomProfileNextSlot(next_slot);
			}
		}
		else if(end == 0)
		{
			auto m = prof_pts[slot][0];
			//If we are still on the first point,just use the first point
			custom_profile_set_talon(m.mode, m.setpoint, m.fTerm, joint_id, m.pidSlot, m.zeroPos);
		}
		else
		{
			auto endp = prof_pts[slot][end];
			auto endpm1 = prof_pts[slot][end - 1];
			//Allows for mode flipping while in profile execution
			//We don't want to interpolate between positional and velocity setpoints
			if(endp.mode != endpm1.mode)
			{
				ROS_WARN_STREAM("mid profile mode flip. If intendped, Cooooooooollllll. If not, fix the code : " << endp.mode << " from " << endpm1.mode);
				custom_profile_set_talon(endp.mode, endp.setpoint, endp.fTerm, joint_id, endp.pidSlot, endp.zeroPos);
				// consider adding a check to see which is closer
			}
			else
			{
				// linear interpolation of the points this particular iteration time
				// falls between
#if 0
				ROS_INFO_STREAM("prof_pts[" << slot <<"]["<<end<<"] setpoint:" << endp.setpoint <<
						" fTerm:" << endp.fTerm);
				ROS_INFO_STREAM("prof_pts[" << slot <<"]["<<end-1<<"] setpoint:" << endpm1.setpoint <<
						" fTerm:" << endpm1.fTerm);
#endif

				const double time_percent = (time_since_start - prof_times[slot][end-1]) / (prof_times[slot][end] - prof_times[slot][end-1]);
				const double setpoint = endpm1.setpoint + (endp.setpoint - endpm1.setpoint) * time_percent;

				const double fTerm = endpm1.fTerm + (endp.fTerm - endpm1.fTerm) * time_percent;
				custom_profile_set_talon(endp.mode, setpoint, fTerm, joint_id, endp.pidSlot, endpm1.zeroPos);
			}
		}
	}
	else
	{
		ps.outOfPoints = false;
	}

	// Update talon profile status
	ps.remainingPoints.resize(prof_pts.size());

	for(size_t i = 0; i < prof_pts.size(); i++)
	{
		if(i == static_cast<size_t>(ps.slotRunning))
		{
			ps.remainingPoints[i] = tc.getCustomProfileCount(i) - cps.points_run_;
			if(tc.getCustomProfileTimeCount(i) > 0)
			{
				ps.remainingTime = tc.getCustomProfileEndTime(i) - (ros::Time::now().toSec() - cps.time_start_);
			}
			else
			{
				ps.remainingTime = 0.0;
			}
		}
		else
		{
			ps.remainingPoints[i] = tc.getCustomProfileCount(i);
		}
	}

	ps.running = run;
	ts.setCustomProfileStatus(ps);
}

void FRCRobotInterface::reset()
{
}

void FRCRobotInterface::printState()
{
	// WARNING: THIS IS NOT REALTIME SAFE
	// FOR DEBUGGING ONLY, USE AT YOUR OWN ROBOT's RISK!
	ROS_INFO_STREAM_THROTTLE(1,
							 std::endl << "State" <<
							 std::endl << printStateHelper());
}

std::string FRCRobotInterface::printStateHelper()
{
	std::stringstream ss;
	std::cout.precision(15);

	ss << "    CAN ID       position        velocity        effort" << std::endl;
	for (std::size_t i = 0; i < num_can_ctre_mcs_; ++i)
	{
		ss << "j" << i << ":    " ;
		ss << talon_state_[i].getCANID() << "\t ";
		ss << std::fixed << talon_state_[i].getPosition() << "\t ";
		ss << std::fixed << talon_state_[i].getSpeed() << "\t ";
		ss << std::fixed << talon_state_[i].getOutputVoltage() << std::endl;
	}
	return ss.str();
}

std::string FRCRobotInterface::printCommandHelper()
{
	std::stringstream ss;
	std::cout.precision(15);
	ss << "    setpoint" << std::endl;
	for (std::size_t i = 0; i < num_can_ctre_mcs_; ++i)
		ss << "j" << i << ": " << std::fixed << talon_command_[i].get() << std::endl;
	return ss.str();
}

void FRCRobotInterface::loadURDF(ros::NodeHandle &/*nh*/, std::string /*param_name*/)
{
	return;
#if 0
	std::string urdf_string;
	urdf_model_ = new urdf::Model();

	// search and wait for robot_description on param server
	while (urdf_string.empty() && ros::ok())
	{
		std::string search_param_name;
		if (nh.searchParam(param_name, search_param_name))
		{
			ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
								  nh.getNamespace() << search_param_name);
			nh.getParam(search_param_name, urdf_string);
		}
		else
		{
			ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
								  nh.getNamespace() << param_name);
			nh.getParam(param_name, urdf_string);
		}

		usleep(100000);
	}

	if (!urdf_model_->initString(urdf_string))
		ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
	else
		ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
#endif
}

}  // namespace
