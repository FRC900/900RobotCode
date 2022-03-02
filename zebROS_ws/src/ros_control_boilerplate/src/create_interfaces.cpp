#include <ros_control_boilerplate/frc_robot_interface.h>

namespace ros_control_boilerplate
{

void FRCRobotInterface::createInterfaces(void)
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
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering Talon Interface for : " << can_ctre_mc_names_[i] << " at hw ID " << can_ctre_mc_can_ids_[i]);

		// Add this controller to the list of tracked TalonHWState objects
		talon_state_.emplace_back(hardware_interface::TalonHWState(can_ctre_mc_can_ids_[i]));
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
		custom_profile_state_.emplace_back(CustomProfileState());
	}

	num_canifiers_ = canifier_names_.size();
	// Create vectors of the correct size for
	// canifier HW state and commands
	canifier_command_.resize(num_canifiers_);

	for (size_t i = 0; i < num_canifiers_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering CANifier Interface for : " << canifier_names_[i] << " at hw ID " << canifier_can_ids_[i]);
		canifier_state_.emplace_back(hardware_interface::canifier::CANifierHWState(canifier_can_ids_[i]));
	}
	for (size_t i = 0; i < num_canifiers_; i++)
	{
		// Create state interface for the given CANifier
		// and point it to the data stored in the
		// corresponding canifier_state array entry
		hardware_interface::canifier::CANifierStateHandle csh(canifier_names_[i], &canifier_state_[i]);
		canifier_state_interface_.registerHandle(csh);

		// Do the same for a command interface for
		// the same CANifier
		hardware_interface::canifier::CANifierCommandHandle cch(csh, &canifier_command_[i]);
		canifier_command_interface_.registerHandle(cch);
		if (!canifier_local_updates_[i])
		{
			hardware_interface::canifier::CANifierWritableStateHandle cwsh(canifier_names_[i], &canifier_state_[i]); /// writing directly to state?
			canifier_remote_state_interface_.registerHandle(cwsh);
		}
	}

	num_cancoders_ = cancoder_names_.size();
	// Create vectors of the correct size for
	// cancoder HW state and commands
	cancoder_command_.resize(num_cancoders_);

	for (size_t i = 0; i < num_cancoders_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering CANCoder Interface for : " << cancoder_names_[i] << " at hw ID " << cancoder_can_ids_[i]);
		cancoder_state_.emplace_back(hardware_interface::cancoder::CANCoderHWState(cancoder_can_ids_[i]));
	}
	for (size_t i = 0; i < num_cancoders_; i++)
	{
		// Create state interface for the given CANCoder
		// and point it to the data stored in the
		// corresponding cancoder_state array entry
		hardware_interface::cancoder::CANCoderStateHandle csh(cancoder_names_[i], &cancoder_state_[i]);
		cancoder_state_interface_.registerHandle(csh);

		// Do the same for a command interface for
		// the same CANCoder
		hardware_interface::cancoder::CANCoderCommandHandle cch(csh, &cancoder_command_[i]);
		cancoder_command_interface_.registerHandle(cch);
		if (!cancoder_local_updates_[i])
		{
			hardware_interface::cancoder::CANCoderWritableStateHandle cwsh(cancoder_names_[i], &cancoder_state_[i]); /// writing directly to state?
			cancoder_remote_state_interface_.registerHandle(cwsh);
		}
	}

	num_spark_maxs_ = spark_max_names_.size();
	// Create vectors of the correct size for
	// Spark Max HW state and commands
	spark_max_command_.resize(num_spark_maxs_);

	for (size_t i = 0; i < num_spark_maxs_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering Spark Max Interface for " << spark_max_names_[i] << " at hw ID " << spark_max_can_ids_[i]);

		// Create joint state interface
		spark_max_state_.push_back(hardware_interface::SparkMaxHWState(spark_max_can_ids_[i], spark_max_motor_types_[i]));
	}
	for (size_t i = 0; i < num_spark_maxs_; i++)
	{
		// Create state interface for the given Talon
		// and point it to the data stored in the
		// corresponding spark_max_state array entry
		hardware_interface::SparkMaxStateHandle smsh(spark_max_names_[i], &spark_max_state_[i]);
		spark_max_state_interface_.registerHandle(smsh);

		// Do the same for a command interface for
		// the same spark_max
		hardware_interface::SparkMaxCommandHandle smch(smsh, &spark_max_command_[i]);
		spark_max_command_interface_.registerHandle(smch);
		if (!spark_max_local_updates_[i])
		{
			hardware_interface::SparkMaxWritableStateHandle smwsh(spark_max_names_[i], &spark_max_state_[i]); /// writing directly to state?
			spark_max_remote_state_interface_.registerHandle(smwsh);
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
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering NIDEC interface for : " << nidec_brushless_names_[i] << " at PWM channel " << nidec_brushless_pwm_channels_[i] << " / DIO channel " << nidec_brushless_dio_channels_[i]);

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
	solenoid_pwm_state_.resize(num_solenoids_);
	solenoid_command_.resize(num_solenoids_);
	solenoid_mode_.resize(num_solenoids_);
	prev_solenoid_mode_.resize(num_solenoids_);
	for (size_t i = 0; i < num_solenoids_; i++)
	{
		std::stringstream s;
		s <<  "FRCRobotInterface: Registering interface for : " << solenoid_names_[i];
		if (solenoid_local_hardwares_[i])
		{
			s << " at channel " << solenoid_channels_[i] <<
				" at " << (solenoid_module_types_[i] == frc::PneumaticsModuleType::CTREPCM ? "ctrepcm" : "revph") <<
				" id " << solenoid_module_ids_[i];
		}
		else
		{
			s << " on remote hardware";
		}
		ROS_INFO_STREAM_NAMED(name_, s.str());

		solenoid_state_[i] = std::numeric_limits<double>::max();
		solenoid_pwm_state_[i] = 0;
		solenoid_command_[i] = 0;
		solenoid_mode_[i] = hardware_interface::JointCommandModes::MODE_POSITION;
		prev_solenoid_mode_[i] = hardware_interface::JointCommandModes::BEGIN;

		hardware_interface::JointStateHandle ssh(solenoid_names_[i], &solenoid_state_[i], &solenoid_state_[i], &solenoid_pwm_state_[i]);
		joint_state_interface_.registerHandle(ssh);

		hardware_interface::JointHandle sch(ssh, &solenoid_command_[i]);
		joint_position_interface_.registerHandle(sch);
		if (!solenoid_local_updates_[i])
			joint_remote_interface_.registerHandle(sch);

		hardware_interface::JointModeHandle smh(solenoid_names_[i], &solenoid_mode_[i]);
		joint_mode_interface_.registerHandle(smh);
		if (!solenoid_local_updates_[i])
			joint_mode_remote_interface_.registerHandle(smh);
	}

	num_double_solenoids_ = double_solenoid_names_.size();
	double_solenoid_state_.resize(num_double_solenoids_);
	double_solenoid_command_.resize(num_double_solenoids_);
	for (size_t i = 0; i < num_double_solenoids_; i++)
	{
		std::stringstream s;
		s << "FRCRobotInterface: Registering interface for : " << double_solenoid_names_[i];
		if (double_solenoid_local_hardwares_[i])
		{
			s << " at forward channel " << double_solenoid_forward_channels_[i] <<
				" & reverse channel " << double_solenoid_reverse_channels_[i] <<
				" at " << (double_solenoid_module_types_[i] == frc::PneumaticsModuleType::CTREPCM ? "ctrepcm" : "revph") <<
				" id " << double_solenoid_module_ids_[i];
		}
		else
		{
			s << " on remote hardware";
		}
		ROS_INFO_STREAM_NAMED(name_, s.str());

		double_solenoid_state_[i] = std::numeric_limits<double>::max();
		double_solenoid_command_[i] = 0;

		hardware_interface::JointStateHandle dssh(double_solenoid_names_[i], &double_solenoid_state_[i], &double_solenoid_state_[i], &double_solenoid_state_[i]);
		joint_state_interface_.registerHandle(dssh);

		hardware_interface::JointHandle dsch(dssh, &double_solenoid_command_[i]);
		joint_position_interface_.registerHandle(dsch);
		if (!double_solenoid_local_updates_[i])
			joint_remote_interface_.registerHandle(dsch);
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
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering AS726x Interface for : " << as726x_names_[i]
				<< " at port " << as726x_ports_[i]
				<< " at address " << as726x_addresses_[i]);
		as726x_state_.emplace_back(hardware_interface::as726x::AS726xState(as726x_ports_[i], as726x_addresses_[i]));
	}
	for (size_t i = 0; i < num_as726xs_; i++)
	{
		hardware_interface::as726x::AS726xStateHandle ash(as726x_names_[i], &as726x_state_[i]);
		as726x_state_interface_.registerHandle(ash);

		hardware_interface::as726x::AS726xCommandHandle ach(ash, &as726x_command_[i]);
		as726x_command_interface_.registerHandle(ach);
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
	num_pcms_ = pcm_names_.size();
	pcm_compressor_closed_loop_enable_state_.resize(num_pcms_);
	pcm_compressor_closed_loop_enable_command_.resize(num_pcms_);
	for (size_t i = 0; i < num_pcms_; i++)
		pcm_state_.emplace_back(hardware_interface::PCMState(pcm_ids_[i]));
	for (size_t i = 0; i < num_pcms_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering interface for PCM : "
				<< pcm_names_[i] << " at pcm_id " << pcm_ids_[i]);

		// Default compressors to running and set state to a different
		// value to force a write to HW the first write() iteration
		pcm_compressor_closed_loop_enable_command_[i] = 1;
		pcm_compressor_closed_loop_enable_state_[i] = std::numeric_limits<double>::max();

		hardware_interface::JointStateHandle csh(pcm_names_[i],
												&pcm_compressor_closed_loop_enable_state_[i],
												&pcm_compressor_closed_loop_enable_state_[i],
												&pcm_compressor_closed_loop_enable_state_[i]);

		hardware_interface::JointHandle cch(csh, &pcm_compressor_closed_loop_enable_command_[i]);
		joint_position_interface_.registerHandle(cch);
		if (!pcm_local_updates_[i])
			joint_remote_interface_.registerHandle(cch);

		hardware_interface::PCMStateHandle pcmsh(pcm_names_[i], &pcm_state_[i]);
		pcm_state_interface_.registerHandle(pcmsh);
		if (!pcm_local_updates_[i])
		{
			hardware_interface::PCMWritableStateHandle rpcmsh(pcm_names_[i], &pcm_state_[i]);
			pcm_remote_state_interface_.registerHandle(rpcmsh);
		}
	}

	num_phs_ = ph_names_.size();
	for (size_t i = 0; i < num_phs_; i++)
		ph_state_.emplace_back(hardware_interface::PHHWState(ph_ids_[i]));
	ph_command_.resize(num_phs_);
	for (size_t i = 0; i < num_phs_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering interface for PH : "
				<< ph_names_[i] << " at ph_id " << ph_ids_[i]);

		hardware_interface::PHStateHandle phsh(ph_names_[i], &ph_state_[i]);
		ph_state_interface_.registerHandle(phsh);

		hardware_interface::PHCommandHandle phch(phsh, &ph_command_[i]);
		ph_command_interface_.registerHandle(phch);

		if (!ph_local_updates_[i])
		{
			hardware_interface::PHWritableStateHandle pwsh(ph_names_[i], &ph_state_[i]);
			ph_remote_state_interface_.registerHandle(pwsh);
		}
	}

	num_pdhs_ = pdh_names_.size();
	for (size_t i = 0; i < num_pdhs_; i++)
		pdh_state_.emplace_back(hardware_interface::PDHHWState(pdh_modules_[i]));
	pdh_command_.resize(num_pdhs_);
	for (size_t i = 0; i < num_pdhs_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering interface for PDH : "
				<< pdh_names_[i] << " at module ID " << pdh_modules_[i]);
		hardware_interface::PDHStateHandle psh(pdh_names_[i], &pdh_state_[i]);
		pdh_state_interface_.registerHandle(psh);

		hardware_interface::PDHCommandHandle pch(psh, &pdh_command_[i]);
		pdh_command_interface_.registerHandle(pch);
		if (!pdh_local_updates_[i])
		{
			hardware_interface::PDHWritableStateHandle pwsh(pdh_names_[i], &pdh_state_[i]);
			pdh_remote_state_interface_.registerHandle(pwsh);
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
	for (const auto &d : dummy_joints)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering interface for DummyVar : " << d.name_);

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
		ROS_INFO_NAMED(name_, "FRCRobotInterface: Registering match data interface");
		hardware_interface::MatchStateHandle msh("match_data", &match_data_);
		match_state_interface_.registerHandle(msh);
	}
	else
	{
		ROS_INFO_NAMED(name_, "FRCRobotInterface: Registering remote match data interface");
		hardware_interface::MatchStateWritableHandle msh("match_data", &match_data_);
		match_remote_state_interface_.registerHandle(msh);
	}

	num_joysticks_ = joystick_names_.size();
	for (size_t i = 0; i < num_joysticks_; i++)
	{
		joystick_state_.push_back(hardware_interface::JoystickState(joystick_names_[i].c_str(), joystick_ids_[i]));
	}
	for (size_t i = 0; i < num_joysticks_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering Joystick interface for : " << joystick_names_[i] << " at hw ID " << joystick_ids_[i]);

		// Create state interface for the given orchestra
		// and point it to the data stored in the
		// corresponding orchestra_state array entry
		hardware_interface::JoystickStateHandle jsh(joystick_names_[i], &joystick_state_[i]);
		joystick_state_interface_.registerHandle(jsh);
	}

	if (run_hal_robot_)
	{
		hardware_interface::RobotControllerStateHandle rcsh("robot_controller_name", &robot_controller_state_);
		robot_controller_state_interface_.registerHandle(rcsh);
	}

	num_talon_orchestras_ = talon_orchestra_names_.size();
	orchestra_command_.resize(num_talon_orchestras_);

	for (size_t i = 0; i < num_talon_orchestras_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface: Registering Orchestra Interface for : " << talon_orchestra_names_[i] << " at hw ID " << talon_orchestra_ids_[i]);

		// Create orchestra state interface
		orchestra_state_.emplace_back(hardware_interface::OrchestraState(talon_orchestra_ids_[i]));
	}
	for (size_t i = 0; i < num_talon_orchestras_; i++)
	{
		// Create state interface for the given orchestra
		// and point it to the data stored in the
		// corresponding orchestra_state array entry
		hardware_interface::OrchestraStateHandle osh(talon_orchestra_names_[i], &orchestra_state_[i]);
		talon_orchestra_state_interface_.registerHandle(osh);

		// Do the same for a command interface for
		// the same orchestra
		hardware_interface::OrchestraCommandHandle och(osh, &orchestra_command_[i]);
		talon_orchestra_command_interface_.registerHandle(och);
	}

	// Publish various FRC-specific data using generic joint state for now
	// For simple things this might be OK, but for more complex state
	// (e.g. joystick) it probably makes more sense to write a
	// RealtimePublisher() for the data coming in from
	// the DS
	registerInterface(&talon_state_interface_);
	registerInterface(&talon_command_interface_);
	registerInterface(&canifier_state_interface_);
	registerInterface(&canifier_command_interface_);
	registerInterface(&cancoder_state_interface_);
	registerInterface(&cancoder_command_interface_);
	registerInterface(&spark_max_state_interface_);
	registerInterface(&spark_max_command_interface_);
	registerInterface(&joint_state_interface_);
	registerInterface(&joint_command_interface_);
	registerInterface(&joint_position_interface_);
	registerInterface(&joint_velocity_interface_);
	registerInterface(&joint_effort_interface_); // empty for now
	registerInterface(&imu_interface_);
	registerInterface(&pcm_state_interface_);
	registerInterface(&pdh_state_interface_);
	registerInterface(&pdh_command_interface_);
	registerInterface(&pdp_state_interface_);
	registerInterface(&ph_state_interface_);
	registerInterface(&robot_controller_state_interface_);
	registerInterface(&joystick_state_interface_);
	registerInterface(&match_state_interface_);
	registerInterface(&as726x_state_interface_);
	registerInterface(&as726x_command_interface_);
	registerInterface(&talon_orchestra_state_interface_);
	registerInterface(&talon_orchestra_command_interface_);

	registerInterface(&talon_remote_state_interface_);
	registerInterface(&canifier_remote_state_interface_);
	registerInterface(&cancoder_remote_state_interface_);
	registerInterface(&spark_max_remote_state_interface_);
	registerInterface(&joint_remote_interface_); // list of Joints defined as remote
	registerInterface(&pcm_remote_state_interface_);
	registerInterface(&pdh_remote_state_interface_);
	registerInterface(&pdp_remote_state_interface_);
	registerInterface(&ph_remote_state_interface_);
	registerInterface(&imu_remote_interface_);
	registerInterface(&match_remote_state_interface_);
	registerInterface(&as726x_remote_state_interface_);

	registerInterface(&joint_mode_interface_);
	registerInterface(&joint_mode_remote_interface_);
}

} // namespace
