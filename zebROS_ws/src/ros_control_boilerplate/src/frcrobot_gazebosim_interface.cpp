#include "ros_control_boilerplate/frcrobot_gazebosim_interface.h"

namespace frcrobot_control
{
	bool FRCRobotGazeboSimInterface::initSim(const std::string &robot_namespace,
											 ros::NodeHandle model_nh,
											 gazebo::physics::ModelPtr parent_model,
											 const urdf::Model *const urdf_model,
											 const std::vector<transmission_interface::TransmissionInfo> &transmissions)
	{
		ros::NodeHandle not_used;
		frcrobot_sim_interface_.init(model_nh, not_used);
		if (!frcrobot_sim_interface_.gazeboSimInit(model_nh, parent_model))
		{
			return false;
		}
		e_stop_active_ = false;
		last_e_stop_active_ = false;

#if 0

		for (size_t i = 0; i < num_solenoids_; i++)
		{
			ROS_INFO_STREAM_NAMED("frcrobot_gazebosim_interface",
								  "Connecting to gazebo : solenoid joint " << i << "=" << solenoid_names_[i] << (solenoid_local_updates_[i] ? " local" : " remote") << " update, " << (solenoid_local_hardwares_[i] ? "local" : "remote") << " hardware "
																		   << " as Solenoid " << solenoid_ids_[i]);

			if (solenoid_local_hardwares_[i])
			{
				gazebo::physics::JointPtr joint = parent_model->GetJoint(solenoid_names_[i]);
				if (!joint)
				{
					ROS_WARN_STREAM_NAMED("frcrobot_gazebosim_interface", "This robot has a joint named \"" << solenoid_names_[i]
																											<< "\" which is not in the gazebo model.");
				}
				sim_joints_solenoids_.push_back(joint);
			}
			else
			{
				ROS_INFO_STREAM("   skipping non-local hardware");
				sim_joints_solenoids_.push_back(nullptr);
			}
		}
		for (size_t i = 0; i < num_double_solenoids_; i++)
		{
			ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
								  "Connecting to gazebo : double solenoid " << i << "=" << double_solenoid_names_[i] << (double_solenoid_local_updates_[i] ? " local" : " remote") << " update, " << (double_solenoid_local_hardwares_[i] ? "local" : "remote") << " hardware "
																			<< " as Double Solenoid forward " << double_solenoid_forward_ids_[i] << " reverse " << double_solenoid_reverse_ids_[i]);

			if (double_solenoid_local_hardwares_[i])
			{
				gazebo::physics::JointPtr joint = parent_model->GetJoint(double_solenoid_names_[i]);
				if (!joint)
				{
					ROS_WARN_STREAM_NAMED("frcrobot_gazebosim_interface", "This robot has a joint named \"" << double_solenoid_names_[i]
																											<< "\" which is not in the gazebo model.");
				}
				sim_joints_double_solenoids_.push_back(joint);
			}
			else
			{
				ROS_INFO_STREAM("   skipping non-local hardware");
				sim_joints_double_solenoids_.push_back(nullptr);
			}
		}
#endif

		// get physics engine type
#if GAZEBO_MAJOR_VERSION >= 8
		gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();
#else
		gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->GetPhysicsEngine();
#endif
		physics_type_ = physics->GetType();
		if (physics_type_.empty())
		{
			ROS_WARN_STREAM_NAMED("frcrobot_gazebosim_interface", "No physics type found.");
		}

		registerInterfaceManager(&frcrobot_sim_interface_);

		ROS_INFO_STREAM("FRCRobotGazeboSim Ready on " << model_nh.getNamespace());

		return true;
	}

	// Called at the start of each gazebo update callback
	// Use this to 
	//  - explicitiy call the base FRCRobotWimInterface read()
	//    method. Since this code takes the place of the 
	//    main loop in the other sim code, if we don't call
	//    ::read here, it won't get called ... and none of the
	//    device states will ever get updated
	//  - read current position and velocity from gazebo, update
	//    the CTRE motor control sim with those sensor updates
	//  - get the voltage from the CTRE motor sim, use it to calculate
	//    a torque value to apply to each simulated motor in gazebo
	void FRCRobotGazeboSimInterface::readSim(ros::Time time, ros::Duration period)
	{
		frcrobot_sim_interface_.read(time, period);
		#if 0 // this might not be needed if we can cram everything into read()
		for (auto &d : devices_)
		{
			d->gazeboSimRead(time, period, *read_tracer_);
		}
		#endif
	}

	// Called at the end of each gazebo update, after all the controllers'
	// update methods are run
	// Use this to 
	// - explicitly call the base class write code, which will copy from
	//   each device's command buffer into the simulated harware for that device
	// And I think that's it.  The gazebo updates should happen in readSim, above.
	void FRCRobotGazeboSimInterface::writeSim(ros::Time time, ros::Duration period)
	{
		frcrobot_sim_interface_.write(time, period);
#if 0
		for (auto &d : devices_)
		{
			d->gazeboSimWrite(time, period, *read_tracer_, e_stop_active_);
		}
		for (size_t i = 0; i < num_solenoids_; i++)
		{
			if (sim_joints_solenoids_[i] && solenoid_local_hardwares_[i])
			{
				const double sign = solenoid_state_[i] ? 1.0 : -1.0;
				sim_joints_solenoids_[i]->SetForce(0, sign * 5.0);
			}
		}
		for (size_t i = 0; i < num_double_solenoids_; i++)
		{
			if (sim_joints_double_solenoids_[i] && double_solenoid_local_hardwares_[i])
			{
				const double sign = double_solenoid_state_[i] ? 1.0 : -1.0;
				sim_joints_double_solenoids_[i]->SetForce(0, sign * 5.0);
			}
		}
#endif
	}
} // namespace

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(frcrobot_control::FRCRobotGazeboSimInterface, gazebo_ros_control::RobotHWSim)