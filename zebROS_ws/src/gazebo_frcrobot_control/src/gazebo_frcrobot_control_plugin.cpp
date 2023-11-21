/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
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
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
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

/* Original Author: Dave Coleman, Jonathan Bohren
   Desc:   Gazebo plugin for ros_control that allows 'hardware_interfaces' to be plugged in
   using pluginlib
*/

#include <optional>
#include <gazebo_frcrobot_control/gazebo_frcrobot_control_plugin.h>
#include <urdf/model.h>
#include <chrono>
#include <thread>

namespace gazebo_ros_control
{

GazeboFRCRobotControlPlugin::GazeboFRCRobotControlPlugin()
  : class_loader_("gazebo_ros_control", "gazebo_ros_control::RobotHWSim")
{
}

GazeboFRCRobotControlPlugin::~GazeboFRCRobotControlPlugin()
{
  // Disconnect from gazebo events
  update_connection_.reset();
}

// Overloaded Gazebo entry point
void GazeboFRCRobotControlPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  ROS_INFO_STREAM_NAMED("gazebo_ros_control","Loading gazebo_ros_control plugin");

  // Save pointers to the model
  parent_model_ = parent;

  // Error message if the model couldn't be found
  if (!parent_model_)
  {
    ROS_ERROR_STREAM_NAMED("loadThread","parent model is NULL");
    return;
  }

  // Check that ROS has been initialized
  if(!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("gazebo_ros_control","A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Get namespace for nodehandle
  if(sdf->HasElement("robotNamespace"))
  {
    robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  }
  else
  {
    robot_namespace_ = parent_model_->GetName(); // default
  }

  // Get robot_description ROS param name
  if (sdf->HasElement("robotParam"))
  {
    robot_description_ = sdf->GetElement("robotParam")->Get<std::string>();
  }
  else
  {
    robot_description_ = "robot_description"; // default
  }

  // Get the Gazebo simulation period
#if GAZEBO_MAJOR_VERSION >= 8
  ros::Duration gazebo_period(parent_model_->GetWorld()->Physics()->GetMaxStepSize());
#else
  ros::Duration gazebo_period(parent_model_->GetWorld()->GetPhysicsEngine()->GetMaxStepSize());
#endif

  // Decide the plugin control period
  if(sdf->HasElement("controlPeriod"))
  {
    control_period_ = ros::Duration(sdf->Get<double>("controlPeriod"));

    // Check the period against the simulation period
    if( control_period_ < gazebo_period )
    {
      ROS_ERROR_STREAM_NAMED("gazebo_ros_control","Desired controller update period ("<<control_period_
        <<" s) is faster than the gazebo simulation period ("<<gazebo_period<<" s).");
    }
    else if( control_period_ > gazebo_period )
    {
      ROS_WARN_STREAM_NAMED("gazebo_ros_control","Desired controller update period ("<<control_period_
        <<" s) is slower than the gazebo simulation period ("<<gazebo_period<<" s).");
    }
  }
  else
  {
    control_period_ = gazebo_period;
    ROS_DEBUG_STREAM_NAMED("gazebo_ros_control","Control period not found in URDF/SDF, defaulting to Gazebo period of "
      << control_period_);
  }

  // Get parameters/settings for controllers from ROS param server
  model_nh_ = ros::NodeHandle(robot_namespace_);

  // Initialize the emergency stop code.
  e_stop_active_ = false;
  last_e_stop_active_ = false;
  if (sdf->HasElement("eStopTopic"))
  {
    const std::string e_stop_topic = sdf->GetElement("eStopTopic")->Get<std::string>();
    e_stop_sub_ = model_nh_.subscribe(e_stop_topic, 1, &GazeboFRCRobotControlPlugin::eStopCB, this);
  }

  ROS_INFO_NAMED("gazebo_ros_control", "Starting gazebo_ros_control plugin in namespace: %s", robot_namespace_.c_str());

  // Read urdf from ros parameter server then
  // setup actuators and mechanism control node.
  // This call will block if ROS is not properly initialized.
  const std::string urdf_string = getURDF(robot_description_);
  if (!parseTransmissionsFromURDF(urdf_string))
  {
    ROS_ERROR_NAMED("gazebo_ros_control", "Error parsing URDF in gazebo_ros_control plugin, plugin not active.\n");
    return;
  }

  std::vector<std::string> robots;
  const std::string param_name = "robot_hardware";
  if (!model_nh_.getParam(param_name, robots))
  {
    ROS_ERROR("Param %s not in %s - can't read list of RobotHWSim interfaces",
			param_name.c_str(), model_nh_.getNamespace().c_str());
    return;
  }

  urdf::Model urdf_model;
  const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

  for (const auto &name: robots)
  {
    ROS_DEBUG("Will load robot HW '%s'", name.c_str());

    // Load the RobotHWSim abstraction to interface the controllers with the gazebo model
    try
    {

      // Create local node handle in namespace of this simulated hardware
      ros::NodeHandle c_nh;
      try
      {
        c_nh = ros::NodeHandle(model_nh_, name);
      }
      catch (std::exception const& e)
      {
        ROS_ERROR("Exception thrown while constructing nodehandle for robot HW with name '%s':\n%s", name.c_str(),
                  e.what());
        return;
      }

      // Get class type of this hardware from param
      std::string type;
      if (!c_nh.getParam("type", type))
      {
        ROS_ERROR("Could not load robot HW '%s' because the type was not specified. Did you load the robot HW "
                  "configuration on the parameter server (namespace: '%s')?",
                  name.c_str(), c_nh.getNamespace().c_str());
        return;
      }

      // Load in plugin for this type
      boost::shared_ptr<RobotHWSim> robot_hw;
      try
      {
        robot_hw = class_loader_.createInstance(type);
      }
      catch (const pluginlib::PluginlibException& ex)
      {
        ROS_ERROR("Could not load class %s: %s", type.c_str(), ex.what());
        return;
      }

      // Initializes the robot HW
      ROS_DEBUG("Initializing robot HW '%s'", name.c_str());
      try
      {
        if (!robot_hw->initSim(robot_namespace_, c_nh, parent_model_, urdf_model_ptr, transmissions_))
		{
			ROS_FATAL_NAMED("gazebo_ros_control","Could not initialize robot simulation interface %s", name.c_str());
			return;
		}
      }
      catch (std::exception& e)
      {
        ROS_ERROR("Exception thrown while initializing robot HW %s.\n%s", name.c_str(), e.what());
        return;
      }

      // Register hardware and add to vector
      robot_hw_sims_.push_back(robot_hw);

      // Create the controller manager
      ROS_DEBUG_STREAM_NAMED("ros_control_plugin","Loading controller_manager");
      controller_managers_.push_back(
			  std::make_shared<controller_manager::ControllerManager>(robot_hw.get(), c_nh));

    }
    catch(pluginlib::LibraryLoadException &ex)
    {
      ROS_FATAL_STREAM_NAMED("gazebo_ros_control","Failed to create robot simulation interface loader: "
			  << name << " : "<< ex.what());
    }
  }

  // Listen to the update event. This event is broadcast every simulation iteration.
  update_connection_ =
    gazebo::event::Events::ConnectWorldUpdateBegin
    (std::bind(&GazeboFRCRobotControlPlugin::Update, this));

  ROS_INFO_NAMED("gazebo_ros_control", "Loaded gazebo_ros_control.");
}

// Called by the world update start event
void GazeboFRCRobotControlPlugin::Update()
{
  // Get the simulation time and period
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::common::Time gz_time_now = parent_model_->GetWorld()->SimTime();
#else
  gazebo::common::Time gz_time_now = parent_model_->GetWorld()->GetSimTime();
#endif
  ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
  // TODO : use periodic iterval counter
  ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  robot_hw_sims_[0]->eStopActive(e_stop_active_);

  // Check if we should update the controllers
  if (sim_period >= control_period_)
  {
    // Store this simulation time
    last_update_sim_time_ros_ = sim_time_ros;

    // Update the robot simulation with the state of the gazebo model
    for (const auto &r : robot_hw_sims_)
      r->readSim(sim_time_ros, sim_period);

    // Compute the controller commands
    bool reset_ctrlrs;
    if (e_stop_active_)
    {
      reset_ctrlrs = false;
      last_e_stop_active_ = true;
    }
    else
    {
      if (last_e_stop_active_)
      {
        reset_ctrlrs = true;
        last_e_stop_active_ = false;
      }
      else
      {
        reset_ctrlrs = false;
      }
    }
    for (const auto &c : controller_managers_)
      c->update(sim_time_ros, sim_period, reset_ctrlrs);
  }

  // Update the gazebo model with the result of the controller computation
  // TODO : should this be run each Update, or only inside the previous
  //        if () block checking if enough time has elapsed
  for (const auto &r : robot_hw_sims_)
    r->writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);
  last_write_sim_time_ros_ = sim_time_ros;
}

// Called on world reset
void GazeboFRCRobotControlPlugin::Reset()
{
  // Reset timing variables to not pass negative update periods to controllers on world reset
  last_update_sim_time_ros_ = ros::Time();
  last_write_sim_time_ros_ = ros::Time();
}

// Get the URDF XML from the parameter server
std::string GazeboFRCRobotControlPlugin::getURDF(std::string param_name) const
{
  std::string urdf_string;

  // search and wait for robot_description on param server
  while (urdf_string.empty())
  {
    std::string search_param_name;
    if (model_nh_.searchParam(param_name, search_param_name))
    {
      ROS_INFO_ONCE_NAMED("gazebo_ros_control", "gazebo_ros_control plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

      model_nh_.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_ONCE_NAMED("gazebo_ros_control", "gazebo_ros_control plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", robot_description_.c_str());

      model_nh_.getParam(param_name, urdf_string);
    }

    std::this_thread::sleep_for(std::chrono::microseconds(100000));
  }
  ROS_DEBUG_STREAM_NAMED("gazebo_ros_control", "Recieved urdf from param server, parsing...");

  return urdf_string;
}

// Get Transmissions from the URDF
bool GazeboFRCRobotControlPlugin::parseTransmissionsFromURDF(const std::string& urdf_string)
{
  transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);
  return true;
}

// Emergency stop callback
void GazeboFRCRobotControlPlugin::eStopCB(const std_msgs::BoolConstPtr& e_stop_active)
{
  e_stop_active_ = e_stop_active->data;
}


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboFRCRobotControlPlugin);
} // namespace
