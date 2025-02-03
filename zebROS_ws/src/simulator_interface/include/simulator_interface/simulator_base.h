#ifndef SIMULATOR_INTERFACE_SIMULATOR_BASE_H_
#define SIMULATOR_INTERFACE_SIMULATOR_BASE_H_
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "ctre/phoenix6/core/CoreCANcoder.hpp"
#include "ctre_interfaces/talonfxpro_state_interface.h"
#include "ros/ros.h"
#include "ctre_interfaces/talonfxpro_sim_command_interface.h"
#include "ctre_interfaces/cancoder_sim_command_interface.h"
#include <optional>

namespace simulator_base
{
  class Simulator
  {
    public:
      virtual void init(const XmlRpc::XmlRpcValue &simulator_info) {

      };
      virtual void update(const std::string &name, const ros::Time &time, const ros::Duration &period, hardware_interface::talonfxpro::TalonFXProSimCommand *talonfxpro, const hardware_interface::talonfxpro::TalonFXProHWState *state, std::optional<hardware_interface::cancoder::CANCoderSimCommand*> cancoder) {
        
      };
      virtual ~Simulator(){}
    
    protected:
      Simulator(){}
  };
};
#endif