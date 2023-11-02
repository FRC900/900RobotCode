#ifndef PH_COMMAND_INTERFACE_INC_
#define PH_COMMAND_INTERFACE_INC_

#include "frc_interfaces/ph_state_interface.h"
#include "state_handle/command_handle.h"

namespace hardware_interface
{

class PHHWCommand
{
	public:
		void setCompressorMinAnalogVoltage(double compressor_min_analog_voltage)
		{
			if (compressor_min_analog_voltage == compressor_max_analog_voltage_)
				return;
			compressor_max_analog_voltage_ = compressor_min_analog_voltage;
			closed_loop_control_changed_ = true;
		}
		double getCompressorMinAnalogVoltage(void) const { return compressor_min_analog_voltage_;}

		void setCompressorMaxAnalogVoltage(double compressor_max_analog_voltage)
		{
			if (compressor_max_analog_voltage == compressor_max_analog_voltage_)
				return;
			compressor_max_analog_voltage_ = compressor_max_analog_voltage;
			closed_loop_control_changed_ = true;
		}
		double getCompressorMaxAnalogVoltage(void) const { return compressor_max_analog_voltage_;}

		void setCompressorForceDisable(bool compressor_force_disable)
		{
			if (compressor_force_disable == compressor_force_disable_)
				return;
			compressor_force_disable_ = compressor_force_disable;
			closed_loop_control_changed_ = true;
		}
		bool getCompressorForceDisable(void) const {return compressor_force_disable_;}

		void setCompressorUseDigital(bool compressor_use_digital)
		{
			if (compressor_use_digital == compressor_use_digital_)
				return;
			compressor_use_digital_ = compressor_use_digital;
			closed_loop_control_changed_ = true;
		}
		bool getCompressorUseDigital(void) const {return compressor_use_digital_;}

		bool closedLoopControlChanged(double &compressor_min_analog_voltage,
				double &compressor_max_analog_voltage,
				bool &compressor_force_disable,
				bool &compressor_use_digital)
		{
			compressor_min_analog_voltage = compressor_min_analog_voltage_;
			compressor_max_analog_voltage = compressor_max_analog_voltage_;
			compressor_force_disable = compressor_force_disable_;
			compressor_use_digital = compressor_use_digital_;
			const bool ret = closed_loop_control_changed_;
			closed_loop_control_changed_ = false;
			return ret;
		}
		void resetClosedLoopControl(void)
		{
			closed_loop_control_changed_ = true;
		}

	private:
		double                compressor_min_analog_voltage_{0.};
		double                compressor_max_analog_voltage_{13.5};
		bool                  compressor_force_disable_{false};
		bool                  compressor_use_digital_{true};
		bool                  closed_loop_control_changed_{true};
};

// Create a handle pointing to a type PHHWCommand / PHHWState pair
using PHCommandHandle = CommandHandle<PHHWCommand, PHHWState, PHStateHandle>;

// Use ClaimResources here since we only want 1 controller
// to be able to access a given PH at any particular time
class PHCommandInterface : public HardwareResourceManager<PHCommandHandle, ClaimResources> {};

} // namespace hardware_interface

#endif

