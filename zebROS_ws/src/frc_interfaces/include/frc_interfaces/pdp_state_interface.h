#ifndef PDP_STATE_INTERFACE_INC_
#define PDP_STATE_INTERFACE_INC_

#include <array>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>

namespace hardware_interface
{

//holds data from the PDP
class PDPHWState
{
	public:
		//access and set
		double getVoltage(void) const		{return voltage_;}
		double getTemperature(void) const	{return temperature_;}
		double getTotalCurrent(void) const	{return total_current_;}
		double getTotalPower(void) const	{return total_power_;}
		double getTotalEnergy(void) const	{return total_energy_;}
		double getCurrent(size_t channel) const {
			if(channel >= current_.size())
			{
				ROS_WARN_STREAM("Invalid channel. Cannot read current.");
				return 0.0;
			}
			return current_[channel];
		}

		void setVoltage(double voltage)				{voltage_ = voltage;}
		void setTemperature(double temperature)		{temperature_ = temperature;}
		void setTotalCurrent(double total_current)	{total_current_ = total_current;}
		void setTotalPower(double total_power)		{total_power_ = total_power;}
		void setTotalEnergy(double total_energy)	{total_energy_ = total_energy;}
		void setCurrent(double current, size_t channel) {
			if(channel >= current_.size())
			{
				ROS_WARN_STREAM("Invalid channel. Cannot set current.");
				return;
			}
			current_[channel] = current;
		}

	private:
		double voltage_{0.};
		double temperature_{0.};
		double total_current_{0.};
		double total_power_{0.};
		double total_energy_{0.};
		std::array<double, 16> current_{0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.};
};

using PDPStateHandle = StateHandle<const PDPHWState>;
using PDPWritableStateHandle = StateHandle<PDPHWState>;

class PDPStateInterface       : public HardwareResourceManager<PDPStateHandle> {};
class RemotePDPStateInterface : public HardwareResourceManager<PDPWritableStateHandle, ClaimResources> {};
}
#endif
