#ifndef PDH_COMMAND_INTERFACE_INC_
#define PDH_COMMAND_INTERFACE_INC_

#include "frc_interfaces/pdh_state_interface.h"
#include "state_handle/command_handle.h"

namespace hardware_interface
{

class PDHHWCommand
{
	public:
		void setSwitchableChannelEnable(bool enable)
		{
			if (enable == switchable_channel_enable_)
				return;
			switchable_channel_enable_ = enable;
			switchable_Channel_enable_changed_ = true;
		}
		bool getSwitchableChannelEnable(void) const { return switchable_channel_enable_;}
		bool switchableChannelEnableChanged(bool &enable)
		{
			enable = switchable_channel_enable_;
			const bool ret = switchable_Channel_enable_changed_;
			switchable_Channel_enable_changed_ = false;
			return ret;
		}
		void resetSwitchableChannelEnable(void) { switchable_Channel_enable_changed_ = true; }

		void setClearStickyFaults(void) { clear_sticky_faults_ = true; }
		bool getClearStickyFaults(void) const { return clear_sticky_faults_; }
		bool clearStickyFaultsChanged(void)
		{
			if (!clear_sticky_faults_)
				return false;
			clear_sticky_faults_ = false;
			return true;
		}

		void setIdentifyPDH(void) { identify_pdh_ = true; }
		bool getIdentifyPDH(void) const { return identify_pdh_; }
		bool identifyPDHChanged(void)
		{
			if (!identify_pdh_)
				return false;
			identify_pdh_ = false;
			return true;
		}

	private:
		bool switchable_channel_enable_{false};
		bool switchable_Channel_enable_changed_{true};

		bool clear_sticky_faults_{false};
		bool identify_pdh_{false};
};

// Create a handle pointing to a type PDHHWCommand / PDHHWState pair
typedef CommandHandle<PDHHWCommand, PDHHWState, PDHStateHandle> PDHCommandHandle;

// Use ClaimResources here since we only want 1 controller
// to be able to access a given PDH at any particular time
class PDHCommandInterface : public HardwareResourceManager<PDHCommandHandle, ClaimResources> {};

} // namespace hardware_interface

#endif

