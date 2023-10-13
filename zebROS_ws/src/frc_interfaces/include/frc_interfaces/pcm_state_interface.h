#ifndef PCM_STATE_INTERFACE_INC__
#define PCM_STATE_INTERFACE_INC__

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>

namespace hardware_interface
{

class PCMState
{
	public:
		PCMState(int32_t id)
			: id_(id)
		{
		}

		int32_t  getId(void)                   const { return id_; }
		bool     getCompressorEnabled(void)    const { return compressor_enabled_; }
		bool     getPressureSwitch(void)       const { return pressure_switch_; }
		double   getCompressorCurrent(void)    const { return compressor_current_; }
		bool     getClosedLoopControl(void)    const { return closed_loop_control_; }
		bool     getCurrentTooHigh(void)       const { return current_too_high_; }
		bool     getCurrentTooHighSticky(void) const { return current_too_high_sticky_; }
		bool     getShorted(void)              const { return shorted_; }
		bool     getShortedSticky(void)        const { return shorted_sticky_; }
		bool     getNotConnected(void)         const { return not_connected_; }
		bool     getNotConnectedSticky(void)   const { return not_connected_sticky_; }
		bool     getVoltageFault(void)         const { return voltage_fault_; }
		bool     getVoltageSticky(void)        const { return voltage_sticky_; }
		uint32_t getSolenoidDisabledList(void) const { return solenoid_disabled_list_; }

		void setCompressorEnabled(bool compressor_enabled)            { compressor_enabled_ = compressor_enabled; }
		void setPressureSwitch(bool pressure_switch)                  { pressure_switch_ = pressure_switch; }
		void setCompressorCurrent(double compressor_current)          { compressor_current_ = compressor_current; }
		void setClosedLoopControl(bool closed_loop_control)           { closed_loop_control_ = closed_loop_control; }
		void setCurrentTooHigh(bool current_too_high)                 { current_too_high_ = current_too_high; }
		void setCurrentTooHighSticky(bool current_too_high_sticky)    { current_too_high_sticky_ = current_too_high_sticky; }
		void setShorted(bool shorted)                                 { shorted_ = shorted; }
		void setShortedSticky(bool shorted_sticky)                    { shorted_sticky_ = shorted_sticky; }
		void setNotConntected(bool not_connected)                     { not_connected_ = not_connected; }
		void setNotConnecteSticky(bool not_connected_sticky)          { not_connected_sticky_ = not_connected_sticky; }
		void setVoltageFault(bool voltage_fault)                      { voltage_fault_ = voltage_fault; }
		void setVoltageSticky(bool voltage_sticky)                    { voltage_sticky_ = voltage_sticky; }
		void setSolenoidDisabledList(uint32_t solenoid_disabled_list) { solenoid_disabled_list_ = solenoid_disabled_list; }

	private:
		int32_t  id_;
		bool     compressor_enabled_{false};
		bool     pressure_switch_{false};
		double   compressor_current_{0};
		bool     closed_loop_control_{false};
		bool     current_too_high_{false};
		bool     current_too_high_sticky_{false};
		bool     shorted_{false};
		bool     shorted_sticky_{false};
		bool     not_connected_{false};
		bool     not_connected_sticky_{false};
		bool     voltage_fault_{false};
		bool     voltage_sticky_{false};
		uint32_t solenoid_disabled_list_{0};
};

typedef StateHandle<const PCMState> PCMStateHandle;
typedef StateHandle<PCMState> PCMWritableStateHandle;
class PCMStateInterface       : public HardwareResourceManager<PCMStateHandle> {};
class RemotePCMStateInterface : public HardwareResourceManager<PCMWritableStateHandle, ClaimResources> {};

} // namespace

#endif