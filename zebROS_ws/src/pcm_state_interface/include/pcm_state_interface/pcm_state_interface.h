#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>

namespace hardware_interface
{

class PCMState
{
	public:
		PCMState(int32_t pcm_id) :
			pcm_id_(pcm_id)
			, enabled_(false)
			, pressure_switch_(false)
			, compressor_current_(0)
			, closed_loop_control_(0)
			, current_too_high_(false)
			, current_too_high_sticky_(false)
			, shorted_(false)
			, shorted_sticky_(false)
			, not_connected_(false)
			, not_connected_sticky_(false)
			, voltage_fault_(false)
			, voltage_sticky_fault_(false)
			, solenoid_blacklist_(0)
		{
		}

		int32_t  getPCMId(void)                const { return pcm_id_; }
		bool     getEnabled(void)              const { return enabled_; }
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
		bool     getVoltageStickyFault(void)   const { return voltage_sticky_fault_; }
		uint32_t getSolenoidBlacklist(void)    const { return solenoid_blacklist_; }

		void setEnabled(bool enabled)                              { enabled_ = enabled; }
		void setPressureSwitch(bool pressure_switch)               { pressure_switch_ = pressure_switch; }
		void setCompressorCurrent(double compressor_current)       { compressor_current_ = compressor_current; }
		void setClosedLoopControl(bool closed_loop_control)        { closed_loop_control_ = closed_loop_control; }
		void setCurrentTooHigh(bool current_too_high)              { current_too_high_ = current_too_high; }
		void setCurrentTooHighSticky(bool current_too_high_sticky) { current_too_high_sticky_ = current_too_high_sticky; }
		void setShorted(bool shorted)                              { shorted_ = shorted; }
		void setShortedSticky(bool shorted_sticky)                 { shorted_sticky_ = shorted_sticky; }
		void setNotConntected(bool not_connected)                  { not_connected_ = not_connected; }
		void setNotConnecteSticky(bool not_connected_sticky)       { not_connected_sticky_ = not_connected_sticky; }
		void setVoltageFault(bool voltage_fault)                   { voltage_fault_ = voltage_fault; }
		void setVoltageStickFault(bool voltage_sticky_fault)       { voltage_sticky_fault_ = voltage_sticky_fault; }
		void setSolenoidBlacklist(uint32_t solenoid_blacklist)     { solenoid_blacklist_ = solenoid_blacklist; }

	private:
		int32_t  pcm_id_;
		bool     enabled_;
		bool     pressure_switch_;
		double   compressor_current_;
		bool     closed_loop_control_;
		bool     current_too_high_;
		bool     current_too_high_sticky_;
		bool     shorted_;
		bool     shorted_sticky_;
		bool     not_connected_;
		bool     not_connected_sticky_;
		bool     voltage_fault_;
		bool     voltage_sticky_fault_;
		uint32_t solenoid_blacklist_;
};

typedef StateHandle<const PCMState> PCMStateHandle;
typedef StateHandle<PCMState> PCMWritableStateHandle;
class PCMStateInterface: public HardwareResourceManager<PCMStateHandle> {};
} // namespace
