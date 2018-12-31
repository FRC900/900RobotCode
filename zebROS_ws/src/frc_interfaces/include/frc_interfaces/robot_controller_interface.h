#pragma once

// Hardare interface to data read from DriverStation about
// current state of the Rio.  See e.g.
// http://first.wpi.edu/FRC/roborio/beta/docs/cpp/classfrc_1_1RobotController.html
// for details

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>

namespace hardware_interface
{

class RobotControllerState
{
	public :
		RobotControllerState(void)
			: fpga_version_(0)
		    , fpga_revision_(0)
		    , fpga_time_(0)
		    , user_button_(false)
			, is_sys_active_(false)
			, is_browned_out_(false)
			, input_voltage_(0.0)
			, input_current_(0.0)
			, voltage3v3_(0.0)
			, current3v3_(0.0)
			, enabled3v3_(false)
			, fault_count3v3_(0)
			, voltage5v_(0.0)
			, current5v_(0.0)
			, enabled5v_(false)
			, fault_count5v_(0)
			, voltage6v_(0.0)
			, current6v_(0.0)
			, enabled6v_(false)
			, fault_count6v_(0)
			, can_percent_bus_utilization_(0.0)
			, can_bus_off_count_(0)
			, can_tx_full_count_(0)
			, can_receive_error_count_(0)
			, can_transmit_error_count_(0)
		{
		}
		int      GetFPGAVersion(void) const              { return fpga_version_; }
		int64_t  GetFPGARevision(void) const             { return fpga_revision_; }
		uint64_t GetFPGATime(void) const                 { return fpga_time_; }
		bool     GetUserButton(void) const               { return user_button_; }
		bool     GetIsSysActive(void) const              { return is_sys_active_; }
		bool     GetIsBrownedOut(void) const             { return is_browned_out_; }
		double   GetInputVoltage(void) const             { return input_voltage_; }
		double   GetInputCurrent(void) const             { return input_current_; }
		double   GetVoltage3V3(void) const               { return voltage3v3_; }
		double   GetCurrent3V3(void) const               { return current3v3_; }
		bool     GetEnabled3V3(void) const               { return enabled3v3_ ; }
		int      GetFaultCount3V3(void) const            { return fault_count3v3_; }
		double   GetVoltage5V(void) const                { return voltage5v_; }
		double   GetCurrent5V(void) const                { return current5v_; }
		bool     GetEnabled5V(void) const                { return enabled5v_ ; }
		int      GetFaultCount5V(void) const             { return fault_count5v_; }
		double   GetVoltage6V(void) const                { return voltage6v_; }
		double   GetCurrent6V(void) const                { return current6v_; }
		bool     GetEnabled6V(void) const                { return enabled6v_ ; }
		int      GetFaultCount6V(void) const             { return fault_count6v_; }
		double   GetCANPercentBusUtilization(void) const { return can_percent_bus_utilization_; }
		int      GetCANBusOffCount(void) const           { return can_bus_off_count_; }
		int      GetCANTxFullCount(void) const           { return can_tx_full_count_; }
		int      GetCANReceiveErrorCount(void) const     { return can_receive_error_count_; }
		int      GetCANTransmitErrorCount(void) const    { return can_transmit_error_count_; }

		void SetFPGAVersion(int fpga_version)                                { fpga_version_ = fpga_version; }
		void SetFPGARevision(int64_t fpga_revision)                          { fpga_revision_ = fpga_revision; }
		void SetFPGATime(uint64_t fpga_time)                                 { fpga_time_ = fpga_time; }
		void SetUserButton(bool user_button)                                 { user_button_ = user_button; }
		void SetIsSysActive(bool is_sys_active)                              { is_sys_active_ = is_sys_active; }
		void SetIsBrownedOut(bool is_browned_out)                            { is_browned_out_ = is_browned_out; }
		void SetInputVoltage(double input_voltage)                           { input_voltage_ = input_voltage; }
		void SetInputCurrent(double input_current)                           { input_current_ = input_current; }
		void SetVoltage3V3(double voltage3v3)                                { voltage3v3_ = voltage3v3; }
		void SetCurrent3V3(double current3v3)                                { current3v3_ = current3v3; }
		void SetEnabled3V3(bool enabled3v3)                                  { enabled3v3_  = enabled3v3; }
		void SetFaultCount3V3(int fault_count3v3)                            { fault_count3v3_ = fault_count3v3; }
		void SetVoltage5V(double voltage5v)                                  { voltage5v_ = voltage5v; }
		void SetCurrent5V(double current5v)                                  { current5v_ = current5v; }
		void SetEnabled5V(bool enabled5v)                                    { enabled5v_  = enabled5v; }
		void SetFaultCount5V(int fault_count5v)                              { fault_count5v_ = fault_count5v; }
		void SetVoltage6V(double voltage6v)                                  { voltage6v_ = voltage6v; }
		void SetCurrent6V(double current6v)                                  { current6v_ = current6v; }
		void SetEnabled6V(bool enabled6v)                                    { enabled6v_  = enabled6v; }
		void SetFaultCount6V(int fault_count6v)                              { fault_count6v_ = fault_count6v; }
		void SetCANPercentBusUtilization(double can_percent_bus_utilization) { can_percent_bus_utilization_ = can_percent_bus_utilization; }
		void SetCANBusOffCount(int can_bus_off_count)                        { can_bus_off_count_ = can_bus_off_count; }
		void SetCANTxFullCount(int can_tx_full_count)                        { can_tx_full_count_ = can_tx_full_count; }
		void SetCANReceiveErrorCount(int can_receive_error_count)            { can_receive_error_count_ = can_receive_error_count; }
		void SetCANTransmitErrorCount(int can_transmit_error_count)          { can_transmit_error_count_ = can_transmit_error_count; }

	private:
		int      fpga_version_;
		int64_t  fpga_revision_;
		uint64_t fpga_time_;
		bool     user_button_;
		bool     is_sys_active_;
		bool     is_browned_out_;
		double   input_voltage_;
		double   input_current_;
		double   voltage3v3_;
		double   current3v3_;
		bool     enabled3v3_;
		int      fault_count3v3_;
		double   voltage5v_;
		double   current5v_;
		bool     enabled5v_;
		int      fault_count5v_;
		double   voltage6v_;
		double   current6v_;
		bool     enabled6v_;
		int      fault_count6v_;
		float    can_percent_bus_utilization_;
		int      can_bus_off_count_;
		int      can_tx_full_count_;
		int      can_receive_error_count_;
		int      can_transmit_error_count_;
};

typedef StateHandle<const RobotControllerState> RobotControllerStateHandle;
class RobotControllerStateInterface: public HardwareResourceManager<RobotControllerStateHandle> {};
}
