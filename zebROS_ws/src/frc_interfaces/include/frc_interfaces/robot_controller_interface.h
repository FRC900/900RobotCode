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
		int         GetFPGAVersion(void) const              { return fpga_version_; }
		int64_t     GetFPGARevision(void) const             { return fpga_revision_; }
		uint64_t    GetFPGATime(void) const                 { return fpga_time_; }
		bool        GetUserButton(void) const               { return user_button_; }
		bool        GetIsSysActive(void) const              { return is_sys_active_; }
		bool        GetIsBrownedOut(void) const             { return is_browned_out_; }
		double      GetInputVoltage(void) const             { return input_voltage_; }
		double      GetInputCurrent(void) const             { return input_current_; }
		double      GetVoltage3V3(void) const               { return voltage3v3_; }
		double      GetCurrent3V3(void) const               { return current3v3_; }
		bool        GetEnabled3V3(void) const               { return enabled3v3_ ; }
		int         GetFaultCount3V3(void) const            { return fault_count3v3_; }
		double      GetVoltage5V(void) const                { return voltage5v_; }
		double      GetCurrent5V(void) const                { return current5v_; }
		bool        GetEnabled5V(void) const                { return enabled5v_ ; }
		int         GetFaultCount5V(void) const             { return fault_count5v_; }
		double      GetVoltage6V(void) const                { return voltage6v_; }
		double      GetCurrent6V(void) const                { return current6v_; }
		bool        GetEnabled6V(void) const                { return enabled6v_ ; }
		int         GetFaultCount6V(void) const             { return fault_count6v_; }
		double      GetCANPercentBusUtilization(void) const { return can_percent_bus_utilization_; }
		int         GetCANBusOffCount(void) const           { return can_bus_off_count_; }
		int         GetCANTxFullCount(void) const           { return can_tx_full_count_; }
		int         GetCANReceiveErrorCount(void) const     { return can_receive_error_count_; }
		int         GetCANTransmitErrorCount(void) const    { return can_transmit_error_count_; }

		std::string GetFPGAVersionStatus(void) const        { return fpga_version_status_; }
		std::string GetFPGARevisionStatus(void) const       { return fpga_revision_status_; }
		std::string GetFPGATimeStatus(void) const           { return fpga_time_status_; }
		std::string GetUserButtonStatus(void) const         { return user_button_status_; }
		std::string GetIsSysActiveStatus(void) const        { return is_sys_active_status_; }
		std::string GetIsBrownedOutStatus(void) const       { return is_browned_out_status_; }
		std::string GetInputVoltageStatus(void) const       { return input_voltage_status_; }
		std::string GetInputCurrentStatus(void) const       { return input_current_status_; }
		std::string GetVoltage3V3Status(void) const         { return voltage3v3_status_; }
		std::string GetCurrent3V3Status(void) const         { return current3v3_status_; }
		std::string GetEnabled3V3Status(void) const         { return enabled3v3_status_; }
		std::string GetFaultCount3V3Status(void) const      { return fault_count3v3_status_; }
		std::string GetVoltage5VStatus(void) const          { return voltage5v_status_; }
		std::string GetCurrent5VStatus(void) const          { return current5v_status_; }
		std::string GetEnabled5VStatus(void) const          { return enabled5v_status_; }
		std::string GetFaultCount5VStatus(void) const       { return fault_count5v_status_; }
		std::string GetVoltage6VStatus(void) const          { return voltage6v_status_; }
		std::string GetCurrent6VStatus(void) const          { return current6v_status_; }
		std::string GetEnabled6VStatus(void) const          { return enabled6v_status_; }
		std::string GetFaultCount6VStatus(void) const       { return fault_count6v_status_; }
		std::string GetCANDataStatus(void) const            { return can_data_status_; }


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

		void SetFPGAVersionStatus(const std::string &fpga_version_status)    { fpga_version_status_ = fpga_version_status; }
		void SetFPGARevisionStatus(const std::string &fpga_revision_status)  { fpga_revision_status_ = fpga_revision_status; }
		void SetFPGATimeStatus(const std::string &fpga_time_status)          { fpga_time_status_ = fpga_time_status; }
		void SetUserButtonStatus(const std::string &user_button_status)      { user_button_status_ = user_button_status; }
		void SetIsSysActiveStatus(const std::string &is_sys_active_status)   { is_sys_active_status_ = is_sys_active_status; }
		void SetIsBrownedOutStatus(const std::string &is_browned_out_status) { is_browned_out_status_ = is_browned_out_status; }
		void SetInputVoltageStatus(const std::string &input_voltage_status)  { input_voltage_status_ = input_voltage_status; }
		void SetInputCurrentStatus(const std::string &input_current_status)  { input_current_status_ = input_current_status; }
		void SetVoltage3V3Status(const std::string &voltage3v3_status)       { voltage3v3_status_ = voltage3v3_status; }
		void SetCurrent3V3Status(const std::string &current3v3_status)       { current3v3_status_ = current3v3_status; }
		void SetEnabled3V3Status(const std::string &enabled3v3_status)       { enabled3v3_status_  = enabled3v3_status; }
		void SetFaultCount3V3Status(const std::string &fault_count3v3_status){ fault_count3v3_status_ = fault_count3v3_status; }
		void SetVoltage5VStatus(const std::string &voltage5v_status)         { voltage5v_status_ = voltage5v_status; }
		void SetCurrent5VStatus(const std::string &current5v_status)         { current5v_status_ = current5v_status; }
		void SetEnabled5VStatus(const std::string &enabled5v_status)         { enabled5v_status_  = enabled5v_status; }
		void SetFaultCount5VStatus(const std::string &fault_count5v_status)  { fault_count5v_status_ = fault_count5v_status; }
		void SetVoltage6VStatus(const std::string &voltage6v_status)         { voltage6v_status_ = voltage6v_status; }
		void SetCurrent6VStatus(const std::string &current6v_status)         { current6v_status_ = current6v_status; }
		void SetEnabled6VStatus(const std::string &enabled6v_status)         { enabled6v_status_  = enabled6v_status; }
		void SetFaultCount6VStatus(const std::string &fault_count6v_status)  { fault_count6v_status_ = fault_count6v_status; }
		void SetCANDataStatus(const std::string &can_data_status)            { can_data_status_ = can_data_status; }

	private:
		int         fpga_version_{0};
		int64_t     fpga_revision_{0};
		uint64_t    fpga_time_{0};
		bool        user_button_{false};
		bool        is_sys_active_{false};
		bool        is_browned_out_{false};
		double      input_voltage_{0};
		double      input_current_{0};
		double      voltage3v3_{0};
		double      current3v3_{0};
		bool        enabled3v3_{0};
		int         fault_count3v3_{0};
		double      voltage5v_{0};
		double      current5v_{0};
		bool        enabled5v_{0};
		int         fault_count5v_{0};
		double      voltage6v_{0};
		double      current6v_{0};
		bool        enabled6v_{0};
		int         fault_count6v_{0};
		float       can_percent_bus_utilization_{0};
		int         can_bus_off_count_{0};
		int         can_tx_full_count_{0};
		int         can_receive_error_count_{0};
		int         can_transmit_error_count_{0};

		//status values
		std::string fpga_version_status_{"0: "};
		std::string fpga_revision_status_{"0: "};
		std::string fpga_time_status_{"0: "};
		std::string user_button_status_{"0: "};
		std::string is_sys_active_status_{"0: "};
		std::string is_browned_out_status_{"0: "};
		std::string input_voltage_status_{"0: "};
		std::string input_current_status_{"0: "};
		std::string voltage3v3_status_{"0: "};
		std::string current3v3_status_{"0: "};
		std::string enabled3v3_status_{"0: "};
		std::string fault_count3v3_status_{"0: "};
		std::string voltage5v_status_{"0: "};
		std::string current5v_status_{"0: "};
		std::string enabled5v_status_{"0: "};
		std::string fault_count5v_status_{"0: "};
		std::string voltage6v_status_{"0: "};
		std::string current6v_status_{"0: "};
		std::string enabled6v_status_{"0: "};
		std::string fault_count6v_status_{"0: "};
		std::string can_data_status_{"0: "};
};

typedef StateHandle<const RobotControllerState> RobotControllerStateHandle;
class RobotControllerStateInterface: public HardwareResourceManager<RobotControllerStateHandle> {};
}
