#ifndef CANCODER_COMMAND_INTERFACE_INC__
#define CANCODER_COMMAND_INTERFACE_INC__

#include "ctre_interfaces/cancoder_state_interface.h"
#include "state_handle/command_handle.h"

namespace hardware_interface::cancoder
{
class CANCoderHWCommand
{
	public:
		CANCoderHWCommand(void);

		void setSetPosition(const double set_position);
		double getSetPosition(void) const;
		bool setPositionChanged(double &set_position);
		void resetSetPosition(void);

		SensorDirection getSensorDirection(void) const;
		void setSensorDirection(const SensorDirection sensor_direction);

		double getMagnetOffset(void) const;
		void setMagnetOffset(const double magnet_offset);

		double getAbsoluteSensorDiscontinuityPoint(void) const;
		void setAbsoluteSensorDiscontinuityPoint(const double absolute_sensor_range);

		bool magnetSensorConfigsChanged(SensorDirection &sensor_direction,
										double &magnet_offset,
										double &absoulte_sensor_discontinuity_point);
		void resetMagnetSensorConfigs();

		void setClearStickyFaults(void);
		bool getClearStickyFaults(void) const;
		bool clearStickyFaultsChanged(void);

		void   setConversionFactor(const double conversion_factor);
		double getConversionFactor(void) const;

		void setEnableReadThread(const bool enable_read_thread);
		bool getEnableReadThread(void) const;

	private:
		double          set_position_{};
		bool            set_position_changed_{false};
		SensorDirection sensor_direction_{SensorDirection::CounterClockwise_Positive};
		double          magnet_offset_{0.0};
		double          absolute_sensor_discontinuity_point_{M_PI};
		bool            magnet_sensor_configs_changed_{true};
		bool            clear_sticky_faults_{false};
		double          conversion_factor_{1.0};
		bool            enable_read_thread_{true};
};

// Handle - used by each controller to get, by name of the
// corresponding joint, an interface with which to send commands
// to a CANCoder
using CANCoderCommandHandle = CommandHandle<CANCoderHWCommand, CANCoderHWState, CANCoderStateHandle>;


// Use ClaimResources here since we only want 1 controller
// to be able to access a given CANCoder at any particular time
class CANCoderCommandInterface : public HardwareResourceManager<CANCoderCommandHandle, ClaimResources> {};

} // namespace hardware_interface::cancoder

#endif