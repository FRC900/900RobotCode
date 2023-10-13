#ifndef CANCODER_COMMAND_INTERFACE_INC__
#define CANCODER_COMMAND_INTERFACE_INC__

#include "ctre_interfaces/cancoder_state_interface.h"
#include "state_handle/command_handle.h"

namespace hardware_interface
{
namespace cancoder
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

		AbsoluteSensorRange getAbsoluteSensorRange(void) const;
		void setAbsoluteSensorRange(const AbsoluteSensorRange absolute_sensor_range);

		bool magnetSensorConfigsChanged(SensorDirection &sensor_direction,
										double &magnet_offset,
										AbsoluteSensorRange &absoulte_sensor_range);
		void resetMagnetSensorConfigs();

		void setClearStickyFaults(void);
		bool getClearStickyFaults(void) const;
		bool clearStickyFaultsChanged(void);

		void   setConversionFactor(const double conversion_factor);
		double getConversionFactor(void) const;

	private:
		double              set_position_{};
		bool                set_position_changed_{false};
		SensorDirection     sensor_direction_{SensorDirection::CounterClockwise_Positive};
		double              magnet_offset_{0.0};
		AbsoluteSensorRange absolute_sensor_range_{AbsoluteSensorRange::Unsigned_0To1};
		bool                magnet_sensor_configs_changed_{true};
		bool                clear_sticky_faults_{false};
		double              conversion_factor_{1.0};
};

// Handle - used by each controller to get, by name of the
// corresponding joint, an interface with which to send commands
// to a CANCoder
typedef CommandHandle<CANCoderHWCommand, CANCoderHWState, CANCoderStateHandle> CANCoderCommandHandle;


// Use ClaimResources here since we only want 1 controller
// to be able to access a given CANCoder at any particular time
class CANCoderCommandInterface : public HardwareResourceManager<CANCoderCommandHandle, ClaimResources> {};

} // namespace cancoder
} // namespace hardware_interface

#endif