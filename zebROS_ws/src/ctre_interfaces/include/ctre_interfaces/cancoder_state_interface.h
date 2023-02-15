#pragma once

#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include "state_handle/state_handle.h"

namespace hardware_interface
{
namespace cancoder
{

enum SensorVelocityMeasPeriod
{
	Sensor_Period_1Ms = 1,
	Sensor_Period_2Ms = 2,
	Sensor_Period_5Ms = 5,
	Sensor_Period_10Ms = 10,
	Sensor_Period_20Ms = 20,
	Sensor_Period_25Ms = 25,
	Sensor_Period_50Ms = 50,
	Sensor_Period_100Ms = 100,
};
enum AbsoluteSensorRange
{
	/**
	 * Express the absolute position as an unsigned value.
	 * E.g. [0,+1) rotations or [0,360) deg.
	 */
	Unsigned_0_to_360 = 0,
	/**
	 * Express the absolute position as an signed value.
	 * E.g. [-0.5,+0.5) rotations or [-180,+180) deg.
	 */
	Signed_PlusMinus180 = 1,
};
enum SensorInitializationStrategy
{
	/**
	 * On boot up, set position to zero.
	 */
	BootToZero = 0,
	/**
	 * On boot up, sync to the Absolute Position signal.  The Absolute position signal will be signed according to the selected Absolute Sensor Range.
	 */
	BootToAbsolutePosition = 1,
};
enum SensorTimeBase
{
	/**
	 * Legacy Mode
	 */
	Per100Ms_Legacy = 0,
	/**
	 * Per-Second Velocities
	 */
	PerSecond = 1,
	/**
	 * Per-Minute Velocities
	 */
	PerMinute = 2,
};
enum MagnetFieldStrength
{
	/** Magnet Field strength cannot be determined */
	Invalid_Unknown = 0,
	/** Magnet field is far too low (too far) or far too high (too close). */
	BadRange_RedLED = 1,
	/** Magnet field is adequate, sensor can be used in this range with slightly reduced accuracy. */
	Adequate_OrangeLED = 2,
	/** Magnet field is ideal */
	Good_GreenLED = 3,
};

class CANCoderHWState
{
	public:
		CANCoderHWState(int device_number);
		int getDeviceNumber(void) const;

		double getPosition(void) const;
		void setPosition(double position);

		double getVelocity(void) const;
		void setVelocity(double velocity);

		double getAbsolutePosition(void) const;
		void setAbsolutePosition(double absolute_position);

		SensorVelocityMeasPeriod getVelocityMeasPeriod(void) const;
		void setVelocityMeasPeriod(SensorVelocityMeasPeriod velocity_meas_period);

		int getVelocityMeasWindow(void) const;
		void setVelocityMeasWindow(int velocity_meas_window);

		AbsoluteSensorRange getAbsoluteSensorRange(void) const;
		void setAbsoluteSensorRange(AbsoluteSensorRange absolute_sensor_range);

		double getMagnetOffset(void) const;
		void setMagnetOffset(double magnet_offset);

		SensorInitializationStrategy getInitializationStrategy(void) const;
		void setInitializationStrategy(SensorInitializationStrategy initialization_strategy);

		double getFeedbackCoefficient(void) const;
		void setFeedbackCoefficient(double feedback_coefficient);

		std::string getUnitString(void) const;
		void setUnitString(const std::string &unit_string);

		SensorTimeBase getTimeBase(void) const;
		void setTimeBase(SensorTimeBase time_base);

		double getBusVoltage(void) const;
		void setBusVoltage(double bus_voltage);

		MagnetFieldStrength getMagnetFieldStrength(void) const;
		void setMagnetFieldStrength(MagnetFieldStrength magnet_field_strength);

		bool getDirection(void) const;
		void setDirection(bool direction);

		double getLastTimestamp(void) const;
		void setLastTimestamp(double last_timestamp);

		int getSensorDataStatusFramePeriod(void) const;
		void setSensorDataStatusFramePeriod(int sensor_data_status_frame_period);

		int getVbatAndFaultsStatusFramePeriod(void) const;
		void setVbatAndFaultsStatusFramePeriod(int vbat_and_faults_status_frame_period);

		int getFirmwareVersion(void) const;
		void setFirmwareVersion(int firmware_version);

		int getFaults(void) const;
		void setFaults(int faults);

		int getStickyFaults(void) const;
		void setStickyFaults(int sticky_faults);

		void   setConversionFactor(double conversion_factor);
		double getConversionFactor(void) const;

	private :
		int                          device_number_;
		double                       position_;
		double                       velocity_;
		double                       absolute_position_;
		SensorVelocityMeasPeriod     velocity_meas_period_;
		int                          velocity_meas_window_;
		AbsoluteSensorRange          absolute_sensor_range_;
		double                       magnet_offset_;
		SensorInitializationStrategy initialization_strategy_;
		double                       feedback_coefficient_;
		std::string                  unit_string_;
		SensorTimeBase               time_base_;
		double                       bus_voltage_;
		MagnetFieldStrength          magnet_field_strength_;
		bool                         direction_;
		double                       last_timestamp_;
		int                          sensor_data_status_frame_period_;
		int                          vbat_and_faults_status_frame_period_;
		int                          firmware_version_;
		int                          faults_;
		int                          sticky_faults_;
		double                       conversion_factor_;
};
// Glue code to let this be registered in the list of
// hardware resources on the robot.  Since state is
// read-only, allow multiple controllers to register it.
typedef StateHandle<const CANCoderHWState> CANCoderStateHandle;
typedef StateHandle<CANCoderHWState>       CANCoderWritableStateHandle;
class CANCoderStateInterface : public HardwareResourceManager<CANCoderStateHandle> {};
class RemoteCANCoderStateInterface : public HardwareResourceManager<CANCoderWritableStateHandle> {};

} // namespace cancoder
} // namespace hardware_interface
