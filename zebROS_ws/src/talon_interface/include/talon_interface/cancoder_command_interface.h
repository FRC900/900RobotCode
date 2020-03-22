#pragma once

#include "talon_interface/cancoder_state_interface.h"
#include "state_handle/command_handle.h"

namespace hardware_interface
{
namespace cancoder
{
class CANCoderHWCommand
{
	public:
		CANCoderHWCommand(void);

		double getPosition(void) const;
		void setPosition(double position);
		bool positionChanged(double &position);
		void resetPosition(void);

		bool getPositionToAbsolute(void) const;
		void setPositionToAbsolute(void);
		bool positionToAbsoluteChanged(void);

		SensorVelocityMeasPeriod getVelocityMeasPeriod(void) const;
		void setVelocityMeasPeriod(SensorVelocityMeasPeriod velocity_meas_period);
		bool velocityMeasPeriodChanged(SensorVelocityMeasPeriod &velocity_meas_period);
		void resetVelocityMeasPeriod(void);

		int getVelocityMeasWindow(void) const;
		void setVelocityMeasWindow(int velocity_meas_window);
		bool velocityMeasWindowChanged(int &velocity_meas_window);
		void resetVelocityMeasWindow(void);

		AbsoluteSensorRange getAbsoluteSensorRange(void) const;
		void setAbsoluteSensorRange(AbsoluteSensorRange absolute_sensor_range);
		bool absoluteSensorRangeChanged(AbsoluteSensorRange &absolute_sensor_range);
		void resetAbsoluteSensorRange(void);

		double getMagnetOffset(void) const;
		void setMagnetOffset(double magnet_offset);
		bool magnetOffsetChanged(double &magnet_offset);
		void resetMagnetOffset(void);

		double getInitializationStrategy(void) const;
		void setInitializationStrategy(SensorInitializationStrategy initialization_strategy);
		bool InitializationStrategyChanged(SensorInitializationStrategy &initialization_strategy);
		void resetInitializationStrategy(void);

		double getFeedbackCoefficient(void) const;
		void setFeedbackCoefficient(double feedback_coefficient);

		std::string getUnitString(void) const;
		void setUnitString(const std::string &unit_string);
		SensorTimeBase getTimeBase(void) const;
		void setTimeBase(SensorTimeBase time_base);
		bool feedbackCoefficientChanged(
				double &feedback_coefficient,
				std::string &unit_string,
				SensorTimeBase &time_base);
		void resetFeedbackCoefficient(void);

		double getDirection(void) const;
		void setDirection(bool direction);
		bool directionChanged(bool &direction);
		void resetDirection(void);

		int getSensorDataStatusFramePeriod(void) const;
		void setSensorDataStatusFramePeriod(int sensor_data_status_frame_period);
		bool sensorDataStatusFramePeriodChanged(int &sensor_data_status_frame_period);
		void resetSensorDataStatusFramePeriod(void);

		int getVBatAndFaultsStatusFramePeriod(void) const;
		void setVBatAndFaultsStatusFramePeriod(int vbat_and_faults_status_frame_period);
		bool vbatAndFaultsStatusFramePeriodChanged(int &vbat_and_faults_status_frame_period);
		void resetVBatAndFaultsStatusFramePeriod(void);

		void setClearStickyFaults(void);
		bool getClearStickyFaults(void) const;
		bool clearStickyFaultsChanged(void);

		void   setConversionFactor(double conversion_factor);
		double getConversionFactor(void) const;

	private:
		double                       position_;
		bool                         position_changed_;
		bool                         set_position_to_absolute_; // one-shot, no need for changed flag
		SensorVelocityMeasPeriod     velocity_meas_period_;
		bool                         velocity_meas_period_changed_;
		int                          velocity_meas_window_;
		bool                         velocity_meas_window_changed_;
		AbsoluteSensorRange          absolute_sensor_range_;
		bool                         absolute_sensor_range_changed_;
		double                       magnet_offset_;
		bool                         magnet_offset_changed_;
		SensorInitializationStrategy initialization_strategy_;
		bool                         initialization_strategy_changed_;
		double                       feedback_coefficient_;
		std::string                  unit_string_;
		SensorTimeBase               time_base_;
		bool                         feedback_coefficient_changed_;
		bool                         direction_;
		bool                         direction_changed_;
		int                          sensor_data_status_frame_period_;
		bool                         sensor_data_status_frame_period_changed_;
		int                          vbat_and_faults_status_frame_period_;
		bool                         vbat_and_faults_status_frame_period_changed_;
		bool                         clear_sticky_faults_;
		double                       conversion_factor_;
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
