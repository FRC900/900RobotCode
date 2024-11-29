#ifndef CANCODER_STATE_INTERFACE_INC__
#define CANCODER_STATE_INTERFACE_INC__

#include <hardware_interface/internal/hardware_resource_manager.h>
#include "state_handle/state_handle.h"

namespace hardware_interface::cancoder
{

enum class SensorDirection
{
	CounterClockwise_Positive,
	Clockwise_Positive
};
enum class MagnetHealth
{
	Red,
	Orange,
	Green,
	Invalid
};

class CANCoderHWState
{
	public:
		explicit CANCoderHWState(const int device_number);
		int getDeviceNumber(void) const;

		SensorDirection getSensorDirection(void) const;
		void setSensorDirection(const SensorDirection sensor_direction);

		double getMagnetOffset(void) const;
		void setMagnetOffset(const double magnet_offset);

		double getAbsoluteSensorDiscontinuityPoint(void) const;
		void setAbsoluteSensorDiscontinuityPoint(const double absolute_sensor_discontinuity_point);

		void   setConversionFactor(const double conversion_factor);
		double getConversionFactor(void) const;

		void setEnableReadThread(const bool enable_read_thread);
		bool getEnableReadThread(void) const;

		void setVersionMajor(const int version_major);
		int  getVersionMajor(void) const;

		void setVersionMinor(const int version_minor);
		int  getVersionMinor(void) const;

		void setVersionBugfix(const int version_bugfix);
		int  getVersionBugfix(void) const;

		void setVersionBuild(const int version_build);
		int  getVersionBuild(void) const;

		double getVelocity(void) const;
		void setVelocity(const double velocity);

		double getPosition(void) const;
		void setPosition(const double position);

		double getAbsolutePosition(void) const;
		void setAbsolutePosition(const double absolute_position);

		double getUnfilteredVelocity(void) const;
		void setUnfilteredVelocity(const double unfiltered_velocity);

		double getPositionSinceBoot(void) const;
		void setPositionSinceBoot(const double position_since_boot);

		double getSupplyVoltage(void) const;
		void setSupplyVoltage(const double supply_voltage);

		MagnetHealth getMagnetHealth(void) const;
		void setMagnetHealth(const MagnetHealth magnet_health);

		void setFaultHardware(const bool fault_hardware);
		bool getFaultHardware(void) const;
		void setFaultUndervoltage(const bool fault_undervolage);
		bool getFaultUndervoltage(void) const;
		void setFaultBootDuringEnable(const bool fault_boot_during_enable);
		bool getFaultBootDuringEnable(void) const;
		void setFaultUnlicensedFeatureInUse(const bool fault_unlicensed_feature_in_use);
		bool getFaultUnlicensedFeatureInUse(void) const;
		void setFaultBadMagnet(const bool bad_magnet);
		bool getFaultBadMagnet(void) const;

		void setStickyFaultHardware(const bool sticky_fault_hardware);
		bool getStickyFaultHardware(void) const;
		void setStickyFaultUndervoltage(const bool sticky_fault_undervolage);
		bool getStickyFaultUndervoltage(void) const;
		void setStickyFaultBootDuringEnable(const bool sticky_fault_boot_during_enable);
		bool getStickyFaultBootDuringEnable(void) const;
		void setStickyFaultUnlicensedFeatureInUse(const bool sticky_fault_unlicensed_feature_in_use);
		bool getStickyFaultUnlicensedFeatureInUse(void) const;
		void setStickyFaultBadMagnet(const bool sticky_fault_bad_magnet);
		bool getStickyFaultBadMagnet(void) const;

	private :
		int             device_number_{};

		SensorDirection sensor_direction_{SensorDirection::CounterClockwise_Positive};
		double          magnet_offset_{0.0};
		double          absolute_sensor_discontinuity_point_{M_PI};

		double          conversion_factor_{1.0};

		int             version_major_{0};
		int             version_minor_{0};
		int             version_bugfix_{0};
		int             version_build_{0};
		double          velocity_{0.0};
		double          position_{0.0};
		double          absolute_position_{0.0};
		double          unfiltered_velocity_{0.0};
		double          position_since_boot_{0.0};
		double          supply_voltage_{0.0};
		MagnetHealth    magnet_health_{MagnetHealth::Invalid};

		bool            fault_hardware_{false};
		bool            fault_undervolage_{false};
		bool            fault_boot_during_enable_{false};
		bool            fault_unlicensed_feature_in_use_{false};
		bool            fault_bad_magnet_{false};
                  
		bool            sticky_fault_hardware_{false};
		bool            sticky_fault_undervolage_{false};
		bool            sticky_fault_boot_during_enable_{false};
		bool            sticky_fault_unlicensed_feature_in_use_{false};
		bool            sticky_fault_bad_magnet_{false};

		bool            enable_read_thread_{true};
};
// Glue code to let this be registered in the list of
// hardware resources on the robot.  Since state is
// read-only, allow multiple controllers to register it.
using CANCoderStateHandle = StateHandle<const CANCoderHWState>;
using CANCoderWritableStateHandle = StateHandle<CANCoderHWState>;
class CANCoderStateInterface : public HardwareResourceManager<CANCoderStateHandle> {};
class RemoteCANcoderStateInterface : public HardwareResourceManager<CANCoderWritableStateHandle, ClaimResources> {};

} // namespace hardware_interface::cancoder

#endif