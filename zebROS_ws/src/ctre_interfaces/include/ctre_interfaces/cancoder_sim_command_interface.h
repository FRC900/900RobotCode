#ifndef CANCODER_SIM_COMMAND_INTERFACE_INC__
#define CANCODER_SIM_COMMAND_INTERFACE_INC__

#include "ctre_interfaces/cancoder_state_interface.h"
#include "state_handle/command_handle.h"

namespace hardware_interface::cancoder
{
class CANCoderSimCommand
{
public:
        CANCoderSimCommand(void);
        void setSupplyVoltage(const double supply_voltage);
        double getSupplyVoltage(void) const;
        bool supplyVoltageChanged(double &supply_voltage);
        void resetSupplyVoltage(void);

        void setRawPosition(const double position);
        double getRawPosition(void) const;
        bool rawPositionChanged(double &position);
        void resetRawPosition(void);

        void setAddPosition(const double add_position);
        double getAddPosition(void) const;
        bool addPositionChanged(double &add_position);
        void resetAddPosition(void);

        void setVelocity(const double velocity);
        double getVelocity(void) const;
        bool velocityChanged(double &velocity);
        void resetVelocity(void);

        void setMagnetHealth(const MagnetHealth magnet_health);
        MagnetHealth getMagnetHealth(void) const;
        bool magnetHealthChanged(MagnetHealth &magnet_health);
        void resetMagnetHealth(void);

private:
        // Set these to NaN so that any new value will be different from
        // the default initialized value and trigger a write to sim-hardware
        double supply_voltage_{std::numeric_limits<double>::quiet_NaN()};
        bool supply_voltage_changed_{false};
        double raw_position_{std::numeric_limits<double>::quiet_NaN()};
        bool raw_position_changed_{false};
        double add_position_{std::numeric_limits<double>::quiet_NaN()};
        bool add_position_changed_{false};
        double velocity_{std::numeric_limits<double>::quiet_NaN()};
        bool velocity_changed_{false};
        MagnetHealth magnet_health_{MagnetHealth::Invalid};
        bool magnet_health_changed_{false};
};

// Handle - used by each controller to get, by name of the
// corresponding joint, an interface with which to send commands
// to a CANCoderimulator
using CANCoderSimCommandHandle = CommandHandle<CANCoderSimCommand, CANCoderHWState, CANCoderStateHandle>;


// Use ClaimResources here since we only want 1 controller
// to be able to access a given CANCoder at any particular time
// TODO : is this true?  For sim stuff, do we need to worry about this?
class CANCoderSimCommandInterface : public HardwareResourceManager<CANCoderSimCommandHandle, ClaimResources> {};

} // namespace hardware_interface::cancoder

#endif