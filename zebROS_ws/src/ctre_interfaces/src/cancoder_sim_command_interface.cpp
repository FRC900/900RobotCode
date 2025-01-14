#include "ctre_interfaces/cancoder_sim_command_interface.h"

namespace hardware_interface::cancoder
{
    CANCoderSimCommand::CANCoderSimCommand(void) = default;


    double CANCoderSimCommand::getSupplyVoltage(void) const
    {
        return supply_voltage_;
    }
    void CANCoderSimCommand::setSupplyVoltage(const double supply_voltage)
    {
        if (supply_voltage != supply_voltage_)
        {
            supply_voltage_ = supply_voltage;
            supply_voltage_changed_ = true;
        }
    }
    bool CANCoderSimCommand::supplyVoltageChanged(double &supply_voltage)
    {
        supply_voltage = supply_voltage_;
        const bool rc = supply_voltage_changed_;
        supply_voltage_changed_ = false;
        return rc;
    }
    void CANCoderSimCommand::resetSupplyVoltage(void)
    {
        supply_voltage_changed_ = true;
    }

    double CANCoderSimCommand::getRawPosition(void) const
    {
        return raw_position_;
    }
    void CANCoderSimCommand::setRawPosition(const double raw_position)
    {
        if (raw_position != raw_position_)
        {
            raw_position_ = raw_position;
            raw_position_changed_ = true;
        }
    }
    bool CANCoderSimCommand::rawPositionChanged(double &raw_position)
    {
        raw_position = raw_position_;
        const bool rc = raw_position_changed_;
        raw_position_changed_ = false;
        return rc;
    }
    void CANCoderSimCommand::resetRawPosition(void)
    {
        raw_position_changed_ = true;
    }

    double CANCoderSimCommand::getAddPosition(void) const
    {
        return add_position_;
    }
    void CANCoderSimCommand::setAddPosition(const double add_position)
    {
        add_position_ = add_position;
        add_position_changed_ = true;
    }
    bool CANCoderSimCommand::addPositionChanged(double &add_position)
    {
        add_position = add_position_;
        const bool rc = add_position_changed_;
        add_position_changed_ = false;
        return rc;
    }
    void CANCoderSimCommand::resetAddPosition(void)
    {
        add_position_changed_ = true;
    }

    double CANCoderSimCommand::getVelocity(void) const
    {
        return velocity_;
    }
    void CANCoderSimCommand::setVelocity(const double velocity)
    {
        if (velocity != velocity_)
        {
            velocity_ = velocity;
            velocity_changed_ = true;
        }
    }
    bool CANCoderSimCommand::velocityChanged(double &velocity)
    {
        velocity = velocity_;
        const bool rc = velocity_changed_;
        velocity_changed_ = false;
        return rc;
    }
    void CANCoderSimCommand::resetVelocity(void)
    {
        velocity_changed_ = true;
    }

    MagnetHealth CANCoderSimCommand::getMagnetHealth(void) const
    {
        return magnet_health_;
    }
    void CANCoderSimCommand::setMagnetHealth(const MagnetHealth magnet_health)
    {
        if (magnet_health != magnet_health_)
        {
            magnet_health_ = magnet_health;
            magnet_health_changed_ = true;
        }
    }
    bool CANCoderSimCommand::magnetHealthChanged(MagnetHealth &magnet_health)
    {
        magnet_health = magnet_health_;
        const bool rc = magnet_health_changed_;
        magnet_health_changed_ = false;
        return rc;
    }
    void CANCoderSimCommand::resetMagnetHealth(void)
    {
        magnet_health_changed_ = true;
    }

} // namespace hardware_interface::cancoder
