#include "ctre_interfaces/talonfxpro_sim_command_interface.h"

namespace hardware_interface::talonfxpro
{
TalonFXProSimCommand::TalonFXProSimCommand(void) = default;

void TalonFXProSimCommand::setSupplyVoltage(const double supply_voltage)
{
    if (supply_voltage != supply_voltage_)
    {
        supply_voltage_ = supply_voltage;
        supply_voltage_changed_ = true;
    }
}
double TalonFXProSimCommand::getSupplyVoltage(void) const
{
    return supply_voltage_;
}
bool TalonFXProSimCommand::supplyVoltageChanged(double &supply_voltage)
{
    supply_voltage = supply_voltage_;
    const bool rc = supply_voltage_changed_;
    supply_voltage_changed_ = false;
    return rc;
}
void TalonFXProSimCommand::resetSupplyVoltage(void)
{
    supply_voltage_changed_ = true;
}

bool TalonFXProSimCommand::forwardLimitChanged(bool &forward_limit)
{
    forward_limit = forward_limit_;
    const bool rc = forward_limit_changed_;
    forward_limit_changed_ = false;
    return rc;
}
bool TalonFXProSimCommand::getForwardLimit(void) const
{
    return forward_limit_;
}
void TalonFXProSimCommand::setForwardLimit(const bool forward_limit)
{
    if (forward_limit != forward_limit_)
    {
        forward_limit_ = forward_limit;
        forward_limit_changed_ = true;
    }
}
void TalonFXProSimCommand::resetForwardLimit(void)
{
    forward_limit_changed_ = true;
}

bool TalonFXProSimCommand::reverseLimitChanged(bool &reverse_limit)
{
    reverse_limit = reverse_limit_;
    const bool rc = reverse_limit_changed_;
    reverse_limit_changed_ = false;
    return rc;
}
bool TalonFXProSimCommand::getReverseLimit(void) const
{
    return reverse_limit_;
}
void TalonFXProSimCommand::setReverseLimit(const bool reverse_limit)
{
    if (reverse_limit != reverse_limit_)
    {
        reverse_limit_ = reverse_limit;
        reverse_limit_changed_ = true;
    }
}
void TalonFXProSimCommand::resetReverseLimit(void)
{
    reverse_limit_changed_ = true;
}

void TalonFXProSimCommand::setRawRotorPosition(const double rotor_position)
{
    if (rotor_position != rotor_position_)
    {
        rotor_position_ = rotor_position;
        rotor_position_changed_ = true;
    }
}
double TalonFXProSimCommand::getRawRotorPosition(void) const
{
    return rotor_position_;
}
bool TalonFXProSimCommand::rawPositionChanged(double &rotor_position)
{
    rotor_position = rotor_position_;
    const bool rc = rotor_position_changed_;
    rotor_position_changed_ = false;
    return rc;
}
void TalonFXProSimCommand::resetRawRotorPosition(void)
{
    rotor_position_changed_ = true;
}

void TalonFXProSimCommand::setAddRotorPosition(const double add_rotor_position)
{
    if (add_rotor_position != add_rotor_position_)
    {
        add_rotor_position_ = add_rotor_position;
        add_rotor_position_changed_ = true;
    }
}
double TalonFXProSimCommand::getAddRotorPosition(void) const
{
    return add_rotor_position_;
}
bool TalonFXProSimCommand::addPositionChanged(double &add_rotor_position)
{
    add_rotor_position = add_rotor_position_;
    const bool rc = add_rotor_position_changed_;
    add_rotor_position_changed_ = false;
    return rc;
}
void TalonFXProSimCommand::resetAddRotorPosition(void)
{
    add_rotor_position_changed_ = true;
}

void TalonFXProSimCommand::setRotorVelocity(const double rotor_velocity)
{
    if (rotor_velocity != rotor_velocity_)
    {
        rotor_velocity_ = rotor_velocity;
        rotor_velocity_changed_ = true;
    }
}
double TalonFXProSimCommand::getRotorVelocity(void) const
{
    return rotor_velocity_;
}
bool TalonFXProSimCommand::rotorVelocityChanged(double &rotor_velocity)
{
    rotor_velocity = rotor_velocity_;
    const bool rc = rotor_velocity_changed_;
    rotor_velocity_changed_ = false;
    return rc;
}
void TalonFXProSimCommand::resetRotorVelocity(void)
{
    rotor_velocity_changed_ = true;
}

void TalonFXProSimCommand::setRotorAcceleration(const double acceleration)
{
    if (acceleration != rotor_acceleration_)
    {
        rotor_acceleration_ = acceleration;
        rotor_acceleration_changed_ = true;
    }
}
double TalonFXProSimCommand::getRotorAcceleration(void) const
{
    return rotor_acceleration_;
}
bool TalonFXProSimCommand::accelerationChanged(double &acceleration)
{
    acceleration = rotor_acceleration_;
    const bool rc = rotor_acceleration_changed_;
    rotor_acceleration_changed_ = false;
    return rc;
}
void TalonFXProSimCommand::resetRotorAcceleration(void)
{
    rotor_acceleration_changed_ = true;
}

} // namespace hardware_interface::talonfxpro