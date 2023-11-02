#include "frc/Joystick.h"
#include <FRC_NetworkCommunication/FRCComm.h>
#include "hal/DriverStationTypes.h"                        // for HAL_kMaxJoystickAxes
#include "hal/simulation/DriverStationData.h"

#include "frc_interfaces/joystick_interface.h"
#include "periodic_interval_counter/periodic_interval_counter.h"
#include "ros_control_boilerplate/joystick_device.h"

struct HAL_JoystickAxesInt
{
    int16_t count;
    int16_t axes[HAL_kMaxJoystickAxes];
};

JoystickDevice::JoystickDevice(const int joint_index,
                               const std::string &name,
                               const uint8_t id,
                               const double read_hz)
    : name_{name}
    , id_{id}
    , joystick_{std::make_unique<frc::Joystick>(id_)}
    , state_{std::make_unique<hardware_interface::JoystickState>(name_.c_str(), id_)}
    , interval_counter_{std::make_unique<PeriodicIntervalCounter>(read_hz)}
{
    ROS_INFO_STREAM("Loading joint " << joint_index << "=" << name_ <<
                    " as joystick with ID " << id_ <<
                    " running at " << read_hz << "Hz");
}

JoystickDevice::~JoystickDevice() = default;

void JoystickDevice::registerInterfaces(hardware_interface::JoystickStateInterface &state_interface) const
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering Joystick interface for : " << name_ << " at hw ID " << id_);
    hardware_interface::JoystickStateHandle state_handle(name_, state_.get());
    state_interface.registerHandle(state_handle);
}

void JoystickDevice::read(const ros::Time &/*time*/, const ros::Duration &period)
{
    // check if sufficient time has passed since last read
    if (interval_counter_->update(period))
    {
        state_->clear();
        const auto axis_count = joystick_->GetAxisCount();
        for (auto i = 0; i < axis_count; i++)
        {
            state_->addAxis(joystick_->GetRawAxis(i));
        }
        const auto button_count = joystick_->GetButtonCount();
        for (auto i = 0; i < button_count; i++)
        {
            state_->addButton(joystick_->GetRawButton(i + 1));
        }
        const auto pov_count = joystick_->GetPOVCount();
        for (auto i = 0; i < pov_count; i++)
        {
            state_->addPOV(joystick_->GetPOV(i));
        }

        // wpilib struct: Declared near top
        HAL_JoystickAxesInt axesInt;
        int retVal = FRC_NetworkCommunication_getJoystickAxes(id_,
                                                              reinterpret_cast<JoystickAxes_t *>(&axesInt),
                                                              HAL_kMaxJoystickAxes);
        for (auto i = 0; i < axesInt.count; i++)
        {
            const auto value = axesInt.axes[i];
            // ROS_INFO_STREAM("Stick=" << joystick_ids_[joystick] << " axis=" << i << " value=" << std::hex << (((unsigned int)value) &0xff));
            state_->addRawAxis(value);
        }
    }
}