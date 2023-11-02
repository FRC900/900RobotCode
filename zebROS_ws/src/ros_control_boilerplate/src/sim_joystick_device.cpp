#include "hal/DriverStationTypes.h"                        // for HAL_kMaxJoystickAxes
#include "hal/simulation/DriverStationData.h"

#include "ros_control_boilerplate/sim_joystick_device.h"

SimJoystickDevice::SimJoystickDevice(const int joint_index,
                                     const std::string &name,
                                     const int id,
                                     const double read_hz)
    : JoystickDevice(joint_index, name, id, read_hz)
    , mutex_{std::make_unique<std::mutex>()}
{
    ROS_INFO_STREAM("Sim Joystick " << name << " initialized");
}

SimJoystickDevice::~SimJoystickDevice() = default;

void SimJoystickDevice::read(const ros::Time &time, const ros::Duration &period)
{
    std::unique_lock<std::mutex> l(*mutex_, std::try_to_lock);
    if (l.owns_lock())
    {
        JoystickDevice::read(time, period);
    }
}

void SimJoystickDevice::simInit(ros::NodeHandle nh, size_t joint_index)
{
    std::stringstream s;
    s << "js" << joint_index << "_in";
    sim_sub_ = nh.subscribe<sensor_msgs::Joy>(s.str(), 1, &SimJoystickDevice::joystickCallback, this);
}

void SimJoystickDevice::joystickCallback(const sensor_msgs::JoyConstPtr &msg)
{
    HAL_JoystickAxes hal_axes;
    hal_axes.count = std::min(msg->axes.size(), static_cast<std::size_t>(6)); // the last two entries (6,7) are for POV
    std::memset(hal_axes.axes, 0, sizeof(hal_axes.axes));
    for (int i = 0; i < hal_axes.count; i++)
    {
        hal_axes.axes[i] = msg->axes[i];
    }

    HAL_JoystickButtons hal_buttons;
    hal_buttons.count = std::min(msg->buttons.size(), static_cast<std::size_t>(32)); // DriverStationGui.cpp used 32, comment below says 16?
    hal_buttons.buttons = 0;
    for (size_t i = 0; i < msg->buttons.size(); i++)
    {
        // TODO This is probably so wrong
        // GenericHID.h : The buttons are returned in a single 16 bit
        // value with one bit representing the state of each button
        hal_buttons.buttons = ((msg->buttons[i] ? 1 : 0) << i) | hal_buttons.buttons;
    }

    HAL_JoystickPOVs hal_povs;
    hal_povs.count = 1;
    std::memset(hal_povs.povs, -1, sizeof(hal_povs.povs));
    if (msg->axes.size() >= 8)
    {
        // TODO Do we have a standard epsilon somewhere in here?
        // TODO - also check see if it needs to be < -1e-5
        const bool direction_left = msg->axes[6] > 1e-5;
        const bool direction_right = msg->axes[6] < -1e-5;
        const bool direction_up = msg->axes[7] > 1e-5;
        const bool direction_down = msg->axes[7] < -1e-5;

        if (direction_up && !direction_left && !direction_right)
        {
            hal_povs.povs[0] = 0;
        }
        else if (direction_up && direction_right)
        {
            hal_povs.povs[0] = 45;
        }
        else if (!direction_up && !direction_down && direction_right)
        {
            hal_povs.povs[0] = 90;
        }
        else if (direction_down && direction_right)
        {
            hal_povs.povs[0] = 135;
        }
        else if (direction_down && !direction_left && !direction_right)
        {
            hal_povs.povs[0] = 180;
        }
        else if (direction_down && direction_left)
        {
            hal_povs.povs[0] = 225;
        }
        else if (!direction_up && !direction_down && direction_left)
        {
            hal_povs.povs[0] = 270;
        }
        else if (direction_up && direction_left)
        {
            hal_povs.povs[0] = 315;
        }
    }
    // TODO check default pov?
    // TODO do you need to set JoystickDescriptor?

    std::lock_guard l(*mutex_);
    HALSIM_SetJoystickPOVs(getId(), &hal_povs);
    HALSIM_SetJoystickAxes(getId(), &hal_axes);
    HALSIM_SetJoystickButtons(getId(), &hal_buttons);
    HALSIM_NotifyDriverStationNewData();
}