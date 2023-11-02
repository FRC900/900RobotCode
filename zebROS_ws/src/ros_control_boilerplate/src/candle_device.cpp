#include <ctre/phoenix/led/CANdle.h>
#include <ctre/phoenix/led/ColorFlowAnimation.h>
#include <ctre/phoenix/led/FireAnimation.h>
#include <ctre/phoenix/led/LarsonAnimation.h>
#include <ctre/phoenix/led/RainbowAnimation.h>
#include <ctre/phoenix/led/RgbFadeAnimation.h>
#include <ctre/phoenix/led/SingleFadeAnimation.h>
#include <ctre/phoenix/led/StrobeAnimation.h>
#include <ctre/phoenix/led/TwinkleAnimation.h>
#include <ctre/phoenix/led/TwinkleOffAnimation.h>
#include "ctre_interfaces/candle_command_interface.h"
#include "ros_control_boilerplate/candle_device.h"

static ctre::phoenix::led::ColorFlowAnimation::Direction convertCANdleDirection(int direction);
static std::shared_ptr<ctre::phoenix::led::BaseStandardAnimation> convertBaseStandardAnimation(hardware_interface::candle::Animation animation);
static std::shared_ptr<ctre::phoenix::led::BaseTwoSizeAnimation> convertBaseTwoAnimation(hardware_interface::candle::Animation animation);

CANdleDevice::CANdleDevice(const std::string &name_space,
                           const int joint_index,
                           const std::string &joint_name,
                           const int candle_id,
                           const std::string &can_bus,
                           const bool local_hardware,
                           const bool local_update)
    : CTREV5Device(name_space, "CANdle", joint_name, candle_id)
    , local_hardware_{local_hardware}
    , local_update_{local_update}
    , candle_{local_hardware_ ? std::make_unique<ctre::phoenix::led::CANdle>(candle_id, can_bus) : nullptr}
    , state_{std::make_unique<hardware_interface::candle::CANdleHWState>(candle_id)}
    , command_{std::make_unique<hardware_interface::candle::CANdleHWCommand>()}
{
    ROS_INFO_STREAM_NAMED("frc_robot_interface",
                          "Loading joint " << joint_index << "=" << joint_name <<
                          (local_update_ ? " local" : " remote") << " update, " <<
                          (local_hardware_ ? "local" : "remote") << " hardware" <<
                          " as CANdle " << candle_id << " on bus " << can_bus);
}

CANdleDevice::~CANdleDevice() = default;

void CANdleDevice::registerInterfaces(hardware_interface::candle::CANdleStateInterface &state_interface,
                                      hardware_interface::candle::CANdleCommandInterface &command_interface,
                                      hardware_interface::candle::RemoteCANdleStateInterface &remote_state_interface) const
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for CANdle : " << getName() << " at CAN id " << getId());

    hardware_interface::candle::CANdleStateHandle state_handle(getName(), state_.get());
    state_interface.registerHandle(state_handle);

    hardware_interface::candle::CANdleCommandHandle command_handle(state_handle, command_.get());
    command_interface.registerHandle(command_handle);

    if (!local_update_)
    {
        hardware_interface::candle::CANdleWritableStateHandle remote_handle(getName(), state_.get());
        remote_state_interface.registerHandle(remote_handle);
    }
}

void CANdleDevice::write(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (!local_hardware_)
    {
        return;
    }

    // For each Command property, check if it changed
    //  if it did change, update the actual CANdle, and update the state

    // Brightness
    if (double brightness; command_->brightnessChanged(brightness)) {
        if (safeCall(
            candle_->ConfigBrightnessScalar(brightness),
            "candle_->ConfigBrightnessScalar"
        )) {
            ROS_INFO_STREAM("CANdle " << getName() << " : Set brightness to " << brightness);
            state_->setBrightness(brightness);
        } else {
            command_->resetBrightnessChanged();
        }
    }

    // Show status LED when active
    if (bool status_led_when_active; command_->statusLEDWhenActiveChanged(status_led_when_active)) {
        if (safeCall(
            candle_->ConfigStatusLedState(!status_led_when_active),
            "candle_->ConfigStatusLedState"
        )) {
            ROS_INFO_STREAM("CANdle " << getName() << " : Set status led when active to " << status_led_when_active);
            state_->setStatusLEDWhenActive(status_led_when_active);
        } else {
            command_->resetStatusLEDWhenActiveChanged();
        }
    }

    // Enabled
    if (bool enabled; command_->enabledChanged(enabled)) {
        if (safeCall(
            candle_->configV5Enabled(enabled),
            "candle_->configV5Enabled"
        )) {
            ROS_INFO_STREAM("CANdle " << getName() << " : Set enabled to " << enabled);
            state_->setEnabled(enabled);
        } else {
            command_->resetEnabledChanged();
        }
    }

    // Stop animations
    if (bool stop_animations; command_->stopAnimationsChanged(stop_animations)) {
        size_t max_animations = (size_t)candle_->GetMaxSimultaneousAnimationCount();
        for (size_t i = 0; i < max_animations; i++) {
            candle_->ClearAnimation(i);
        }

        command_->drainAnimations();
        state_->setMaxAnimations(max_animations);
        state_->clearAnimations();
    }

    // Animation
    if (std::vector<hardware_interface::candle::Animation> candle_animations;
        command_->animationsChanged(candle_animations)) {
        for (hardware_interface::candle::Animation& candle_animation : candle_animations) {
            // Clear any old animations
            size_t group_max = (size_t)(candle_animation.start + candle_animation.count);
            for (size_t led_id = candle_animation.start; led_id < group_max; led_id++) {
                std::optional<hardware_interface::candle::LED> led = state_->getLED(led_id);
                if (led.has_value() && led->type == hardware_interface::candle::LEDType::Animated) {
                    int animation_id = led->getAnimationID().value();
                    candle_->ClearAnimation(animation_id);

                    hardware_interface::candle::Animation existing_animation = state_->getAnimation(animation_id).value();
                    candle_->SetLEDs(
                        0,
                        0,
                        0,
                        0,
                        existing_animation.start,
                        existing_animation.count
                    );

                    state_->clearAnimation(animation_id);
                }
            }

            // A pointer to the animation, after it gets converted
            std::shared_ptr<ctre::phoenix::led::Animation> animation;

            // Convert from Animation to the appropriate CTRE animation class
            if (candle_animation.class_type == hardware_interface::candle::AnimationClass::BaseStandard) {
                animation = convertBaseStandardAnimation(candle_animation);
            } else {
                animation = convertBaseTwoAnimation(candle_animation);
            }

            size_t slot = state_->getNextAnimationSlot();

            if (safeCall(
                candle_->Animate(*animation, slot),
                "candle_->Animate"
            )) {
                ROS_INFO_STREAM("CANdle " << getName() << " : Changed its animation");
                state_->setAnimation(candle_animation);
            } else {
                // If we fail to animate, just re-add the animation to the queue
                command_->setAnimation(candle_animation);
            }
        }
        command_->drainAnimations();
    }

    // LEDs
    std::vector<hardware_interface::candle::LEDGroup> led_groups;
    if (command_->ledGroupsChanged(led_groups)) {
        command_->drainLEDGroups();
        for (hardware_interface::candle::LEDGroup group : led_groups) {
            size_t group_max = (size_t)(group.start + group.count);

            for (size_t led_id = group.start; led_id < group_max; led_id++) {
                std::optional<hardware_interface::candle::LED> led = state_->getLED(led_id);
                if (led.has_value() && led->type == hardware_interface::candle::LEDType::Animated) {
                    int animation_id = led->getAnimationID().value();
                    candle_->ClearAnimation(animation_id);

                    hardware_interface::candle::Animation animation = state_->getAnimation(animation_id).value();
                    candle_->SetLEDs(
                        0,
                        0,
                        0,
                        0,
                        animation.start,
                        animation.count
                    );

                    state_->clearAnimation(animation_id);
                }
            }

            if (safeCall(
                candle_->SetLEDs(
                    group.colour.red,
                    group.colour.green,
                    group.colour.blue,
                    group.colour.white,
                    group.start,
                    group.count
                ),
                "candle_->SetLEDs"
            )) {
                for (size_t led_id = group.start; led_id < group_max; led_id++) {
                    state_->setLED(led_id, group.colour);
                }

                ROS_INFO_STREAM("CANdle " << getName() << " : Changed colours");
            } else {
                command_->setLEDGroup(group);
            }
        }
    }
}

using namespace hardware_interface::candle;
using namespace ctre::phoenix::led;
static ColorFlowAnimation::Direction convertCANdleDirection(int direction) {
    switch (direction) {
        case 0:
            return ColorFlowAnimation::Direction::Forward;
        case 1:
            return ColorFlowAnimation::Direction::Backward;
        default:
            ROS_ERROR_STREAM("Invalid int to convert to CANdle direction! Defaulting to Forwards.");
            return ColorFlowAnimation::Direction::Forward;
    }
}

static std::shared_ptr<BaseStandardAnimation> convertBaseStandardAnimation(hardware_interface::candle::Animation animation) {
    switch (animation.type) {
        case AnimationType::Fire: {
            return std::make_shared<FireAnimation>(
                animation.brightness,
                animation.speed,
                animation.count,
                animation.param4,
                animation.param5,
                animation.reversed,
                animation.start
            );
        }
        case AnimationType::Rainbow: {
            return std::make_shared<RainbowAnimation>(
                animation.brightness,
                animation.speed,
                animation.count,
                animation.reversed,
                animation.start
            );
        }
        case AnimationType::RGBFade: {
            return std::make_shared<RgbFadeAnimation>(
                animation.brightness,
                animation.speed,
                animation.count,
                animation.start
            );
        }
        default: {
            ROS_ERROR_STREAM("Invalid animation type in " << __FUNCTION__ << " " << static_cast<int>(animation.type));
            return {};
        }
    }
}

static std::shared_ptr<BaseTwoSizeAnimation> convertBaseTwoAnimation(hardware_interface::candle::Animation animation) {
    Colour colour = animation.colour;
    switch (animation.type) {
        case AnimationType::ColourFlow: {
            return std::make_shared<ColorFlowAnimation>(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation.speed,
                animation.count,
                convertCANdleDirection(animation.direction),
                animation.start
            );
        }
        case AnimationType::Larson:
        {
            return std::make_shared<LarsonAnimation>(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation.speed,
                animation.count,
                // TODO: Store Bounce mode and Size arguments in animation class
                LarsonAnimation::BounceMode::Front,
                2,
                animation.start
            );
        }
        case AnimationType::SingleFade:
        {
            return std::make_shared<SingleFadeAnimation>(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation.speed,
                animation.count,
                animation.start
            );
        }
        case AnimationType::Strobe:
        {
            return std::make_shared<StrobeAnimation>(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation.speed,
                animation.count,
                animation.start
            );
        }
        case AnimationType::Twinkle:
        {
            return std::make_shared<TwinkleAnimation>(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation.speed,
                animation.count,
                // TODO: Store actual Divider value
                TwinkleAnimation::TwinklePercent::Percent100,
                animation.start
            );
        }
        case AnimationType::TwinkleOff:
        {
            return std::make_shared<TwinkleOffAnimation>(
                colour.red,
                colour.green,
                colour.blue,
                colour.white,
                animation.speed,
                animation.count,
                // TODO: Store actual Divider value
                TwinkleOffAnimation::TwinkleOffPercent::Percent100,
                animation.start
            );
        }
        default: {
            ROS_ERROR_STREAM("Invalid animation type in " << __FUNCTION__ << " " << static_cast<int>(animation.type));
            return {};
        }
    }
}