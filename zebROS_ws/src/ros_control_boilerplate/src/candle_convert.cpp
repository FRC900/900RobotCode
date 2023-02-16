#include <ros/console.h>
#include "ros_control_boilerplate/candle_convert.h"

using namespace hardware_interface::candle;
using namespace ctre::phoenix::led;

namespace candle_convert {

ColorFlowAnimation::Direction convertCANdleDirection(int direction) {
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

std::shared_ptr<BaseStandardAnimation> convertBaseStandardAnimation(hardware_interface::candle::Animation animation) {
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
std::shared_ptr<BaseTwoSizeAnimation> convertBaseTwoAnimation(hardware_interface::candle::Animation animation) {
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

} // namespace candle_convert
