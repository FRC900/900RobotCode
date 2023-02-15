#pragma once

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
#include "ctre_interfaces/candle_state_interface.h"
#include <memory>

namespace candle_convert {

ctre::phoenix::led::ColorFlowAnimation::Direction convertCANdleDirection(int direction);
std::shared_ptr<ctre::phoenix::led::BaseStandardAnimation> convertBaseStandardAnimation(hardware_interface::candle::Animation animation);
std::shared_ptr<ctre::phoenix::led::BaseTwoSizeAnimation> convertBaseTwoAnimation(hardware_interface::candle::Animation animation);

} // namespace candle_convert
