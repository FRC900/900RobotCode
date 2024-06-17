#include "ctre_interfaces/candle_state_interface.h"

namespace hardware_interface::candle {

Colour::Colour(int red, int green, int blue, int white) 
    : red_{red}
    , green_{green}
    , blue_{blue}
    , white_{white}
{
}
bool Colour::operator!=(const Colour& rhs) const {
    return !(*this == rhs);
}
bool Colour::operator==(const Colour& rhs) const {
    return (
        red_ == rhs.red_ &&
        green_ == rhs.green_ &&
        blue_ == rhs.blue_ &&
        white_ == rhs.white_
    );
}

Animation::Animation(double speed, int start, int count, AnimationType type, double brightness, bool reversed, double param4, double param5) 
    : speed_{speed}
    , start_{start}
    , count_{count}
    , type_{type}
    , class_type_{AnimationClass::BaseStandard}
    , brightness_{brightness}
    , reversed_{reversed}
    , param4_{param4}
    , param5_{param5}
{
}
Animation::Animation(double speed, int start, int count, AnimationType type, int red, int green, int blue, int white, int direction) 
    : speed_{speed}
    , start_{start}
    , count_{count}
    , type_{type}
    , class_type_{AnimationClass::BaseTwo}
    , colour_{red, green, blue, white}
    , direction_{direction}
{
}

bool Animation::operator==(const Animation& rhs) const {
    if (class_type_ == rhs.class_type_) {
        if (class_type_ == AnimationClass::BaseStandard) {
            return (
                (speed_ == rhs.speed_) &&
                (start_ == rhs.start_) &&
                (count_ == rhs.count_) &&
                (brightness_ == rhs.brightness_) &&
                (param4_ == rhs.param4_) &&
                (param5_ == rhs.param5_) &&
                (type_ == rhs.type_)
            );
        } else {
            return (
                (speed_ == rhs.speed_) &&
                (start_ == rhs.start_) &&
                (count_ == rhs.count_) &&
                (colour_ == rhs.colour_) &&
                (direction_ == rhs.direction_) &&
                (type_ == rhs.type_)
            );
        }
    } else {
        return false;
    }
}
bool Animation::operator!=(const Animation& rhs) const {
    return !(*this == rhs);
}

CANdleHWState::CANdleHWState(int id) :
    device_id_{id}
{
}

void CANdleHWState::setLED(const size_t id, const Colour &colour) {
    if (id < leds_.size()) {
        leds_[id].emplace(colour);
    } else {
        if (id > leds_.size()) {
            leds_.resize(id);
        }
        leds_.emplace_back(LED(colour));
    }
}
void CANdleHWState::setLED(const size_t id, const size_t animation_id) {
    if (id < leds_.size()) {
        leds_[id].emplace(animation_id);
    } else {
        if (id > leds_.size()) {
            leds_.resize(id);
        }
        leds_.emplace_back(LED(animation_id));
    }
}
void CANdleHWState::setLEDOff(const size_t id) {
    leds_[id].reset();
}
std::optional<LED> CANdleHWState::getLED(const size_t id) const {
    if (id < leds_.size()) {
        return leds_[id];
    } else {
        return std::nullopt;
    }
}

size_t CANdleHWState::getLEDCount() const {
    return leds_.size();
}

void CANdleHWState::setBrightness(const double brightness) {
    brightness_ = brightness;
}
double CANdleHWState::getBrightness() const {
    return brightness_;
}

void CANdleHWState::setStatusLEDWhenActive(const bool show) {
    show_led_when_active_ = show;
}
bool CANdleHWState::getStatusLEDWhenActive() const {
    return show_led_when_active_;
}

void CANdleHWState::setEnabled(const bool enabled) {
    enabled_ = enabled;
}
bool CANdleHWState::getEnabled() const {
    return enabled_;
}

void CANdleHWState::setAnimation(const Animation& animation) {
    size_t animation_id = this->getNextAnimationSlot();
    if (animation_id == MAX_ANIMATION_SLOTS) {
        ROS_ERROR_STREAM("Failed to set CANdle animation - no available slots!");
        return;
    }
    if (animation_id == animations_.size()) {
        animations_.emplace_back(animation);
    } else {
        animations_[animation_id].emplace(animation);
    }
    for (int led_id = animation.start_; led_id < (animation.start_ + animation.count_); led_id++) {
        this->setLED(led_id, animation_id);
    }
}
void CANdleHWState::clearAnimation(const size_t id) {
    if (id < animations_.size() && animations_[id].has_value()) {
        const Animation animation = animations_[id].value();
        for (size_t led_id = animation.start_; led_id < (size_t)(animation.start_ + animation.count_); led_id++) {
            this->setLEDOff(led_id);
        }
        animations_[id].reset();
    } else {
        ROS_ERROR_STREAM("Attempted to clear CANdle animation with invalid ID!");
    }
}
void CANdleHWState::setMaxAnimations(const size_t max) {
    animations_.resize(max, std::nullopt);
}
void CANdleHWState::clearAnimations() {
    for (auto &a : animations_) {
        a.reset();
    }
}
std::optional<Animation> CANdleHWState::getAnimation(const size_t id) const {
    if (id < animations_.size()) {
        return animations_[id];
    }

    return std::nullopt;
}
size_t CANdleHWState::getNextAnimationSlot() {
    size_t anim_idx = 0;
    while (anim_idx < animations_.size() && animations_[anim_idx].has_value()) {
        anim_idx++;
    }
    if (anim_idx < MAX_ANIMATION_SLOTS) {
        return anim_idx;
    }
    ROS_ERROR_STREAM("Failed to get available animation slot for CANdle!");
    return MAX_ANIMATION_SLOTS;
}
size_t CANdleHWState::getAnimationCount() const {
    return animations_.size();
}

int CANdleHWState::getDeviceID() const {
    return device_id_;
}

} // namespace hardware_interface::candle