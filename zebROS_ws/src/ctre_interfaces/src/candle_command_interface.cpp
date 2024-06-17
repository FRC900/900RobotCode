#include "ctre_interfaces/candle_command_interface.h"

namespace hardware_interface::candle {

CANdleHWCommand::CANdleHWCommand()
{
    for (size_t i = 0; i < 8; i++) {
        leds_.emplace_back(Colour(255, 255, 255, 255));
    }
}

void CANdleHWCommand::setLEDGroup(const LEDGroup& leds)
{
    size_t max = leds.start_ + leds.count_;
    for (size_t i = leds.start_; i < max; i++) {
        if (i < leds_.size()) {
            leds_[i] = Colour(
                leds.colour_.red_,
                leds.colour_.green_,
                leds.colour_.blue_,
                leds.colour_.white_
            );
        } else {
            if ((i - leds_.size()) > 1) {
                leds_.resize(i);
            }

            leds_.emplace_back(Colour(
                leds.colour_.red_,
                leds.colour_.green_,
                leds.colour_.blue_,
                leds.colour_.white_
            ));
        }
    }
    leds_changed_ = true;
}
bool CANdleHWCommand::ledGroupsChanged(std::vector<LEDGroup>& groups) {
    groups.clear();

    if (leds_changed_) {
        std::optional<Colour> previous_colour;
        for (size_t i = 0; i < leds_.size(); i++) {
            std::optional<Colour> led = leds_[i];

            if (led.has_value()) {
                // If the previous colour is none, or the colour has changed, start a new group
                if (!previous_colour.has_value() || previous_colour.value() != led.value()) {
                    groups.emplace_back(
                        i,
                        1,
                        Colour(
                            led->red_,
                            led->green_,
                            led->blue_,
                            led->white_
                        )
                    );
                // If the colour matches the last colour, just increase the number of LEDs in that group
                } else {
                    groups.back().count_ += 1;
                }
            }

            previous_colour = led;
        }
        leds_changed_ = false;
        leds_.clear();
        return true;
    }
    return false;
}
void CANdleHWCommand::drainLEDGroups() {
    leds_.clear();
}

void CANdleHWCommand::setBrightness(double brightness) {
    if (brightness_ != brightness) {
        ROS_INFO_STREAM("Setting CANdle brightness to: " << brightness);
        brightness_ = brightness;
        brightness_changed_ = true;
    }
}
double CANdleHWCommand::getBrightness() const{
    return brightness_;
}
bool CANdleHWCommand::brightnessChanged(double& brightness) {
    if (brightness_changed_) {
        brightness_changed_ = false;
        brightness = brightness_;
        return true;
    }
    return false;
}
void CANdleHWCommand::resetBrightnessChanged() {
    brightness_changed_ = true;
}

void CANdleHWCommand::setStatusLEDWhenActive(bool show) {
    if (show_status_led_when_active_ != show) {
        show_status_led_when_active_ = show;
        status_led_changed_ = true;
    }
}
bool CANdleHWCommand::getStatusLEDWhenActive() const {
    return show_status_led_when_active_;
}
bool CANdleHWCommand::statusLEDWhenActiveChanged(bool& show) {
    if (status_led_changed_) {
        status_led_changed_ = false;
        show = show_status_led_when_active_;
        return true;
    }
    return false;
}
void CANdleHWCommand::resetStatusLEDWhenActiveChanged() {
    status_led_changed_ = true;
}

void CANdleHWCommand::setEnabled(bool enabled) {
    if (enabled_ != enabled) {
        enabled_ = enabled;
        enabled_changed_ = true;
    }
}
bool CANdleHWCommand::getEnabled() const {
    return enabled_;
}
bool CANdleHWCommand::enabledChanged(bool& enabled) {
    if (enabled_changed_) {
        enabled_changed_ = false;
        enabled = enabled_;
        return true;
    }
    return false;
}
void CANdleHWCommand::resetEnabledChanged() {
    enabled_changed_ = true;
}

void CANdleHWCommand::setAnimation(const Animation &animation) {
    // Check if the animation is already in the list to
    // update this time
    for (const Animation& existing_animation : animations_) {
        if (animation == existing_animation) {
            ROS_INFO_STREAM("Identical animation detected. Not updating");
            return;
        }
    }

    // Check if the animation matches an existing one already
    // written to hardware. This will prevent restarting the
    // animation if it's already running.
    for (const Animation& a : current_animations_) {
        if (a == animation) {
            ROS_INFO_STREAM("New animation matching existing animation. Not updating");
            return;
        }
    }

    animations_.push_back(animation);
    animation_changed_ = true;

    // Previously checked for an exact match, so here we
    // can just check start and size and if there's an
    // existing animation with the same start and size,
    // overwrite the rest of the info with the requested animation
    for (size_t i = 0; i < current_animations_.size(); i++) {
        if ((current_animations_[i].start_ == animation.start_) &&
            (current_animations_[i].count_ == animation.count_)) {
            current_animations_[i] = animation;
            return;
        }
    }
    current_animations_.push_back(animation);
}
bool CANdleHWCommand::animationsChanged(std::vector<Animation>& animations) {
    if (animation_changed_) {
        animation_changed_ = false;
        animations = animations_;
        animations_.clear();
        return true;
    }
    return false;
}
void CANdleHWCommand::drainAnimations() {
    animations_.clear();
}

// Clear an animation from the list of currently active animations
// Do this by passing the animation config rather than the index
// since I'm not sure if the indexes will get out of sync between
// here and the state interface
void CANdleHWCommand::clearCurrentAnimation(const Animation &animation) {
    for (size_t i = 0; i < current_animations_.size(); i++) {
        if (current_animations_[i] == animation) {
            current_animations_.erase(current_animations_.begin() + i);
            return;
        }
    }
}

void CANdleHWCommand::stopAnimations() {
    stop_animations_ = true;
    current_animations_.clear();
}
bool CANdleHWCommand::stopAnimationsChanged(bool& stop) {
    if (stop_animations_) {
        stop = stop_animations_;
        stop_animations_ = false;
        return true;
    }
    return false;
}

} // namespace hardware_interface::candle