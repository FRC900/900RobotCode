#pragma once

#include "ctre_interfaces/candle_state_interface.h"
#include "state_handle/command_handle.h"
#include <vector>
#include <optional>

namespace hardware_interface {
namespace candle {

struct LEDGroup {
    // Start LED
    int start;
    // Number of LEDs this group covers
    int count;
    // Colour to set the LEDs to
    Colour colour;

    // Constructor
    LEDGroup(int start, int count, Colour colour) :
        start{start},
        count{count},
        colour{colour}
    {}
    // Blank constructor for arrays and vectors
    LEDGroup() {}
};

class CANdleHWCommand {
    public:
        // Constructor
        CANdleHWCommand();

        // Set colour of LEDs
        void setLEDGroup(const LEDGroup& leds);
        bool ledGroupsChanged(std::vector<LEDGroup>& groups);
        void drainLEDGroups();

        // Set brightness of LEDs
        void setBrightness(double brightness);
        double getBrightness();
        bool brightnessChanged(double& brightness);
        void resetBrightnessChanged();

        // If the status LED should be shown when running
        void setStatusLEDWhenActive(bool show);
        bool getStatusLEDWhenActive();
        bool statusLEDWhenActiveChanged(bool& show);
        void resetStatusLEDWhenActiveChanged();

        // If the CANdle is enabled
        void setEnabled(bool enabled);
        bool getEnabled();
        bool enabledChanged(bool& enabled);
        void resetEnabledChanged();

        // The CANdle's animation
        void setAnimation(Animation animation);
        bool animationsChanged(std::vector<Animation>& animation);
        void drainAnimations();

        // Stop CANdle animations
        void stopAnimations();
        bool stopAnimationsChanged(bool& stop);

    private:
        // LEDs to be written
        std::vector<std::optional<Colour>> leds;
        bool leds_changed;
        // Brightness of LEDs
        double brightness;
        bool brightness_changed;
        // Status LED when active
        bool show_status_led_when_active;
        bool status_led_changed;
        // If the CANdle is enabled
        bool enabled;
        bool enabled_changed;
        // Animations to be written
        std::vector<Animation> animations;
        bool animation_changed;
        // If we should reset all the animations
        bool stop_animations;
};


typedef CommandHandle<CANdleHWCommand, CANdleHWState, CANdleStateHandle> CANdleCommandHandle;
class CANdleCommandInterface : public HardwareResourceManager<CANdleCommandHandle, ClaimResources> {};

} // namespace candle
} // namespace hardware_interface
