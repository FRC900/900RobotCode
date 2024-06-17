#ifndef CANDLE_COMMAND_INTERFACE_INC__
#define CANDLE_COMMAND_INTERFACE_INC__

#include "ctre_interfaces/candle_state_interface.h"
#include "state_handle/command_handle.h"
#include <optional>
#include <vector>

namespace hardware_interface::candle {

struct LEDGroup {
    // Start LED
    int start_;
    // Number of LEDs this group covers
    int count_;
    // Colour to set the LEDs to
    Colour colour_;

    // Constructor
    LEDGroup(int start, int count, const Colour &colour) :
        start_{start},
        count_{count},
        colour_{colour}
    {}
    // Blank constructor for arrays and vectors
    LEDGroup() = default;
};

class CANdleHWCommand {
    public:
        // Constructor
        CANdleHWCommand();

        CANdleHWCommand(const CANdleHWCommand& other) = delete;
        CANdleHWCommand(CANdleHWCommand&& other) noexcept = delete;
        CANdleHWCommand& operator=(const CANdleHWCommand& other) = delete;
        CANdleHWCommand& operator=(CANdleHWCommand&& other) noexcept = delete;
        virtual ~CANdleHWCommand() = default;

        // Set colour of LEDs
        void setLEDGroup(const LEDGroup& leds);
        bool ledGroupsChanged(std::vector<LEDGroup>& groups);
        void drainLEDGroups();

        // Set brightness of LEDs
        void setBrightness(const double brightness);
        double getBrightness() const;
        bool brightnessChanged(double& brightness);
        void resetBrightnessChanged();

        // If the status LED should be shown when running
        void setStatusLEDWhenActive(const bool show);
        bool getStatusLEDWhenActive() const;
        bool statusLEDWhenActiveChanged(bool& show);
        void resetStatusLEDWhenActiveChanged();

        // If the CANdle is enabled
        void setEnabled(const bool enabled);
        bool getEnabled() const;
        bool enabledChanged(bool& enabled);
        void resetEnabledChanged();

        // The CANdle's animation
        void setAnimation(const Animation &animation);
        bool animationsChanged(std::vector<Animation>& animation);
        void drainAnimations();

        void clearCurrentAnimation(const Animation &animation);

        // Stop CANdle animations
        void stopAnimations();
        bool stopAnimationsChanged(bool& stop);

    private:
        // LEDs to be written
        std::vector<std::optional<Colour>> leds_;
        bool leds_changed_{true};
        // Brightness of LEDs
        double brightness_{1.0};
        bool brightness_changed_{true};
        // Status LED when active
        bool show_status_led_when_active_{true};
        bool status_led_changed_{true};
        // If the 5v CANdle output is enabled
        bool enabled_{true};
        bool enabled_changed_{true};
        // Animations to be written
        std::vector<Animation> animations_;
        bool animation_changed_{false};

        // Animations already written - used to prevent duplicate
        // writes of already-programmed animations
        std::vector<Animation> current_animations_;

        // If we should reset all the animations
        bool stop_animations_{true};
};


using CANdleCommandHandle = CommandHandle<CANdleHWCommand, CANdleHWState, CANdleStateHandle>;
class CANdleCommandInterface : public HardwareResourceManager<CANdleCommandHandle, ClaimResources> {};

} // namespace hardware_interface::candle

#endif