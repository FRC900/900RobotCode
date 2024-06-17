#ifndef CANDLE_STATE_INTERFACE_INC__
#define CANDLE_STATE_INTERFACE_INC__

#include <hardware_interface/internal/hardware_resource_manager.h>
#include "state_handle/state_handle.h"
#include <optional>
#include <variant>
#include <vector>

namespace hardware_interface::candle {

static constexpr size_t MAX_ANIMATION_SLOTS = 8;

// An enum for all the animations the CANdle can play
enum class AnimationType {
    ColourFlow,
    Fire,
    Larson,
    Rainbow,
    RGBFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff
};

enum class AnimationClass {
    BaseStandard,
    BaseTwo
};

// A CANdle colour
struct Colour {
    int red_{0};
    int green_{0};
    int blue_{0};
    int white_{0};

    // Constructor
    Colour(int red, int green, int blue, int white);
    Colour() = default;

    bool operator!=(const Colour& rhs) const;
    bool operator==(const Colour& rhs) const;
};

// An animation for the CANdle to play
struct Animation {
    // Animation speed
    double speed_{0.0};
    // Initial LED to animate
    int start_{0};
    // Number of LEDs to animation
    int count_{0};
    // The animation to play
    AnimationType type_{AnimationType::ColourFlow};
    // The animation class type
    AnimationClass class_type_{AnimationClass::BaseStandard};

    // For Base Two animations
    // Animation colour
    Colour colour_{};
    // Animation direction
    // Also encodes twinke percent for Twinkle/TwinkleOff animations
    int direction_{0};

    // For Base Standard animations
    // Animation brightness
    double brightness_{1.0};
    // If the animation is reversed
    bool reversed_{false};
    // Extra params
    double param4_{0.0};
    double param5_{0.0};

    // Constructor for BaseStandard animations
    Animation(double speed, int start, int count, AnimationType type, double brightness, bool reversed, double param4, double param5);
    // Constructor for BaseTwo animations
    Animation(double speed, int start, int count, AnimationType type, int red, int green, int blue, int white, int direction);
    // Blank constructor/null
    Animation() = default;

    // Comparison methods
    bool operator==(const Animation& rhs) const;
    bool operator!=(const Animation& rhs) const;
};

// An LED on the CANdle
class LED {
    public:
        explicit LED(size_t animation_id) :
            display_{animation_id}
        {
        }
        explicit LED(Colour colour) :
            display_{colour}
        {
        }

        std::optional<Colour> getColour() const {
            return std::holds_alternative<Colour>(display_) ? std::optional<Colour>(std::get<Colour>(display_)) : std::nullopt;
        }
        std::optional<size_t> getAnimationID() const {
            return std::holds_alternative<size_t>(display_) ? std::optional<size_t>(std::get<size_t>(display_)) : std::nullopt;
        }

        void setColour(Colour colour) {
            display_ = colour;
        }
        void setAnimationID(size_t animation_id) {
            display_ = animation_id;
        }

    private:
        std::variant<Colour, size_t> display_;
};

class CANdleHWState {
    public:
        // Constructor and method to get device ID
        explicit CANdleHWState(int id);
        CANdleHWState(const CANdleHWState& other) = delete;
        CANdleHWState(CANdleHWState&& other) noexcept = delete;
        CANdleHWState& operator=(const CANdleHWState& other) = delete;
        CANdleHWState& operator=(CANdleHWState&& other) noexcept = delete;
        virtual ~CANdleHWState() = default;

        int getDeviceID() const;

        // Set the colour of an LED
        void setLED(const size_t id, const Colour &colour);
        void setLED(const size_t id, const size_t animation_id);
        void setLEDOff(const size_t id);
        std::optional<LED> getLED(size_t id) const;

        size_t getLEDCount() const;

        // Set the brightness of the CANdle's LEDs
        void setBrightness(const double brightness);
        double getBrightness() const;

        // Show status LED when the CANdle is being controlled
        void setStatusLEDWhenActive(const bool show);
        bool getStatusLEDWhenActive() const;

        // If the CANdle is enabled
        void setEnabled(const bool enabled);
        bool getEnabled() const;

        // The CANdle's animation
        void setAnimation(const Animation& animation);
        void clearAnimation(const size_t id);
        void clearAnimations();
        void setMaxAnimations(const size_t max);
        size_t getAnimationCount(void) const;
        std::optional<Animation> getAnimation(const size_t id) const;
        size_t getNextAnimationSlot();

    private:
        // The CAN ID of this CANdle
        int device_id_;
        // All of the LED groups to colour
        std::vector<std::optional<LED>> leds_;
        // The brightness of the LEDs in the CANdle, from 0->1
        double brightness_{1.0};
        // If the status LED should be on when the CANdle is being controlled
        bool show_led_when_active_{false};
        // If the 5v CANdle output is enabled
        bool enabled_{false};
        // The CANdle's animations
        std::vector<std::optional<Animation>> animations_;
};

using CANdleStateHandle = StateHandle<const CANdleHWState>;
using CANdleWritableStateHandle = StateHandle<CANdleHWState>;
class CANdleStateInterface : public HardwareResourceManager<CANdleStateHandle> {};
class RemoteCANdleStateInterface : public HardwareResourceManager<CANdleWritableStateHandle, ClaimResources> {};

} // namespace hardware_interface::candle

#endif // CANDLE_STATE_INTERFACE_INC__