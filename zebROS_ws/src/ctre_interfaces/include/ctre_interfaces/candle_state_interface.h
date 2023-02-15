#pragma once

#include <vector>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include "state_handle/state_handle.h"
#include <optional>

namespace hardware_interface {
namespace candle {

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
    int red;
    int green;
    int blue;
    int white;

    // Constructor
    Colour(int red, int green, int blue, int white);
    Colour();

    bool operator!=(const Colour& rhs);
    bool operator==(const Colour& rhs);
};

// An animation for the CANdle to play
struct Animation {
    // Animation speed
    double speed;
    // Initial LED to animate
    int start;
    // Number of LEDs to animation
    int count;
    // The animation to play
    AnimationType type;
    // The animation class type
    AnimationClass class_type;

    // For Base Two animations
    // Animation colour
    Colour colour;
    // Animation direction
    int direction;

    // For Base Standard animations
    // Animation brightness
    double brightness;
    // If the animation is reversed
    bool reversed;
    // Extra params
    double param4;
    double param5;

    // Constructor for BaseStandard animations
    Animation(double speed, int start, int count, AnimationType type, double brightness, bool reversed, double param4, double param5);
    // Constructor for BaseTwo animations
    Animation(double speed, int start, int count, AnimationType type, int red, int green, int blue, int white, int direction);
    // Blank constructor/null
    Animation();

    // Comparison methods
    bool operator==(const Animation& rhs);
    bool operator!=(const Animation& rhs);
};

// An LED on the CANdle
enum LEDType {
    Coloured,
    Animated
};
union LEDDisplay {
    Colour colour;
    int animation_id;

    LEDDisplay() :
        animation_id{0}
    {}
};
class LED {
    public:
        LEDType type;
        
        LED(int animation_id) :
            type{LEDType::Animated}
        {
            this->display.animation_id = animation_id;
        }
        LED(Colour colour) :
            type{LEDType::Coloured}
        {
            this->display.colour = colour;
        }

        std::optional<Colour> getColour() {
            return this->type == LEDType::Coloured ? std::optional<Colour>(this->display.colour) : std::nullopt;
        }
        std::optional<int> getAnimationID() {
            return this->type == LEDType::Animated ? std::optional<int>(this->display.animation_id) : std::nullopt;
        }

        void setColour(Colour colour) {
            this->display.colour = colour;
        }
        void setAnimationID(int animation_id) {
            this->display.animation_id = animation_id;
        }

    private:
        LEDDisplay display;
};

class CANdleHWState {
    public:
        // Constructor and method to get device ID
        CANdleHWState(int id);
        int getDeviceID() const;

        // Set the colour of an LED
        void setLED(size_t id, Colour colour);
        void setLED(size_t id, int animation_id);
        void setLEDOff(size_t id);
        std::optional<LED> getLED(size_t id);

        // Set the brightness of the CANdle's LEDs
        void setBrightness(double brightness);
        double getBrightness();

        // Show status LED when the CANdle is being controlled
        void setStatusLEDWhenActive(bool show);
        bool getStatusLEDWhenActive();

        // If the CANdle is enabled
        void setEnabled(bool enabled);
        bool getEnabled();

        // The CANdle's animation
        void setAnimation(const Animation& animation);
        void clearAnimation(size_t id);
        void clearAnimations();
        void setMaxAnimations(size_t max);
        std::optional<Animation> getAnimation(size_t id);
        size_t getNextAnimationSlot();

    private:
        // The CAN ID of this CANdle
        int device_id;
        // All of the LED groups to colour
        std::vector<std::optional<LED>> leds;
        // The brightness of the LEDs in the CANdle, from 0->1
        double brightness;
        // If the status LED should be on when the CANdle is being controlled
        bool show_led_when_active;
        // If the CANdle is enabled
        bool enabled;
        // The CANdle's animations
        std::vector<std::optional<Animation>> animations;
};


typedef StateHandle<const CANdleHWState> CANdleStateHandle;
typedef StateHandle<CANdleHWState>       CANdleWritableStateHandle;
class CANdleStateInterface : public HardwareResourceManager<CANdleStateHandle> {};
class RemoteCANdleStateInterface : public HardwareResourceManager<CANdleWritableStateHandle> {};

} // namespace candle
} // namespace hardware_interface
