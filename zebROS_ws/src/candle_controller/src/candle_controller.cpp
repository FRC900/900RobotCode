#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <ctre_interfaces/candle_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <pluginlib/class_list_macros.h>

#include <string>
#include <optional>

#include <candle_controller_msgs/Brightness.h>
#include <candle_controller_msgs/Colour.h>
#include <candle_controller_msgs/Animation.h>

using namespace hardware_interface::candle;

AnimationType convertAnimationType(int type) {
    switch (type) {
        case 0: {
            return AnimationType::ColourFlow;
        }
        case 1: {
            return AnimationType::Fire;
        }
        case 2: {
            return AnimationType::Larson;
        }
        case 3: {
            return AnimationType::Rainbow;
        }
        case 4: {
            return AnimationType::RGBFade;
        }
        case 5: {
            return AnimationType::SingleFade;
        }
        case 6: {
            return AnimationType::Strobe;
        }
        case 7: {
            return AnimationType::Twinkle;
        }
        case 8: {
            return AnimationType::TwinkleOff;
        }
        default: {
            ROS_ERROR_STREAM("Invalid int to convert to AnimationType! Defaulting to ColourFlow.");
            return AnimationType::ColourFlow;
        }
    }
}
AnimationClass getAnimationClass(AnimationType type) {
    switch (type) {
        case AnimationType::ColourFlow: {
            return AnimationClass::BaseTwo;
        }
        case AnimationType::Fire: {
            return AnimationClass::BaseStandard;
        }
        case AnimationType::Larson: {
            return AnimationClass::BaseTwo;
        }
        case AnimationType::Rainbow: {
            return AnimationClass::BaseStandard;
        }
        case AnimationType::RGBFade: {
            return AnimationClass::BaseStandard;
        }
        case AnimationType::SingleFade: {
            return AnimationClass::BaseTwo;
        }
        case AnimationType::Strobe: {
            return AnimationClass::BaseTwo;
        }
        case AnimationType::Twinkle: {
            return AnimationClass::BaseTwo;
        }
        case AnimationType::TwinkleOff: {
            return AnimationClass::BaseTwo;
        }
        default: {
            ROS_ERROR_STREAM("Invalid AnimationType passed to candle_controller::getAnimationClass! Defaulting to AnimationClass::BaseTwo.");
            return AnimationClass::BaseTwo;
        }
    }
}

namespace candle_controller {
class CANdleController : public controller_interface::Controller<CANdleCommandInterface> {
public:
    CANdleController() {
        this->brightness_buffer.writeFromNonRT(1.0);
    }

    bool init(
        CANdleCommandInterface* candle_command_interface,
        ros::NodeHandle& /*root_nh*/,
        ros::NodeHandle& controller_nh
    ) override {
        std::string candle_name;
        if (!controller_nh.getParam("name", candle_name)) {
            ROS_ERROR("Cannot initialize candle! Failed to read the 'name' field.==========");
            return false;
        }

        this->candle_handle = candle_command_interface->getHandle(candle_name);

        this->colour_service = controller_nh.advertiseService("colour", &CANdleController::colourCallback, this);
        this->brightness_service = controller_nh.advertiseService("brightness", &CANdleController::brightnessCallback, this);
        this->animation_service = controller_nh.advertiseService("animation", &CANdleController::animationCallback, this);
        return true;
    }

    void starting(const ros::Time&) override {}

    void update(const ros::Time&, const ros::Duration&) override {
        const LEDGroupStamped leds = *(this->led_buffer.readFromRT());
        const AnimationStamped animation = *(this->animation_buffer.readFromRT());
        const double brightness = *(this->brightness_buffer.readFromRT());

        // Brightness isn't exclusive to colours/animations, so it gets special treatment
        this->candle_handle->setBrightness(brightness);

        if (leds.time > this->last_write || animation.time > this->last_write) {
            if (leds.time > animation.time) {
                ROS_INFO_STREAM("Writing LED group to CANdle");
                this->last_write = leds.time;
                this->candle_handle->setLEDGroup(leds.group);
            } else {
                ROS_INFO_STREAM("Writing new animation to CANdle");
                this->last_write = animation.time;
                this->candle_handle->setAnimation(animation.animation);
            }
        }
    }

    void stopping(const ros::Time&) override {}

private:
    struct LEDGroupStamped {
        ros::Time time;
        LEDGroup group;

        LEDGroupStamped() {}
    };

    struct AnimationStamped {
        Animation animation;
        ros::Time time;

        AnimationStamped() {}
    };

    CANdleCommandHandle candle_handle;
    ros::ServiceServer colour_service;
    ros::ServiceServer brightness_service;
    ros::ServiceServer animation_service;
    ros::Time last_write;

    realtime_tools::RealtimeBuffer<LEDGroupStamped> led_buffer;
    realtime_tools::RealtimeBuffer<AnimationStamped> animation_buffer;
    realtime_tools::RealtimeBuffer<double> brightness_buffer;

    bool colourCallback(candle_controller_msgs::Colour::Request& req, candle_controller_msgs::Colour::Response&) {
        if (this->isRunning()) {
            LEDGroupStamped leds;
            leds.group = LEDGroup(req.start, req.count, Colour(req.red, req.green, req.blue, req.white));
            leds.time = ros::Time::now();

            this->led_buffer.writeFromNonRT(leds);

            return true;
        } else {
            ROS_ERROR_STREAM("Can't accept new commands. CANdleController is not running.");
            return false;
        }
    }

    bool brightnessCallback(candle_controller_msgs::Brightness::Request& req, candle_controller_msgs::Brightness::Response&) {
        if (this->isRunning()) {
            this->brightness_buffer.writeFromNonRT(req.brightness);

            return true;
        } else {
            ROS_ERROR_STREAM("Can't accept new commands. CANdleController is not running.");
            return false;
        }
    }

    bool animationCallback(candle_controller_msgs::Animation::Request& req, candle_controller_msgs::Animation::Response&) {
        if (this->isRunning()) {
            AnimationStamped animation;
            animation.time = ros::Time::now();
            AnimationType animation_type = convertAnimationType(req.animation_type);
            AnimationClass animation_class = getAnimationClass(animation_type);

            switch (animation_class) {
                // Base Standard animation
                case AnimationClass::BaseStandard: {
                    animation.animation = Animation(
                        req.speed,
                        req.start,
                        req.count,
                        animation_type,
                        req.brightness,
                        req.reversed,
                        req.param4,
                        req.param5
                    );
                }
                break;
                // Base Two animation
                case AnimationClass::BaseTwo: {
                    animation.animation = Animation(
                        req.speed,
                        req.start,
                        req.count,
                        animation_type,
                        req.red,
                        req.green,
                        req.blue,
                        req.white,
                        req.direction
                    );
                }
                break;
            }

            ROS_INFO_STREAM("Setting CANdle animation to animation with ID " << (int) animation.animation.class_type);
            
            this->animation_buffer.writeFromNonRT(animation);

            return true;
        } else {
            ROS_ERROR_STREAM("Can't accept new commands. CANdleController is not running.");
            return false;
        }
    }
};
} // namespace candle_controller

PLUGINLIB_EXPORT_CLASS(candle_controller::CANdleController, controller_interface::ControllerBase)
