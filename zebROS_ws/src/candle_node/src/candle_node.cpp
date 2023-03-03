#include <candle_controller_msgs/Colour.h>
#include <candle_controller_msgs/Animation.h>
#include <ros/ros.h>
#include <frc_msgs/MatchSpecificData.h>
#include <frc_msgs/ButtonBoxState2023.h>
#include <behavior_actions/AutoState.h>

constexpr uint8_t MAX_LED = 34;
constexpr uint8_t MID_START = 0;
constexpr uint8_t MID_COUNT = 8;
constexpr uint8_t RIGHT_START = 8;
constexpr uint8_t RIGHT_COUNT = 13;
constexpr uint8_t LEFT_START = 21;
constexpr uint8_t LEFT_COUNT = 13;

struct NodeCTX {
    uint8_t team_colour;
    bool cone_button_pressed;
    bool cube_button_pressed;
    bool updated;
    bool disabled;
    uint8_t auto_mode;


    NodeCTX() :
        team_colour{3},
        cone_button_pressed{false},
        cube_button_pressed{false},
        updated{false}
    {}

    void team_colour_callback(const frc_msgs::MatchSpecificData& msg) {
        disabled = msg.Disabled;
        if (msg.allianceColor != this->team_colour) {
            ROS_INFO_STREAM("Updating alliance colour LEDs...");
            this->team_colour = msg.allianceColor;
            this->updated = true;
        }
    }

    void button_box_callback(const frc_msgs::ButtonBoxState2023& msg) {
        if (disabled) {
            if (1 <= auto_mode && auto_mode <= 3) {
                
            }
        }

        if (msg.topMiddleConePress && !this->cone_button_pressed) {
            this->cone_button_pressed = true;
            this->updated = true;
        } else if (msg.topMiddleConeRelease && this->cone_button_pressed) {
            this->cone_button_pressed = false;
            this->updated = true;
        } else if (msg.topRightCubePress && !this->cube_button_pressed) {
            this->cube_button_pressed = true;
            this->updated = true;
        } else if (msg.topRightCubeRelease && this->cube_button_pressed) {
            this->cube_button_pressed bv= false;
            this->updated = true;
        }
    }

    void auto_mode_callback(const behavior_actions::AutoMode& msg) {
        auto_mode = msg.auto_mode;
    }

};

int main(int argc, char **argv) {
    // ROS node
    ros::init(argc, argv, "candle_node");
    ros::NodeHandle node;
    ros::Duration delay(1, 0);
    NodeCTX ctx;

    // Team colour subscriber/callback
    ros::Subscriber team_colour_subscriber = node.subscribe("/frcrobot_rio/match_data", 0, &NodeCTX::team_colour_callback, &ctx);
    ros::Subscriber button_box_subscriber = node.subscribe("/frcrobot_rio/js1", 100, &NodeCTX::button_box_callback, &ctx);

    // ROS service clients (setting the CANdle)
    ros::ServiceClient colour_client = node.serviceClient<candle_controller_msgs::Colour>("candle_controller/colour");
    ros::ServiceClient animation_client = node.serviceClient<candle_controller_msgs::Animation>("candle_controller/animation");

    // Wait for the CANdle services to start
    colour_client.waitForExistence();
    animation_client.waitForExistence();

    // Clear the CANdle LEDs
    candle_controller_msgs::Colour clear_req;
    clear_req.request.start = 0;
    clear_req.request.count = MAX_LED;
    while (!colour_client.call(clear_req)) {
        delay.sleep();
    }
    ROS_INFO_STREAM("Candle node running; cleared LEDs.");
    delay.sleep();

    // ROS loop
    while (ros::ok()) {
        // Robot alliance colour changed
        if (ctx.updated) {
            candle_controller_msgs::Colour colour_req;
            colour_req.request.start = 0;
            colour_req.request.count = MAX_LED;

            if (ctx.cone_button_pressed) {
                // Yellow colour
                colour_req.request.red = 255;
                colour_req.request.green = 150;
            } else if (ctx.cube_button_pressed) {
                // Purple colour
                colour_req.request.red = 150;
                colour_req.request.blue = 255;
            } else if (ctx.team_colour == 0) {
                // Red colour
                colour_req.request.red = 255;
            } else if (ctx.team_colour == 1) {
                // Blue colour
                colour_req.request.blue = 255;
            }

            if (colour_client.call(colour_req)) {
                ROS_INFO_STREAM("Updated LEDs");
                ctx.updated = false;
            } else {
                ROS_ERROR_STREAM("Failed to update LEDs");
            }
        }

        ros::spinOnce();
    }
    return 0;
}
