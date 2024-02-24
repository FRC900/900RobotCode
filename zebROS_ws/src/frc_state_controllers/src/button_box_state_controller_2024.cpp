#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <frc_msgs/ButtonBoxState2024.h>
#include <frc_interfaces/joystick_interface.h>
#include <pluginlib/class_list_macros.h>
#include "periodic_interval_counter/periodic_interval_counter.h"

namespace button_box_state_controller
{
class ButtonBoxStateController_2024 : public controller_interface::Controller<hardware_interface::JoystickStateInterface>
{
private:
 hardware_interface::JoystickStateHandle button_box_state_;
 std::unique_ptr<realtime_tools::RealtimePublisher<frc_msgs::ButtonBoxState2024>> realtime_pub_;
 std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
 double publish_rate_{50};
 frc_msgs::ButtonBoxState2024 prev_button_box_msg_;

public:
 ButtonBoxStateController_2024() = default;
 bool init(hardware_interface::JoystickStateInterface *hw,
          ros::NodeHandle                             &root_nh,
          ros::NodeHandle                             &controller_nh) override
 {
     ROS_INFO_STREAM_NAMED("button_box_state_controller_2024", "init is running");
     std::string name;
     if (!controller_nh.getParam("name", name))
     {
         ROS_ERROR("Could not read button box name parameter in ButtonBox State Controller");
         return false;
     }

     if (!controller_nh.getParam("publish_rate", publish_rate_))
     {
         ROS_WARN_STREAM("Could not read publish_rate in ButtonBox state controller, using default " << publish_rate_);
     }
     else if (publish_rate_ <= 0.0)
     {
         ROS_ERROR_STREAM("Invliad publish_rate in ButtonBox state controller (" << publish_rate_ << ")");
         return false;
     }
     interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

     std::vector<std::string> button_box_names = hw->getNames();
     const auto it = std::find(button_box_names.begin(), button_box_names.end(), name);
     if (it == button_box_names.cend())
     {
         ROS_ERROR_STREAM("Could not find requested name " << name << " in button box interface list");
         return false;
     }

     button_box_state_ = hw->getHandle(*it);
     const auto id = button_box_state_->getId();
     std::stringstream pub_name;
     // TODO : maybe use pub_names instead, or joy id unconditionally?
     pub_name << "js" << id;

     realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<frc_msgs::ButtonBoxState2024>>(root_nh, pub_name.str(), 1);

     return true;
 }

 void starting(const ros::Time &time) override
 {
     interval_counter_->reset();
 }

 void update(const ros::Time &time, const ros::Duration &period) override
 {
     if (interval_counter_->update(period))
     {
         if (realtime_pub_->trylock())
         {
             const auto &bbs = button_box_state_;
             auto &m = realtime_pub_->msg_;

             m.header.stamp = time;

             // If the buttons are pressed
             // TODO : test this using a real driver station
             m.lockingSwitchButton = bbs->getButton(6);
             m.rightGreenButton = bbs->getButton(16);
             m.leftGreenButton = bbs->getButton(9);
             m.topGreenButton = bbs->getButton(15);
             m.bottomGreenButton = bbs->getButton(8);
             m.zeroButton = bbs->getButton(1);
             m.backupButton1Button = bbs->getButton(10);
             m.redButton = bbs->getButton(11);
             m.backupButton2Button = bbs->getButton(13);
             m.speedSwitchUpButton = bbs->getButton(5);
             m.speedSwitchDownButton = bbs->getButton(4);
             m.shooterArmUpButton = bbs->getButton(3);
             m.shooterArmDownButton = bbs->getButton(7);
             // Skip 13 (that's the Arduino LED)
             m.trapButton = bbs->getButton(17);
             m.climbButton = bbs->getButton(0);
             m.subwooferShootButton = bbs->getButton(2);

            constexpr uint8_t FIRST_AUTO_BUTTON = 19;
            constexpr uint8_t NUM_AUTO_BUTTONS = 6;
            m.auto_mode = 0;
            for (uint8_t i = 0; i < NUM_AUTO_BUTTONS; i++)
            {
                if (bbs->getButton(FIRST_AUTO_BUTTON + i))
                {
                    m.auto_mode = i + 1;
                    break;
                }
            }

             // Creating press booleans by comparing the last publish to the current one
             m.lockingSwitchPress   = !prev_button_box_msg_.lockingSwitchButton   && m.lockingSwitchButton;
             m.rightGreenPress      = !prev_button_box_msg_.rightGreenButton      && m.rightGreenButton;
             m.leftGreenPress       = !prev_button_box_msg_.leftGreenButton       && m.leftGreenButton;
             m.topGreenPress        = !prev_button_box_msg_.topGreenButton        && m.topGreenButton;
             m.bottomGreenPress     = !prev_button_box_msg_.bottomGreenButton     && m.bottomGreenButton;
             m.zeroPress            = !prev_button_box_msg_.zeroButton            && m.zeroButton;
             m.backupButton1Press   = !prev_button_box_msg_.backupButton1Button   && m.backupButton1Button;
             m.redPress             = !prev_button_box_msg_.redButton             && m.redButton;
             m.backupButton2Press   = !prev_button_box_msg_.backupButton2Button   && m.backupButton2Button;
             m.speedSwitchUpPress   = !prev_button_box_msg_.speedSwitchUpButton   && m.speedSwitchUpButton;
             m.speedSwitchDownPress = !prev_button_box_msg_.speedSwitchDownButton && m.speedSwitchDownButton;
             m.shooterArmUpPress    = !prev_button_box_msg_.shooterArmUpButton    && m.shooterArmUpButton;
             m.shooterArmDownPress  = !prev_button_box_msg_.shooterArmDownButton  && m.shooterArmDownButton;
             m.trapPress            = !prev_button_box_msg_.trapButton            && m.trapButton;
             m.climbPress           = !prev_button_box_msg_.climbButton           && m.climbButton;
             m.subwooferShootPress  = !prev_button_box_msg_.subwooferShootButton  && m.subwooferShootButton;

             // Creating release booleans by comparing the last publish to the current one
             m.lockingSwitchRelease   = prev_button_box_msg_.lockingSwitchButton   && !m.lockingSwitchButton;
             m.rightGreenRelease      = prev_button_box_msg_.rightGreenButton      && !m.rightGreenButton;
             m.leftGreenRelease       = prev_button_box_msg_.leftGreenButton       && !m.leftGreenButton;
             m.topGreenRelease        = prev_button_box_msg_.topGreenButton        && !m.topGreenButton;
             m.bottomGreenRelease     = prev_button_box_msg_.bottomGreenButton     && !m.bottomGreenButton;
             m.zeroRelease            = prev_button_box_msg_.zeroButton            && !m.zeroButton;
             m.backupButton1Release   = prev_button_box_msg_.backupButton1Button   && !m.backupButton1Button;
             m.redRelease             = prev_button_box_msg_.redButton             && !m.redButton;
             m.backupButton2Release   = prev_button_box_msg_.backupButton2Button   && !m.backupButton2Button;
             m.speedSwitchUpRelease   = prev_button_box_msg_.speedSwitchUpButton   && !m.speedSwitchUpButton;
             m.speedSwitchDownRelease = prev_button_box_msg_.speedSwitchDownButton && !m.speedSwitchDownButton;
             m.shooterArmUpRelease    = prev_button_box_msg_.shooterArmUpButton    && !m.shooterArmUpButton;
             m.shooterArmDownRelease  = prev_button_box_msg_.shooterArmDownButton  && !m.shooterArmDownButton;
             m.trapRelease            = prev_button_box_msg_.trapButton            && !m.trapButton;
             m.climbRelease           = prev_button_box_msg_.climbButton           && !m.climbButton;
             m.subwooferShootRelease  = prev_button_box_msg_.subwooferShootButton  && !m.subwooferShootButton;

             realtime_pub_->unlockAndPublish();
             prev_button_box_msg_ = m;
         }
         else
         {
             interval_counter_->force_publish();
         }
     }
 }

 void stopping(const ros::Time & ) override
 {
 }

}; //class

} // namespace

PLUGINLIB_EXPORT_CLASS(button_box_state_controller::ButtonBoxStateController_2024, controller_interface::ControllerBase)
