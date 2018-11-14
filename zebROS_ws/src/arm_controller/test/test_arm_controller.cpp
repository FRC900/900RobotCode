
// GTest includes
#include <gtest/gtest.h>
// #include <gtest/gmock.h>

// STL and Boost
#include <vector>
#include <boost/shared_ptr.hpp>

// Standard ROS includes required
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include <std_srvs/SetBool.h>
#include <realtime_tools/realtime_publisher.h> //code for real-time buffer - stop multple things writing to same variable at same time
#include <controller_interface/controller.h> //for writing controllers

#include <hardware_interface/joint_state_interface.h> //other than talon data

// Talon interfaces
#include <talon_interface/talon_state_interface.h> // "
#include <talon_controllers/talon_controller.h> // "
#include <talon_controllers/talon_controller_interface.h> // "

// Arm controller messages
#include <arm_controller/SetArmState.h>
#include <arm_controller/CurArmCommand.h>

/**
 * @brief Test fixture for the arm controller
 * 
 **/
using namespace arm_controller;

// class ArmControllerTest : public ::testing:Test
// {

// public:

//     ArmControllerTest() : 
//         root_nh_(ros::NodeHandle()),
//         controller_nh_(""),
//         robo_iface_(),
//         sub_(),
//         recv_msg_count_(0),
//         last_msg_()
//     {


//         // Setup the joint state interface
//         hardware_interface::RobotHW state_handle();
//         robo_iface_.registerHandle(state_handle);

//         // Subscribe to the messages
//         sub_ = root_nh_.subscribe<std_msgs::Float64>(
//             "talon_states",
//             1,
//             &ArmControllerTest::armControllerCallback,
//             this
//         );

//     }


// protected:
//     ros::NodeHandle root_nh_; //!< Root node handle
//     ros::NodeHandle controller_nh_; //!< Controller node handle
//     ros::Subscriber sub_;
//     hardware_interface::RobotHW robo_iface_;

//     int recv_msg_count_; //!< Counter for received messages
//     std_msgs::Float64 last_msg_; //!< Last received message


// } // end ArmControllerTest

/**
 * @brief 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 **/
int main(int argc, char **argv)
{
    // testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "talon_state_controller_test");

    // int ret = RUN_ALL_TESTS();
    // ros::shutdown();
    return 0;
}
