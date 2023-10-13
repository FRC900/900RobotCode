#ifndef PIGEON2_DEVICE_INC__
#define PIGEON2_DEVICE_INC__
#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include <nav_msgs/Odometry.h>
#include "ros_control_boilerplate/ctre_v6_device.h"

namespace ctre::phoenix6::hardware::core
{
    class CorePigeon2;
}
namespace hardware_interface
{
    class ImuSensorInterface;
    class RemoteImuSensorInterface;
    namespace pigeon2
    {
        class Pigeon2HWState;
        class Pigeon2HWCommand;
        class Pigeon2StateInterface;
        class Pigeon2CommandInterface;
        class RemotePigeon2StateInterface;
    }
}
class Tracer;

class Pigeon2Device : public CTREV6Device
{
public:
    Pigeon2Device(const std::string &name_space,
                  const int joint_index,
                  const std::string &joint_name,
                  const int pigeon2_id,
                  const std::string &can_bus,
                  const std::string &frame_id,
                  const bool local_hardware,
                  const bool local_update,
                  const double read_hz);
    Pigeon2Device(const Pigeon2Device &) = delete;
    Pigeon2Device(Pigeon2Device &&other) noexcept = delete;
    virtual ~Pigeon2Device();

    Pigeon2Device &operator=(const Pigeon2Device &) = delete;
    Pigeon2Device &operator=(Pigeon2Device &&) noexcept = delete;

    void registerInterfaces(hardware_interface::pigeon2::Pigeon2StateInterface &state_interface,
                            hardware_interface::pigeon2::Pigeon2CommandInterface &command_interface,
                            hardware_interface::pigeon2::RemotePigeon2StateInterface &remote_state_interface,
                            hardware_interface::ImuSensorInterface &imu_interface,
                            hardware_interface::RemoteImuSensorInterface &imu_remote_interface);
    void read(const ros::Time &/*time*/, const ros::Duration &/*period*/);
    void write(const ros::Time &/*time*/, const ros::Duration &/*period*/);
    void simInit(ros::NodeHandle nh, size_t joint_index);
    void simRead(const ros::Time &/*time*/, const ros::Duration &/*period*/);

private:
    const std::string frame_id_;
    const bool local_hardware_;
    const bool local_update_;

    std::unique_ptr<ctre::phoenix6::hardware::core::CorePigeon2> pigeon2_;

    std::unique_ptr<hardware_interface::pigeon2::Pigeon2HWState> state_;
    std::unique_ptr<hardware_interface::pigeon2::Pigeon2HWCommand> command_;
    std::array<double, 4> imu_orientation_{};            // x,y,z,w
    std::array<double, 9> imu_orientation_covariance_{}; // [x,y,z] x [x,y,z]
    std::array<double, 3> imu_angular_velocity_{};       // x,y,z
    std::array<double, 9> imu_angular_velocity_covariance_{};
    std::array<double, 3> imu_linear_acceleration_{};    // x,y,z
    std::array<double, 9> imu_linear_acceleration_covariance_{};
    std::unique_ptr<hardware_interface::pigeon2::Pigeon2HWState> read_thread_state_;
    std::unique_ptr<std::mutex> read_state_mutex_;
    std::unique_ptr<std::thread> read_thread_;
    void read_thread(std::unique_ptr<Tracer> tracer,
                     const double poll_frequency);

    std::atomic<double> sim_yaw_{0};
    ros::Subscriber sim_sub_;
    void imuOdomCallback(const nav_msgs::OdometryConstPtr &msg);
};

#endif