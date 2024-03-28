#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <image_transport/image_transport.h>

namespace frc971_gpu_apriltag
{
class DeeptagRosNodelet : public nodelet::Nodelet
{
public:
    DeeptagRosNodelet(void) = default;
    ~DeeptagRosNodelet() override = default;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle base_nh_;
    std::unique_ptr<image_transport::ImageTransport> base_it_;
    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::CameraSubscriber camera_sub_;
    ros::Publisher pub_apriltag_detections_;
    ros::Publisher pub_apriltag_poses_;

    ddynamic_reconfigure::DDynamicReconfigure ddr_;
    ros::ServiceServer save_input_image_srv_;
    std::atomic<bool> save_input_image_{false};
};
} // namespace
