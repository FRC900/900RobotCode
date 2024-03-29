
#include <nodelet/nodelet.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace frc971_gpu_apriltag
{
class FRC971GpuApriltagNodelet : public nodelet::Nodelet
{
public:
    FRC971GpuApriltagNodelet(void) = default;
    ~FRC971GpuApriltagNodelet() override = default;
    void onInit() override;
    void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info);
};
}
int main(void)
{

frc971_gpu_apriltag::FRC971GpuApriltagNodelet nodelet;
nodelet.onInit();
sensor_msgs::ImageConstPtr image;
sensor_msgs::CameraInfoConstPtr camera_info;
nodelet.callback(image, camera_info);
return 0;

}