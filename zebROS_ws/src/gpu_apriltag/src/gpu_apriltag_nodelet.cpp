#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <image_transport/image_transport.h>

#include "gpu_apriltag/gpu_apriltag.h"
#include "apriltag_msgs/ApriltagArrayStamped.h"
#include "apriltag_msgs/ApriltagPoseStamped.h"

namespace frc971_gpu_apriltag
{
class FRC971GpuApriltagNodelet : public nodelet::Nodelet
{
public:
    FRC971GpuApriltagNodelet(void) = default;
    ~FRC971GpuApriltagNodelet() override = default;

    void onInit() override
    {
        nh_ = getMTPrivateNodeHandle();
        auto base_nh = getNodeHandle();

        image_transport::ImageTransport base_it(base_nh);
        camera_sub_ = base_it.subscribeCamera("image_rect_color", 1, &FRC971GpuApriltagNodelet::callback, this);
        pub_apriltag_detections_ = nh_.advertise<apriltag_msgs::ApriltagArrayStamped>("tags", 2);
        pub_apriltag_poses_ = nh_.advertise<apriltag_msgs::ApriltagPoseStamped>("poses", 2);
        image_transport::ImageTransport it(nh_);
        // pub_debug_image_ = it.advertise("debug_image", 2);
        // pub_stage1_grid_debug_image_ = it.advertise("stage1_grid_debug_image", 2);
        // pub_stage1_ssd_debug_image_ = it.advertise("stage1_ssd_debug_image", 2);
        // save_input_image_srv_ = base_nh.advertiseService("save_input_image", &DeeptagRosNodelet::saveInputImageCallback, this);
    }
    void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info)
    {
        if (!detector_)
        {
            detector_ = std::make_unique<frc971_gpu_apriltag::FRC971GpuApriltagDetector>(camera_info);
        }

        cv_bridge::CvImageConstPtr cv_frame = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);

        std::vector<GpuApriltagResult> results;
        std::vector<std::array<cv::Point2f, 4>> rejected_margin_corners;
        std::vector<std::array<cv::Point2f, 4>> rejected_noconverge_corners;
        detector_->Detect(results,
                          rejected_margin_corners,
                          rejected_noconverge_corners,
                          cv_frame->image);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle base_nh_;
    std::unique_ptr<image_transport::ImageTransport> base_it_;
    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::CameraSubscriber camera_sub_;
    ros::Publisher pub_apriltag_detections_;
    ros::Publisher pub_apriltag_poses_;

    std::unique_ptr<frc971_gpu_apriltag::FRC971GpuApriltagDetector> detector_;

    ddynamic_reconfigure::DDynamicReconfigure ddr_;
    ros::ServiceServer save_input_image_srv_;
    std::atomic<bool> save_input_image_{false};
};

} // namespace
