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
        camera_sub_ = base_it.subscribeCamera("image_rect_color", 10, &FRC971GpuApriltagNodelet::callback, this);
        pub_apriltag_detections_ = nh_.advertise<apriltag_msgs::ApriltagArrayStamped>("tags", 1);
        pub_apriltag_poses_ = nh_.advertise<apriltag_msgs::ApriltagPoseStamped>("poses", 1);
        image_transport::ImageTransport it(nh_);
        pub_debug_image_ = it.advertise("debug_image", 1);
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
        std::vector<std::array<cv::Point2d, 4>> rejected_margin_corners;
        std::vector<std::array<cv::Point2d, 4>> rejected_noconverge_corners;
        detector_->Detect(results,
                          rejected_margin_corners,
                          rejected_noconverge_corners,
                          cv_frame->image);

        apriltag_msgs::ApriltagArrayStamped apriltag_array;
        apriltag_array.header = image->header;
        apriltag_array.header.frame_id = camera_info->header.frame_id;
        for (const auto &result : results)
        {
            apriltag_array.apriltags.emplace_back();

            auto &msg = apriltag_array.apriltags.back();
            msg.id = result.id_;
            msg.hamming = result.hamming_;
            msg.family = "36h11";
            msg.center.x = result.center_.x;
            msg.center.y = result.center_.y;
            
            for (size_t i = 0; i < msg.corners.size(); i++)
            {
                msg.corners[i].x = result.undistorted_corners_[i].x;
                msg.corners[i].y = result.undistorted_corners_[i].y;
            }
        }
        pub_apriltag_detections_.publish(apriltag_array);

        if (pub_debug_image_.getNumSubscribers() > 0)
        {
            debug_image_.header = cv_frame->header;
            debug_image_.encoding = sensor_msgs::image_encodings::BGR8;
            debug_image_.image = cv_frame->image.clone();
            for (const auto &corner : rejected_margin_corners)
            {
                drawCorner(debug_image_.image, corner, cv::Scalar(255, 0, 0));
            }
            for (const auto &corner : rejected_noconverge_corners)
            {
                drawCorner(debug_image_.image, corner, cv::Scalar(255, 166, 0));
            }
            for (const auto &result : results)
            {
                drawCorner(debug_image_.image, result.original_corners_, cv::Scalar(0, 255, 0));
                drawCorner(debug_image_.image, result.undistorted_corners_, cv::Scalar(255, 200, 200));
            }
            pub_debug_image_.publish(debug_image_.toImageMsg());
        }
    }

private:
    void drawCorner(cv::Mat &image, const std::array<cv::Point2d, 4> &corner, const cv::Scalar &color) const
    {
        for (size_t i = 0; i < corner.size(); i++)
        {
            cv::line(image, corner[i], corner[(i + 1) % corner.size()], color, 2);
        }
    }
    ros::NodeHandle nh_;
    ros::NodeHandle base_nh_;
    std::unique_ptr<image_transport::ImageTransport> base_it_;
    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::CameraSubscriber camera_sub_;
    ros::Publisher pub_apriltag_detections_;
    ros::Publisher pub_apriltag_poses_;

    image_transport::Publisher pub_debug_image_;
    cv_bridge::CvImage debug_image_;

    std::unique_ptr<frc971_gpu_apriltag::FRC971GpuApriltagDetector> detector_;

    ddynamic_reconfigure::DDynamicReconfigure ddr_;
    ros::ServiceServer save_input_image_srv_;
    std::atomic<bool> save_input_image_{false};
};

} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(frc971_gpu_apriltag::FRC971GpuApriltagNodelet, nodelet::Nodelet)