#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <image_transport/image_transport.h>
#include <glog/logging.h>

#include "gpu_apriltag/gpu_apriltag.h"
#include "apriltag_msgs/ApriltagArrayStamped.h"
#include "apriltag_msgs/ApriltagPoseStamped.h"
#include "field_obj/TFDetection.h"

namespace frc971_gpu_apriltag
{
class FRC971GpuApriltagNodelet : public nodelet::Nodelet
{
public:
    FRC971GpuApriltagNodelet(void) = default;
    ~FRC971GpuApriltagNodelet() override = default;

    void onInit() override
    {
        // Initialize Google's logging library.
        // TODO : hook it up to print using ROS_INFO calls somehow?
        // FLAGS_v = 6; // TODO : make this a param?
        google::InitGoogleLogging(getName().c_str());
        nh_ = getMTPrivateNodeHandle();
        auto base_nh = getNodeHandle();

        image_transport::ImageTransport base_it(base_nh);
        camera_sub_ = base_it.subscribeCamera("image_rect_color", 1, &FRC971GpuApriltagNodelet::callback, this);
        // Publisher for apriltag detections
        pub_apriltag_detections_ = nh_.advertise<apriltag_msgs::ApriltagArrayStamped>("tags", 1);
        // And a publisher to publish to a topic that screen to world can use directly
        pub_objdetect_ = nh_.advertise<field_obj::TFDetection>("tag_detection_msg", 1);
        // Publisher for debug image
        image_transport::ImageTransport it(nh_);
        pub_debug_image_ = it.advertise("debug_image", 1);

        // TODO: test
        // Load config for legal apriltags
        std::vector<int> _legal_tags_vec;
        nh_.param<std::vector<int>>("legal_tags", _legal_tags_vec, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16});
        for (int legal_tag : _legal_tags_vec)
        {
            legal_tags_.emplace(legal_tag);
            std::cout << legal_tag;
        }
    }

    void callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::CameraInfoConstPtr &camera_info)
    {
        ROS_ERROR_STREAM("image step = " << image->step);
        auto cv_frame = cv_bridge::toCvShare(image);

        if (!detector_)
        {
            frc971::apriltag::InputFormat input_format;
            if (cv_frame->encoding == sensor_msgs::image_encodings::MONO8)
            {
                input_format = frc971::apriltag::InputFormat::Mono8;
            }
            else if (cv_frame->encoding == sensor_msgs::image_encodings::MONO16)
            {
                input_format = frc971::apriltag::InputFormat::Mono16;
            }
            else if (cv_frame->encoding == sensor_msgs::image_encodings::BGR8)
            {
                input_format = frc971::apriltag::InputFormat::BGR8;
            }
            else if (cv_frame->encoding == sensor_msgs::image_encodings::BGRA8)
            {
                input_format = frc971::apriltag::InputFormat::BGRA8;
            }
            else
            {
                ROS_ERROR_STREAM_THROTTLE(1.0, "Unsupported image encoding " << cv_frame->encoding);
                return;
            }
            if (camera_info->height == 0 || camera_info->width == 0)
            {
                ROS_ERROR_STREAM_THROTTLE(1.0, "Camera info not available - make sure .yaml file is loaded from ~/.ros/camera_info/filename.yaml.");
                return;
            }

            detector_ = std::make_unique<frc971_gpu_apriltag::FRC971GpuApriltagDetector>(camera_info, input_format);
        }

        std::vector<GpuApriltagResult> results;
        std::vector<std::array<cv::Point2d, 4>> rejected_margin_corners;
        std::vector<std::array<cv::Point2d, 4>> rejected_noconverge_corners;
        detector_->Detect(results,
                          rejected_margin_corners,
                          rejected_noconverge_corners,
                          cv_frame->image);

        // Publish both an apriltag message and a td detection message
        // The later lets us skip using a separate node to process
        // the results into the format needed by screen to world
        apriltag_msgs::ApriltagArrayStamped apriltag_array;
        apriltag_array.header = image->header;
        apriltag_array.header.frame_id = camera_info->header.frame_id;

        field_obj::TFDetection objdetect_msg;
        objdetect_msg.header = image->header;
        objdetect_msg.header.frame_id = camera_info->header.frame_id;
        for (const auto &result : results)
        {
            // if tag id isn't found in legal tags, don't record it
            if (legal_tags_.count(result.id_) == 0)
            {
                ROS_WARN_STREAM("Filtering tag with id " << result.id_);
                continue;
            }

            apriltag_array.apriltags.emplace_back();

            auto &msg = apriltag_array.apriltags.back();
            msg.id = result.id_;
            msg.hamming = result.hamming_;
            msg.family = "36h11";
            msg.center.x = result.center_.x;
            msg.center.y = result.center_.y;

            double br_x = -std::numeric_limits<double>::max();
            double br_y = -std::numeric_limits<double>::max();
            double tl_x =  std::numeric_limits<double>::max();
            double tl_y =  std::numeric_limits<double>::max();
            for (size_t i = 0; i < msg.corners.size(); i++)
            {
                msg.corners[i].x = result.original_corners_[i].x;
                msg.corners[i].y = result.original_corners_[i].y;
                br_x = std::max(br_x, result.undistorted_corners_[i].x);
                br_y = std::max(br_y, result.undistorted_corners_[i].y);
                tl_x = std::min(tl_x, result.undistorted_corners_[i].x);
                tl_y = std::min(tl_y, result.undistorted_corners_[i].y);
            }

            objdetect_msg.objects.emplace_back();
            auto &obj = objdetect_msg.objects.back();
            obj.br.x = static_cast<float>(br_x);
            obj.br.y = static_cast<float>(br_y);
            obj.tl.x = static_cast<float>(tl_x);
            obj.tl.y = static_cast<float>(tl_y);
            obj.id = result.id_;
            obj.label = std::to_string(result.id_);
            obj.confidence = 1.0;
        }
        pub_apriltag_detections_.publish(apriltag_array);
        pub_objdetect_.publish(objdetect_msg);

        if (pub_debug_image_.getNumSubscribers() > 0)
        {
            debug_image_.header = cv_frame->header;
            debug_image_.encoding = sensor_msgs::image_encodings::BGR8;
            debug_image_.image = cv_frame->image.clone();
            // If the image is not already BGR8, convert it to BGR8 for visualization
            // with pretty(?) colors
            if (cv_frame->encoding == sensor_msgs::image_encodings::MONO8)
            {
                cv::cvtColor(debug_image_.image, debug_image_.image, cv::COLOR_GRAY2BGR);
            }
            else if (cv_frame->encoding == sensor_msgs::image_encodings::BGRA8)
            {
                cv::cvtColor(debug_image_.image, debug_image_.image, cv::COLOR_BGRA2BGR);
            }
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
                drawCorner(debug_image_.image, result.undistorted_corners_, cv::Scalar(255, 255, 0));
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
    image_transport::CameraSubscriber camera_sub_;
    ros::Publisher pub_apriltag_detections_;
    ros::Publisher pub_apriltag_poses_;
    ros::Publisher pub_objdetect_;

    image_transport::Publisher pub_debug_image_;
    cv_bridge::CvImage debug_image_;

    std::unique_ptr<frc971_gpu_apriltag::FRC971GpuApriltagDetector> detector_;

    ddynamic_reconfigure::DDynamicReconfigure ddr_;
    ros::ServiceServer save_input_image_srv_;
    std::atomic<bool> save_input_image_{false};

    std::set<int> legal_tags_;
};

} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(frc971_gpu_apriltag::FRC971GpuApriltagNodelet, nodelet::Nodelet)