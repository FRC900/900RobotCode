#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <image_transport/image_transport.h>
#include <glog/logging.h>
#include "gpu_apriltag_msgs/SetAllowedTags.h"

#include "gpu_apriltag/gpu_apriltag.h"
#include "apriltag_msgs/ApriltagArrayStamped.h"
// #include "apriltag_msgs/ApriltagPoseStamped.h"
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
        camera_sub_ = base_it.subscribeCamera("image_raw", 1, &FRC971GpuApriltagNodelet::callback, this);
        // Publisher for apriltag detections
        pub_apriltag_detections_ = nh_.advertise<apriltag_msgs::ApriltagArrayStamped>("tags", 1);
        // And a publisher to publish to a topic that screen to world can use directly
        pub_objdetect_ = nh_.advertise<field_obj::TFDetection>("tag_detection_msg", 1);
        // Publisher for debug image
        image_transport::ImageTransport it(nh_);
        pub_debug_image_ = it.advertise("debug_image", 1);

        // TODO: test this
        set_allowed_tags_service_ = nh_.advertiseService("set_allowed_tags_service", &FRC971GpuApriltagNodelet::cmd_service, this);
        // Load config for legal apriltags
        std::vector<int> _legal_tags_vec;
        nh_.param<std::vector<int>>("legal_tags", _legal_tags_vec, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16});
        for (int legal_tag : _legal_tags_vec)
        {
            legal_tags_.emplace(legal_tag);
            ROS_INFO_STREAM("Allowing tag " << std::to_string(legal_tag));
        }
        nh_.param<int>("min_white_black_diff", min_white_black_diff_, min_white_black_diff_);
        ROS_INFO_STREAM("Setting min_white_black_diff to " << min_white_black_diff_);
    }

    void callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::CameraInfoConstPtr &camera_info)
    {
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

            detector_ = std::make_unique<frc971_gpu_apriltag::FRC971GpuApriltagDetector>(camera_info, input_format, min_white_black_diff_);
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
                ROS_WARN_STREAM_THROTTLE(5, "Filtering tag with id " << result.id_);
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
                msg.corners[i].x = result.undistorted_corners_[i].x;
                msg.corners[i].y = result.undistorted_corners_[i].y;
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
            auto debug_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
            for (const auto &corner : rejected_margin_corners)
            {
                drawCorner(debug_image->image, corner, cv::Scalar(255, 0, 0));
            }
            for (const auto &corner : rejected_noconverge_corners)
            {
                drawCorner(debug_image->image, corner, cv::Scalar(255, 166, 0));
            }
            for (const auto &result : results)
            {
                drawCorner(debug_image->image, result.original_corners_, cv::Scalar(0, 255, 0));
                drawCorner(debug_image->image, result.undistorted_corners_, cv::Scalar(255, 255, 0));
            }
            pub_debug_image_.publish(debug_image->toImageMsg());
        }
    }

private:
    bool cmd_service(gpu_apriltag_msgs::SetAllowedTags::Request &req,
                    gpu_apriltag_msgs::SetAllowedTags::Response &res) {
        ROS_ERROR_STREAM("CMD service for gpu apriltag called");
        legal_tags_.clear();
        for (int legal_tag : req.allowed_tags)
        {
            legal_tags_.emplace(legal_tag);
        }
        std::stringstream info_stream;
        info_stream << "Allowed tags: ";
        std::copy(legal_tags_.begin(), legal_tags_.end(), std::ostream_iterator<int>(info_stream, " "));
        info_stream << std::endl;
        ROS_INFO_STREAM(info_stream.str());
        res.success = true;
        return true;
    } 
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
    // cv_bridge::CvImage debug_image_;

    std::unique_ptr<frc971_gpu_apriltag::FRC971GpuApriltagDetector> detector_;

    ddynamic_reconfigure::DDynamicReconfigure ddr_;
    ros::ServiceServer save_input_image_srv_;
    std::atomic<bool> save_input_image_{false};

    std::set<int> legal_tags_;
    ros::ServiceServer set_allowed_tags_service_; // service for receiving commands listing allowed apriltags
    int min_white_black_diff_{5};
};

} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(frc971_gpu_apriltag::FRC971GpuApriltagNodelet, nodelet::Nodelet)