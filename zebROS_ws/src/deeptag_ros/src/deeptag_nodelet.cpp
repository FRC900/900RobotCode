#include <optional>

#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Trigger.h>

#include "apriltag_msgs/ApriltagArrayStamped.h"
#include "apriltag_msgs/ApriltagPoseStamped.h"
#include "deeptag_ros/deeptag.h"
#include "field_obj/TFDetection.h"

namespace deeptag_ros
{
class DeeptagRosNodelet : public nodelet::Nodelet
{
public:
    DeeptagRosNodelet(void) = default;
    ~DeeptagRosNodelet() override = default;

private:
    void onInit() override
    {
        nh_ = getMTPrivateNodeHandle();
        base_nh_ = getNodeHandle();

        base_it_ = std::make_unique<image_transport::ImageTransport>(base_nh_);
        camera_sub_ = base_it_->subscribeCamera("image_rect_color", 1, &DeeptagRosNodelet::callback, this);
        // Publishers for apriltag detections
        pub_apriltag_detections_ = nh_.advertise<apriltag_msgs::ApriltagArrayStamped>("tags", 1);
        pub_apriltag_poses_ = nh_.advertise<apriltag_msgs::ApriltagPoseStamped>("poses", 1);
        // A publisher to publish to a topic that screen to world can use directly
        pub_objdetect_ = nh_.advertise<field_obj::TFDetection>("tag_detection_msg", 1);
        it_ = std::make_unique<image_transport::ImageTransport>(nh_);
        pub_debug_image_ = it_->advertise("debug_image", 1);
        pub_stage1_grid_debug_image_ = it_->advertise("stage1_grid_debug_image", 1);
        pub_stage1_ssd_debug_image_ = it_->advertise("stage1_ssd_debug_image", 1);
        pub_stage2_debug_image_ = it_->advertise("stage2_debug_image", 1);
        save_input_image_srv_ = base_nh_.advertiseService("save_input_image", &DeeptagRosNodelet::saveInputImageCallback, this);
    }

    void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info)
    {
        if (!deep_tag_)
        {
            image_geometry::PinholeCameraModel model;
            model.fromCameraInfo(*camera_info);

            double tagSizeInMeter = 0.1651;
            if (!nh_.getParam("tag_size_in_meter", tagSizeInMeter))
            {
                ROS_ERROR("Failed to get tag_size_in_meter from the parameter server");
                return;
            }
            std::string model_path = ros::package::getPath("deeptag_ros") + "/models";
            std::string detect_onnx_model_filename = "arucotag_roi_detector.onnx";
            std::string decode_onnx_model_filename = "arucotag_decoder.onnx";
            nh_.param("detect_onnx_model_filename", detect_onnx_model_filename, detect_onnx_model_filename);
            nh_.param("decode_onnx_model_filename", decode_onnx_model_filename, decode_onnx_model_filename);
            using enum DeepTagType;
            auto tag_type = APRILTAG_36H11;
            if (std::string tag_type_str;
                nh_.getParam("tag_type", tag_type_str))
            {
                if (tag_type_str == "16h11")
                {
                    tag_type = APRILTAG_16H5;
                }
                else if (tag_type_str == "25h9")
                {
                    tag_type = APRILTAG_25H9;
                }
                else if (tag_type_str == "36h10")
                {
                    tag_type = APRILTAG_36H10;
                }
                else if (tag_type_str == "36h11")
                {
                    tag_type = DeepTagType::APRILTAG_36H11;
                }
                else
                {
                    ROS_ERROR_STREAM("Unknown tag type: " << tag_type_str << " expecting 16h11, 25h9, 36h10, or 36h11");
                    return;
                }
            }

            const cv::Mat intrinsic_matrix = (cv::Mat_<double>(3, 3) << model.fx(), 0, model.cx(),
                                                                        0, model.fy(), model.cy(),
                                                                        0, 0, 1);

            const cv::Mat distortion_coeffs = (cv::Mat_<double>(1, 8) << model.distortionCoeffs().at<double>(0), model.distortionCoeffs().at<double>(1), model.distortionCoeffs().at<double>(2), model.distortionCoeffs().at<double>(3), model.distortionCoeffs().at<double>(4), 0, 0, 0);

            bool use_tiled_detection = false;
            nh_.param("use_tiled_detection", use_tiled_detection, use_tiled_detection);
            bool use_scaled_image = true;
            nh_.param("use_scaled_image", use_scaled_image, use_scaled_image);

            deep_tag_ = std::make_unique<DeepTag>(model.fullResolution(),      // input image, used for image resolution
                                                  use_tiled_detection,         // use tiled detection - config item
                                                  use_scaled_image,            // use scaled-down full image in addition to tiles - config item
                                                  tag_type,                    // tag type - config item
                                                  intrinsic_matrix,            // from camera info
                                                  distortion_coeffs,           // from camera info
                                                  tagSizeInMeter,              // physical tag size - config item
                                                  model_path,                  // use rospkg to find?
                                                  detect_onnx_model_filename,  // onnx detect model filename - config item?
                                                  decode_onnx_model_filename); // onnx decode model filename - config item?
            double corner_min_center_score(deep_tag_->getCornerMinCenterScore());
            nh_.param("corner_min_center_score", corner_min_center_score, corner_min_center_score);
            double ssd_min_center_score(deep_tag_->getSSDMinCenterScore());
            nh_.param("ssd_min_center_score", ssd_min_center_score, ssd_min_center_score);
            double grid_grouper_sigma(deep_tag_->getGridGrouperSigma());
            nh_.param("grid_grouper_sigma", grid_grouper_sigma, grid_grouper_sigma);
            double ssd_grouper_sigma(deep_tag_->getSSDGrouperSigma());
            nh_.param("ssd_grouper_sigma", ssd_grouper_sigma, ssd_grouper_sigma);

            double nms_confidence_threshold;
            nh_.param("nms_confidence_threshold", nms_confidence_threshold, nms_confidence_threshold);
            double nms_nms_threshold;
            nh_.param("nms_nms_threshold", nms_nms_threshold, nms_nms_threshold);

            double min_grid_match_ratio;
            nh_.param("min_grid_match_ratio", min_grid_match_ratio, min_grid_match_ratio);

            ddr_.registerVariable<double>("corner_min_center_score",
                                          [this]() { return deep_tag_->getCornerMinCenterScore();},
                                          [this](const double param) { deep_tag_->setCornerMinCenterScore(param);},
                                          "Corner min center detection score",
                                          0.0, 1.0);
            ddr_.registerVariable<double>("ssd_min_center_score",
                                          [this]() { return deep_tag_->getSSDMinCenterScore();},
                                          [this](const double param) { deep_tag_->setSSDMinCenterScore(param);},
                                          "SSD min center detection score",
                                          0.0, 1.0);
            ddr_.registerVariable<int>   ("grid_grouper_sigma",
                                          [this]() { return deep_tag_->getGridGrouperSigma();},
                                          [this](const int param) { deep_tag_->setGridGrouperSigma(param);},
                                          "Grid grouper sigma",
                                          1, 10000);
            ddr_.registerVariable<int>   ("ssd_grouper_sigma",
                                          [this]() { return deep_tag_->getSSDGrouperSigma();},
                                          [this](const int param) { deep_tag_->setSSDGrouperSigma(param);},
                                          "SSD grouper sigma",
                                          1, 10000);
            ddr_.registerVariable<double>("nms_confidence_threshold",
                                          [this]() { return deep_tag_->getNMSConfidenceThreshold();},
                                          [this](const double param) { deep_tag_->setNMSConfidenceThreshold(param);},
                                          "NMS Confidence Threshold",
                                          0.0, 1.0);
            ddr_.registerVariable<double>("nms_nms_threshold",
                                          [this]() { return deep_tag_->getNMSNMSThreshold();},
                                          [this](const double param) { deep_tag_->setNMSNMSThreshold(param);},
                                          "NMS NMS Threshold",
                                          0.0, 1.0);
            ddr_.registerVariable<double>("min_grid_match_ratio",
                                          [this]() { return deep_tag_->getMinGridMatchRatio();},
                                          [this](const double param) { deep_tag_->setMinGridMatchRatio(param);},
                                          "Stage 2 min grid match ratio",
                                          0.0, 1.0);
            ddr_.publishServicesTopics();
        }
        cv_bridge::CvImageConstPtr cv_frame = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
        // One-shot to save current post-proc input image to a file
        if (save_input_image_.exchange(false))
        {
            deep_tag_->saveInputImage();
        }
        const auto result = deep_tag_->runInference(cv_frame->image);

        apriltag_msgs::ApriltagArrayStamped apriltag_array_msg;
        apriltag_array_msg.header = image->header;

        field_obj::TFDetection objdetect_msg;
        objdetect_msg.header = image->header;
        objdetect_msg.header.frame_id = camera_info->header.frame_id;

        for (const auto &r: result)
        {
            apriltag_array_msg.apriltags.emplace_back();
            
            auto &msg = apriltag_array_msg.apriltags.back();
            msg.id = r.m_tagId;
            msg.center.x = r.m_center.x;
            msg.center.y = r.m_center.y;

            double br_x = -std::numeric_limits<double>::max();
            double br_y = -std::numeric_limits<double>::max();
            double tl_x =  std::numeric_limits<double>::max();
            double tl_y =  std::numeric_limits<double>::max();
            for (size_t i = 0; i < 4; i++)
            {
                msg.corners[i].x = r.m_stage2Corners[i].x;
                msg.corners[i].y = r.m_stage2Corners[i].y;
                br_x = std::max(br_x, msg.corners[i].x);
                br_y = std::max(br_y, msg.corners[i].y);
                tl_x = std::min(tl_x, msg.corners[i].x);
                tl_y = std::min(tl_y, msg.corners[i].y);
            }

            objdetect_msg.objects.emplace_back();
            auto &obj = objdetect_msg.objects.back();
            obj.br.x = static_cast<float>(br_x);
            obj.br.y = static_cast<float>(br_y);
            obj.tl.x = static_cast<float>(tl_x);
            obj.tl.y = static_cast<float>(tl_y);
            obj.id = r.m_tagId;
            obj.label = std::to_string(r.m_tagId);
            obj.confidence = r.m_centerScore;
        }
        pub_apriltag_detections_.publish(apriltag_array_msg);
        pub_objdetect_.publish(objdetect_msg);

        apriltag_msgs::ApriltagPoseStamped apriltag_pose_msg;
        apriltag_pose_msg.header = image->header;
        apriltag_pose_msg.apriltags = apriltag_array_msg.apriltags;
        apriltag_pose_msg.posearray.header = image->header;

        for (const auto &r: result)
        {
            apriltag_pose_msg.posearray.poses.emplace_back();  
            
            auto &msg = apriltag_pose_msg.posearray.poses.back();
            msg.position.x = r.m_tVec.at<double>(0);
            msg.position.y = r.m_tVec.at<double>(1);
            msg.position.z = r.m_tVec.at<double>(2);
            tf2::Quaternion quaternion_tf2;
            quaternion_tf2.setRPY(r.m_rVec.at<double>(0), r.m_rVec.at<double>(1), r.m_rVec.at<double>(2));
            msg.orientation = tf2::toMsg(quaternion_tf2);
        }
        pub_apriltag_poses_.publish(apriltag_pose_msg);

        if (pub_debug_image_.getNumSubscribers() > 0)
        {
            debug_image_.header = image->header;
            debug_image_.encoding = sensor_msgs::image_encodings::BGR8;
            debug_image_.image = cv_frame->image.clone();
            deep_tag_->visualize(debug_image_.image, result);
            pub_debug_image_.publish(debug_image_.toImageMsg());
        }
        if (pub_stage1_grid_debug_image_.getNumSubscribers() > 0)
        {
            stage1_grid_debug_image_.header = image->header;
            stage1_grid_debug_image_.encoding = sensor_msgs::image_encodings::BGR8;
            stage1_grid_debug_image_.image = cv_frame->image.clone();
            deep_tag_->visualizeStage1Grid(stage1_grid_debug_image_.image);
            pub_stage1_grid_debug_image_.publish(stage1_grid_debug_image_.toImageMsg());
        }
        if (pub_stage1_ssd_debug_image_.getNumSubscribers() > 0)
        {
            stage1_ssd_debug_image_.header = image->header;
            stage1_ssd_debug_image_.encoding = sensor_msgs::image_encodings::BGR8;
            stage1_ssd_debug_image_.image = cv_frame->image.clone();
            deep_tag_->visualizeStage1SSD(stage1_ssd_debug_image_.image);
            pub_stage1_ssd_debug_image_.publish(stage1_ssd_debug_image_.toImageMsg());
        }
        if (pub_stage2_debug_image_.getNumSubscribers() > 0)
        {
            stage2_debug_image_.header = image->header;
            stage2_debug_image_.encoding = sensor_msgs::image_encodings::BGR8;
            stage2_debug_image_.image = cv_frame->image.clone();
            deep_tag_->visualizeStage2(stage2_debug_image_.image, result);
            pub_stage2_debug_image_.publish(stage2_debug_image_.toImageMsg());
        }

    }

    bool saveInputImageCallback(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
    {
        save_input_image_ = true;
        res.success = true;
        res.message = "Input image will be saved";
        return true;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle base_nh_;
    std::unique_ptr<image_transport::ImageTransport> base_it_;
    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::CameraSubscriber camera_sub_;
    image_transport::Publisher pub_debug_image_;
    image_transport::Publisher pub_stage1_grid_debug_image_;
    image_transport::Publisher pub_stage1_ssd_debug_image_;
    image_transport::Publisher pub_stage2_debug_image_;
    cv_bridge::CvImage debug_image_;
    cv_bridge::CvImage stage1_grid_debug_image_;
    cv_bridge::CvImage stage1_ssd_debug_image_;
    cv_bridge::CvImage stage2_debug_image_;
    ros::Publisher pub_apriltag_detections_;
    ros::Publisher pub_apriltag_poses_;
    ros::Publisher pub_objdetect_;

    std::unique_ptr<DeepTag> deep_tag_;

    ddynamic_reconfigure::DDynamicReconfigure ddr_;
    ros::ServiceServer save_input_image_srv_;
    std::atomic<bool> save_input_image_{false};
};
}

PLUGINLIB_EXPORT_CLASS(deeptag_ros::DeeptagRosNodelet, nodelet::Nodelet)