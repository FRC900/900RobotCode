/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */
#include <set>
#include <cassert>
#include <chrono>
#include "cuda.h"
#include "cuda_runtime.h"
#include <opencv2/opencv.hpp>
#include "nvapriltags/nvAprilTags.h"
#include <ros/ros.h>
#include <string>
#include <sstream>
#include <vector>
#include <map>

#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "cuda_apriltag_ros/AprilTagDetectionArray.h"

struct AprilTagsImpl
{
    // Handle used to interface with the stereo library.
    nvAprilTagsHandle april_tags_handle = nullptr;
    // Camera intrinsics
    nvAprilTagsCameraIntrinsics_t cam_intrinsics;

    // Output vector of detected Tags
    std::vector<nvAprilTagsID_t> tags;

    // CUDA stream
    cudaStream_t main_stream = {};

    // CUDA buffers to store the input image.
    nvAprilTagsImageInput_t input_image;

    // CUDA memory buffer container for RGBA images.
    uchar4 *input_image_buffer = nullptr;

    // Size of image buffer
    size_t input_image_buffer_size = 0;

    int max_tags;

    void initialize(const uint32_t width,
                    const uint32_t height, const size_t image_buffer_size,
                    const size_t pitch_bytes,
                    const float fx, const float fy, const float cx, const float cy,
                    float tag_edge_size_, int max_tags_)
    {
        assert(!april_tags_handle);

        // Get camera intrinsics
        cam_intrinsics = {fx, fy, cx, cy};

        // Create AprilTags detector instance and get handle
        const int error = nvCreateAprilTagsDetector(
                              &april_tags_handle, width, height, nvAprilTagsFamily::NVAT_TAG36H11,
                              &cam_intrinsics, tag_edge_size_);
        if (error != 0)
        {
            throw std::runtime_error(
                "Failed to create NV April Tags detector (error code " +
                std::to_string(error) + ")");
        }

        // Create stream for detection
        cudaStreamCreate(&main_stream);

        // Allocate the output vector to contain detected AprilTags.
        tags.resize(max_tags_);
        max_tags = max_tags_;
        // Setup input image CUDA buffer.
        const cudaError_t cuda_error =
            cudaMalloc(&input_image_buffer, image_buffer_size);
        if (cuda_error != cudaSuccess)
        {
            throw std::runtime_error("Could not allocate CUDA memory (error code " +
                                     std::to_string(cuda_error) + ")");
        }

        // Setup input image.
        input_image_buffer_size = image_buffer_size;
        input_image.width = width;
        input_image.height = height;
        input_image.dev_ptr = reinterpret_cast<uchar4 *>(input_image_buffer);
        input_image.pitch = pitch_bytes;
    }

    ~AprilTagsImpl()
    {
        if (april_tags_handle != nullptr)
        {
            cudaStreamDestroy(main_stream);
            nvAprilTagsDestroy(april_tags_handle);
            cudaFree(input_image_buffer);
        }
    }
};

sensor_msgs::CameraInfo caminfo;
bool caminfovalid {false};
// Capture camera info published about the camera - needed for screen to world to work
void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &info)
{
    caminfo = *info;
    caminfovalid = true;
}

class CudaApriltagDetector
{
    public:
        CudaApriltagDetector(ros::NodeHandle &n, const std::string &sub_topic, const std::string &pub_topic, const double tag_size, const std::vector<int> &tag_id_vector)
            :   //what should the camera topic be on the robot?
                sub_(n.subscribe(sub_topic, 1, &CudaApriltagDetector::imageCallback, this))
            , pub_(n.advertise<cuda_apriltag_ros::AprilTagDetectionArray>(pub_topic, 2))
            , impl_(std::make_unique<AprilTagsImpl>())
            , tag_ids_{tag_id_vector.cbegin(), tag_id_vector.cend()}

        {
            tag_size_ = tag_size;
        }

        geometry_msgs::Transform ToTransformMsg(const nvAprilTagsID_t &detection)
        {

            geometry_msgs::Transform t;
            t.translation.x = detection.translation[0];
            t.translation.y = detection.translation[1];
            t.translation.z = detection.translation[2];


            //
            auto o = detection.orientation;
            auto matrix = tf2::Matrix3x3();
            matrix.setValue(o[0], o[3], o[6], o[1], o[4], o[7], o[2], o[5], o[8]);
            tf2::Quaternion q;
            matrix.getRotation(q);


            // Rotation matrix from nvAprilTags is column major
            //const Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::ColMajor>>
            //orientation(detection.orientation);
            //const Eigen::Quaternion<float> q(orientation);

            t.rotation.w = q.w();
            t.rotation.x = q.x();
            t.rotation.y = q.y();
            t.rotation.z = q.z();

            //ROS_INFO_STREAM("t.translation = " << t.translation << " t.rotation = " << t.rotation);
            return t;

        }

        void imageCallback (const sensor_msgs::ImageConstPtr &image_rect)
        {

            if (!caminfovalid)
            {
                ROS_WARN_STREAM("Waiting for camera info");
                return;
            }
            // Lazy updates:
            // When there are no subscribers _and_ when tf is not published,
            // skip detection.
            // TODO : remove sub_ check, since if we're getting the callback the subscriber
            //        has a publisher
            if (pub_.getNumSubscribers() == 0 &&
                    sub_.getNumPublishers() == 0)
            {
                ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
                return;
            }
            cv::Mat img;
            // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run processing
            try
            {
                img = cv_bridge::toCvShare(image_rect, "rgba8")->image;
            }
            catch (cv_bridge::Exception &e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            if (impl_->april_tags_handle == nullptr)
            {
                impl_->initialize(img.cols, img.rows,
                                  img.total() * img.elemSize(),  img.step,
                                  float(caminfo.P[0]), float(caminfo.P[5]), float(caminfo.P[2]), float(caminfo.P[6]),
                                  tag_size_,
                                  40);
            }

            //   ROS_ERROR_STREAM("CUDA Apriltag callback ");
            const cudaError_t cuda_error =
                cudaMemcpyAsync(impl_->input_image_buffer, (uchar4 *)img.ptr<unsigned char>(0),
                                impl_->input_image_buffer_size, cudaMemcpyHostToDevice, impl_->main_stream);

            if (cuda_error != cudaSuccess)
            {
                throw std::runtime_error(
                    "Could not memcpy to device CUDA memory (error code " +
                    std::to_string(cuda_error) + ")");
            }


            uint32_t num_detections;
            const int error = nvAprilTagsDetect(
                                  impl_->april_tags_handle, &(impl_->input_image), impl_->tags.data(),
                                  &num_detections, impl_->max_tags, impl_->main_stream);


            if (error != 0)
            {
                throw std::runtime_error("Failed to run AprilTags detector (error code " +
                                         std::to_string(error) + ")");
            }


            cuda_apriltag_ros::AprilTagDetectionArray msg_detections;
            msg_detections.header = image_rect->header;
            for (uint32_t i = 0; i < num_detections; i++)
            {
                const nvAprilTagsID_t &detection = impl_->tags[i];
                cuda_apriltag_ros::AprilTagDetection msg_detection;
                msg_detection.family = tag_family_;
                msg_detection.id = detection.id;

                if (tag_ids_.count(detection.id) != 1)
                {
                    ROS_WARN_STREAM("Skipping tag with ID=" << detection.id);
                    continue;
                }

#if 0
                ROS_INFO_STREAM("corners0 = " << detection.corners[0].x << " " << detection.corners[0].y);
                ROS_INFO_STREAM("corners1 = " << detection.corners[1].x << " " << detection.corners[1].y);
                ROS_INFO_STREAM("corners2 = " << detection.corners[2].x << " " << detection.corners[2].y);
                ROS_INFO_STREAM("corners3 = " << detection.corners[3].x << " " << detection.corners[3].y);
                ROS_INFO_STREAM("translation = " << detection.translation[0] << " " << detection.translation[1] << " " << detection.translation[2]);
                ROS_INFO_STREAM("orientation = " << detection.orientation[0] << " " << detection.orientation[1] << " " << detection.orientation[2] << " " << detection.orientation[3] << " " << detection.orientation[4] << " " << detection.orientation[5]  << " " << detection.orientation[6]  << " " << detection.orientation[7]  << " " << detection.orientation[8] );
#endif

                // detection


                // corners
                for (int corner_idx = 0; corner_idx < 4; corner_idx++)
                {
                    msg_detection.corners.data()[corner_idx].x =
                        detection.corners[corner_idx].x;
                    msg_detection.corners.data()[corner_idx].y =
                        detection.corners[corner_idx].y;
                }

                // center
                const float slope_1 = (detection.corners[2].y - detection.corners[0].y) /
                                      (detection.corners[2].x - detection.corners[0].x);
                const float slope_2 = (detection.corners[3].y - detection.corners[1].y) /
                                      (detection.corners[3].x - detection.corners[1].x);
                const float intercept_1 = detection.corners[0].y -
                                          (slope_1 * detection.corners[0].x);
                const float intercept_2 = detection.corners[3].y -
                                          (slope_2 * detection.corners[3].x);
                msg_detection.center.x = (intercept_2 - intercept_1) / (slope_1 - slope_2);
                msg_detection.center.y = (slope_2 * intercept_1 - slope_1 * intercept_2) /
                                         (slope_2 - slope_1);

                // Timestamped Pose3 transform
                //ROS_INFO_STREAM("Transforming tag ID " << detection.id << " header = " << image_rect->header << " ");
                geometry_msgs::TransformStamped tf;
                tf.header = image_rect->header;
                tf.child_frame_id = tag_family_ + std::to_string(detection.id);
                tf.transform = ToTransformMsg(detection);
                br_.sendTransform(tf);

                msg_detection.pose.pose.pose.position.x = tf.transform.translation.x;
                msg_detection.pose.pose.pose.position.y = tf.transform.translation.y;
                msg_detection.pose.pose.pose.position.z = tf.transform.translation.z;
                msg_detection.pose.pose.pose.orientation = tf.transform.rotation;
                msg_detections.detections.push_back(msg_detection);
            }

            pub_.publish(msg_detections);

        }

    private:
        // I need to combine two
        ros::Subscriber sub_;
        ros::Publisher pub_;
        std::unique_ptr<AprilTagsImpl> impl_;
        double tag_size_;
        std::string tag_family_ = "36h11";
        tf2_ros::TransformBroadcaster br_;
        std::set<int> tag_ids_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cuda_apriltag_ros");
    ros::NodeHandle nh;

    std::string image_topic, pub_topic, camera_info;
    double tag_size;

    if (!nh.getParam("tag_size", tag_size))
    {
        ROS_ERROR("tag_size not specified");
        return -1;
    }
    if (!nh.getParam("image_topic", image_topic))
    {
        ROS_ERROR("image_topic not specified");
        return -1;
    }
    if (!nh.getParam("pub_topic", pub_topic))
    {
        ROS_ERROR("pub_topic not specified");
        return -1;
    }
    if (!nh.getParam("camera_info", camera_info))
    {
        ROS_ERROR("camera_info not specified");
        return -1;
    }
    std::vector<int> tag_ids;
    if (!nh.getParam("tag_ids", tag_ids))
    {
        ROS_ERROR("tag_ids not specified");
        return -1;
    }


    ros::Subscriber camera_info_sub_ = nh.subscribe(camera_info, 1, camera_info_callback);

    CudaApriltagDetector detection_node(nh, image_topic, pub_topic, tag_size, tag_ids);

    ros::spin();
    return 0;
}
