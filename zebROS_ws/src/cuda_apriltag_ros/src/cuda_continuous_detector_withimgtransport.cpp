// @TODO 
// This version of cuda_continous_detector is out of date, but uses image transport which is faster
// Use image trans


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
#include <tf/transform_broadcaster.h>
#include "apriltag_ros/AprilTagDetectionArray.h"

struct AprilTagsImpl {
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
                    float tag_edge_size_, int max_tags_) {
        assert(!april_tags_handle);

        // Get camera intrinsics
        cam_intrinsics = {fx, fy, cx, cy};

        // Create AprilTags detector instance and get handle
        const int error = nvCreateAprilTagsDetector(
                &april_tags_handle, width, height, nvAprilTagsFamily::NVAT_TAG36H11,
                &cam_intrinsics, tag_edge_size_);
        if (error != 0) {
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
        if (cuda_error != cudaSuccess) {
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

    ~AprilTagsImpl() {
        if (april_tags_handle != nullptr) {
            cudaStreamDestroy(main_stream);
            nvAprilTagsDestroy(april_tags_handle);
            cudaFree(input_image_buffer);
        }
    }
};

class CudaApriltagDetector
{
	public:
		CudaApriltagDetector(ros::NodeHandle &n)
			:
        it_(new image_transport::ImageTransport(n))
			, pub_(n.advertise<apriltag_ros::AprilTagDetectionArray>("cuda_tag_detections", 1))
		
    {
      n.param<std::string>("transport_hint", transport_hint, "raw");
      sub_ = it_->subscribe("image_rect", 1,
                          &CudaApriltagDetector::imageCallback, this,
                          image_transport::TransportHints(transport_hint));
      
		}

  void imageCallback (const sensor_msgs::ImageConstPtr& image_rect) {
      // Seems like this should be a paramater
      float fx = 388.239;
      float fy = 388.239;
      float ppx = 317.285;
      float ppy = 245.185;

    // Lazy updates:
    // When there are no subscribers _and_ when tf is not published,
    // skip detection.
    if (pub_.getNumSubscribers() == 0 &&
        sub_.getNumPublishers() == 0)
    {
      // ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run processing
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat img = cv_ptr->image;

    std::unique_ptr<AprilTagsImpl> impl_ = std::make_unique<AprilTagsImpl>();
    impl_->initialize(img.cols, img.rows,
              img.total() * img.elemSize(),  img.step,
              fx,fy,ppx,ppy,
              0.5,
              256);


    const auto start = std::chrono::system_clock::now();

    const cudaError_t cuda_error =
            cudaMemcpyAsync(impl_->input_image_buffer, (uchar4 *)img.ptr<unsigned char>(0),
              impl_->input_image_buffer_size, cudaMemcpyHostToDevice, impl_->main_stream);

    if (cuda_error != cudaSuccess) {
        throw std::runtime_error(
                "Could not memcpy to device CUDA memory (error code " +
                std::to_string(cuda_error) + ")");
    }

    uint32_t num_detections;
    const int error = nvAprilTagsDetect(
            impl_->april_tags_handle, &(impl_->input_image), impl_->tags.data(),
            &num_detections, impl_->max_tags, impl_->main_stream);
    if (error != 0) {
        throw std::runtime_error("Failed to run AprilTags detector (error code " +
                                  std::to_string(error) + ")");
    }
    const auto end = std::chrono::system_clock::now();



    // PUBLISH IMAGES HERE
    //pub_.publish(    );

    // 
  }

	private:
    
		ros::Publisher pub_;
    std::string transport_hint;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber sub_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cuda_apriltag_ros");
  ros::NodeHandle nh;

	/* old code 

  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nh));
  
  
  std::string transport_hint;
  pnh.param<std::string>("transport_hint", transport_hint, "raw");

  camera_image_subscriber_ =
      it_->subscribeCamera("image_rect", 1,
                          &ContinuousDetector::imageCallback, this,
                          image_transport::TransportHints(transport_hint));
  tag_detections_publisher_ =
      nh.advertise<AprilTagDetectionArray>("cuda_tag_detections", 1);


  
  add debug option      
  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
  }
  */
  CudaApriltagDetector detection_node(nh);

  ros::spin();
  return 0;
}



