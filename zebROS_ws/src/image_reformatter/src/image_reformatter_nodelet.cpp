// Nodelet which subscribes to an image topic, changes the channel, size and format
// without modifying the image in any way, and then republishes it.  Initial use
// is for the OV2311 camera on the Jetson Nano, which publishes a Bayer RGGB8 image
// as a flat 2M x 1 row single channel image. The cv_camera node sets the encoding
// to mono8. This nodelet will convert the image back to Bayer RGGB8 and set the
// shape to 1600x1300 pixels.
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>

namespace image_reformatter
{
class ImageReformatterNodelet : public nodelet::Nodelet
{
public:
    ImageReformatterNodelet(void) = default;
    ~ImageReformatterNodelet() override = default;

    void onInit() override
    {
        auto nh = getMTPrivateNodeHandle();
        auto base_nh = getNodeHandle();

        nh.param<std::string>("output_encoding", output_encoding_, "mono8");
        nh.param<int>("output_height", output_height_, 1300);

        image_transport::ImageTransport base_it(base_nh);
        camera_sub_ = base_it.subscribeCamera("image_raw", 1, &ImageReformatterNodelet::callback, this);

        image_transport::ImageTransport it(nh);
        camera_pub_ = it.advertiseCamera("image_reformatted", 1);
    }

private:
    void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info)
    {
        cv_bridge::CvImage output_image;
        cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(image);
        output_image.header = image->header;
        output_image.encoding = output_encoding_;
        output_image.image = cv_image->image.reshape(1, output_height_);

        camera_pub_.publish(output_image.toImageMsg(), camera_info);
    }

    image_transport::CameraSubscriber camera_sub_;
    image_transport::CameraPublisher camera_pub_;

    std::string output_encoding_{};
    int output_height_{};
};
}  // namespace image_reformatter

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_reformatter::ImageReformatterNodelet, nodelet::Nodelet)
