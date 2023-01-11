
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
/*
fx=528.945
fy=528.42
cx=644.31
cy=340.988
k1=-0.0409595
k2=0.00860526
k3=-0.00429503
p1=0.000209693
p2=0.000237899
*/

int main (int argc, char **argv)
{
	ros::init(argc, argv, "dummy_caminfo");
	ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<sensor_msgs::CameraInfo>("/zed_objdetect/left/camera_info", 1 ,true);

    constexpr double fx = 528.945;
    constexpr double fy = 528.42;
    constexpr double cx = 644.31;
    constexpr double cy = 340.988;
    constexpr double k1 = -0.0409595;
    constexpr double k2 = 0.00860526;
    constexpr double k3 = -0.00429503;
    constexpr double p1 = 0.000209693;
    constexpr double p2 = 0.000237899;

    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.distortion_model = "plumb_bob";
    cam_info_msg.D.resize(5);
    cam_info_msg.D[0] = k1;
    cam_info_msg.D[1] = k2;
    cam_info_msg.D[2] = k3;
    cam_info_msg.D[3] = p1;
    cam_info_msg.D[4] = p2;

    cam_info_msg.K.fill(0.0);
    cam_info_msg.K[0] = fx;
    cam_info_msg.K[2] = cx;
    cam_info_msg.K[4] = fy;
    cam_info_msg.K[5] = cy;
    cam_info_msg.K[8] = 1.0;
    cam_info_msg.R.fill(0.0);

    for (size_t i = 0; i < 3; i++)
    {
        cam_info_msg.R[i + i * 3] = 1; // identity
    }

    cam_info_msg.P.fill(0.0);
    cam_info_msg.P[0] = fx;
    cam_info_msg.P[2] = cx;
    cam_info_msg.P[5] = fy;
    cam_info_msg.P[6] = cy;
    cam_info_msg.P[10] = 1.0;
    cam_info_msg.width = 1280;
    cam_info_msg.height = 720;
    cam_info_msg.header.frame_id = "zed_objdetect_left_optical_frame";

    ros::Rate r(5);
    while (ros::ok())
    {
        pub.publish(cam_info_msg);
        r.sleep();
    }

    return 0;
}