#include <ros/ros.h>
#include "gpu_apriltag/gpu_apriltag.h"
#include "sensor_msgs/CameraInfo.h"

int main(int argc, char** argv)
{
    sensor_msgs::CameraInfo camera_info;
    auto detector frc971_gpu_apriltag::FRC971GpuApriltagDetector<frc971::apriltag::InputFormat::Mono8>(camera_info);
}