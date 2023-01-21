/*
NODE NAME: imu_zero_node

SUBSCRIBES TO:
sub_topic

PUBLISHES TO:
pub_topic

SERVICES:
set_imu_zero

FUNCTIONALITY:
Zeros z-axis orientation from IMU.
Other IMU data is passed through.
Zero point in degrees is set using service call.
*/

#include <string>

#include <ros/ros.h>
#include <angles/angles.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

#include <tf2_ros/transform_listener.h>

#include <imu_zero/ImuZeroAngle.h>
#include <std_srvs/Trigger.h>
#include <robot_localization/SetPose.h>
#include <zed_interfaces/reset_tracking.h>
#include <zed_interfaces/reset_odometry.h>

const std::string sub_topic = "imu/data";
const std::string pub_topic = "zeroed_imu";
const std::string service_name = "set_imu_zero";
const std::string bias_service_name = "imu/bias_estimate";
const std::string ukf_set_pose_name = "/swerve_imu_ukf/set_pose";
const std::string zed_reset_odometry_name = "/zed_objdetect/reset_odometry";
const std::string zed_reset_tracking_name = "/zed_objdetect/reset_tracking";
ros::Publisher pub;
tf2::Quaternion zero_rot;
tf2::Quaternion last_raw;
ros::ServiceClient bias_estimate;
ros::ServiceClient ukf_zero_pos;
ros::ServiceClient zed_reset_odometry;
ros::ServiceClient zed_reset_tracking;

tf2_ros::Buffer tfBuffer;
std::unique_ptr<tf2_ros::TransformListener> tfListener;

// From https://github.com/ros-perception/imu_pipeline/blame/noetic-devel/imu_transformer/include/imu_transformer/tf2_sensor_msgs.h
namespace tf2
{

/**********/
/** IMU  **/
/**********/

/**
* method to extract timestamp from object
*/
/**
* Transforms a covariance array from one frame to another
*/
  inline
  void transformCovariance(const boost::array<double, 9>& in, boost::array<double, 9>& out, Eigen::Quaternion<double> r){

    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > cov_in(in.data());
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > cov_out(out.data());
    cov_out = r * cov_in * r.inverse();

  }

/**
* Transforms sensor_msgs::Imu data from one frame to another
*/
  template <>
  inline
  void doTransform(const sensor_msgs::Imu &imu_in, sensor_msgs::Imu &imu_out, const geometry_msgs::TransformStamped& t_in)
  {

    imu_out.header = t_in.header;

    // Discard translation, only use orientation for IMU transform
    Eigen::Quaternion<double> r(
        t_in.transform.rotation.w, t_in.transform.rotation.x, t_in.transform.rotation.y, t_in.transform.rotation.z);
    Eigen::Transform<double,3,Eigen::Affine> t(r);

    Eigen::Vector3d vel = t * Eigen::Vector3d(
        imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);

    imu_out.angular_velocity.x = vel.x();
    imu_out.angular_velocity.y = vel.y();
    imu_out.angular_velocity.z = vel.z();

    transformCovariance(imu_in.angular_velocity_covariance, imu_out.angular_velocity_covariance, r);

    Eigen::Vector3d accel = t * Eigen::Vector3d(
        imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);


    imu_out.linear_acceleration.x = accel.x();
    imu_out.linear_acceleration.y = accel.y();
    imu_out.linear_acceleration.z = accel.z();

    transformCovariance(imu_in.linear_acceleration_covariance, imu_out.linear_acceleration_covariance, r);

    // Orientation expresses attitude of the new frame_id in a fixed world frame. This is why the transform here applies
    // in the opposite direction.
    Eigen::Quaternion<double> orientation = Eigen::Quaternion<double>(
        imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z) * r.inverse();

    imu_out.orientation.w = orientation.w();
    imu_out.orientation.x = orientation.x();
    imu_out.orientation.y = orientation.y();
    imu_out.orientation.z = orientation.z();

    // Orientation is measured relative to the fixed world frame, so it doesn't change when applying a static
    // transform to the sensor frame.
    imu_out.orientation_covariance = imu_in.orientation_covariance;

  }

}

double getYaw(const tf2::Quaternion &q)
{
	double roll, pitch, yaw;
	tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
	return yaw;
}

void zeroCallback(const sensor_msgs::Imu::ConstPtr& raw_msg) {
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer.lookupTransform("base_link", raw_msg->header.frame_id, ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "imu_zero : error looking up transform from "
                               << raw_msg->header.frame_id
                               << " to base_link : "
                               << ex.what());
    return;
  }
  sensor_msgs::Imu zeroed_imu = *raw_msg;
  tf2::doTransform(zeroed_imu, zeroed_imu, transformStamped);
  tf2::convert(zeroed_imu.orientation, last_raw);
  tf2::Quaternion zeroed = zero_rot * last_raw;
  zeroed.normalize();

  zeroed_imu.orientation = tf2::toMsg(zeroed);
  if (!std::isfinite(getYaw(zeroed)))
  {
	  ROS_WARN_STREAM_THROTTLE(5., "zeroCallback : NaN yaw result"
			  << "\n\traw_msg.orientation = " << raw_msg->orientation
			  << "\n\tzeroed_imu.orientation = " << zeroed_imu.orientation
			  << "\n\tlast_raw = " << last_raw[0] << " " << last_raw[1] << " " << last_raw[2] << " " << last_raw[3]
			  << "\n\tzero_rot = " << zero_rot[0] << " " << zero_rot[1] << " " << zero_rot[2] << " " << zero_rot[3]
			  << "\n\tzeroed = " << zeroed[0] << " " << zeroed[1] << " " << zeroed[2] << " " << zeroed[3]);
  }
  else
  {
	  pub.publish(zeroed_imu);
  }
}

bool zeroSet(imu_zero::ImuZeroAngle::Request& req,
             imu_zero::ImuZeroAngle::Response& /*res*/) {
  double roll, pitch, yaw;
  tf2::Matrix3x3(last_raw).getRPY(roll, pitch, yaw);

  // takes zero angle in degrees, converts to radians
  ROS_INFO_STREAM("Setting IMU zero angle = " << req.angle << " degrees");
  const double a = angles::from_degrees(req.angle);

  zero_rot.setRPY(0.0, 0.0, angles::shortest_angular_distance(yaw, a));
  zero_rot.normalize();

  if(bias_estimate.exists())
  {
    std_srvs::Trigger biasCall;
    if (!bias_estimate.call(biasCall))
    {
      ROS_ERROR("imu_zero : bias_estimate call failed");
    }
    ROS_INFO("Bias estimate: %s", biasCall.response.message.c_str());
  }
  #if 0
  if (zed_reset_odometry.exists())
  {
    zed_interfaces::reset_odometry resetOdomCall;
    if (!zed_reset_odometry.call(resetOdomCall))
    {
      ROS_ERROR("imu_zero : zed_reset_odometry call failed");
    }
  }
  if (zed_reset_tracking.exists())
  {
    zed_interfaces::reset_tracking resetTrackingCall;
    if (!zed_reset_tracking.call(resetTrackingCall))
    {
      ROS_ERROR("imu_zero : zed_reset_tracking call failed");
    }
  }
  #endif
  if (ukf_zero_pos.exists())
  {
	  robot_localization::SetPose zeroPose;
	  zeroPose.request.pose.header.stamp = ros::Time::now();
	  zeroPose.request.pose.header.frame_id = "odom";
	  // Needs more pose.pose.pose
	  zeroPose.request.pose.pose.pose.position.x = 0;
	  zeroPose.request.pose.pose.pose.position.y = 0;
	  zeroPose.request.pose.pose.pose.position.z = 0;
	  tf2::Quaternion tf2_quat;
	  tf2_quat.setRPY(0,0,0);
	  tf2_quat.normalize();
	  zeroPose.request.pose.pose.pose.orientation = tf2::toMsg(tf2_quat);
	  if (!ukf_zero_pos.call(zeroPose))
		  ROS_ERROR("imu_zero : ukf_zero_pos call failed");
	  //ROS_INFO("Zero pose: %s", zeroPose.response.message.c_str());
  }
  return true;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "imu_zero_node");
  ros::NodeHandle node;
  tfListener = std::make_unique<tf2_ros::TransformListener>(tfBuffer);
  pub = node.advertise<sensor_msgs::Imu>(pub_topic, 1);
  ros::Subscriber sub = node.subscribe(sub_topic, 1, zeroCallback);
  ros::ServiceServer svc = node.advertiseService(service_name, zeroSet);
  zero_rot.setRPY(0,0,0);
  bias_estimate = node.serviceClient<std_srvs::Trigger>(bias_service_name);
  ukf_zero_pos = node.serviceClient<robot_localization::SetPose>(ukf_set_pose_name);
  zed_reset_odometry = node.serviceClient<zed_interfaces::reset_odometry>(zed_reset_odometry_name);
  zed_reset_tracking = node.serviceClient<zed_interfaces::reset_tracking>(zed_reset_tracking_name);

  ros::spin();
  return 0;
}
