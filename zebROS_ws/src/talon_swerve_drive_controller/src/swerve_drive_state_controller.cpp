#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <ctre_interfaces/latency_compensation_state_interface.h>
#include <talon_state_msgs/LatencyCompensationState.h>
#include <nav_msgs/Odometry.h>
#include <periodic_interval_counter/periodic_interval_counter.h>
#include <Eigen/Dense>
#include <talon_swerve_drive_controller/get_wheel_names.h>

namespace swerve_drive_state_controller
{
template<size_t WHEELCOUNT>
class SwerveDriveStateController: public controller_interface::Controller<hardware_interface::latency_compensation::CTRELatencyCompensationStateInterface>
{
private:
	hardware_interface::latency_compensation::CTRELatencyCompensationStateHandle latency_compensation_state_;
	std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> realtime_pub_;
	std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
	double publish_rate_;
	size_t num_hw_joints_;

	std::string latency_compensation_group_;
	size_t latency_compensation_group_index_;

	std::array<std::string, WHEELCOUNT> speed_names_;
	std::array<std::string, WHEELCOUNT> steering_names_;

	std::array<double, WHEELCOUNT> offsets_;

	double wheel_radius_ = 0.050; // meters
	double encoder_to_rotations_ = 0.148148148; // 1/6.75

	Eigen::Matrix<double, WHEELCOUNT * 2, 3> inverse_kinematics_matrix_; // Desired chassis speeds -> swerve module states
	Eigen::Matrix<double, 3, WHEELCOUNT * 2> forward_kinematics_matrix_; // Moore-Penrose pseudoinverse of the inverse kinematics matrix

public:
	bool init(hardware_interface::latency_compensation::CTRELatencyCompensationStateInterface *hw,
			  ros::NodeHandle &root_nh,
			  ros::NodeHandle &controller_nh)
	{
		// get name of our latency compensation joint
		if (!controller_nh.getParam("latency_compensation_group", latency_compensation_group_))
		{
			ROS_ERROR("Parameter 'latency_compensation_group' not set in swerve drive state controller");
			return false;
		}

		// get all joint names from the hardware interface
		const std::vector<std::string> &joint_names = hw->getNames();
		latency_compensation_group_index_ = std::find(joint_names.begin(), joint_names.end(), latency_compensation_group_) - joint_names.begin();
		if (latency_compensation_group_index_ >= joint_names.size()) {
			ROS_ERROR_STREAM("Latency compensation group " << latency_compensation_group_ << " not found in swerve drive state controller");
			return false;
		}

		// get publishing period
		if (!controller_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_ERROR("Parameter 'publish_rate' not set in swerve drive state controller");
			return false;
		}
		else if (publish_rate_ <= 0.0)
		{
			ROS_ERROR_STREAM("Invalid publish rate in swerve drive state controller (" << publish_rate_ << ")");
			return false;
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

        // Get joint names from the parameter server
        if (!getWheelNames(controller_nh, "swerve_drive_state_controller", "speed", speed_names_) or
            !getWheelNames(controller_nh, "swerve_drive_state_controller", "steering", steering_names_))
        {
            return false;
        }

		XmlRpc::XmlRpcValue wheel_coords;

		if(!controller_nh.getParam("wheel_coords", wheel_coords))
		{
			ROS_ERROR("swerve_drive_state_controller : could not read wheel_coords");
			return false;
		}
		if(wheel_coords.getType() != XmlRpc::XmlRpcValue::TypeArray )
		{
			ROS_ERROR("swerve_drive_state_controller : param 'wheel_coords' is not a list");
			return false;
		}
		if (wheel_coords.size() != WHEELCOUNT)
		{
			ROS_ERROR_STREAM("swerve_drive_state_controller : param 'wheel_coords' is not correct length (expecting WHEELCOUNT = " << WHEELCOUNT << ")");
			return false;
		}
		for(int i=0; i < wheel_coords.size(); ++i)
		{
			if(wheel_coords[i].getType() != XmlRpc::XmlRpcValue::TypeArray )
			{
				ROS_ERROR("swerve_drive_state_controller : param wheel_coords[%d] is not a list", i);
				return false;
			}
			if(wheel_coords[i].size() != 2)
			{
				ROS_ERROR("swerve_drive_state_controller: param wheel_coords[%d] is not a pair", i);
				return false;
			}
			if(	wheel_coords[i][0].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
				wheel_coords[i][1].getType() != XmlRpc::XmlRpcValue::TypeDouble)
			{
				ROS_ERROR("swerve_drive_state_controller : param wheel_coords[%d] is not a pair of doubles", i);
				return false;
			}

			// See https://www.overleaf.com/read/hjncvyqkrvzn
			inverse_kinematics_matrix_(i*2, 0) = 1;
			inverse_kinematics_matrix_(i*2, 1) = 0;
			inverse_kinematics_matrix_(i*2, 2) = -(double)wheel_coords[i][1];

			inverse_kinematics_matrix_(i*2+1, 0) = 0;
			inverse_kinematics_matrix_(i*2+1, 1) = 1;
			inverse_kinematics_matrix_(i*2+1, 2) = (double)wheel_coords[i][0];
		}

		// A_dagger = inv(A_transpose * A) * A_transpose
		forward_kinematics_matrix_.noalias() = (inverse_kinematics_matrix_.transpose() * inverse_kinematics_matrix_).inverse() * inverse_kinematics_matrix_.transpose();

		for (size_t i = 0; i < WHEELCOUNT; i++)
		{
			ros::NodeHandle nh(controller_nh, steering_names_[i]);
			if (!nh.getParam("offset", offsets_[i]))
			{
				ROS_ERROR_STREAM("Can not read offset for " << steering_names_[i]);
				return false;
			}
		}
		for (const auto o : offsets_)
		{
			ROS_INFO_STREAM("\t SWERVE: offset = " << o);
		}

		// get wheel radius and conversion factor
		if (!controller_nh.getParam("wheel_radius", wheel_radius_))
		{
			ROS_ERROR("Parameter 'wheel_radius' not set in swerve drive state controller");
			return false;
		}
		if (!controller_nh.getParam("ratio_encoder_to_rotations", encoder_to_rotations_))
		{
			ROS_ERROR("Parameter 'ratio_encoder_to_rotations' not set in swerve drive state controller");
			return false;
		}

		// realtime publisher
		realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<nav_msgs::Odometry>>(controller_nh, "odom", 2);

		auto &m = realtime_pub_->msg_;

		// get joints and allocate message
		latency_compensation_state_ = hw->getHandle(joint_names[latency_compensation_group_index_]);

		return true;
	}

	void starting(const ros::Time &time)
	{
		interval_counter_->reset();
	}

	void update(const ros::Time &time, const ros::Duration & period)
	{
		// Compute odometry (Eigen magic)
		Eigen::Matrix<double, WHEELCOUNT * 2, 1> wheel_states_vector;

		ros::Time ts = time;

		for (size_t i = 0; i < WHEELCOUNT; i++) {
			std::string steering_name = steering_names_[i];
			std::string speed_name = speed_names_[i];
			
			double angle = latency_compensation_state_->getLatencyCompensatedValue(steering_name, time) - offsets_[i];

			double value, velocity;
			
			// The timestamped slope is probably close enough for speed joint velocity
			latency_compensation_state_->getEntry(speed_name, ts, value, velocity);

			wheel_states_vector(i*2, 0) = velocity * wheel_radius_ * encoder_to_rotations_ * cos(angle);
			wheel_states_vector(i*2 + 1, 0) = velocity * wheel_radius_ * encoder_to_rotations_ * sin(angle);

		}

		Eigen::Matrix<double, 3, 1> chassis_speeds = forward_kinematics_matrix_ * wheel_states_vector; // least-squares solution
		// (multiplying on the left by the Moore-Penrose pseudoinverse)

		// limit rate of publishing
		if (interval_counter_->update(period))
		{
			// try to publish
			if (realtime_pub_->trylock())
			{
				auto &odom = realtime_pub_->msg_;
				odom.header.stamp = time;
				odom.header.frame_id = "base_link";
				// just doing twist for now
				// x = -y, y = x
				odom.twist.twist.linear.x = -chassis_speeds(1);
				odom.twist.twist.linear.y = chassis_speeds(0);
				odom.twist.twist.angular.z = chassis_speeds(2);
				realtime_pub_->unlockAndPublish();
			}
			else
			{
				interval_counter_->force_publish();
			}
		}
	}

	void stopping(const ros::Time & /*time*/)
	{
	}

}; // class

} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(swerve_drive_state_controller::SwerveDriveStateController<4>, controller_interface::ControllerBase)
