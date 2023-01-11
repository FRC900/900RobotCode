#ifndef INC_SMARK_MAX_CONTROLLER_INTERFACE__
#define INC_SMARK_MAX_CONTROLLER_INTERFACE__

#include <algorithm>
#include <atomic>
#include <thread>
#ifdef __linux__
#include <sched.h>
#endif
#include <dynamic_reconfigure/server.h>
#include <spark_max_interface/spark_max_command_interface.h>
#include <spark_max_controllers/SparkMaxConfig.h>
#include <XmlRpcValue.h>

namespace spark_max_controllers
{

class SparkMaxCIParams
{
	public:
		SparkMaxCIParams(void)
		{
			p_gain_.fill(0);
			i_gain_.fill(0);
			d_gain_.fill(0);
			f_gain_.fill(0);
			i_zone_.fill(0);
			d_filter_.fill(0);
			pidf_output_min_.fill(-1);
			pidf_output_max_.fill(1);
			pidf_reference_ctrl_.fill(hardware_interface::kDutyCycle);
			pidf_arb_feed_forward_.fill(0);
			pidf_arb_feed_forward_units_.fill(hardware_interface::ArbFFUnits::kVoltage);
		}

		SparkMaxCIParams(const SparkMaxConfig &config)
		{
			p_gain_[0]  = config.p_gain0;
			i_gain_[0]  = config.i_gain0;
			d_gain_[0]  = config.d_gain0;
			f_gain_[0]  = config.f_gain0;
			i_zone_[0]   = config.i_zone0;
			d_filter_[0] = config.d_filter0;
			pidf_output_min_[0] = config.pidf_output_min0;
			pidf_output_max_[0] = config.pidf_output_max0;
			pidf_arb_feed_forward_[0] = config.pidf_arb_feed_forward0;
			pidf_arb_feed_forward_units_[0] = static_cast<hardware_interface::ArbFFUnits>(config.pidf_arb_feed_forward_units0);
			p_gain_[1]  = config.p_gain1;
			i_gain_[1]  = config.i_gain1;
			d_gain_[1]  = config.d_gain1;
			f_gain_[1]  = config.f_gain1;
			i_zone_[1]   = config.i_zone1;
			d_filter_[1] = config.d_filter1;
			pidf_output_min_[1] = config.pidf_output_min1;
			pidf_output_max_[1] = config.pidf_output_max1;
			pidf_arb_feed_forward_[1] = config.pidf_arb_feed_forward1;
			pidf_arb_feed_forward_units_[1] = static_cast<hardware_interface::ArbFFUnits>(config.pidf_arb_feed_forward_units1);
			p_gain_[2]  = config.p_gain2;
			i_gain_[2]  = config.i_gain2;
			d_gain_[2]  = config.d_gain2;
			f_gain_[2]  = config.f_gain2;
			i_zone_[2]   = config.i_zone2;
			d_filter_[2] = config.d_filter2;
			pidf_output_min_[2] = config.pidf_output_min2;
			pidf_output_max_[2] = config.pidf_output_max2;
			pidf_arb_feed_forward_[2] = config.pidf_arb_feed_forward2;
			pidf_arb_feed_forward_units_[2] = static_cast<hardware_interface::ArbFFUnits>(config.pidf_arb_feed_forward_units2);
			p_gain_[3]  = config.p_gain3;
			i_gain_[3]  = config.i_gain3;
			d_gain_[3]  = config.d_gain3;
			f_gain_[3]  = config.f_gain3;
			i_zone_[3]   = config.i_zone3;
			d_filter_[3] = config.d_filter3;
			pidf_output_min_[3] = config.pidf_output_min3;
			pidf_output_max_[3] = config.pidf_output_max3;
			pidf_arb_feed_forward_[3] = config.pidf_arb_feed_forward3;
			pidf_arb_feed_forward_units_[3] = static_cast<hardware_interface::ArbFFUnits>(config.pidf_arb_feed_forward_units3);

			pidf_reference_slot_ = config.pidf_reference_slot;

			forward_limit_switch_polarity_  = static_cast<hardware_interface::LimitSwitchPolarity>(config.forward_limit_switch_polarity);
			forward_limit_switch_enabled_   = config.forward_limit_switch_enabled;
			reverse_limit_switch_polarity_  = static_cast<hardware_interface::LimitSwitchPolarity>(config.reverse_limit_switch_polarity);
			reverse_limit_switch_enabled_   = config.reverse_limit_switch_enabled;

			current_limit_ = config.current_limit;
			current_limit_stall_ = config.current_limit_stall;
			current_limit_free_ = config.current_limit_free;
			current_limit_RPM_ = config.current_limit_rpm;
			secondary_current_limit_ = config.secondary_current_limit;
			secondary_current_limit_cycles_ = config.secondary_current_limit_cycles;

			idle_mode_ = static_cast<hardware_interface::IdleMode>(config.idle_mode);

			voltage_compensation_enable_ = config.voltage_compensation_enable;
			voltage_compensation_nominal_voltage_ = config.voltage_compensation_nominal_voltage;

			open_loop_ramp_rate_ = config.open_loop_ramp_rate;
			closed_loop_ramp_rate_ = config.closed_loop_ramp_rate;

			follower_type_   = static_cast<hardware_interface::ExternalFollower>(config.follower_type);
			follower_id_     = config.follower_id;
			follower_invert_ = config.follower_invert;

			forward_softlimit_enable_ = config.forward_softlimit_enable;
			forward_softlimit_        = config.forward_softlimit;
			reverse_softlimit_enable_ = config.reverse_softlimit_enable;
			reverse_softlimit_        = config.reverse_softlimit;

			encoder_ticks_per_rotation_ = config.encoder_ticks_per_rotation;
			encoder_type_ = static_cast<hardware_interface::SensorType>(config.encoder_type);
		}

		SparkMaxConfig toConfig(void) const
		{
			SparkMaxConfig config;

			config.p_gain0   = p_gain_[0];
			config.i_gain0   = i_gain_[0];
			config.d_gain0   = d_gain_[0];
			config.f_gain0   = f_gain_[0];
			config.i_zone0   = i_zone_[0];
			config.d_filter0 = d_filter_[0];
			config.pidf_output_min0 = pidf_output_min_[0];
			config.pidf_output_max0 = pidf_output_max_[0];
			config.pidf_arb_feed_forward0 = pidf_arb_feed_forward_[0];
			config.pidf_arb_feed_forward_units0 = static_cast<int>(pidf_arb_feed_forward_units_[0]);
			config.p_gain1   = p_gain_[1];
			config.i_gain1   = i_gain_[1];
			config.d_gain1   = d_gain_[1];
			config.f_gain1   = f_gain_[1];
			config.i_zone1   = i_zone_[1];
			config.d_filter1 = d_filter_[1];
			config.pidf_output_min1 = pidf_output_min_[1];
			config.pidf_output_max1 = pidf_output_max_[1];
			config.pidf_arb_feed_forward1 = pidf_arb_feed_forward_[1];
			config.pidf_arb_feed_forward_units1 = static_cast<int>(pidf_arb_feed_forward_units_[1]);
			config.p_gain2   = p_gain_[2];
			config.i_gain2   = i_gain_[2];
			config.d_gain2   = d_gain_[2];
			config.f_gain2   = f_gain_[2];
			config.i_zone2   = i_zone_[2];
			config.d_filter2 = d_filter_[2];
			config.pidf_output_min2 = pidf_output_min_[2];
			config.pidf_output_max2 = pidf_output_max_[2];
			config.pidf_arb_feed_forward2 = pidf_arb_feed_forward_[2];
			config.pidf_arb_feed_forward_units2 = static_cast<int>(pidf_arb_feed_forward_units_[2]);
			config.p_gain3   = p_gain_[3];
			config.i_gain3   = i_gain_[3];
			config.d_gain3   = d_gain_[3];
			config.f_gain3   = f_gain_[3];
			config.i_zone3   = i_zone_[3];
			config.d_filter3 = d_filter_[3];
			config.pidf_output_min3 = pidf_output_min_[3];
			config.pidf_output_max3 = pidf_output_max_[3];
			config.pidf_arb_feed_forward3 = pidf_arb_feed_forward_[3];
			config.pidf_arb_feed_forward_units3 = static_cast<int>(pidf_arb_feed_forward_units_[3]);

			config.pidf_reference_slot = pidf_reference_slot_;

			config.forward_limit_switch_polarity = forward_limit_switch_polarity_;
			config.forward_limit_switch_enabled  = forward_limit_switch_enabled_;
			config.reverse_limit_switch_polarity = reverse_limit_switch_polarity_;
			config.reverse_limit_switch_enabled  = reverse_limit_switch_enabled_;

			config.current_limit = current_limit_;
			config.current_limit_stall = current_limit_stall_;
			config.current_limit_free = current_limit_free_;
			config.current_limit_rpm = current_limit_RPM_;
			config.secondary_current_limit = secondary_current_limit_;
			config.secondary_current_limit_cycles = secondary_current_limit_cycles_;

			config.idle_mode = idle_mode_;
			config.voltage_compensation_enable = voltage_compensation_enable_;
			config.voltage_compensation_nominal_voltage = voltage_compensation_nominal_voltage_;

			config.open_loop_ramp_rate = open_loop_ramp_rate_;
			config.closed_loop_ramp_rate = closed_loop_ramp_rate_;

			config.follower_type   = follower_type_;
			config.follower_id     = follower_id_;
			config.follower_invert = follower_invert_;

			config.forward_softlimit_enable = forward_softlimit_enable_;
			config.forward_softlimit        = forward_softlimit_;
			config.reverse_softlimit_enable = reverse_softlimit_enable_;
			config.reverse_softlimit        = reverse_softlimit_;

			config.encoder_ticks_per_rotation = encoder_ticks_per_rotation_;
			config.encoder_type = encoder_type_;
			return config;
		}

		// Read a joint name from the given nodehandle's params
		bool readJointName(ros::NodeHandle &n)
		{
			if (!n.getParam("joint", joint_name_))
			{
				ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
				return false;
			}
			return true;
		}

		bool readCloseLoopParams(ros::NodeHandle &n)
		{
			XmlRpc::XmlRpcValue pid_param_list;

			int val;
			if (n.getParam("pid_slot", val))
			{
				pidf_reference_slot_ = val;
			}
			if (!n.getParam("close_loop_values", pid_param_list))
				return true;
			if (static_cast<size_t>(pid_param_list.size()) <= hardware_interface::SPARK_MAX_PID_SLOTS)
			{
				for (int i = 0; i < pid_param_list.size(); i++)
				{
					XmlRpc::XmlRpcValue &pidparams = pid_param_list[i];

					findFloatParam("p", pidparams, p_gain_[i]);
					findFloatParam("i", pidparams, i_gain_[i]);
					findFloatParam("d", pidparams, d_gain_[i]);
					findFloatParam("f", pidparams, f_gain_[i]);
					findFloatParam("i_zone", pidparams, i_zone_[i]);
					findFloatParam("d_filter", pidparams, d_filter_[i]);
					findFloatParam("output_min", pidparams, pidf_output_min_[i]);
					findFloatParam("output_max", pidparams, pidf_output_max_[i]);
					findFloatParam("arb_feed_forward", pidparams, pidf_arb_feed_forward_[i]);
					std::string arb_feed_forward_units_string;
					findStringParam("arb_feed_forward_units", pidparams, arb_feed_forward_units_string);
					if (arb_feed_forward_units_string.length())
					{
						if (arb_feed_forward_units_string == "voltage")
							pidf_arb_feed_forward_units_[i] = hardware_interface::ArbFFUnits::kVoltage;
						else if (arb_feed_forward_units_string == "percent_out")
							pidf_arb_feed_forward_units_[i] = hardware_interface::ArbFFUnits::kPercentOut;
						else
						{
							ROS_ERROR_STREAM("Invalid entry for arb_feed_forward_units : " << arb_feed_forward_units_string
									<< " (valid options are voltage and percent_out");
							return false;
						}
					}
				}
				return true;
			}
			else
			{
				throw std::runtime_error("Too many pid_param values");
			}
			return false;
		}

		bool readLimitSwitches(ros::NodeHandle &n)
		{
			std::string str_val;
			if (n.getParam("forward_limit_switch_polarity", str_val))
			{
				hardware_interface::LimitSwitchPolarity limit_switch_polarity;
				if (!stringToLimitSwitchPolarity(str_val, limit_switch_polarity))
					return false;
				forward_limit_switch_polarity_ = limit_switch_polarity;
			}
			n.getParam("reverse_limit_switch_enabled", reverse_limit_switch_enabled_);
			if (n.getParam("reverse_limit_switch_polarity", str_val))
			{
				hardware_interface::LimitSwitchPolarity limit_switch_polarity;
				if (!stringToLimitSwitchPolarity(str_val, limit_switch_polarity))
					return false;
				reverse_limit_switch_polarity_ = limit_switch_polarity;
			}
			n.getParam("reverse_limit_switch_enabled", reverse_limit_switch_enabled_);
			return true;
		}

		bool readCurrentLimits(ros::NodeHandle &n)
		{
			int val;
			if (n.getParam("current_limit", val))
			{
				current_limit_ = val;
			}
			if (n.getParam("current_limit_stall", val))
			{
				current_limit_stall_ = val;
			}
			if (n.getParam("current_limit_free", val))
			{
				current_limit_free_ = val;
			}
			if (n.getParam("current_limit_rpm", val))
			{
				current_limit_RPM_ = val;
			}
			if (n.getParam("secondary_current_limit", val))
			{
				secondary_current_limit_ = val;
			}
			if (n.getParam("secondary_current_limit_cycles", val))
			{
				secondary_current_limit_cycles_ = val;
			}
			return true;
		}

		bool readIdleMode(ros::NodeHandle &n)
		{
			std::string str_val;
			if (n.getParam("idle_mode", str_val))
			{
				hardware_interface::IdleMode idle_mode;
				if (!stringToIdleMode(str_val, idle_mode))
					return false;
				idle_mode_ = idle_mode;
			}
			return true;
		}

		bool readVoltageCompensation(ros::NodeHandle &n)
		{
			const bool read_enable = n.getParam("voltage_compensation_enable", voltage_compensation_enable_);
			const bool read_voltage = n.getParam("voltage_compensation_nominal_voltage", voltage_compensation_nominal_voltage_);
			if (read_enable && !read_voltage)
			{
				ROS_WARN("voltage_compensation_enable set without setting voltage_compensation_nominal_voltage, enabling with default value");
			}
			else if (!read_enable && read_voltage)
			{
				ROS_WARN("voltage_compensation_nominal_voltage set without setting voltage_compensation_enable, voltage compenstaion remains disabled");
			}

			return true;
		}

		bool readRampRate(ros::NodeHandle &n)
		{
			n.getParam("open_loop_ramp_rate", open_loop_ramp_rate_);
			n.getParam("closed_loop_ramp_rate", closed_loop_ramp_rate_);
			return true;
		}

		bool readSoftlimit(ros::NodeHandle &n)
		{
			const bool read_forward_enable = n.getParam("forward_softlimit_enable", forward_softlimit_enable_);
			const bool read_forward_limit  = n.getParam("forward_softlimit", forward_softlimit_);
			if (read_forward_enable && !read_forward_limit)
			{
				ROS_ERROR("Can not enable forward limit without forward_softlimit set");
				return false;
			}
			else if (!read_forward_enable && read_forward_limit)
			{
				ROS_WARN("forward softlimit value set but forward_softlimit_enable not set");
			}
			const bool read_reverse_enable = n.getParam("reverse_softlimit_enable", reverse_softlimit_enable_);
			const bool read_reverse_limit  = n.getParam("reverse_softlimit", reverse_softlimit_);
			if (read_reverse_enable && !read_reverse_limit)
			{
				ROS_ERROR("Can not enable reverse limit without reverse_softlimit set");
				return false;
			}
			else if (!read_reverse_enable && read_reverse_limit)
			{
				ROS_WARN("reverse softlimit value set but reverse_softlimit_enable not set");
			}

			return true;
		}
		bool readFollower(ros::NodeHandle &n)
		{
			std::string follower_type_str;
			const bool read_follower_type = n.getParam("follower_type", follower_type_str);
			if (read_follower_type)
			{
				if (follower_type_str == "disabled")
					follower_type_ = hardware_interface::kFollowerDisabled;
				else if (follower_type_str == "spark_max")
					follower_type_ = hardware_interface::kFollowerSparkMax;
				else if (follower_type_str == "phoenix")
					follower_type_ = hardware_interface::kFollowerPhoenix;
				else
				{
					ROS_ERROR_STREAM("Invalid value for follower_type : " << follower_type_str
							<< " (valid options are disabled, spark_max, phoenix)");
					return false;
				}
			}
			const bool read_follower_id = n.getParam("follower_id", follower_id_);
			if (read_follower_type && (follower_type_ != hardware_interface::kFollowerDisabled) && !read_follower_id)
			{
				ROS_ERROR("Follower type set without setting follower_id");
				return false;
			}
			n.getParam("follower_invert", follower_invert_);
			return true;
		}
		bool readEncoder(ros::NodeHandle &n)
		{
			int val;
			if (n.getParam("encoder_ticks_per_roation", val))
			{
				encoder_ticks_per_rotation_ = val;
			}
			std::string encoder_type_str;
			if (n.getParam("encoder_type", encoder_type_str))
			{
				if (encoder_type_str == "no_sensor")
					encoder_type_ = hardware_interface::kNoSensor;
				else if (encoder_type_str == "hall_sensor")
					encoder_type_ = hardware_interface::kHallSensor;
				else if (encoder_type_str == "quadrature")
					encoder_type_ = hardware_interface::kQuadrature;
				else
				{
					ROS_ERROR_STREAM("Invalid value for encoder_type : " << encoder_type_str
							<< " (valid options are no_sensor, hall_sensor, quadrature, sensorless");
					return false;
				}
			}
			return true;
		}

		std::string  joint_name_;
		bool         inverted_{false};

		// PID Controller
		std::array<double,                          hardware_interface::SPARK_MAX_PID_SLOTS> p_gain_{0};
		std::array<double,                          hardware_interface::SPARK_MAX_PID_SLOTS> i_gain_{0};
		std::array<double,                          hardware_interface::SPARK_MAX_PID_SLOTS> d_gain_{0};
		std::array<double,                          hardware_interface::SPARK_MAX_PID_SLOTS> f_gain_{0};
		std::array<double,                          hardware_interface::SPARK_MAX_PID_SLOTS> i_zone_{0};
		std::array<double,                          hardware_interface::SPARK_MAX_PID_SLOTS> d_filter_{0};
		std::array<double,                          hardware_interface::SPARK_MAX_PID_SLOTS> pidf_output_min_{-1};
		std::array<double,                          hardware_interface::SPARK_MAX_PID_SLOTS> pidf_output_max_{1};
		std::array<hardware_interface::ControlType, hardware_interface::SPARK_MAX_PID_SLOTS> pidf_reference_ctrl_{hardware_interface::kDutyCycle};
		std::array<double,                          hardware_interface::SPARK_MAX_PID_SLOTS> pidf_arb_feed_forward_{0};
		std::array<hardware_interface::ArbFFUnits,  hardware_interface::SPARK_MAX_PID_SLOTS> pidf_arb_feed_forward_units_{hardware_interface::ArbFFUnits::kVoltage};
		size_t                                                                               pidf_reference_slot_{0};


		// Forward and Reverse Limit switches
		hardware_interface::LimitSwitchPolarity forward_limit_switch_polarity_{hardware_interface::kNormallyOpen};
		bool                                    forward_limit_switch_enabled_{false};
		hardware_interface::LimitSwitchPolarity reverse_limit_switch_polarity_{hardware_interface::kNormallyOpen};
		bool                                    reverse_limit_switch_enabled_{false};

		//TODO :: better defaults for these?
		unsigned int current_limit_{0};
		unsigned int current_limit_stall_{0};
		unsigned int current_limit_free_{0};
		unsigned int current_limit_RPM_{0};
		double       secondary_current_limit_{0};
		int          secondary_current_limit_cycles_{0};

		hardware_interface::IdleMode idle_mode_{hardware_interface::kCoast};

		bool         voltage_compensation_enable_{false};
		double       voltage_compensation_nominal_voltage_{12.};

		double       open_loop_ramp_rate_{0};
		double       closed_loop_ramp_rate_{0};

		hardware_interface::ExternalFollower follower_type_{hardware_interface::kFollowerDisabled};
		int          follower_id_{-1};
		bool         follower_invert_{false};

		bool         forward_softlimit_enable_{false};
		double       forward_softlimit_{0.0};

		bool         reverse_softlimit_enable_{false};
		double       reverse_softlimit_{0.0};

		unsigned int encoder_ticks_per_rotation_{4096};

		hardware_interface::SensorType encoder_type_{hardware_interface::kNoSensor};

	private:
		// Read a double named <param_name> from the array/map in params
		bool findFloatParam(std::string param_name, XmlRpc::XmlRpcValue &params, double &val) const
		{
			if (!params.hasMember(param_name))
				return false;
			XmlRpc::XmlRpcValue &param = params[param_name];
			if (!param.valid())
				throw std::runtime_error(param_name + " was not a double valid type");
			if (param.getType() == XmlRpc::XmlRpcValue::TypeDouble)
			{
				val = (double)param;
				return true;
			}
			else if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
			{
				val = (int)param;
				return true;
			}
			else
				throw std::runtime_error("A non-double value was passed for" + param_name);

			return false;
		}

		// Read an integer named <param_name> from the array/map in params
		bool findIntParam(std::string param_name, XmlRpc::XmlRpcValue &params, int &val) const
		{
			if (!params.hasMember(param_name))
				return false;
			XmlRpc::XmlRpcValue &param = params[param_name];
			if (!param.valid())
				throw std::runtime_error(param_name + " was not a valid int type");
			if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
				val = (int)param;
			else
				throw std::runtime_error("A non-int value was passed for" + param_name);
			return false;
		}
		//
		// Read an string named <param_name> from the array/map in params
		bool findStringParam(std::string param_name, XmlRpc::XmlRpcValue &params, std::string &val) const
		{
			if (!params.hasMember(param_name))
				return false;
			XmlRpc::XmlRpcValue &param = params[param_name];
			if (!param.valid())
				throw std::runtime_error(param_name + " was not a valid string type");
			if (param.getType() == XmlRpc::XmlRpcValue::TypeString)
				val = (int)param;
			else
				throw std::runtime_error("A non-string value was passed for" + param_name);
			return false;
		}

		bool stringToLimitSwitchPolarity(const std::string &str,
				hardware_interface::LimitSwitchPolarity &limit_switch_polarity)
		{
			if (str == "normally_open")
				limit_switch_polarity = hardware_interface::kNormallyOpen;
			else if (str == "normally_closed")
				limit_switch_polarity = hardware_interface::kNormallyClosed;
			else
			{
				ROS_ERROR_STREAM("Invalid limit switch polarity : " << str);
				return false;
			}
			return true;
		}

		bool stringToIdleMode(const std::string &str,
				hardware_interface::IdleMode &idle_mode)
		{
			if (str == "coast")
				idle_mode = hardware_interface::kCoast;
			else if (str == "brake")
				idle_mode = hardware_interface::kBrake;
			else
			{
				ROS_ERROR_STREAM("Invalid idle mode : " << str);
				return false;
			}
			return true;
		}
};

class SparkMaxControllerInterface
{
	public:
		SparkMaxControllerInterface(void)
			: srv_mutex_{std::make_shared<boost::recursive_mutex>()}
			, srv_update_thread_flag_{std::make_shared<std::atomic_flag>()}
		    , srv_update_thread_active_{std::make_shared<std::atomic<bool>>(true)}
		    , srv_update_thread_{nullptr}
		{
			srv_update_thread_flag_->test_and_set();
		}

		virtual ~SparkMaxControllerInterface()
		{
			if (srv_update_thread_ && srv_update_thread_->joinable())
			{
				*srv_update_thread_active_ = false;
				srv_update_thread_->join();
			}
		}

		virtual bool readParams(ros::NodeHandle &n, SparkMaxCIParams &params)
		{
			return params.readJointName(n) &&
				   params.readCloseLoopParams(n) &&
				   params.readLimitSwitches(n) &&
				   params.readCurrentLimits(n) &&
				   params.readIdleMode(n) &&
				   params.readVoltageCompensation(n) &&
				   params.readRampRate(n) &&
				   params.readSoftlimit(n) &&
				   params.readFollower(n) &&
				   params.readEncoder(n);
		}

		// Read params from config file and use them to
		// initialize the Spark Max hardware
		virtual bool initWithNode(hardware_interface::SparkMaxCommandInterface *smci,
								  hardware_interface::SparkMaxStateInterface * /*smsi*/,
								  ros::NodeHandle &n)
		{
			return init(smci, n, spark_max_, srv_mutex_, false) &&
				   setInitialMode();
		}

		// Same as above, except pass in an array
		// of node handles. First entry is set up as the master
		// spark max and the rest are set in follower mode to follow
		// the leader
		virtual bool initWithNode(hardware_interface::SparkMaxCommandInterface *smci,
								  hardware_interface::SparkMaxStateInterface *smsi,
								  std::vector<ros::NodeHandle> &n)
		{
			if (!initWithNode(smci, smsi, n[0]))
				return false;

			const int follow_can_id = spark_max_.state()->getDeviceId();

			follower_spark_maxs_.resize(n.size() - 1);
			for (size_t i = 1; i < n.size(); i++)
			{
				follower_srv_mutexes_.push_back(nullptr);
				follower_srvs_.push_back(nullptr);
				if (!init(smci, n[i], follower_spark_maxs_[i-1], nullptr, true))
					return false;
				follower_spark_maxs_[i-1]->setFollowerType(hardware_interface::kFollowerSparkMax);
				follower_spark_maxs_[i-1]->setFollowerID(follow_can_id);
				ROS_INFO_STREAM("Set up Spark Max " << follower_spark_maxs_[i-1].getName()
						<< " to follow Spark Max CAN ID " << follow_can_id
						<< " (" << spark_max_.getName() << ")");
			}

			return true;
		}

		// Init with XmlRpcValue instead of NodeHandle - XmlRpcValue
		// will be either a string or an array of strings of joints
		// to load
		virtual bool initWithNode(hardware_interface::SparkMaxCommandInterface *smci,
								  hardware_interface::SparkMaxStateInterface *smsi,
								  ros::NodeHandle &controller_nh,
								  XmlRpc::XmlRpcValue param)
		{
			std::vector<ros::NodeHandle> joint_nodes;

			if (param.getType() == XmlRpc::XmlRpcValue::TypeArray)
			{
				if (param.size() == 0)
				{
					ROS_ERROR_STREAM("Joint param is an empty list");
					return false;
				}

				for (int i = 0; i < param.size(); ++i)
				{
					if (param[i].getType() != XmlRpc::XmlRpcValue::TypeString)
					{
						ROS_ERROR_STREAM("Joint param #" << i << " isn't a string.");
						return false;
					}
				}

				for (int i = 0; i < param.size(); ++i)
					joint_nodes.push_back(ros::NodeHandle(controller_nh,
								static_cast<std::string>(param[i])));
			}
			else if (param.getType() == XmlRpc::XmlRpcValue::TypeString)
			{
				joint_nodes.push_back(ros::NodeHandle(controller_nh,
							static_cast<std::string>(param)));
			}
			else
			{
				ROS_ERROR_STREAM("Joint param is neither a list of strings nor a string.");
				return false;
			}

			return initWithNode(smci, smsi, joint_nodes);
		}

		void callback(SparkMaxConfig &config, uint32_t /*level*/)
		{
			// TODO : this list is rapidly getting out of date.
			// Update it or remove the printout?
			ROS_INFO_STREAM("Reconfigure request : %s" << spark_max_.getName().c_str());
			writeParamsToHW(SparkMaxCIParams(config), spark_max_);
		}

		// Set the setpoint for the motor controller
		virtual void setCommand(const double command)
		{
			spark_max_->setSetPoint(command);
		}

		// Set the mode of the motor controller
		virtual void setMode(hardware_interface::ControlType mode)
		{
			spark_max_->setPIDFReferenceCtrl(params_.pidf_reference_slot_, mode);
		}

		virtual bool setPIDFSlot(size_t slot)
		{
			if (slot >= hardware_interface::SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR ("PID slot out of range in SparkMaxControllerInterface::setPIDFSlot()");
				return false;
			}
			params_.pidf_reference_slot_ = slot;

			// If dynamic reconfigure is running update
			// the reported config there with the new internal
			// state
			syncDynamicReconfigure();

			spark_max_->setPIDFReferenceSlot(slot);
			return true;
		}

	protected:
		hardware_interface::SparkMaxCommandHandle                    spark_max_;
		SparkMaxCIParams                                             params_;
		std::shared_ptr<boost::recursive_mutex>                      srv_mutex_;
		//
		// Variables for a thread which updates the dynamic reconfigure server
		// when vars are updated from calls to the interafce. This keeps the
		// values in the dynamic reconfigure GUI in sync with the values set
		// via functions called in this interface
		std::shared_ptr<std::atomic_flag>                    srv_update_thread_flag_;
		std::shared_ptr<std::atomic<bool>>                   srv_update_thread_active_;
		std::shared_ptr<std::thread>                         srv_update_thread_;

		// List of follower talons associated with the master
		// listed above
		std::vector<std::shared_ptr<dynamic_reconfigure::Server<SparkMaxConfig>>> follower_srvs_;
		std::vector<std::shared_ptr<boost::recursive_mutex>>                      follower_srv_mutexes_;
		std::vector<hardware_interface::SparkMaxCommandHandle>                    follower_spark_maxs_;

		// Used to set initial (and only) talon
		// mode for FixedMode derived classes
		virtual bool setInitialMode(void)
		{
			ROS_INFO_STREAM("SparkMax " << spark_max_.getName() << " Base class setInitialMode");
			return true;
		}

	private :
		virtual bool init(hardware_interface::SparkMaxCommandInterface *smci,
						  ros::NodeHandle &n,
						  hardware_interface::SparkMaxCommandHandle &spark_max,
						  std::shared_ptr<boost::recursive_mutex> srv_mutex,
						  bool follower)
		{
			ROS_WARN("spark max init start");
			// Read params from startup and intialize SparkMax using them
			SparkMaxCIParams params;
			if (!readParams(n, params))
			   return false;

			ROS_WARN("spark max init past readParams");

			spark_max = smci->getHandle(params.joint_name_);
			if (!writeParamsToHW(params, spark_max, !follower))
				return false;

			ROS_WARN("spark_max init past writeParamsToHW");
			bool dynamic_reconfigure = false;
			if (!follower)
			{
				n.param<bool>("dynamic_reconfigure", dynamic_reconfigure, false);
			}
			if (dynamic_reconfigure)
			{
				// Create dynamic_reconfigure Server. Pass in n
				// so that all the vars for the class are grouped
				// under the node's name.  Doing so allows multiple
				// copies of the class to be started, each getting
				// their own namespace.
				auto srv = std::make_shared<dynamic_reconfigure::Server<SparkMaxConfig>>(*srv_mutex, n);

				ROS_WARN("init updateConfig");
				// Without this, the first call to callback()
				// will overwrite anything passed in from the
				// launch file
				srv->updateConfig(params_.toConfig());

				ROS_WARN("init setCallback");
				// Register a callback function which is run each
				// time parameters are changed using
				// rqt_reconfigure or the like
				srv->setCallback(boost::bind(&SparkMaxControllerInterface::callback, this, _1, _2));
				// Create a thread to update the server with new values written by users of this interface
				srv_update_thread_ = std::make_shared<std::thread>(std::bind(&SparkMaxControllerInterface::srvUpdateThread, this, srv));
			}

			ROS_WARN("spark max init returning");

			return true;
		}

		// If dynamic reconfigure is running then update
		// the reported config there with the new internal
		// state
		// Trigger a write of the current CIParams to the DDR server. This needs
		// to happen if the values have been updated via the interface code.
		// The meaning of the flag - set == no new updates, cleared == data has
		// been updated via code and the reconfigure gui needs that new data sent
		// to it to stay in sync
		// This call is on the control loop update path and needs to be quick.
		// atomic_flags are lock free, so clearing it should be a fast operation
		// which never blocks.  Thus, with any luck it won't slow down the
		// control loop by any significant amount.
		void syncDynamicReconfigure()
		{
			srv_update_thread_flag_->clear();
		}

		// Loop forever, waiting for requests from the main thread
		// to update values from this class to the DDR server.
		void srvUpdateThread(std::shared_ptr<dynamic_reconfigure::Server<SparkMaxConfig>> srv)
		{
			ROS_INFO_STREAM("srvUpdateThread started for joint " << params_.joint_name_);
#ifdef __linux__
			struct sched_param sp;
			sp.sched_priority = 0;
			sched_setscheduler(0, SCHED_IDLE, &sp); // GUI update is low priority compared to driving the robot
			pthread_setname_np(pthread_self(), "tci_ddr_upd");
			ROS_INFO_STREAM("srvUpdateThread priority set for joint " << params_.joint_name_);
#endif
			*srv_update_thread_active_ = true;
			ros::Rate r(10);
			while (*srv_update_thread_active_)
			{
				// Loop forever, periodically checking for the flag to be cleared
				// Test and set returns the previous value of the variable and sets
				// it, all in one atomic operation.  The set will reset the flag
				// so after running updateConfig() the code will loop back and wait
				// here for the next time the flag is cleared by syncDynamicReconfigure.
				while (srv_update_thread_flag_->test_and_set())
				{
					r.sleep();
				}

				if (srv)
				{
					SparkMaxConfig config(params_.toConfig());
					srv->updateConfig(config);
				}
			}
		}


		// Use data in params to actually set up SparkMax
		// hardware. Make this a separate method outside of
		// init() so that dynamic reconfigure callback can write
		// values using this method at any time
		virtual bool writeParamsToHW(const SparkMaxCIParams &params,
									 hardware_interface::SparkMaxCommandHandle &spark_max,
									 bool update_params = true)
		{
			spark_max->setInverted(params.inverted_);
			for (size_t i = 0; i < hardware_interface::SPARK_MAX_PID_SLOTS; i++)
			{
				spark_max->setPGain(i, params.p_gain_[i]);
				spark_max->setIGain(i, params.i_gain_[i]);
				spark_max->setDGain(i, params.d_gain_[i]);
				spark_max->setFGain(i, params.f_gain_[i]);
				spark_max->setIZone(i, params.i_zone_[i]);
				spark_max->setDFilter(i, params.d_filter_[i]);
				spark_max->setPIDFOutputMin(i, params.pidf_output_min_[i]);
				spark_max->setPIDFOutputMax(i, params.pidf_output_max_[i]);
				spark_max->setPIDFArbFeedForward(i, params.pidf_arb_feed_forward_[i]);
				spark_max->setPIDFArbFeedForwardUnits(i, params.pidf_arb_feed_forward_units_[i]);
			}
			spark_max->setPIDFReferenceSlot(params.pidf_reference_slot_);

			spark_max->setForwardLimitSwitchPolarity(params.forward_limit_switch_polarity_);
			spark_max->setForwardLimitSwitchEnabled(params.forward_limit_switch_enabled_);
			spark_max->setReverseLimitSwitchPolarity(params.reverse_limit_switch_polarity_);
			spark_max->setReverseLimitSwitchEnabled(params.reverse_limit_switch_enabled_);

			spark_max->setCurrentLimit(params.current_limit_);
			spark_max->setCurrentLimitStall(params.current_limit_stall_);
			spark_max->setCurrentLimitFree(params.current_limit_free_);
			spark_max->setCurrentLimitRPM(params.current_limit_RPM_);
			spark_max->setSecondaryCurrentLimit(params.secondary_current_limit_);
			spark_max->setSecondaryCurrentLimitCycles(params.secondary_current_limit_cycles_);

			spark_max->setIdleMode(params.idle_mode_);

			spark_max->setVoltageCompensationNominalVoltage(params.voltage_compensation_nominal_voltage_);
			spark_max->setVoltageCompensationEnable(params.voltage_compensation_enable_);

			spark_max->setOpenLoopRampRate(params.open_loop_ramp_rate_);
			spark_max->setClosedLoopRampRate(params.closed_loop_ramp_rate_);

			spark_max->setForwardSoftlimit(params.forward_softlimit_);
			spark_max->setForwardSoftlimitEnable(params.forward_softlimit_enable_);
			spark_max->setReverseSoftlimit(params.reverse_softlimit_);
			spark_max->setReverseSoftlimitEnable(params.reverse_softlimit_enable_);

			spark_max->setEncoderTicksPerRotation(params.encoder_ticks_per_rotation_);
			spark_max->setEncoderType(params.encoder_type_);

			// Save copy of params written to HW
			// so they can be queried later?
			if (update_params)
				params_ = params;

			return true;
		}
};

// Base --^

// A derived class which disables mode switching. Any
// single-mode CI class should derive from this class
class SparkMaxFixedModeControllerInterface : public SparkMaxControllerInterface
{
	protected:
		// Disable changing mode for controllers derived from this class
		void setMode(hardware_interface::ControlType /*mode*/) override
		{
			ROS_WARN("Can't reset mode using this SparkMaxControllerInterface");
		}
};



// Duty Cycle
class SparkMaxDutyCycleControllerInterface : public SparkMaxFixedModeControllerInterface
{
	protected:
		bool setInitialMode(void) override
		{
			// Set the mode at init time - since this
			// class is derived from the FixedMode class
			// it can't be reset
			spark_max_->setPIDFReferenceCtrl(params_.pidf_reference_slot_, hardware_interface::kDutyCycle);
			ROS_INFO_STREAM("Set up spark max" << spark_max_.getName() << " in duty cycle mode");
			return true;
		}
		// Maybe disable the setPIDFSlot call since that makes
		// no sense for a non-PID controller mode?
};

// Position
// Velocity
// Voltage
class SparkMaxCloseLoopControllerInterface : public SparkMaxFixedModeControllerInterface
{
};

class SparkMaxPositionCloseLoopControllerInterface : public SparkMaxCloseLoopControllerInterface
{
	protected:
		bool setInitialMode(void) override
		{
			// Set to speed close loop mode
			spark_max_->setPIDFReferenceCtrl(params_.pidf_reference_slot_, hardware_interface::kPosition);
			ROS_INFO_STREAM("Set up spark_max " << spark_max_.getName() << " in Close Loop Position mode");

			return true;
		}
};

class SparkMaxVelocityCloseLoopControllerInterface : public SparkMaxCloseLoopControllerInterface
{
	protected:
		bool setInitialMode(void) override
		{
			// Set to speed close loop mode
			spark_max_->setPIDFReferenceCtrl(params_.pidf_reference_slot_, hardware_interface::kVelocity);
			ROS_INFO_STREAM("Set up spark_max " << spark_max_.getName() << " in Close Loop Velocity mode");

			return true;
		}
};

class SparkMaxVoltageCloseLoopControllerInterface : public SparkMaxCloseLoopControllerInterface
{
	protected:
		bool setInitialMode(void) override
		{
			// Set to speed close loop mode
			spark_max_->setPIDFReferenceCtrl(params_.pidf_reference_slot_, hardware_interface::kVoltage);
			ROS_INFO_STREAM("Set up spark_max " << spark_max_.getName() << " in Close Loop Voltage mode");

			return true;
		}
};
class SparkMaxFollowerControllerInterface : public SparkMaxFixedModeControllerInterface
{
	public:
		bool initWithNode(hardware_interface::SparkMaxCommandInterface *smci,
						  hardware_interface::SparkMaxStateInterface   *smsi,
						  ros::NodeHandle &n) override
		{
			if (!smsi)
			{
				ROS_ERROR("NULL SparkMaxStateInterface in SparkMaxFollowerCommandInterface");
				return false;
			}

			std::string follow_joint_name;
			if (!n.getParam("follow_joint", follow_joint_name))
			{
				ROS_ERROR("No follow joint specified for SparkMaxFollowerControllerInterface");
				return false;
			}

			hardware_interface::SparkMaxStateHandle follow_handle = smsi->getHandle(follow_joint_name);
			const int follow_device_id = follow_handle->getDeviceId();

			if (!commonInit(smci, n, hardware_interface::kFollowerSparkMax, follow_device_id))
				return false;

			ROS_INFO_STREAM("Launching follower " << params_.joint_name_ <<
					" to follow Spark Max CAN ID " << follow_device_id <<
					" (" << follow_handle.getName() << ")");
			return true;
		}

#if 0 // Does phoenix follower mode work?
		bool initWithNode(hardware_interface::SparkMaxCommandInterface *smci,
						  hardware_interface::TalonStateInterface      *tsi,
						  ros::NodeHandle &n) override
		{
			if (!tsi)
			{
				ROS_ERROR("NULL SparkMaxStateInterface in SparkMaxFollowerCommandInterface");
				return false;
			}

			std::string follow_joint_name;
			if (!n.getParam("follow_joint", follow_joint_name))
			{
				ROS_ERROR("No follow joint specified for SparkMaxFollowerControllerInterface");
				return false;
			}

			hardware_interface::TalonStateHandle follow_handle = tsi->getHandle(follow_joint_name);
			const int follow_device_id = follow_handle->getDeviceId();
j
			if (!commonInit(smci, n, hardware_interface::kFollowerPhoenix, follow_device_id))
				return false;

			ROS_INFO_STREAM("Launching follower " << params_.joint_name_ <<
					" to follow Talon CAN ID " << follow_device_id <<
					" (" << follow_handle.getName() << ")");
			return true;
		}
#endif

		// Maybe disable the setPIDFSlot call since that makes
		// no sense for a non-PID controller mode?
		void setCommand(const double /*command*/) override
		{
			ROS_WARN("Can't set a command in follower mode!");
		}

	private:
		bool commonInit(hardware_interface::SparkMaxCommandInterface *tci,
						ros::NodeHandle &n,
						hardware_interface::ExternalFollower follow_device_type,
						int follow_device_id)
		{
			// Call base-class init to load config params
			if (!SparkMaxControllerInterface::initWithNode(tci, nullptr, n))
			{
				ROS_ERROR("SparkMaxFollowerController base initWithNode failed");
				return false;
			}
			// Set the mode and CAN ID of talon to follow at init time -
			// since this class is derived from the FixedMode class
			// these can't be reset. Hopefully we never have a case
			// where a follower mode SparkMax changes which other
			// SparkMax it is following during a match?
			spark_max_->setFollowerType(follow_device_type);
			spark_max_->setFollowerID(follow_device_id);
			return true;
		}
};
}

#endif