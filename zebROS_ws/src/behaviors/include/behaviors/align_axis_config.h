// Class to hold configuration values for an alignment axis
// This is just a set of strings passed in to configure the axis
// to monitor a particular set of topics
#pragma once
#include <string>
class AlignActionAxisConfig
{
	public:
		AlignActionAxisConfig(const std::string &name,
							  const std::string &enable_pub_topic,
							  const std::string &error_sub_topic,
							  const std::string &timeout_param,
							  const std::string &error_threshold_param)
			: name_(name)
			, enable_pub_topic_(enable_pub_topic)
			, error_sub_topic_(error_sub_topic)
			, timeout_param_(timeout_param)
			, error_threshold_param_(error_threshold_param)
		{
		}
		std::string name_;
		std::string enable_pub_topic_;
		std::string error_sub_topic_;
		std::string timeout_param_;
		std::string error_threshold_param_;
};

