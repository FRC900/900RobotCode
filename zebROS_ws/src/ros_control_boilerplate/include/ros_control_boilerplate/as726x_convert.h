#include "as726x_interface/as726x_interface.h"
#include "ros_control_boilerplate/AS726x.h"

namespace as726x_convert
{
class AS726xConvert
{
	public:
		bool indLedCurrentLimit(const hardware_interface::as726x::IndLedCurrentLimits input,
				as726x::ind_led_current_limits &output) const;
		bool drvLedCurrentLimit(const hardware_interface::as726x::DrvLedCurrentLimits input,
				as726x::drv_led_current_limits &output) const;
		bool conversionType(const hardware_interface::as726x::ConversionTypes input,
				as726x::conversion_types &output) const;
		bool channelGain(const hardware_interface::as726x::ChannelGain input,
				as726x::channel_gain &output) const;

};

} // namespace as726x_convert
