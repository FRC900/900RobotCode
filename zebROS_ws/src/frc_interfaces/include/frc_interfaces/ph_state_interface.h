#ifndef PH_STATE_INTERFACE_INC__
#define PH_STATE_INTERFACE_INC__

#include <array>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>

namespace hardware_interface
{

class PHHWState
{
	public:
		PHHWState(int32_t id)
			: id_(id)
		{
		}
		PHHWState(const PHHWState &) = delete;
		PHHWState(PHHWState &&) noexcept = delete;
		~PHHWState() = default;

		// Override operator= to only copy fields actually
		// read from ph - write() will set vales of
		// write-only config items sent to PH
		PHHWState& operator=(const PHHWState &other)
		{
			if (this != &other)
			{
				compressor_enabled_            = other.compressor_enabled_;
				pressure_switch_               = other.pressure_switch_;
				compressor_current_            = other.compressor_current_;
				analog_voltage_                = other.analog_voltage_;
				compressor_min_analog_voltage_ = other.compressor_max_analog_voltage_;
				compressor_max_analog_voltage_ = other.compressor_max_analog_voltage_;
				compressor_force_disable_      = other.compressor_force_disable_;
				compressor_use_digital_        = other.compressor_use_digital_;
				pressure_                      = other.pressure_;
			}
			return *this;
		}
		PHHWState& operator=(PHHWState &&) noexcept = delete;

		int32_t getId(void)                         const { return id_; }
		bool    getCompressorEnabled(void)          const { return compressor_enabled_; }
		bool    getPressureSwitch(void)             const { return pressure_switch_; }
		double  getCompressorCurrent(void)          const { return compressor_current_; }
		double  getCompressorMinAnalogVoltage(void) const { return compressor_min_analog_voltage_; }
		double  getCompressorMaxAnalogVoltage(void) const { return compressor_max_analog_voltage_; }
		bool    getCompressorForceDisable(void)     const { return compressor_force_disable_; }
		bool    getCompressorUseDigital(void)       const { return compressor_use_digital_;; }
		double  getAnalogVoltage(size_t channel)    const
		{
			if (channel >= analog_voltage_.size())
			{
				ROS_ERROR_STREAM("PH getAnalogVoltage channel out of bounds : " << channel);
				return 0.0;
			}
			return analog_voltage_[channel];
		}
		double  getPressure(size_t channel)   const
		{
			if (channel >= pressure_.size())
			{
				ROS_ERROR_STREAM("PH getPressure channel out of bounds : " << channel);
				return 0.0;
			}
			return pressure_[channel];
		}

		void setCompressorEnabled(const bool compressor_enabled)                       { compressor_enabled_ = compressor_enabled; }
		void setPressureSwitch(const bool pressure_switch)                             { pressure_switch_ = pressure_switch; }
		void setCompressorCurrent(const double compressor_current)                     { compressor_current_ = compressor_current; }
		void setCompressorMinAnalogVoltage(const double compressor_min_analog_voltage) { compressor_min_analog_voltage_ = compressor_min_analog_voltage; }
		void setCompressorMaxAnalogVoltage(const double compressor_max_analog_voltage) { compressor_max_analog_voltage_ = compressor_max_analog_voltage; }
		void setCompressorForceDisable(const bool compressor_force_disable)            { compressor_force_disable_ = compressor_force_disable; }
		void setCompressorUseDigital(const bool compressor_use_digital)                { compressor_use_digital_ = compressor_use_digital; }
		void setAnalogVoltage(const double analog_voltage, size_t channel)
		{
			if (channel >= analog_voltage_.size())
			{
				ROS_ERROR_STREAM("PH setAnalogVoltage channel out of bounds : " << channel);
				return;
			}
			analog_voltage_[channel] = analog_voltage;
		}
		void setPressure(double pressure, size_t channel)
		{
			if (channel >= pressure_.size())
			{
				ROS_ERROR_STREAM("PH setPressure channel out of bounds : " << channel);
				return;
			}
			pressure_[channel] = pressure;
		}

	private:
		int32_t               id_;
		bool                  compressor_enabled_{false};
		bool                  pressure_switch_{false};
		double                compressor_current_{0};
		double                compressor_min_analog_voltage_{0.};
		double                compressor_max_analog_voltage_{13.5};
		bool                  compressor_force_disable_{false};
		bool                  compressor_use_digital_{true};
		std::array<double, 2> analog_voltage_{0, 0};
		std::array<double, 2> pressure_{0, 0};
};

using PHStateHandle = StateHandle<const PHHWState>;
using PHWritableStateHandle = StateHandle<PHHWState>;
class PHStateInterface       : public HardwareResourceManager<PHStateHandle> {};
class RemotePHStateInterface : public HardwareResourceManager<PHWritableStateHandle, ClaimResources> {};

} // namespace

#endif