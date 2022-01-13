#pragma once

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

		// Override operator= to only copy fields actually
		// read from ph - write() will set vales of
		// write-only config items sent to PH
		PHHWState& operator=(const PHHWState &other)
		{
			if (this != &other)
			{
				compressor_enabled_ = other.compressor_enabled_;
				pressure_switch_    = other.pressure_switch_;
				compressor_current_ = other.compressor_current_;
				analog_voltage_     = other.analog_voltage_;
			}
			return *this;
		}

		int32_t getId(void)                         const { return id_; }
		bool    getCompressorEnabled(void)          const { return compressor_enabled_; }
		bool    getPressureSwitch(void)             const { return pressure_switch_; }
		double  getCompressorCurrent(void)          const { return compressor_current_; }
		double  getCompressorMinAnalogVoltage(void) const { return compressor_min_analog_voltage_; }
		double  getCompressorMaxAnalogVoltage(void) const { return compressor_max_analog_voltage_; }
		bool    getCompressorForceDisable(void)     const { return compressor_force_disable_; }
		bool    getCompressorUseDigital(void)       const { return compressor_use_digital_;; }
		double  getAnalogVoltage(size_t channel)   const
		{
			if (channel >= analog_voltage_.size())
			{
				ROS_ERROR_STREAM("PH getAnalogVoltage channel out of bounds : " << channel);
				return 0.0;
			}
			return analog_voltage_[channel];
		}

		void setCompressorEnabled(bool compressor_enabled)                       { compressor_enabled_ = compressor_enabled; }
		void setPressureSwitch(bool pressure_switch)                             { pressure_switch_ = pressure_switch; }
		void setCompressorCurrent(double compressor_current)                     { compressor_current_ = compressor_current; }
		void setCompressorMinAnalogVoltage(double compressor_min_analog_voltage) { compressor_min_analog_voltage_ = compressor_min_analog_voltage; }
		void setCompressorMaxAnalogVoltage(double compressor_max_analog_voltage) { compressor_max_analog_voltage_ = compressor_max_analog_voltage; }
		void setCompressorForceDisable(double compressor_force_disable)          { compressor_force_disable_ = compressor_force_disable; }
		void setCompressorUseDigital(double compressor_use_digital)              { compressor_use_digital_ = compressor_use_digital; }
		void setAnalogVoltage(double analog_voltage, size_t channel)
		{
			if (channel >= analog_voltage_.size())
			{
				ROS_ERROR_STREAM("PH setAnalogVoltage channel out of bounds : " << channel);
				return;
			}
			analog_voltage_[channel] = analog_voltage;
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
};

typedef StateHandle<const PHHWState> PHStateHandle;
typedef StateHandle<PHHWState> PHWritableStateHandle;
class PHStateInterface       : public HardwareResourceManager<PHStateHandle> {};
class RemotePHStateInterface : public HardwareResourceManager<PHWritableStateHandle, ClaimResources> {};

} // namespace
