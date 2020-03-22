#pragma once

#include <atomic>
#include <mutex>
#include "ddynamic_reconfigure/ddynamic_reconfigure.h"
#include "talon_interface/canifier_command_interface.h"

namespace canifier_controller_interface
{
class CANifierCIParams
{
	public:
		CANifierCIParams(ros::NodeHandle n);

		void setLEDOutput(hardware_interface::canifier::LEDChannel index, double value, bool update_dynamic = true);
		void setGeneralPinOutputEnable(hardware_interface::canifier::GeneralPin index, bool value, bool update_dynamic = true);
		void setGeneralPinOutput(hardware_interface::canifier::GeneralPin index, bool value, bool update_dynamic = true);
		void setVelocityMeasurementPeriod(int period, bool update_dynamic = true);
		void setVelocityMeasurementWindow(int window, bool update_dynamic = true);
		void setClearPositionOnLimitF(bool value, bool update_dynamic = true);
		void setClearPositionOnLimitR(bool value, bool update_dynamic = true);
		void setClearPositionOnQuadIdx(bool value, bool update_dynamic = true);
		void setPWMOutputEnable(hardware_interface::canifier::PWMChannel index, bool value, bool update_dynamic = true);
		void setPWMOutput(hardware_interface::canifier::PWMChannel index, double value, bool update_dynamic = true);
		void setStatusFramePeriod(hardware_interface::canifier::CANifierStatusFrame frame_id, int period, bool update_dynamic = true);
		void setControlFramePeriod(hardware_interface::canifier::CANifierControlFrame frame_id, int period, bool update_dynamic = true);

		double getLEDOutput(hardware_interface::canifier::LEDChannel index) const;
		bool   getGeneralPinOutputEnable(hardware_interface::canifier::GeneralPin index) const;
		bool   getGeneralPinOutput(hardware_interface::canifier::GeneralPin index) const;
		hardware_interface::canifier::CANifierVelocityMeasPeriod getVelocityMeasurementPeriod(void) const;
		int    getVelocityMeasurementWindow(void) const;
		bool   getClearPositionOnLimitF(void) const;
		bool   getClearPositionOnLimitR(void) const;
		bool   getClearPositionOnQuadIdx(void) const;
		bool   getPWMOutputEnable(hardware_interface::canifier::PWMChannel index) const;
		double getPWMOutput(hardware_interface::canifier::PWMChannel index) const;
		int    getStatusFramePeriod(hardware_interface::canifier::CANifierStatusFrame frame_id) const;
		int    getControlFramePeriod(hardware_interface::canifier::CANifierControlFrame frame_id) const;

	private:
		std::array<std::atomic<double>, hardware_interface::canifier::LEDChannel::LEDChannelLast>                    led_output_;
		std::array<std::atomic<bool>,   hardware_interface::canifier::GeneralPin::GeneralPin_LAST>                   general_pin_output_enable_;
		std::array<std::atomic<bool>,   hardware_interface::canifier::GeneralPin::GeneralPin_LAST>                   general_pin_output_;
		std::atomic<hardware_interface::canifier::CANifierVelocityMeasPeriod>                                        velocity_measurement_period_;
		std::atomic<int>                                                                                             velocity_measurement_window_;
		std::atomic<bool>                                                                                            clear_position_on_limit_f_;
		std::atomic<bool>                                                                                            clear_position_on_limit_r_;
		std::atomic<bool>                                                                                            clear_position_on_quad_idx_;
		std::array<std::atomic<bool>,   hardware_interface::canifier::PWMChannel::PWMChannelLast>                    pwm_output_enable_;
		std::array<std::atomic<double>, hardware_interface::canifier::PWMChannel::PWMChannelLast>                    pwm_output_;
		std::array<std::atomic<int>,    hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Last> status_frame_period_;
		std::array<std::atomic<int>,    hardware_interface::canifier::CANifierControlFrame::CANifier_Control_Last>   control_frame_period_;

		ddynamic_reconfigure::DDynamicReconfigure                                                                    ddr_;

		const std::map<std::string, int> velocity_measurement_period_enum_map_ =
		{
			{"Period_1Ms", static_cast<int>(hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_1Ms)},
			{"Period_2Ms", static_cast<int>(hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_2Ms)},
			{"Period_5Ms", static_cast<int>(hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_5Ms)},
			{"Period_10Ms", static_cast<int>(hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_10Ms)},
			{"Period_20Ms", static_cast<int>(hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_20Ms)},
			{"Period_25Ms", static_cast<int>(hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_25Ms)},
			{"Period_50Ms", static_cast<int>(hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_50Ms)},
			{"Period_100Ms", static_cast<int>(hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_100Ms)}
		};

		template <typename T, size_t S>
		void  readIntoArray(ros::NodeHandle &n, const std::string &name, size_t index, std::array<std::atomic<T>, S> &array)
		{
			if (index >= array.size())
			{
				ROS_ERROR_STREAM ("Internal error reading " << name << " in " << __PRETTY_FUNCTION__ << " : index out of bounds");
				return;
			}
			T val;
			if (n.getParam(name, val))
				array[index] = val;
		}
		template <typename T>
		void readIntoScalar(ros::NodeHandle &n, const std::string &name, std::atomic<T> &scalar)
		{
			T val;
			if (n.getParam(name, val))
				scalar = val;
		}

		template <typename T>
		bool readIntoEnum(ros::NodeHandle &n,
				const std::string &param_name,
				const std::map<std::string, int> &mymap,
				std::atomic<T>&out) const
		{
			std::string param_str;
			if (!n.getParam(param_name, param_str))
				return true;

			const auto it = mymap.find(param_str);
			if (it != mymap.cend())
			{
				out = static_cast<T>(it->second);
				return true;
			}
			ROS_ERROR_STREAM("Could not convert param_name "
					<< param_name
					<< " param_str="
					<< param_str
					<< " didn't match list of valid types");
			return false;
		}
};
class CANifierControllerInterface
{
	public:
		CANifierControllerInterface(ros::NodeHandle &n, const std::string &joint_name, hardware_interface::canifier::CANifierCommandHandle handle);

		void update(void);
		void setQuadraturePosition(double new_quadrature_position);
		void setClearStickyFaults(void);

		void setLEDOutput(hardware_interface::canifier::LEDChannel index, double value);
		void setGeneralPinOutputEnable(hardware_interface::canifier::GeneralPin index, bool value);
		void setGeneralPinOutput(hardware_interface::canifier::GeneralPin index, bool value);
		void setVelocityMeasurementPeriod(int period);
		void setVelocityMeasurementWindow(int window);
		void setClearPositionOnLimitF(bool value);
		void setClearPositionOnLimitR(bool value);
		void setClearPositionOnQuadIdx(bool value);
		void setPWMOutputEnable(hardware_interface::canifier::PWMChannel index, bool value);
		void setPWMOutput(hardware_interface::canifier::PWMChannel index, double value);
		void setStatusFramePeriod(hardware_interface::canifier::CANifierStatusFrame frame_id, int period);
		void setControlFramePeriod(hardware_interface::canifier::CANifierControlFrame frame_id, int period);

	private:
		CANifierCIParams                                    params_;
		hardware_interface::canifier::CANifierCommandHandle handle_;
		std::mutex                                          set_quadrature_position_mutex_;
		bool                                                set_quadrature_position_;
		double                                              new_quadrature_position_;
		std::atomic<bool>                                   clear_sticky_faults_;
};

} // namespace canifier_controller_interfac
