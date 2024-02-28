#ifndef CANCODER_CONTROLLER_INTERFACE_INC__
#define CANCODER_CONTROLLER_INTERFACE_INC__

#include <atomic>
#include <mutex>
#include "ddr_updater/ddr_updater.h"
#include "ctre_interfaces/cancoder_command_interface.h"

namespace cancoder_controller_interface
{
class CANCoderCIParams : public ddr_updater::DDRUpdater
{
	public:
		explicit CANCoderCIParams(const ros::NodeHandle &n);

		void setSensorDirection(const int sensor_direction, bool update_dynamic = true);
		void setMagnetOffset(const double magnet_offset, bool update_dynamic = true);
		void setAbsoluteSensorRange(const int absolute_sensor_range, bool update_dynamic = true);
		void setConversionFactor(const double conversion_factor, bool update_dynamic = true);
		void setEnableReadThread(const bool enable_read_thread, bool update_dynamic = true);

		hardware_interface::cancoder::SensorDirection     getSensorDirection(void) const;
		double                                            getMagnetOffset(void) const;
		hardware_interface::cancoder::AbsoluteSensorRange getAbsoluteSensorRange(void) const;
		double                                            getConversionFactor(void) const;
		bool                                              getEnableReadThread(void) const;

	private:
		std::atomic<hardware_interface::cancoder::SensorDirection>     sensor_direction_{hardware_interface::cancoder::SensorDirection::CounterClockwise_Positive};
		std::atomic<double>                                            magnet_offset_{0.0};
		std::atomic<hardware_interface::cancoder::AbsoluteSensorRange> absolute_sensor_range_{hardware_interface::cancoder::AbsoluteSensorRange::Signed_PlusMinusHalf};
		std::atomic<double>                                            conversion_factor_{1.0};
		std::atomic<bool>                                              enable_read_thread_{true};

		const std::map<std::string, int> sensor_direction_enum_map_ =
		{
			{"CounterClockwise_Positive", static_cast<int>(hardware_interface::cancoder::SensorDirection::CounterClockwise_Positive)},
			{"Clockwise_Positive", static_cast<int>(hardware_interface::cancoder::SensorDirection::Clockwise_Positive)}
		};
		const std::map<std::string, int> absolute_sensor_range_enum_map_ =
		{
			{"Unsigned_0To1", static_cast<int>(hardware_interface::cancoder::AbsoluteSensorRange::Unsigned_0To1)},
			{"Signed_PlusMinusHalf", static_cast<int>(hardware_interface::cancoder::AbsoluteSensorRange::Signed_PlusMinusHalf)}
		};

		template <typename T>
		void readIntoScalar(const ros::NodeHandle &n, const std::string &name, std::atomic<T> &scalar)
		{
			T val;
			if (n.getParam(name, val))
				scalar = val;
		}

		template <typename T>
		bool readIntoEnum(const ros::NodeHandle &n,
						  const std::string &param_name,
						  const std::map<std::string, int> &mymap,
						  std::atomic<T> &out) const
		{
			std::string param_str;
			if (!n.getParam(param_name, param_str))
				return true;

			if (const auto it = mymap.find(param_str); it != mymap.cend())
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
class CANCoderControllerInterface
{
	public:
		CANCoderControllerInterface(const ros::NodeHandle &n, hardware_interface::cancoder::CANCoderCommandHandle handle);

		void update(void);
		void setPosition(const double new_position);
		void setClearStickyFaults(void);

		void setSensorDirection(const hardware_interface::cancoder::SensorDirection sensor_direction);
		void setMagnetOffset(const double magnet_offset);
		void setAbsoluteSensorRange(const hardware_interface::cancoder::AbsoluteSensorRange absolute_sensor_range);
		void setConversionFactor(const double conversion_factor);

	private:
		CANCoderCIParams                                    params_;
		hardware_interface::cancoder::CANCoderCommandHandle handle_;
		std::mutex                                          set_position_mutex_;
		bool                                                set_position_flag_{false};
		double                                              set_position_value_{};
		std::atomic<bool>                                   clear_sticky_faults_{false};
};

} // namespace cancoder_controller_interfac

#endif