#include <thread>

#include "ros_control_boilerplate/AS726x.h"

#include "as726x_interface/as726x_interface.h"
#include "ros_control_boilerplate/as726x_device.h"
#include "ros_control_boilerplate/tracer.h"

static bool convertIndLedCurrentLimit(const hardware_interface::as726x::IndLedCurrentLimits input,
                               as726x::ind_led_current_limits &output);
static bool convertDrvLedCurrentLimit(const hardware_interface::as726x::DrvLedCurrentLimits input,
                               as726x::drv_led_current_limits &output);
static bool convertConversionType(const hardware_interface::as726x::ConversionTypes input,
                           as726x::conversion_types &output);
static bool convertChannelGain(const hardware_interface::as726x::ChannelGain input,
                        as726x::channel_gain &output);
template <bool SIMFLAG>
AS726xDevice<SIMFLAG>::AS726xDevice(const std::string &name_space,
                                    const int joint_index,
                                    const std::string &joint_name,
                                    const std::string &port,
                                    const int address,
                                    const bool local_hardware,
                                    const bool local_update,
                                    const double read_hz_)
    : name_{joint_name}
    , port_{port}
    , address_{address}
    , local_hardware_{local_hardware}
    , local_update_{local_update}
    , as726x_{nullptr}
    , state_{std::make_unique<hardware_interface::as726x::AS726xState>(port_, address_)}
    , command_{std::make_unique<hardware_interface::as726x::AS726xCommand>()}
    , read_thread_state_{local_hardware ? std::make_unique<hardware_interface::as726x::AS726xState>(port_, address_) : nullptr}
    , read_state_mutex_{local_hardware ? std::make_unique<std::mutex>() : nullptr}
    , read_thread_{nullptr}
{
    // TODO : old code had a check for duplicate use of DIO channels? Needed?
    ROS_INFO_STREAM_NAMED("frc_robot_interface",
                        "Loading joint " << joint_index << "=" << name_ <<
                        (local_update_ ? " local" : " remote") << " update, " <<
                        (local_hardware_ ? "local" : "remote") << " hardware" <<
                        " as AS726x with port=" << port_ <<
                        " address=" << address_);
    if (local_hardware_)
    {
        frc::I2C::Port i2c_port;
        if (port_ == "onboard")
        {
            i2c_port = frc::I2C::Port::kOnboard;
        }
        else if (port_ == "mxp")
        {
            i2c_port = frc::I2C::Port::kMXP;
        }
        else
        {
            // Allow arbitrary integer ports to open /dev/i2c-<number> devices
            // on the Jetson or other linux platforms
            try
            {
                i2c_port = static_cast<frc::I2C::Port>(std::stoi(port_));
            }
            catch (...)
            {
                ROS_ERROR_STREAM("Invalid port specified for as726x - " << port_ << "valid options are onboard, mxp, or a number");
                return;
            }
        }

        as726x_ = std::make_unique<as726x::roboRIO_AS726x<SIMFLAG>>(i2c_port, address_);
        if (as726x_)
        {
            read_thread_ = std::make_unique<std::thread>(&AS726xDevice::read_thread, this,
                                                         std::make_unique<Tracer>("AS726x:" + name_ + " " + name_space),
                                                         read_hz_);
        }
        else
        {
            ROS_ERROR_STREAM("Error intializing as726x");
            read_thread_state_.reset();
            read_state_mutex_.reset();
        }
    }
}

template <bool SIMFLAG>
AS726xDevice<SIMFLAG>::~AS726xDevice(void)
{
    if (read_thread_ && read_thread_->joinable())
    {
        read_thread_->join();
    }
}

template <bool SIMFLAG>
void AS726xDevice<SIMFLAG>::registerInterfaces(hardware_interface::as726x::AS726xStateInterface &state_interface,
                                  hardware_interface::as726x::AS726xCommandInterface &command_interface,
                                  hardware_interface::as726x::RemoteAS726xStateInterface &remote_state_interface)
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for AS726x : " << name_ << " at port " << port_ << " address " << address_);

    hardware_interface::as726x::AS726xStateHandle state_handle(name_, state_.get());
    state_interface.registerHandle(state_handle);

    hardware_interface::as726x::AS726xCommandHandle command_handle(state_handle, command_.get());
    command_interface.registerHandle(command_handle);

    if (!local_update_)
    {
        hardware_interface::as726x::AS726xWritableStateHandle remote_handle(name_, state_.get());
        remote_state_interface.registerHandle(remote_handle);
    }
}

template <bool SIMFLAG>
void AS726xDevice<SIMFLAG>::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (local_update_)
    {
        std::lock_guard<std::mutex> l(*read_state_mutex_);
        state_->setTemperature(read_thread_state_->getTemperature());
        state_->setRawChannelData(read_thread_state_->getRawChannelData());
        state_->setCalibratedChannelData(read_thread_state_->getCalibratedChannelData());
    }
}

template <bool SIMFLAG>
void AS726xDevice<SIMFLAG>::write(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    hardware_interface::as726x::IndLedCurrentLimits ind_led_current_limit;
    bool ind_led_enable;
    hardware_interface::as726x::DrvLedCurrentLimits drv_led_current_limit;
    bool drv_led_enable;
    hardware_interface::as726x::ConversionTypes conversion_type;
    hardware_interface::as726x::ChannelGain gain;
    uint8_t integration_time;

    const bool ind_led_current_limit_changed = command_->indLedCurrentLimitChanged(ind_led_current_limit);
    const bool ind_led_enable_changed = command_->indLedEnableChanged(ind_led_enable);
    const bool drv_led_current_limit_changed = command_->drvLedCurrentLimitChanged(drv_led_current_limit);
    const bool drv_led_enable_changed = command_->drvLedEnableChanged(drv_led_enable);
    const bool conversion_type_changed = command_->conversionTypeChanged(conversion_type);
    const bool gain_changed = command_->gainChanged(gain);
    const bool integration_time_changed = command_->integrationTimeChanged(integration_time);
    // Only attempt to lock access to the sensor if there's actually
    // any changed config values to write to it
    if (ind_led_current_limit_changed ||
        ind_led_enable_changed ||
        drv_led_current_limit_changed ||
        drv_led_enable_changed ||
        conversion_type_changed ||
        gain_changed ||
        integration_time_changed)
    {
        // Try to get a lock for this instance of AS726x.  If not available
        // due to the read thread accessing the sensor,
        // continue on to the next sensor in the loop one.  This way the code won't be waiting
        // for a mutex to be released - it'll move on and try again next
        // iteration of write()
        std::unique_lock<std::mutex> l(*(as726x_->getMutex()), std::defer_lock);
        if (!l.try_lock())
        {
            // If we can't get a lock this iteration, reset and try again next time through
            if (ind_led_current_limit_changed)
            {
                command_->resetIndLedCurrentLimit();
            }
            if (ind_led_enable_changed)
            {
                command_->resetIndLedEnable();
            }
            if (drv_led_current_limit_changed)
            {
                command_->resetDrvLedCurrentLimit();
            }
            if (drv_led_enable_changed)
            {
                command_->resetDrvLedEnable();
            }
            if (conversion_type_changed)
            {
                command_->resetConversionType();
            }
            if (gain_changed)
            {
                command_->resetGain();
            }
            if (integration_time_changed)
            {
                command_->resetIntegrationTime();
            }
            return;
        }

        if (ind_led_current_limit_changed)
        {
            if (local_hardware_)
            {
                as726x::ind_led_current_limits as726x_ind_led_current_limit;
                if (convertIndLedCurrentLimit(ind_led_current_limit, as726x_ind_led_current_limit))
                {
                    as726x_->setIndicateCurrent(as726x_ind_led_current_limit);
                }
            }
            state_->setIndLedCurrentLimit(ind_led_current_limit);
            ROS_INFO_STREAM("Wrote as726x " << name_ << " ind_led_current_limit = " << ind_led_current_limit);
        }

        if (ind_led_enable_changed)
        {
            if (local_hardware_)
            {
                as726x_->indicateLED(ind_led_enable);
            }
            state_->setIndLedEnable(ind_led_enable);
            ROS_INFO_STREAM("Wrote as726x " << name_ << " ind_led_enable = " << ind_led_enable);
        }

        if (drv_led_current_limit_changed)
        {
            if (local_hardware_)
            {
                as726x::drv_led_current_limits as726x_drv_led_current_limit;
                if (convertDrvLedCurrentLimit(drv_led_current_limit, as726x_drv_led_current_limit))
                {
                    as726x_->setDrvCurrent(as726x_drv_led_current_limit);
                }
            }
            state_->setDrvLedCurrentLimit(drv_led_current_limit);
            ROS_INFO_STREAM("Wrote as726x  " << name_ << " drv_led_current_limit = " << drv_led_current_limit);
        }

        if (drv_led_enable_changed)
        {
            if (local_hardware_)
            {
                if (drv_led_enable)
                {
                    as726x_->drvOn();
                }
                else
                {
                    as726x_->drvOff();
                }
            }
            state_->setDrvLedEnable(drv_led_enable);
            ROS_INFO_STREAM("Wrote as726x  " << name_ << " drv_led_enable = " << drv_led_enable);
        }

        if (conversion_type_changed)
        {
            if (local_hardware_)
            {
                as726x::conversion_types as726x_conversion_type;
                if (convertConversionType(conversion_type, as726x_conversion_type))
                {
                    as726x_->setConversionType(as726x_conversion_type);
                }
            }
            state_->setConversionType(conversion_type);
            ROS_INFO_STREAM("Wrote as726x  " << name_ << " conversion_type = " << conversion_type);
        }

        if (gain_changed)
        {
            if (local_hardware_)
            {
                as726x::channel_gain as726x_gain;
                if (convertChannelGain(gain, as726x_gain))
                {
                    as726x_->setGain(as726x_gain);
                }
            }
            state_->setGain(gain);
            ROS_INFO_STREAM("Wrote as726x  " << name_ << " channel_gain = " << gain);
        }

        if (integration_time_changed)
        {
            if (local_hardware_)
            {
                as726x_->setIntegrationTime(integration_time);
            }
            state_->setIntegrationTime(integration_time);
            ROS_INFO_STREAM("Wrote as726x  " << name_ << " integration_time = "
                                             << static_cast<int>(integration_time));
        }
    }
    return;
}

template <bool SIMFLAG>
void AS726xDevice<SIMFLAG>::read_thread(std::unique_ptr<Tracer> tracer,
                                        double poll_frequency)
{
#ifdef __linux__
	if (pthread_setname_np(pthread_self(), "as726x_read"))
	{
		ROS_ERROR_STREAM("Error setting thrad name for as726x_read " << errno);
	}
#endif
	ros::Duration(4.75).sleep(); // Sleep for a few seconds to let I2C start up

	uint16_t temperature;
	std::array<uint16_t, 6> raw_channel_data;
	std::array<float, 6> calibrated_channel_data;
    for (ros::Rate r(poll_frequency); ros::ok(); r.sleep())
	{
		tracer->start("main loop");
		// Create a scope for the lock protecting hardware accesses
		{
			std::lock_guard<std::mutex> l(*(as726x_->getMutex()));
			temperature = as726x_->readTemperature();
			as726x_->startMeasurement();
			auto data_ready_start = ros::Time::now();
			bool timeout = false;
			while (!as726x_->dataReady() && !timeout)
			{
				ros::Duration(0.01).sleep();
				if ((ros::Time::now() - data_ready_start).toSec() > 1.5)
				{
					timeout = true;
				}
			}
			if (timeout)
			{
				ROS_WARN("Timeout waiting for as726 data ready");
				tracer->stop();
				continue;
			}

			as726x_->readRawValues(&raw_channel_data[0], 6);
			as726x_->readCalibratedValues(&calibrated_channel_data[0], 6);
		}
		// Create a new scope for the lock protecting the shared state
		{
			std::lock_guard<std::mutex> l(*read_state_mutex_);
			read_thread_state_->setTemperature(temperature);
			read_thread_state_->setRawChannelData(raw_channel_data);
			read_thread_state_->setCalibratedChannelData(calibrated_channel_data);
		}
		tracer->report(60);
		ros::Duration(0.1).sleep(); // allow time for write() to run
	}
}

static bool convertIndLedCurrentLimit(const hardware_interface::as726x::IndLedCurrentLimits input,
                                      as726x::ind_led_current_limits &output)
{
	switch (input)
	{
		case hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_1MA:
			output = as726x::ind_led_current_limits::LIMIT_1MA;
			break;
		case hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_2MA:
			output = as726x::ind_led_current_limits::LIMIT_2MA;
			break;
		case hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_4MA:
			output = as726x::ind_led_current_limits::LIMIT_4MA;
			break;
		case hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_8MA:
			output = as726x::ind_led_current_limits::LIMIT_8MA;
			break;
		default:
			ROS_ERROR("Invalid input in convertAS72xIndLedCurrentLimit");
			return false;
	}
	return true;
}

static bool convertDrvLedCurrentLimit(const hardware_interface::as726x::DrvLedCurrentLimits input,
                                      as726x::drv_led_current_limits &output)
{
	switch (input)
	{
		case hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_12MA5:
			output = as726x::drv_led_current_limits::LIMIT_12MA5;
			break;
		case hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_25MA:
			output = as726x::drv_led_current_limits::LIMIT_25MA;
			break;
		case hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_50MA:
			output = as726x::drv_led_current_limits::LIMIT_50MA;
			break;
		case hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_100MA:
			output = as726x::drv_led_current_limits::LIMIT_100MA;
			break;
		default:
			ROS_ERROR("Invalid input in convertAS72xDrvLedCurrentLimit");
			return false;
	}
	return true;
}

static bool convertConversionType(const hardware_interface::as726x::ConversionTypes input,
                                  as726x::conversion_types &output)
{
    switch (input)
	{
		case hardware_interface::as726x::ConversionTypes::MODE_0:
			output = as726x::conversion_types::MODE_0;
			break;
		case hardware_interface::as726x::ConversionTypes::MODE_1:
			output = as726x::conversion_types::MODE_1;
			break;
		case hardware_interface::as726x::ConversionTypes::MODE_2:
			output = as726x::conversion_types::MODE_2;
			break;
		case hardware_interface::as726x::ConversionTypes::ONE_SHOT:
			output = as726x::conversion_types::ONE_SHOT;
			break;
		default:
			ROS_ERROR("Invalid input in convertAS726xConversionType");
			return false;
	}
	return true;
}

static bool convertChannelGain(const hardware_interface::as726x::ChannelGain input,
                               as726x::channel_gain &output)
{
    switch (input)
	{
		case hardware_interface::as726x::ChannelGain::GAIN_1X:
			output = as726x::channel_gain::GAIN_1X;
			break;
		case hardware_interface::as726x::ChannelGain::GAIN_3X7:
			output = as726x::channel_gain::GAIN_3X7;
			break;
		case hardware_interface::as726x::ChannelGain::GAIN_16X:
			output = as726x::channel_gain::GAIN_16X;
			break;
		case hardware_interface::as726x::ChannelGain::GAIN_64X:
			output = as726x::channel_gain::GAIN_64X;
			break;
		default:
			ROS_ERROR("Invalid input in convertAS726xChannelGain");
			return false;
	}
	return true;
}
