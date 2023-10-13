#include <thread>

#include "ros/ros.h"

#include "ctre/phoenix/CANifier.h"

#include "ctre_interfaces/canifier_command_interface.h"
#include "ros_control_boilerplate/canifier_device.h"
#include "ros_control_boilerplate/get_conversion_factor.h"
#include "ros_control_boilerplate/tracer.h"

#define safeTalonCall(error_code, call_string) \
    SIMFLAG ? true : safeCall(error_code, call_string)

bool convertGeneralPin(const hardware_interface::canifier::GeneralPin input,
                       ctre::phoenix::CANifier::GeneralPin &output);
bool convertPWMChannel(hardware_interface::canifier::PWMChannel input,
                       ctre::phoenix::CANifier::PWMChannel &output);
bool convertLEDChannel(hardware_interface::canifier::LEDChannel input,
                       ctre::phoenix::CANifier::LEDChannel &output);
bool convertVelocityMeasurementPeriod(hardware_interface::canifier::CANifierVelocityMeasPeriod input,
                                      ctre::phoenix::CANifierVelocityMeasPeriod &output);
bool convertStatusFrame(hardware_interface::canifier::CANifierStatusFrame input,
                        ctre::phoenix::CANifierStatusFrame &output);
bool convertControlFrame(hardware_interface::canifier::CANifierControlFrame input,
                         ctre::phoenix::CANifierControlFrame &output);

template <bool SIMFLAG>
CANifierDevice<SIMFLAG>::CANifierDevice(const std::string &name_space,
                                        const int joint_index,
                                        const std::string &joint_name,
                                        const int can_id,
                                        const bool local_hardware,
                                        const bool local_update,
                                        const double read_hz_)
    : CTREV5Device{name_space, "CANifier", joint_name, can_id}
    , local_hardware_{local_hardware}
    , local_update_{local_update}
    , canifier_{nullptr}
    , state_{std::make_unique<hardware_interface::canifier::CANifierHWState>(can_id)}
    , command_{std::make_unique<hardware_interface::canifier::CANifierHWCommand>()}
    , read_thread_state_{nullptr}
    , read_state_mutex_{nullptr}
    , read_thread_{nullptr}
{
    // TODO : old code had a check for duplicate use of DIO channels? Needed?
    ROS_INFO_STREAM_NAMED("frc_robot_interface",
                        "Loading joint " << joint_index << "=" << joint_name <<
                        (local_update_ ? " local" : " remote") << " update, " <<
                        (local_hardware_ ? "local" : "remote") << " hardware" <<
                        " as CANifier " << can_id);

    if constexpr (!SIMFLAG)
    {
        if (local_hardware_)
        {
            canifier_ = std::make_unique<ctre::phoenix::CANifier>(can_id);
            read_thread_state_ = std::make_unique<hardware_interface::canifier::CANifierHWState>(can_id);
            read_state_mutex_ = std::make_unique<std::mutex>();
            read_thread_ = std::make_unique<std::thread>(&CANifierDevice<SIMFLAG>::read_thread, this,
                                                         std::make_unique<Tracer>("canifier_read_" + joint_name + " " + name_space),
                                                         read_hz_);
        }
    }
}

template <bool SIMFLAG>
CANifierDevice<SIMFLAG>::~CANifierDevice(void)
{
    if (read_thread_ && read_thread_->joinable())
    {
        read_thread_->join();
    }
}

template <bool SIMFLAG>
void CANifierDevice<SIMFLAG>::registerInterfaces(hardware_interface::canifier::CANifierStateInterface &state_interface,
                                                 hardware_interface::canifier::CANifierCommandInterface &command_interface,
                                                 hardware_interface::canifier::RemoteCANifierStateInterface &remote_state_interface)
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for CANifier : " << getName() << " at CAN id " << getId());

    hardware_interface::canifier::CANifierStateHandle state_handle(getName(), state_.get());
    state_interface.registerHandle(state_handle);

    hardware_interface::canifier::CANifierCommandHandle command_handle(state_handle, command_.get());
    command_interface.registerHandle(command_handle);

    if (!local_update_)
    {
        hardware_interface::canifier::CANifierWritableStateHandle remote_handle(getName(), state_.get());
        remote_state_interface.registerHandle(remote_handle);
    }
}

template <bool SIMFLAG>
void CANifierDevice<SIMFLAG>::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (local_update_)
    {
        std::lock_guard<std::mutex> l(*read_state_mutex_);

        // These are used to convert position and velocity units - make sure the
        // read thread's local copy of state is kept up to date
        read_thread_state_->setEncoderTicksPerRotation(state_->getEncoderTicksPerRotation());
        read_thread_state_->setConversionFactor(state_->getConversionFactor());

        for (size_t i = hardware_interface::canifier::GeneralPin::GeneralPin_FIRST + 1; i < hardware_interface::canifier::GeneralPin::GeneralPin_LAST; i++)
        {
            const auto pin = static_cast<hardware_interface::canifier::GeneralPin>(i);
            state_->setGeneralPinInput(pin, read_thread_state_->getGeneralPinInput(pin));
        }
        state_->setQuadraturePosition(read_thread_state_->getQuadraturePosition());
        state_->setQuadratureVelocity(read_thread_state_->getQuadratureVelocity());
        state_->setBusVoltage(read_thread_state_->getBusVoltage());

        for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
        {
            const auto pwm_channel = static_cast<hardware_interface::canifier::PWMChannel>(i);
            state_->setPWMInput(pwm_channel, read_thread_state_->getPWMInput(pwm_channel));
        }
        state_->setFaults(read_thread_state_->getFaults());
        state_->setStickyFaults(read_thread_state_->getStickyFaults());
    }
}

template <bool SIMFLAG>
void CANifierDevice<SIMFLAG>::write(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (!local_hardware_)
    {
        return;
    }

    if (canifier_->HasResetOccurred())
    {
        for (size_t i = hardware_interface::canifier::LEDChannel::LEDChannelFirst + 1; i < hardware_interface::canifier::LEDChannel::LEDChannelLast; i++)
        {
            command_->resetLEDOutput(static_cast<hardware_interface::canifier::LEDChannel>(i));
        }
        for (size_t i = hardware_interface::canifier::GeneralPin::GeneralPin_FIRST + 1; i < hardware_interface::canifier::GeneralPin::GeneralPin_LAST; i++)
        {
            command_->resetGeneralPinOutput(static_cast<hardware_interface::canifier::GeneralPin>(i));
        }
        command_->resetQuadraturePosition();
        command_->resetVelocityMeasurementPeriod();
        command_->resetVelocityMeasurementWindow();
        command_->resetClearPositionOnLimitF();
        command_->resetClearPositionOnLimitR();
        command_->resetClearPositionOnQuadIdx();
        for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
        {
            const auto pwm_channel = static_cast<hardware_interface::canifier::PWMChannel>(i);
            command_->resetPWMOutput(pwm_channel);
            command_->resetPWMOutputEnable(pwm_channel);
        }
        for (size_t i = hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_First + 1; i < hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Last; i++)
        {
            command_->resetStatusFramePeriod(static_cast<hardware_interface::canifier::CANifierStatusFrame>(i));
        }
        for (size_t i = hardware_interface::canifier::CANifierControlFrame::CANifier_Control_First + 1; i < hardware_interface::canifier::CANifierControlFrame::CANifier_Control_Last; i++)
        {
            command_->resetControlFramePeriod(static_cast<hardware_interface::canifier::CANifierControlFrame>(i));
        }
    }

    for (size_t i = hardware_interface::canifier::LEDChannel::LEDChannelFirst + 1; i < hardware_interface::canifier::LEDChannel::LEDChannelLast; i++)
    {
        const auto led_channel = static_cast<hardware_interface::canifier::LEDChannel>(i);
        double percent_output;
        ctre::phoenix::CANifier::LEDChannel ctre_led_channel;
        if (command_->ledOutputChanged(led_channel, percent_output) &&
            convertLEDChannel(led_channel, ctre_led_channel))
        {
            if (safeTalonCall(canifier_->SetLEDOutput(percent_output, ctre_led_channel), "canifier->SetLEDOutput"))
            {
                state_->setLEDOutput(led_channel, percent_output);
                ROS_INFO_STREAM("CANifier " << getName() << " id = " << getId()
                                            << " : Set LED channel " << i << " to " << percent_output);
            }
            else
            {
                command_->resetLEDOutput(led_channel);
            }
        }
    }
    for (size_t i = hardware_interface::canifier::GeneralPin::GeneralPin_FIRST + 1; i < hardware_interface::canifier::GeneralPin::GeneralPin_LAST; i++)
    {
        const auto general_pin = static_cast<hardware_interface::canifier::GeneralPin>(i);
        bool value;
        bool output_enable;
        ctre::phoenix::CANifier::GeneralPin ctre_general_pin;
        if (command_->generalPinOutputChanged(general_pin, value, output_enable) &&
            convertGeneralPin(general_pin, ctre_general_pin))
        {
            if (safeTalonCall(canifier_->SetGeneralOutput(ctre_general_pin, value, output_enable), "canifier->SetGeneralOutput"))
            {
                state_->setGeneralPinOutput(general_pin, value);
                state_->setGeneralPinOutputEnable(general_pin, output_enable);
                ROS_INFO_STREAM("CANifier " << getName() << " id = " << getId()
                                            << " : Set General Pin " << i << " to enable=" << output_enable << " value=" << value);
            }
            else
            {
                command_->resetGeneralPinOutput(general_pin);
            }
        }
    }

    // Don't bother with all the changed/reset code here, just copy
    // from command to state each time through the write call
    state_->setEncoderTicksPerRotation(command_->getEncoderTicksPerRotation());
    state_->setConversionFactor(command_->getConversionFactor());
    double position;
    if (command_->quadraturePositionChanged(position))
    {
        const double radian_scale = getConversionFactor(state_->getEncoderTicksPerRotation(), hardware_interface::FeedbackDevice_QuadEncoder, hardware_interface::TalonMode_Position) * state_->getConversionFactor();
        if (safeTalonCall(canifier_->SetQuadraturePosition(position / radian_scale), "canifier->SetQuadraturePosition"))
        {
            // Don't set state encoder position, let it be read at the next read() call
            ROS_INFO_STREAM("CANifier " << getName() << " id = " << getId()
                            << " : Set Quadrature Position to " << position);
        }
        else
        {
            command_->resetQuadraturePosition();
        }
    }

    hardware_interface::canifier::CANifierVelocityMeasPeriod hwi_period;
    ctre::phoenix::CANifierVelocityMeasPeriod ctre_period;
    if (command_->velocityMeasurementPeriodChanged(hwi_period) &&
        convertVelocityMeasurementPeriod(hwi_period, ctre_period))
    {
        if (safeTalonCall(canifier_->ConfigVelocityMeasurementPeriod(ctre_period), "canifier->ConfigVelocityMeasurementPeriod"))
        {
            ROS_INFO_STREAM("CANifier " << getName() << " id = " << getId()
                            << " : Set velocity measurement Period to " << hwi_period);
            state_->setVelocityMeasurementPeriod(hwi_period);
        }
        else
        {
            command_->resetVelocityMeasurementPeriod();
        }
    }

    int window;
    if (command_->velocityMeasurementWindowChanged(window))
    {
        if (safeTalonCall(canifier_->ConfigVelocityMeasurementWindow(window), "canifier->ConfigVelocityMeasurementWindow"))
        {
            ROS_INFO_STREAM("CANifier " << getName() << " id = " << getId()
                            << " : Set velocity measurement window to " << window);
            state_->setVelocityMeasurementWindow(window);
        }
        else
        {
            command_->resetVelocityMeasurementWindow();
        }
    }

    bool clear_position_on_limit_f;
    if (command_->clearPositionOnLimitFChanged(clear_position_on_limit_f))
    {
        if (safeTalonCall(canifier_->ConfigClearPositionOnLimitF(clear_position_on_limit_f), "canifier->ConfigClearPositionOnLimitF"))
        {
            ROS_INFO_STREAM("CANifier " << getName() << " id = " << getId()
                            << " : Set clear position on limit F to " << clear_position_on_limit_f);
            state_->setClearPositionOnLimitF(clear_position_on_limit_f);
        }
        else
        {
            command_->resetClearPositionOnLimitF();
        }
    }

    bool clear_position_on_limit_r;
    if (command_->clearPositionOnLimitRChanged(clear_position_on_limit_r))
    {
        if (safeTalonCall(canifier_->ConfigClearPositionOnLimitR(clear_position_on_limit_r), "canifier->ConfigClearPositionOnLimitR"))
        {
            ROS_INFO_STREAM("CANifier " << getName() << " id = " << getId()
                            << " : Set clear position on limit R to " << clear_position_on_limit_r);
            state_->setClearPositionOnLimitR(clear_position_on_limit_r);
        }
        else
        {
            command_->resetClearPositionOnLimitR();
        }
    }

    bool clear_position_on_quad_idx;
    if (command_->clearPositionOnQuadIdxChanged(clear_position_on_quad_idx))
    {
        if (safeTalonCall(canifier_->ConfigClearPositionOnQuadIdx(clear_position_on_quad_idx), "canifier->ConfigClearPositionOnQuadIdx"))
        {
            ROS_INFO_STREAM("CANifier " << getName() << " id = " << getId()
                            << " : Set clear position on quad idx to " << clear_position_on_quad_idx);
            state_->setClearPositionOnQuadIdx(clear_position_on_quad_idx);
        }
        else
        {
            command_->resetClearPositionOnQuadIdx();
        }
    }

    for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
    {
        const auto pwm_channel = static_cast<hardware_interface::canifier::PWMChannel>(i);
        ctre::phoenix::CANifier::PWMChannel ctre_pwm_channel;
        bool output_enable;
        if (command_->pwmOutputEnableChanged(pwm_channel, output_enable) &&
            convertPWMChannel(pwm_channel, ctre_pwm_channel))
        {
            if (safeTalonCall(canifier_->EnablePWMOutput(ctre_pwm_channel, output_enable), "canifier->EnablePWMOutput"))
            {
                ROS_INFO_STREAM("CANifier " << getName() << " id = " << getId()
                                << " : Set pwm channel " << pwm_channel << " output enable to " << static_cast<int>(output_enable));
                state_->setPWMOutputEnable(pwm_channel, output_enable);
            }
            else
            {
                command_->resetPWMOutputEnable(pwm_channel);
            }
        }
    }

    for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
    {
        const auto pwm_channel = static_cast<hardware_interface::canifier::PWMChannel>(i);
        ctre::phoenix::CANifier::PWMChannel ctre_pwm_channel;
        double output;
        if (command_->pwmOutputChanged(pwm_channel, output) &&
            convertPWMChannel(pwm_channel, ctre_pwm_channel))
        {
            if (safeTalonCall(canifier_->SetPWMOutput(ctre_pwm_channel, output), "canifier->SetPWMOutput"))
            {
                ROS_INFO_STREAM("CANifier " << getName() << " id = " << getId()
                                << " : Set pwm channel " << pwm_channel << " output to " << output);
                state_->setPWMOutput(pwm_channel, output);
            }
            else
            {
                command_->resetPWMOutput(pwm_channel);
            }
        }
    }

    for (size_t i = hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_First + 1; i < hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Last; i++)
    {
        const auto frame_id = static_cast<hardware_interface::canifier::CANifierStatusFrame>(i);
        ctre::phoenix::CANifierStatusFrame ctre_frame_id;
        int hwi_frame_period;
        if (command_->statusFramePeriodChanged(frame_id, hwi_frame_period) &&
            convertStatusFrame(frame_id, ctre_frame_id))
        {
            if (safeTalonCall(canifier_->SetStatusFramePeriod(ctre_frame_id, hwi_frame_period), "canifier->SetStatusFramePeriod"))
            {
                ROS_INFO_STREAM("CANifier " << getName() << " id = " << getId()
                                << " : Set frame_id " << i << " status period to " << hwi_frame_period);
                state_->setStatusFramePeriod(frame_id, hwi_frame_period);
            }
            else
            {
                command_->resetStatusFramePeriod(frame_id);
            }
        }
    }

    for (size_t i = hardware_interface::canifier::CANifierControlFrame::CANifier_Control_First + 1; i < hardware_interface::canifier::CANifierControlFrame::CANifier_Control_Last; i++)
    {
        const auto frame_id = static_cast<hardware_interface::canifier::CANifierControlFrame>(i);
        ctre::phoenix::CANifierControlFrame ctre_frame_id;
        int hwi_frame_period;
        if (command_->controlFramePeriodChanged(frame_id, hwi_frame_period) &&
            convertControlFrame(frame_id, ctre_frame_id))
        {
            if (safeTalonCall(canifier_->SetControlFramePeriod(ctre_frame_id, hwi_frame_period), "canifier->SetControlFramePeriod"))
            {
                ROS_INFO_STREAM("CANifier " << getName() << " id = " << getId()
                                << " : Set frame_id " << i << " control period to " << hwi_frame_period);
                state_->setControlFramePeriod(frame_id, hwi_frame_period);
            }
            else
            {
                command_->resetControlFramePeriod(frame_id);
            }
        }
    }

    if (command_->clearStickyFaultsChanged())
    {
        if (safeTalonCall(canifier_->ClearStickyFaults(), "canifier->ClearStickyFaults()"))
        {
            ROS_INFO_STREAM("CANifier " << getName() << " id = " << getId() << " : cleared sticky faults");
            // No corresponding status field
        }
        else
        {
            command_->setClearStickyFaults();
        }
    }
}

template <bool SIMFLAG>
void CANifierDevice<SIMFLAG>::read_thread(std::unique_ptr<Tracer> tracer,
                                          double poll_frequency)
{
#ifdef __linux__
	std::stringstream thread_name{"canifier_rd_"};
	thread_name << read_thread_state_->getCANId();
	if (pthread_setname_np(pthread_self(), thread_name.str().c_str()))
	{
		ROS_ERROR_STREAM("Error setting thread name for canifier_read " << errno);
	}
#endif
	ros::Duration(3.5 + read_thread_state_->getCANId() * 0.05).sleep(); // Sleep for a few seconds to let CAN start up

	for(ros::Rate rate(poll_frequency); ros::ok(); rate.sleep())
	{
		tracer->start("canifier read main_loop");

		int encoder_ticks_per_rotation;
		double conversion_factor;

		// Update local status with relevant global config
		// values set by write(). This way, items configured
		// by controllers will be reflected in the read_thread_state_ here
		// used when reading from talons.
		// Realistically they won't change much (except maybe mode)
		// but unless it causes performance problems reading them
		// each time through the loop is easier than waiting until
		// they've been correctly set by write() before using them
		// here.
		// Note that this isn't a complete list - only the values
		// used by the read thread are copied over.  Update
		// as needed when more are read
		{
			std::lock_guard<std::mutex> l(*read_state_mutex_);
			encoder_ticks_per_rotation = read_thread_state_->getEncoderTicksPerRotation();
			conversion_factor = read_thread_state_->getConversionFactor();
		}

		std::array<bool, hardware_interface::canifier::GeneralPin::GeneralPin_LAST> general_pins;
		for (size_t i = hardware_interface::canifier::GeneralPin::GeneralPin_FIRST + 1; i < hardware_interface::canifier::GeneralPin::GeneralPin_LAST; i++)
		{
			ctre::phoenix::CANifier::GeneralPin ctre_general_pin;
			if (convertGeneralPin(static_cast<hardware_interface::canifier::GeneralPin>(i), ctre_general_pin))
            {
				general_pins[i] = canifier_->GetGeneralInput(ctre_general_pin);
            }
		}

		// Use FeedbackDevice_QuadEncoder to force getConversionFactor to use the encoder_ticks_per_rotation
		// variable to calculate these values
		const double radians_scale = getConversionFactor(encoder_ticks_per_rotation, hardware_interface::FeedbackDevice_QuadEncoder, hardware_interface::TalonMode_Position) * conversion_factor;
		const double radians_per_second_scale = getConversionFactor(encoder_ticks_per_rotation, hardware_interface::FeedbackDevice_QuadEncoder, hardware_interface::TalonMode_Velocity) * conversion_factor;
		const double quadrature_position = canifier_->GetQuadraturePosition() * radians_scale;
		const double quadrature_velocity = canifier_->GetQuadratureVelocity() * radians_per_second_scale;
		const double bus_voltage = canifier_->GetBusVoltage();

		std::array<std::array<double, 2>, hardware_interface::canifier::PWMChannel::PWMChannelLast> pwm_inputs;
		for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
		{
			ctre::phoenix::CANifier::PWMChannel ctre_pwm_channel;
			if (convertPWMChannel(static_cast<hardware_interface::canifier::PWMChannel>(i), ctre_pwm_channel))
            {
				canifier_->GetPWMInput(ctre_pwm_channel, &pwm_inputs[i][0]);
            }
		}

		ctre::phoenix::CANifierFaults ctre_faults;
		canifier_->GetFaults(ctre_faults);
		const unsigned faults = ctre_faults.ToBitfield();
		ctre::phoenix::CANifierStickyFaults ctre_sticky_faults;
		canifier_->GetStickyFaults(ctre_sticky_faults);
		const unsigned sticky_faults = ctre_sticky_faults.ToBitfield();

		// Actually update the CANifierHWread_thread_state_ shared between
		// this thread and read()
		// Do this all at once so the code minimizes the amount
		// of time with mutex locked
		{
			// Lock the read_thread_state_ entry to make sure writes
			// are atomic - reads won't grab data in
			// the middle of a write
			std::lock_guard<std::mutex> l(*read_state_mutex_);

			for (size_t i = hardware_interface::canifier::GeneralPin::GeneralPin_FIRST + 1; i < hardware_interface::canifier::GeneralPin::GeneralPin_LAST; i++)
			{
				read_thread_state_->setGeneralPinInput(static_cast<hardware_interface::canifier::GeneralPin>(i), general_pins[i]);
			}

			read_thread_state_->setQuadraturePosition(quadrature_position);
			read_thread_state_->setQuadratureVelocity(quadrature_velocity);
			read_thread_state_->setBusVoltage(bus_voltage);

			for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
			{
				read_thread_state_->setPWMInput(static_cast<hardware_interface::canifier::PWMChannel>(i), pwm_inputs[i]);
			}

			read_thread_state_->setFaults(faults);
			read_thread_state_->setStickyFaults(sticky_faults);
		}
		tracer->report(60);
	}
}

template class CANifierDevice<false>;
template class CANifierDevice<true>;

bool convertGeneralPin(const hardware_interface::canifier::GeneralPin input,
		ctre::phoenix::CANifier::GeneralPin &output)
{
	switch (input)
	{
		case hardware_interface::canifier::GeneralPin::QUAD_IDX:
			output = ctre::phoenix::CANifier::GeneralPin::QUAD_IDX;
			break;
		case hardware_interface::canifier::GeneralPin::QUAD_B:
			output = ctre::phoenix::CANifier::GeneralPin::QUAD_B;
			break;
		case hardware_interface::canifier::GeneralPin::QUAD_A:
			output = ctre::phoenix::CANifier::GeneralPin::QUAD_A;
			break;
		case hardware_interface::canifier::GeneralPin::LIMR:
			output = ctre::phoenix::CANifier::GeneralPin::LIMR;
			break;
		case hardware_interface::canifier::GeneralPin::LIMF:
			output = ctre::phoenix::CANifier::GeneralPin::LIMF;
			break;
		case hardware_interface::canifier::GeneralPin::SDA:
			output = ctre::phoenix::CANifier::GeneralPin::SDA;
			break;
		case hardware_interface::canifier::GeneralPin::SCL:
			output = ctre::phoenix::CANifier::GeneralPin::SCL;
			break;
		case hardware_interface::canifier::GeneralPin::SPI_CS:
			output = ctre::phoenix::CANifier::GeneralPin::SPI_CS;
			break;
		case hardware_interface::canifier::GeneralPin::SPI_MISO_PWM2P:
			output = ctre::phoenix::CANifier::GeneralPin::SPI_MISO_PWM2P;
			break;
		case hardware_interface::canifier::GeneralPin::SPI_MOSI_PWM1P:
			output = ctre::phoenix::CANifier::GeneralPin::SPI_MOSI_PWM1P;
			break;
		case hardware_interface::canifier::GeneralPin::SPI_CLK_PWM0P:
			output = ctre::phoenix::CANifier::GeneralPin::SPI_CLK_PWM0P;
			break;
		default:
			ROS_ERROR("Invalid input in convertCANifierGeneralPin");
			return false;
	}
	return true;
}
bool convertPWMChannel(hardware_interface::canifier::PWMChannel input,
		ctre::phoenix::CANifier::PWMChannel &output)
{
	switch (input)
	{
		case hardware_interface::canifier::PWMChannel::PWMChannel0:
			output = ctre::phoenix::CANifier::PWMChannel::PWMChannel0;
			break;
		case hardware_interface::canifier::PWMChannel::PWMChannel1:
			output = ctre::phoenix::CANifier::PWMChannel::PWMChannel1;
			break;
		case hardware_interface::canifier::PWMChannel::PWMChannel2:
			output = ctre::phoenix::CANifier::PWMChannel::PWMChannel2;
			break;
		case hardware_interface::canifier::PWMChannel::PWMChannel3:
			output = ctre::phoenix::CANifier::PWMChannel::PWMChannel3;
			break;
		default:
			ROS_ERROR("Invalid input in convertCANifierPWMChannel");
			return false;
	}
	return true;
}

bool convertLEDChannel(hardware_interface::canifier::LEDChannel input,
		ctre::phoenix::CANifier::LEDChannel &output)
{
	switch (input)
	{
		case hardware_interface::canifier::LEDChannel::LEDChannelA:
			output = ctre::phoenix::CANifier::LEDChannel::LEDChannelA;
			break;
		case hardware_interface::canifier::LEDChannel::LEDChannelB:
			output = ctre::phoenix::CANifier::LEDChannel::LEDChannelB;
			break;
		case hardware_interface::canifier::LEDChannel::LEDChannelC:
			output = ctre::phoenix::CANifier::LEDChannel::LEDChannelC;
			break;
		default:
			ROS_ERROR("Invalid input in convertCANifierLEDChannel");
			return false;
	}
	return true;
}

bool convertVelocityMeasurementPeriod(hardware_interface::canifier::CANifierVelocityMeasPeriod input,
		ctre::phoenix::CANifierVelocityMeasPeriod &output)
{
	switch (input)
	{
		case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_1Ms:
			output = ctre::phoenix::CANifierVelocityMeasPeriod::Period_1Ms;
			break;
		case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_2Ms:
			output = ctre::phoenix::CANifierVelocityMeasPeriod::Period_2Ms;
			break;
		case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_5Ms:
			output = ctre::phoenix::CANifierVelocityMeasPeriod::Period_5Ms;
			break;
		case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_10Ms:
			output = ctre::phoenix::CANifierVelocityMeasPeriod::Period_10Ms;
			break;
		case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_20Ms:
			output = ctre::phoenix::CANifierVelocityMeasPeriod::Period_20Ms;
			break;
		case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_25Ms:
			output = ctre::phoenix::CANifierVelocityMeasPeriod::Period_25Ms;
			break;
		case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_50Ms:
			output = ctre::phoenix::CANifierVelocityMeasPeriod::Period_50Ms;
			break;
		case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_100Ms:
			output = ctre::phoenix::CANifierVelocityMeasPeriod::Period_100Ms;
			break;
		default:
			ROS_ERROR("Invalid input in convertCANifierVelocityMeasurementPeriod");
			return false;
	}
	return true;
}

bool convertStatusFrame(hardware_interface::canifier::CANifierStatusFrame input,
		ctre::phoenix::CANifierStatusFrame &output)
{
	switch (input)
	{
		case hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_1_General:
			output = ctre::phoenix::CANifierStatusFrame::CANifierStatusFrame_Status_1_General;
			break;
		case hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_2_General:
			output = ctre::phoenix::CANifierStatusFrame::CANifierStatusFrame_Status_2_General;
			break;
		case hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_3_PwmInputs0:
			output = ctre::phoenix::CANifierStatusFrame::CANifierStatusFrame_Status_3_PwmInputs0;
			break;
		case hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_4_PwmInputs1:
			output = ctre::phoenix::CANifierStatusFrame::CANifierStatusFrame_Status_4_PwmInputs1;
			break;
		case hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_5_PwmInputs2:
			output = ctre::phoenix::CANifierStatusFrame::CANifierStatusFrame_Status_5_PwmInputs2;
			break;
		case hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_6_PwmInputs3:
			output = ctre::phoenix::CANifierStatusFrame::CANifierStatusFrame_Status_6_PwmInputs3;
			break;
		case hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_8_Misc:
			output = ctre::phoenix::CANifierStatusFrame::CANifierStatusFrame_Status_8_Misc;
			break;
		default:
			ROS_ERROR("Invalid input in convertCANifierStatusFrame");
			return false;
	}
	return true;
}

bool convertControlFrame(hardware_interface::canifier::CANifierControlFrame input,
		ctre::phoenix::CANifierControlFrame &output)
{
	switch (input)
	{
		case hardware_interface::canifier::CANifierControlFrame::CANifier_Control_1_General:
			output = ctre::phoenix::CANifierControlFrame::CANifier_Control_1_General;
			break;
		case hardware_interface::canifier::CANifierControlFrame::CANifier_Control_2_PwmOutput:
			output = ctre::phoenix::CANifierControlFrame::CANifier_Control_2_PwmOutput;
			break;
		default:
			ROS_ERROR("Invalid input in convertCANifierControlFrame");
			return false;
	}
	return true;
}
