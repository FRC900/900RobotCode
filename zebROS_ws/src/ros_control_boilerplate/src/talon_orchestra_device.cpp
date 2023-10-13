#include <ctre/phoenix6/Orchestra.hpp>

#include "ctre_interfaces/orchestra_command_interface.h"
#include "ros_control_boilerplate/talon_orchestra_device.h"

#define safeTalonCall(error_code, call_string) \
    SIMFLAG ? true : safeCall(error_code, call_string)

template <bool SIMFLAG>
TalonOrchestraDevice<SIMFLAG>::TalonOrchestraDevice(const std::string &name_space,
                                                    const int joint_index,
                                                    const std::string &joint_name)
    : CTREV6Device{"TalonOrchestra", joint_name, joint_index}
    , orchestra_{nullptr}
    , state_{std::make_unique<hardware_interface::OrchestraState>()}
    , command_{std::make_unique<hardware_interface::OrchestraCommand>()}
{
    ROS_INFO_STREAM("Loading joint " << joint_index << "=" << joint_name << " as TalonOrchestra ");
    if constexpr (!SIMFLAG)
    {
        orchestra_ = std::make_unique<ctre::phoenix6::Orchestra>();
    }
}

template <bool SIMFLAG>
TalonOrchestraDevice<SIMFLAG>::~TalonOrchestraDevice(void)
{
}

template <bool SIMFLAG>
void TalonOrchestraDevice<SIMFLAG>::registerInterfaces(hardware_interface::OrchestraStateInterface &state_interface,
                                                       hardware_interface::OrchestraCommandInterface &command_interface)
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for TalonOrchestra : " << getName());

    hardware_interface::OrchestraStateHandle state_handle(getName(), state_.get());
    state_interface.registerHandle(state_handle);

    hardware_interface::OrchestraCommandHandle command_handle(state_handle, command_.get());
    command_interface.registerHandle(command_handle);

}

template <bool SIMFLAG>
void TalonOrchestraDevice<SIMFLAG>::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{	
    if constexpr (SIMFLAG)
    {
        return;
    }	
    if(orchestra_->IsPlaying())
    {
        state_->setPlaying();
    }
    else
    {
        state_->setStopped();
    }
}

template <bool SIMFLAG>
void TalonOrchestraDevice<SIMFLAG>::write(const ros::Time &/*time*/, const ros::Duration &/*period*/,
                                          const std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> &talonfxs)
{
    std::string music_file_path;
    std::vector<std::string> instruments;
    if(command_->clearInstrumentsChanged())
    {
        if(safeCall(orchestra_->ClearInstruments(), "ClearInstruments"))
        {
            ROS_INFO_STREAM("Talon Orchestra " << getName() << " cleared instruments.");
        }
        else{
            ROS_ERROR_STREAM("Failed to clear instruments in orchestra.");
        }
    }
    if(command_->instrumentsChanged(instruments))
    {
        if(safeCall(orchestra_->ClearInstruments(), "ClearInstruments"))
        {
            for (const auto &instrument : instruments)
            {
                const auto it = talonfxs.find(instrument);
                if (it == talonfxs.end())
                {
                    ROS_ERROR_STREAM("Talon Orchestra " <<  getName() << " failed to add Falcon " << instrument << " because it does not exist");
                }
                else
                {
                    if (safeTalonCall(orchestra_->AddInstrument(*(it->second)), "AddInstrument"))
                    {
                        ROS_INFO_STREAM("Talon Orchestra " << getName() << " added Falcon " << instrument);
                    }
                    else
                    {
                        ROS_ERROR_STREAM("Failed to add instrument " << instrument << " to orchestra");
                        command_->resetInstruments();
                    }
                }
            }
            state_->setInstruments(instruments);
        }
        else{
            ROS_ERROR_STREAM("Failed to clear instruments in orchestra");
            command_->clearInstruments();
        }
    }
    if(command_->musicChanged(music_file_path))
    {
        if(safeCall(orchestra_->LoadMusic(music_file_path.c_str()), "LoadMusic"))
        {
            state_->setMusic(music_file_path);
            ROS_INFO_STREAM("Talon Orchestra " << getName() << " loaded music at " << music_file_path);
        }
        else{
            ROS_ERROR_STREAM("Failed to load music into orchestra");
            command_->resetMusic();
        }
    }
    if(command_->pauseChanged())
    {
        if(safeCall(orchestra_->Pause(), "Pause"))
        {
            state_->setPaused();
            ROS_INFO_STREAM("Talon Orchestra " << getName() << " pausing");
        }
        else{
            ROS_ERROR_STREAM("Failed to pause orchestra");
            command_->pause();
        }
    }
    if(command_->playChanged())
    {
        if(safeCall(orchestra_->Play(), "Play"))
        {
            state_->setPlaying();
            ROS_INFO_STREAM("Talon Orchestra " << getName() << " playing");
        }
        else{
            ROS_ERROR_STREAM("Failed to play orchestra");
            command_->play();
        }
    }
    if(command_->stopChanged())
    {
        if(safeCall(orchestra_->Stop(), "Stop"))
        {
            //state_->setStopped();
            ROS_INFO_STREAM("Talon Orchestra " << getName() << " stopping");
        }
        else{
            ROS_ERROR_STREAM("Failed to stop orchestra");
            command_->stop();
        }
    }
}

template class TalonOrchestraDevice<false>;
template class TalonOrchestraDevice<true>;