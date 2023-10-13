#include "ctre_interfaces/orchestra_state_interface.h"

namespace hardware_interface
{

OrchestraState::OrchestraState()
{
}

OrchestraState::~OrchestraState()
{
}

void OrchestraState::setMusic(const std::string &music)
{
    music_ = music;
}
std::string OrchestraState::getMusic() const
{
    return music_;
}

void OrchestraState::setInstruments(const std::vector<std::string> &instruments)
{
    instruments_ = instruments;
}
std::vector<std::string> OrchestraState::getInstruments() const
{
    return instruments_;
}

void OrchestraState::setPaused()
{
    is_paused_ = true;
	is_playing_ = false;
}
bool OrchestraState::getIsPaused() const
{
    return is_paused_;
}

void OrchestraState::setPlaying()
{
    is_playing_ = true;
	is_paused_ = false;
}
bool OrchestraState::getIsPlaying() const
{
    return is_playing_;
}

void OrchestraState::setStopped()
{
	is_playing_ = false;
	is_paused_ = false;
}
bool OrchestraState::getIsStopped() const
{
    return !is_playing_ && !is_paused_;
}

void OrchestraState::setCurrentTime(const double current_time)
{
    current_time_ = current_time;
}
double OrchestraState::getCurrentTime(void) const
{
    return current_time_;
}

}