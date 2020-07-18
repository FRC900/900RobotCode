#include "talon_interface/orchestra_state_interface.h"

namespace hardware_interface
{

OrchestraState::OrchestraState(int orchestra_id) :
        orchestra_id_(orchestra_id),
        instruments_{},
        chirp_file_path_(""),
        is_playing_(false),
        is_paused_(false)
{
}

OrchestraState::~OrchestraState()
{
}

void OrchestraState::setChirpFilePath(const std::string &chirp_file_path)
{
    chirp_file_path_ = chirp_file_path;
}
std::string OrchestraState::getChirpFilePath() const
{
    return chirp_file_path_;
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
}
