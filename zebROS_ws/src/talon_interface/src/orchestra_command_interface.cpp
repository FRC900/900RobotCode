#include "talon_interface/orchestra_command_interface.h"

namespace hardware_interface
{

// Set up default values
// We don't want to set any of the 
// changed_ values to true, because if you
// call orchestra commands without music
// loaded then it's an invalid action
OrchestraCommand::OrchestraCommand() :
	pause_changed_(false),
	play_changed_(false),
	stop_changed_(false),
	file_path_(""),
	load_music_changed_(false),
	instruments_{},
	instruments_changed_(false),
	clear_instruments_changed_(false)
{
}

OrchestraCommand::~OrchestraCommand()
{
}

void OrchestraCommand::pause()
{
	pause_changed_ = true;
}
bool OrchestraCommand::getPause(void) const
{
	return pause_changed_;
}
bool OrchestraCommand::pauseChanged()
{
    if(pause_changed_)
    {
        pause_changed_ = false;
        return true;
    }
    return false;
}

void OrchestraCommand::play()
{
	play_changed_ = true;
}
bool OrchestraCommand::getPlay(void) const
{
	return play_changed_;
}
bool OrchestraCommand::playChanged()
{
    if(play_changed_)
    {
        play_changed_ = false;
        return true;
    }
    return false;
}

void OrchestraCommand::stop()
{
	stop_changed_ = true;
}
bool OrchestraCommand::getStop(void) const
{
	return stop_changed_;
}
bool OrchestraCommand::stopChanged()
{
    if(stop_changed_)
    {
        stop_changed_ = false;
        return true;
    }
    return false;
}

void OrchestraCommand::loadMusic(const std::string &file_path)
{
	if(file_path_ != file_path)
	{
		file_path_ = file_path;
		load_music_changed_ = true;
	}
}
std::string OrchestraCommand::getMusic() const
{
    return file_path_;
}
bool OrchestraCommand::musicChanged(std::string &file_path)
{
	file_path = file_path_;
    if(load_music_changed_)
    {
        load_music_changed_ = false;
        return true;
    }
    return false;
}
void OrchestraCommand::resetMusic()
{
    load_music_changed_ = true;
}

void OrchestraCommand::addInstruments(const std::vector<std::string> &instruments)
{
	if(instruments_ != instruments)
	{
		instruments_changed_ = true;
		instruments_ = instruments;
	}
}
std::vector<std::string> OrchestraCommand::getInstruments() const
{
    return instruments_;
}
bool OrchestraCommand::instrumentsChanged(std::vector<std::string> &instruments)
{
	instruments = instruments_;
    if(instruments_changed_)
    {
        instruments_changed_ = false;
        return true;
    }
    return false;
}
void OrchestraCommand::resetInstruments()
{
    instruments_changed_ = true;
}

void OrchestraCommand::clearInstruments()
{
    clear_instruments_changed_ = true;
}
bool OrchestraCommand::getInstrumentsCleared() const
{
    return true;
}
bool OrchestraCommand::clearInstrumentsChanged()
{
    if(clear_instruments_changed_)
    {
        clear_instruments_changed_ = false;
        return true;
    }
    return false;
}
void OrchestraCommand::resetClearInstruments()
{
    clear_instruments_changed_ = true;
}

} // namespace
