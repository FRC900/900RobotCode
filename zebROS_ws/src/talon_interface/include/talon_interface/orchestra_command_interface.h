#pragma once

#include <talon_interface/orchestra_state_interface.h>
#include "state_handle/command_handle.h"

namespace hardware_interface
{
class OrchestraCommand
{
	public:
		OrchestraCommand();
		~OrchestraCommand();

                void pause();
                bool getPause() const;
                bool pauseChanged();
                
                void play();
                bool getPlay() const;
                bool playChanged();

                void stop();
                bool getStop() const;
                bool stopChanged();

                void loadMusic(const std::string &file_path);
				std::string getMusic() const;
                bool musicChanged(std::string &file_path);
                void resetMusic();

                void addInstruments(const std::vector<std::string> &instruments);
				std::vector<std::string> getInstruments() const;
                bool instrumentsChanged(std::vector<std::string> &instruments);
                void resetInstruments();

                void clearInstruments();
                bool getInstrumentsCleared() const;
                bool clearInstrumentsChanged();
                void resetClearInstruments();

	private:
                bool pause_changed_;
                bool play_changed_;
                bool stop_changed_;

                std::string file_path_;
                bool load_music_changed_;
				std::vector<std::string> instruments_;
                bool instruments_changed_;
                bool instruments_cleared_;
                bool clear_instruments_changed_;
};

// Create a handle pointing to a type TalonHWCommand / TalonHWState pair
typedef CommandHandle<OrchestraCommand, OrchestraState, OrchestraStateHandle> OrchestraCommandHandle;

// Use ClaimResources here since we only want 1 controller
// to be able to access a given Orchestra at any particular time
class OrchestraCommandInterface : public HardwareResourceManager<OrchestraCommandHandle, ClaimResources> {};
}
