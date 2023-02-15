#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>

namespace hardware_interface
{
class OrchestraState
{
	public:
		OrchestraState(int orchestra_id);
		~OrchestraState();

                void setChirpFilePath(const std::string &chirp_file_path);
                std::string getChirpFilePath() const;

                void setInstruments(const std::vector<std::string> &instruments);
                std::vector<std::string> getInstruments() const;

                void setPaused();
                bool getIsPaused() const;
                
                void setPlaying();
                bool getIsPlaying() const;

                void setStopped();
                bool getIsStopped() const;

	private:
                int orchestra_id_;
				std::vector<std::string> instruments_;
                std::string chirp_file_path_;
                bool is_playing_;
                bool is_paused_;
};

// Glue code to let this be registered in the list of
// hardware resources on the robot.  Since state is
// read-only, allow multiple controllers to register it.
typedef StateHandle<const OrchestraState> OrchestraStateHandle;
typedef StateHandle<OrchestraState> OrchestraWritableStateHandle;
class OrchestraStateInterface : public HardwareResourceManager<OrchestraStateHandle> {};
}
