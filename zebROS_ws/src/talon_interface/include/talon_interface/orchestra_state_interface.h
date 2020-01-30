#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>

namespace hardware_interface
{
class OrchestraState
{
	public:
		OrchestraState();
		~OrchestraState();

                void setChirpFilePath(std::string chirp_file_path);
                std::string getChirpFilePath() const;

                void setInstruments(std::vector<TalonStateHandle> instruments);
                std::vector<TalonStateHandle> getInstruments() const;

                void setIsPaused(bool is_paused);
                bool getIsPaused() const;
                
                void setIsPlaying(bool is_playing);
                bool getIsPlaying() const;

	private:
		std::vector<TalonStateHandle> instruments_;
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
