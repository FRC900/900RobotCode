#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>

namespace hardware_interface
{
class OrchestraState
{
	public:
		OrchestraState();
		OrchestraState(const OrchestraState &) = delete;
		OrchestraState(OrchestraState &&) = delete;
		~OrchestraState();
        OrchestraState operator=(const OrchestraState &) = delete;
        OrchestraState &operator=(OrchestraState &&) = delete;

                void setMusic(const std::string &music);
                std::string getMusic() const;

                void setInstruments(const std::vector<std::string> &instruments);
                std::vector<std::string> getInstruments() const;

                void setPaused();
                bool getIsPaused() const;
                
                void setPlaying();
                bool getIsPlaying() const;

                void setStopped();
                bool getIsStopped() const;

                void setCurrentTime(const double current_time);
                double getCurrentTime(void) const;

	private:
				std::vector<std::string> instruments_{};
                std::string music_{""};
                bool is_playing_{false};
                bool is_paused_{false};
                double current_time_{0};

};

// Glue code to let this be registered in the list of
// hardware resources on the robot.  Since state is
// read-only, allow multiple controllers to register it.
typedef StateHandle<const OrchestraState> OrchestraStateHandle;
class OrchestraStateInterface : public HardwareResourceManager<OrchestraStateHandle> {};
}
