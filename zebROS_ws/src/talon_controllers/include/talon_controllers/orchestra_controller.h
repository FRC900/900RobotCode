#ifndef ORCHESTRA_CONTROLLER
#define ORCHESTRA_CONTROLLER

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <talon_controller_msgs/LoadInstrumentsSrv.h>
#include <talon_controller_msgs/LoadMusicSrv.h>
#include <talon_controller_msgs/SetOrchestraStateSrv.h>
#include <ctre_interfaces/orchestra_command_interface.h>

namespace orchestra_controller
{
	class OrchestraController: public controller_interface::Controller<hardware_interface::OrchestraCommandInterface>
	{
		public:
			OrchestraController() {}

			virtual bool init(hardware_interface::OrchestraCommandInterface *hw,
					ros::NodeHandle						&root_nh,
					ros::NodeHandle						&controller_nh) override;
			virtual void starting(const ros::Time &time) override;
			virtual void update(const ros::Time &time, const ros::Duration & ) override;
			virtual void stopping(const ros::Time &time) override;

			bool loadMusicService(talon_controller_msgs::LoadMusicSrv::Request &req,
					talon_controller_msgs::LoadMusicSrv::Response &res);

			bool setStateService(talon_controller_msgs::SetOrchestraStateSrv::Request &req,
					talon_controller_msgs::SetOrchestraStateSrv::Response &res);

			bool reloadInstrumentsService(talon_controller_msgs::LoadInstrumentsSrv::Request &req,
					talon_controller_msgs::LoadInstrumentsSrv::Response &res);

		private:
			hardware_interface::OrchestraCommandHandle orchestra_command_handle_;
			realtime_tools::RealtimeBuffer<std::string> music_file_path_;
			realtime_tools::RealtimeBuffer<std::vector<std::string>> instruments_;
			realtime_tools::RealtimeBuffer<int> orchestra_state_;
			int previous_state_{2};

			ros::ServiceServer load_music_server_;
			ros::ServiceServer set_state_server_;
			ros::ServiceServer load_instruments_server_;
	}; //class

} //namespace

#endif
