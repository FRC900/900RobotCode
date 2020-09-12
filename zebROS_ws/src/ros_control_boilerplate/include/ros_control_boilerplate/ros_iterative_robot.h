#pragma once
namespace frc { class DriverStation; }

//Stuff from frcrobot_hw_interface
namespace ros_control_boilerplate
{
class ROSIterativeRobot
{
	public:
		ROSIterativeRobot(void);
		void StartCompetition(void) const;
		void OneIteration(void) const;
	private:
		frc::DriverStation& m_ds;
};

} // namespace ros_control_boilerplate
