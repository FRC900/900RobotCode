#include <cstdio>                   // for fputs, fclose, fopen, FILE
#include <exception>                // for terminate
#include "ros/console.h"
#include "WPILibVersion.h"          // for GetWPILibVersion
#include "frc/DriverStation.h"      // for DriverStation
#include "hal/DriverStation.h"      // for HAL_ObserveUserProgramAutonomous
#include "hal/FRCUsageReporting.h"  // for HAL_Report, kFramework_ROS, kReso...
#include "hal/HALBase.h"            // for HAL_Initialize
#include "ros_control_boilerplate/ros_iterative_robot.h"

namespace ros_control_boilerplate
{
	ROSIterativeRobot::ROSIterativeRobot(void) : m_ds(frc::DriverStation::GetInstance())
	{
		if (!HAL_Initialize(500, 0))
		{
			ROS_ERROR("FATAL ERROR: HAL could not be initialized");
			std::terminate();
		}
		std::FILE* file = nullptr;
		file = std::fopen("/tmp/frc_versions/FRC_Lib_Version.ini", "w");

		if (file != nullptr) {
			std::fputs("C++ ", file);
			std::fputs(GetWPILibVersion(), file);
			std::fclose(file);
		}

		HAL_Report(HALUsageReporting::kResourceType_Framework, HALUsageReporting::kFramework_ROS);
		HAL_Report(HALUsageReporting::kResourceType_RobotDrive, 900, 0, "field centric swerve");
		//HAL_Report(HALUsageReporting::kResourceType_kKinematics, HALUsageReporting::kKinematics_SwerveDrive);
#if 0
		for (int i = 0; i < 900; i++)
			HAL_Report(HALUsageReporting::kResourceType_NidecBrushless, 900);
#endif
		HAL_Report(HALUsageReporting::kResourceType_Language, 900, 0, "C++/CMake/Javascript/Python/Shell/PERL");
	}

	void ROSIterativeRobot::StartCompetition(void) const
	{
		HAL_ObserveUserProgramStarting();
	}

	void ROSIterativeRobot::OneIteration(void) const
	{
		// Call the appropriate function depending upon the current robot mode
		if (m_ds.IsDisabled()) {
			HAL_ObserveUserProgramDisabled();
		} else if (m_ds.IsAutonomous()) {
			HAL_ObserveUserProgramAutonomous();
		} else if (m_ds.IsOperatorControl()) {
			HAL_ObserveUserProgramTeleop();
		} else {
			HAL_ObserveUserProgramTest();
		}
	}

} // namespace ros_control_boilerplate


