#ifndef INC_ROS_ITERATRIVE_ROBOT__
#define INC_ROS_ITERATRIVE_ROBOT__

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
};

} // namespace ros_control_boilerplate
#endif