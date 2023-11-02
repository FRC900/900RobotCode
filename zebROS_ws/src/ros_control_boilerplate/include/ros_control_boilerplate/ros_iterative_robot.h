#ifndef INC_ROS_ITERATRIVE_ROBOT__
#define INC_ROS_ITERATRIVE_ROBOT__

namespace ros_control_boilerplate
{
class ROSIterativeRobot
{
	public:
		ROSIterativeRobot(void);
		ROSIterativeRobot(const ROSIterativeRobot &) = delete;
		ROSIterativeRobot(ROSIterativeRobot &&) noexcept = delete;
		virtual ~ROSIterativeRobot() = default;

		ROSIterativeRobot &operator=(const ROSIterativeRobot &) = delete;
		ROSIterativeRobot &operator=(ROSIterativeRobot &&) noexcept = delete;

		void StartCompetition(void) const;
		void OneIteration(void) const;
};

} // namespace ros_control_boilerplate
#endif