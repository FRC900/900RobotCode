#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <behavior_actions/FourbarElevatorPath2023Action.h>
#include <std_msgs/Float64.h>
#include <talon_state_msgs/TalonState.h>
#include <controllers_2023_msgs/ElevatorSrv.h>
#include <controllers_2023_msgs/FourBarSrv.h>

struct ElevatorFourbarPoint {
	// in that order
	double elevator;
	double fourbar;
};

using Path = std::vector<ElevatorFourbarPoint>;

class FourbarElevatorPathServer2023
{
protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<behavior_actions::FourbarElevatorPath2023Action> as_;
	std::string action_name_;
	// create messages that are used to published result
	behavior_actions::FourbarElevatorPath2023Result result_;

	ros::ServiceClient elevator_srv_;
	ros::ServiceClient fourbar_srv_;

	size_t fourbar_idx_;
	size_t elevator_idx_;
	double fourbar_position_;
	double elevator_position_;

	double fourbar_tolerance_;
	double elevator_tolerance_;

	ros::Subscriber talon_states_sub_;

	std::map<std::string, Path> path_map_;
	
public:

	FourbarElevatorPathServer2023(std::string name) :
		as_(nh_, name, boost::bind(&FourbarElevatorPathServer2023::executeCB, this, _1), false),
		action_name_(name)
	{
		fourbar_idx_ = std::numeric_limits<size_t>::max();
		elevator_idx_ = std::numeric_limits<size_t>::max();
		talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states", 1, &FourbarElevatorPathServer2023::talonStateCallback, this);
		XmlRpc::XmlRpcValue pathList;
    	nh_.getParam("paths", pathList);

		fourbar_tolerance_ = 0.03;
		elevator_tolerance_ = 0.03;

		if (nh_.hasParam("fourbar_tolerance")) {
			nh_.getParam("fourbar_tolerance", fourbar_tolerance_);
			ROS_INFO_STREAM("2023_fourbar_elevator_path_server : fourbar_tolerance is " << fourbar_tolerance_ << " radians");
		} else {
			ROS_INFO_STREAM("2023_fourbar_elevator_path_server : fourbar_tolerance defaulting to 0.03 radians");
		}

		if (nh_.hasParam("elevator_tolerance")) {
			nh_.getParam("elevator_tolerance", elevator_tolerance_);
			ROS_INFO_STREAM("2023_fourbar_elevator_path_server : elevator_tolerance is " << elevator_tolerance_ << " meters");
		} else {
			ROS_INFO_STREAM("2023_fourbar_elevator_path_server : elevator_tolerance defaulting to 0.03 meters");
		}

		for (XmlRpc::XmlRpcValue::iterator path=pathList.begin(); path!=pathList.end(); ++path) {
			// path->first = name
			// path->second = [[e,f],[e,f]...]
			ROS_INFO_STREAM("path name = " << path->first << ". points:");
			Path p;
			for (size_t i = 0; i < path->second.size(); i++) {
				auto pt = ElevatorFourbarPoint();
				pt.elevator = static_cast<double>(path->second[i][0]);
				pt.fourbar = static_cast<double>(path->second[i][1]);
				p.push_back(pt);
				ROS_INFO_STREAM("(" << pt.elevator << ", " << pt.fourbar << ")");
			}
			path_map_[path->first] = p;
		}

		const std::map<std::string, std::string> service_connection_header{{"tcp_nodelay", "1"}};

		elevator_srv_ = nh_.serviceClient<controllers_2023_msgs::ElevatorSrv>("/frcrobot_jetson/elevator_controller_2023/elevator_service", false, service_connection_header);
		if (!elevator_srv_.waitForExistence(ros::Duration(5)))
		{
			ROS_ERROR_STREAM("2023_fourbar_elevator_path_server : =======Could not find elevator service========");
		}

		fourbar_srv_ = nh_.serviceClient<controllers_2023_msgs::FourBarSrv>("/frcrobot_jetson/four_bar_controller_2023/four_bar_service", false, service_connection_header);
		if (!fourbar_srv_.waitForExistence(ros::Duration(5)))
		{
			ROS_ERROR_STREAM("2023_fourbar_elevator_path_server : =======Could not find fourbar service========");
		}
		as_.start();
	}

	~FourbarElevatorPathServer2023(void)
	{
	}

	void talonStateCallback(const talon_state_msgs::TalonState &talon_state)
	{
		if (fourbar_idx_ == std::numeric_limits<size_t>::max()) // could maybe just check for > 0
		{
			for (size_t i = 0; i < talon_state.name.size(); i++)
			{
				if (talon_state.name[i] == "four_bar")
				{
					fourbar_idx_ = i;
					break;
				}
			}
		}
		if (fourbar_idx_ != std::numeric_limits<size_t>::max()) 
		{
			fourbar_position_ = talon_state.position[fourbar_idx_];
		}
		else {
			ROS_ERROR_STREAM_THROTTLE(0.1, "2023_fourbar_elevator_path_server : cannot find talon with name = four_bar");
		}

		if (elevator_idx_ == std::numeric_limits<size_t>::max()) // could maybe just check for > 0
		{
			for (size_t i = 0; i < talon_state.name.size(); i++)
			{
				if (talon_state.name[i] == "elevator_leader")
				{
					elevator_idx_ = i;
					break;
				}
			}
		}
		if (elevator_idx_ != std::numeric_limits<size_t>::max()) 
		{
			elevator_position_ = talon_state.position[elevator_idx_];
		}
		else {
			ROS_ERROR_STREAM_THROTTLE(0.1, "2023_fourbar_elevator_path_server : cannot find talon with name = elevator_leader");
		}
	}

	template<class C, class S>
	bool waitForResultAndCheckForPreempt(const ros::Duration & timeout, actionlib::SimpleActionClient<C> & ac, actionlib::SimpleActionServer<S> & as, bool preempt_at_timeout = false)
	{
		bool negative_timeout = false;
		if (timeout < ros::Duration(0, 0)) {
			ROS_WARN("waitForResultAndCheckForPreempt : Negative timeout, waiting forever");
			negative_timeout = true;
		}

		ros::Time timeout_time = ros::Time::now() + timeout;

		ros::Rate r(10);

		while (ros::ok() && !as.isPreemptRequested()) {
			ros::spinOnce();
			// Determine how long we should wait
			ros::Duration time_left = timeout_time - ros::Time::now();

			// Check if we're past the timeout time
			if (timeout > ros::Duration(0, 0) && time_left <= ros::Duration(0, 0) && !negative_timeout) {
				if (preempt_at_timeout) {
					ac.cancelGoalsAtAndBeforeTime(ros::Time::now());
				}
				break;
			}

			if (ac.getState().isDone()) {
				break;
			}
			r.sleep();
		}

		return ac.getState().isDone();
	}

	bool waitForFourbarAndElevator(double fourbar_setpoint, double elevator_setpoint) {
		ros::Rate r = ros::Rate(10);

		while (true)
		{
			ros::spinOnce();
			ROS_INFO_STREAM_THROTTLE(1, "2023_fourbar_elevator_path_server: Waiting for fourbar and elevator, fourbar: " << fourbar_position_ << " vs " << fourbar_setpoint << ", elevator: " << elevator_position_ << " vs " << elevator_setpoint);

			// essentially just keep fourbar where it is now
			if (as_.isPreemptRequested() || !ros::ok())
			{
				ROS_ERROR_STREAM("2023_fourbar_elevator_path_server : preempted");
				controllers_2023_msgs::FourBarSrv fourbar_req;
				fourbar_req.request.angle = fourbar_position_;
				if (!fourbar_srv_.call(fourbar_req))
				{
					ROS_ERROR_STREAM("2023_fourbar_elevator_path_server : Could not set fourbar to the current setpoint!");
					result_.success = false;
					as_.setAborted(result_);
					return false;
				}
				controllers_2023_msgs::ElevatorSrv elevator_req;
				elevator_req.request.position = elevator_position_;
				if (!elevator_srv_.call(elevator_req))
				{
					ROS_ERROR_STREAM("2023_fourbar_elevator_path_server : Could not set elevator to the current setpoint!");
					result_.success = false;
					as_.setAborted(result_);
					return false;
				}
				return false;
			}

			if (fabs(fourbar_position_ - fourbar_setpoint) <= fourbar_tolerance_ && fabs(elevator_position_ - elevator_setpoint) <= elevator_tolerance_) // make this tolerance configurable
			{
				ROS_INFO_STREAM("2023_fourbar_elevator_path_server : fourbar and elevator reached position! ");
				break;
			}
			r.sleep();
		}
		return true;
	}

	void executeCB(const behavior_actions::FourbarElevatorPath2023GoalConstPtr &goal)
	{
		if (!elevator_srv_.waitForExistence(ros::Duration(5)))
		{
			ROS_ERROR_STREAM("2023_fourbar_elevator_path_server : =======Could not find elevator service========");
			return;
		}

		if (!fourbar_srv_.waitForExistence(ros::Duration(5)))
		{
			ROS_ERROR_STREAM("2023_fourbar_elevator_path_server : =======Could not find fourbar service========");
			return;
		}

		if (path_map_.find(goal->path) == path_map_.end()) {
			ROS_ERROR_STREAM("2023_fourbar_elevator_path_server : path " << goal->path << " not found! Exiting.");
			result_.success = false;
			as_.setAborted(result_);
			return;
		}

		Path path = path_map_[goal->path];
		if (goal->reverse) {
			std::reverse(path.begin(), path.end());
		}
		for (int i = 1; i < path.size(); i++) {
			auto pt = path[i];
			controllers_2023_msgs::FourBarSrv f;
			f.request.angle = pt.fourbar;
			ROS_INFO_STREAM("2023_fourbar_elevator_path_server : calling fourbar with " << pt.fourbar);
			if (!fourbar_srv_.call(f)) {
				ROS_ERROR_STREAM("2023_fourbar_elevator_path_server : four bar controller failed! Exiting.");
				result_.success = false;
				as_.setAborted(result_);
				return;
			}
			controllers_2023_msgs::ElevatorSrv e;
			e.request.position = pt.elevator;
			ROS_INFO_STREAM("2023_fourbar_elevator_path_server : calling elevator with " << pt.elevator);
			if (!elevator_srv_.call(e)) {
				ROS_ERROR_STREAM("2023_fourbar_elevator_path_server : elevator controller failed! Exiting.");
				result_.success = false;
				as_.setAborted(result_);
				return;
			}
			if (!waitForFourbarAndElevator(pt.fourbar, pt.elevator)) {
				return;
			}
		}
		result_.success = true;
		as_.setSucceeded(result_);
	}


};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "fourbar_elevator_path_server_2023");

	FourbarElevatorPathServer2023 pather("fourbar_elevator_path_server_2023");
	ros::spin();

	return 0;
}
