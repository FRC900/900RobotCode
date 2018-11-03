#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "behaviors/IntakeAction.h"
#include "intake_controller/IntakeSrv.h"
#include "sensor_msgs/JointState.h"
#include <atomic>
#include <ros/console.h>

double intake_power;
double intake_hold_power;
double linebreak_debounce_iterations;
double spit_out_time;

class IntakeAction {
    protected:
        ros::NodeHandle nh_;

        actionlib::SimpleActionServer<behaviors::IntakeAction> as_;
        std::string action_name_;
        ros::ServiceClient intake_srv_;
	std::atomic<int> cube_state_true;
	std::atomic<int> cube_state_false;
	behaviors::IntakeResult result_;
	ros::Subscriber cube_state_;
	ros::Subscriber proceed_;
	bool proceed;

    public:
        IntakeAction(const std::string &name) :
            as_(nh_, name, boost::bind(&IntakeAction::executeCB, this, _1), false),
            action_name_(name)
        {
            as_.start();
            std::map<std::string, std::string> service_connection_header;
            service_connection_header["tcp_nodelay"] = "1";
            intake_srv_ = nh_.serviceClient<intake_controller::IntakeSrv>("/frcrobot/intake_controller/intake_command", false, service_connection_header);
            cube_state_ = nh_.subscribe("/frcrobot/joint_states", 1, &IntakeAction::jointStateCallback, this);
            //proceed_ = nh_.subscribe("/frcrobot/auto_interpreter_server/proceed", 1, &IntakeAction::proceedCallback, this);
	}

        ~IntakeAction(void) 
        {
        }

        void executeCB(const behaviors::IntakeGoalConstPtr &goal) {
            ros::Rate r(10);
            double start_time = ros::Time::now().toSec();
            bool timed_out = false;
            bool aborted = false;
            bool success = false;
            if(goal->intake_cube) {
                cube_state_true = 0;
                intake_controller::IntakeSrv srv;
                srv.request.power = intake_power;
                srv.request.intake_in = false;
                if(!intake_srv_.call(srv)) 
                    ROS_ERROR("Srv intake call failed in auto interpreter server intake");
                ros::spinOnce();

                success = false;
                while(!success && !timed_out && !aborted) {
                    success = cube_state_true > linebreak_debounce_iterations;
                    if(as_.isPreemptRequested() || !ros::ok()) {
                        ROS_WARN("%s: Preempted", action_name_.c_str());
                        as_.setPreempted();
                        aborted = true;
                        return;
                    }
                    if (!aborted) {
                        r.sleep();
                        ros::spinOnce();
                        timed_out = (ros::Time::now().toSec()-start_time) > goal->timeout;
                    }
                }

                srv.request.power = 0;
                srv.request.intake_in = true; //soft in
                if(!intake_srv_.call(srv)) 
                    ROS_ERROR("Srv intake call failed in auto interpreter server intake");
            }
            else
            {
                cube_state_false = 0;
                intake_controller::IntakeSrv srv;
                srv.request.power = -1;
                srv.request.intake_in = true; //soft in
                if(!intake_srv_.call(srv)) 
                    ROS_ERROR("Srv intake call failed in auto interpreter server intake");
                ros::spinOnce();

                success = false;
                while(!success && !timed_out && !aborted) {
                    success = cube_state_false > linebreak_debounce_iterations;

                    if(as_.isPreemptRequested() || !ros::ok()) {
                        ROS_WARN("%s: Preempted", action_name_.c_str());
                        as_.setPreempted();
                        aborted = true;
                        return;
                    }
                    if (!aborted) {
                        r.sleep();
                        ros::spinOnce();
                        timed_out = (ros::Time::now().toSec()-start_time) > goal->timeout;
                    }
                }
                //wait another config time before stopping motors to ensure cube is out
                double start_time_extra = ros::Time::now().toSec();
                while(!timed_out && !aborted) {
                    success = (ros::Time::now().toSec() - start_time_extra) > 1;

                    if(as_.isPreemptRequested() || !ros::ok()) {
                        ROS_WARN("%s: Preempted", action_name_.c_str());
                        as_.setPreempted();
                        aborted = true;
                        return;
                    }
                    if (!aborted) {
                        r.sleep();
                        ros::spinOnce();
                        timed_out = (ros::Time::now().toSec()-start_time) > goal->timeout;
                    }
                }

                srv.request.power = 0;
                srv.request.intake_in = true; //soft in
                if(!intake_srv_.call(srv)) 
                    ROS_ERROR("Srv intake call failed in auto interpreter server intake");
            }
            if(timed_out)
            {
                ROS_INFO("%s: Timed Out", action_name_.c_str());
            }
            else if(!aborted)
            {
                ROS_INFO("%s: Succeeded", action_name_.c_str());
            }
            else
            {
                ROS_INFO("%s: Aborted", action_name_.c_str());
            }

            result_.timed_out = timed_out;
            result_.success = success;
            as_.setSucceeded(result_);
            return;
        }

		// Grab various info from hw_interface using
		// dummy joint position values
		void jointStateCallback(const sensor_msgs::JointState &joint_state)
		{
			static size_t cube_idx = std::numeric_limits<size_t>::max();
			if ((cube_idx >= joint_state.name.size()))
			{
				for (size_t i = 0; i < joint_state.name.size(); i++)
				{
					if (joint_state.name[i] == "intake_line_break")
						cube_idx = i;
				}
			}
			if (cube_idx < joint_state.position.size())
			{
				bool cube_state = (joint_state.position[cube_idx] != 0);
				if(cube_state)
				{
					cube_state_true += 1;
					cube_state_false = 0;
				}
				else
				{
					cube_state_true = 0;
					cube_state_false += 1;
				}
			}
			else
			{
				cube_state_true = 0;
				cube_state_false += 1;
			}
		}
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "intake_server");
    IntakeAction intake_action("intake_server");

    ros::NodeHandle n;
    ros::NodeHandle n_params(n, "teleop_params");
    ros::NodeHandle n_actionlib_intake_params(n, "actionlib_intake_params");

    if (!n_params.getParam("intake_power", intake_power))
		ROS_ERROR("Could not read intake_power in intake_server");

    if (!n_params.getParam("intake_hold_power", intake_hold_power))
		ROS_ERROR("Could not read intake_hold_power in intake_server");

    if (!n_actionlib_intake_params.getParam("linebreak_debounce_iterations", linebreak_debounce_iterations))
		ROS_ERROR("Could not read linebreak_debounce_iterations in intake_sever");
    if (!n_actionlib_intake_params.getParam("spit_out_time", spit_out_time))
		ROS_ERROR("Could not read spit_out_time in intake_sever");
    ros::spin();
    return 0;
}
