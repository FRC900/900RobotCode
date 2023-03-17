// so theoretically the best way to do this is to use the values in 2023_grid_locations.yaml to drive to the location knowing our field-relative location using an apriltag.
// that's just a particle filter.
// which we don't trust yet.
// so we can do the equivalent by calculating the offset between a detected apriltag (see 2023_apriltag_locations.yaml) and the desired location
// and averaging if there are multiple.
// find closest apriltag to desired location, calculate offset, call path_to_apriltag
#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include "behavior_actions/AlignToGrid2023Action.h"
#include "behavior_actions/AlignAndPlaceGrid2023Action.h"
#include "behavior_actions/PathToAprilTagAction.h"
#include <actionlib/client/simple_action_client.h>
#include <behavior_actions/Placing2023Action.h>
#include <behavior_actions/AlignToGrid2023Action.h>
#include <path_follower_msgs/PathAction.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

class AlignAndPlaceGridAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<behavior_actions::AlignAndPlaceGrid2023Action> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  behavior_actions::AlignAndPlaceGrid2023Feedback feedback_;
  behavior_actions::AlignAndPlaceGrid2023Result result_;
  ros::Subscriber sub_;
  actionlib::SimpleActionClient<behavior_actions::AlignToGrid2023Action> align_to_goal_ac;
  actionlib::SimpleActionClient<behavior_actions::Placing2023Action> placing_ac;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher orientation_command_pub_;
  ros::Subscriber control_effort_sub_;
  double orient_effort_;
  double xOffset_;
  double holdPosTimeout_;
  double latest_yaw_;
  double percent_complete_{};
  double desired_percent_complete_;
  bool started_moving_elevator_;
  bool moved_ = false;
  uint8_t alliance_;

  double green_button_time_;
  double rotate_time_;

public:

  AlignAndPlaceGridAction(std::string name) :
    as_(nh_, name, boost::bind(&AlignAndPlaceGridAction::executeCB, this, _1), false),
    action_name_(name),
    align_to_goal_ac("/align_to_grid", true),
    placing_ac("/placing/placing_server_2023", true),
    // /teleop/swerve_drive_controller/cmd_vel
    cmd_vel_pub_(nh_.advertise<geometry_msgs::Twist>("/placing/cmd_vel", 1)),
    orientation_command_pub_(nh_.advertise<std_msgs::Float64>("/teleop/orientation_command", 1)),
    control_effort_sub_(nh_.subscribe<std_msgs::Float64>("/teleop/orient_strafing/control_effort", 1, &AlignAndPlaceGridAction::controlEffortCB, this))
  {
    if (!nh_.getParam("green_button_time", green_button_time_)) {
			ROS_WARN_STREAM("2023_align_and_place_grid : could not find green_button_time, defaulting to 1 second");
			green_button_time_ = 1;
		}
    if (!nh_.getParam("rotate_time", rotate_time_)) {
			ROS_WARN_STREAM("2023_align_and_place_grid : could not find rotate_time, defaulting to 0.5 seconds");
			rotate_time_ = 0.5;
		}
    //nh_.getParam("tags", tagList);
    as_.start();
  }

  ~AlignAndPlaceGridAction(void)
  {
  }

  void handle_preempt() {
    ROS_WARN_STREAM("Preempting align and place grid!"); 
    align_to_goal_ac.cancelGoalsAtAndBeforeTime(ros::Time::now());
    placing_ac.cancelGoalsAtAndBeforeTime(ros::Time::now());
    as_.setPreempted();
    result_.success = false;
    as_.setAborted(result_);
  }

  void place(uint8_t node, uint8_t game_piece) {
    behavior_actions::Placing2023Goal goal;
    goal.node = node;
    goal.piece = game_piece;
    goal.override_game_piece = true;
    goal.step = moved_ ? goal.PLACE_RETRACT : goal.MOVE;
    placing_ac.sendGoal(goal);
    moved_ = !moved_;
  }

  void feedbackCB(const  behavior_actions::AlignToGrid2023FeedbackConstPtr& feedback) {
    percent_complete_ = feedback->percent_complete;
  }

  void controlEffortCB(const std_msgs::Float64ConstPtr& msg) {
    orient_effort_ = msg->data;
  }

  void executeCB(const behavior_actions::AlignAndPlaceGrid2023GoalConstPtr &goal)
  {
    if (!align_to_goal_ac.isServerConnected()) {
      ROS_ERROR_STREAM("2023_align_and_place_grid : align to grid server not running!!! this is unlikely to work");
    }
    if (!placing_ac.isServerConnected()) {
      ROS_ERROR_STREAM("2023_align_and_place_grid : placing server not running!!! this is unlikely to work");
    }
    bool started_moving_elevator = false;
    bool path_finished = false;
    ros::Time path_finished_time = ros::Time(0);
    uint8_t game_piece = goal->piece;
    uint8_t node = goal->node;
    ROS_INFO_STREAM("Align and place grid callback!");
    
    behavior_actions::AlignToGrid2023Goal grid_goal;
    
    grid_goal.alliance = goal->alliance;
    grid_goal.grid_id = goal->grid_id; 
    ros::Rate r(10);
    for (int i = 0; i <= 1; ++i) {
        align_to_goal_ac.sendGoal(grid_goal, /* Done cb */ NULL, /*Active*/ NULL, boost::bind(&AlignAndPlaceGridAction::feedbackCB, this, _1));
        
        while (!align_to_goal_ac.getState().isDone()) {
            ros::spinOnce();
            if (as_.isPreemptRequested() || !ros::ok())
            {
                handle_preempt();
                return;
            }
            if (percent_complete_ >= goal->percent_to_extend && !started_moving_elevator) {
                ROS_INFO_STREAM("Sending elevator!");
                started_moving_elevator = true;
                place(node, game_piece);
            }
            r.sleep();
        }
    }

    path_finished_time = ros::Time::now();
    std_msgs::Float64 msg;
    msg.data = M_PI;
    orientation_command_pub_.publish(msg);
    while (ros::Time::now() - path_finished_time < ros::Duration(green_button_time_)) {
        ros::spinOnce();
        ROS_INFO_STREAM_THROTTLE(0.4, "Green buttoning from align and place!");
        orientation_command_pub_.publish(msg);
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.5; // green button states
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = orient_effort_;
        cmd_vel_pub_.publish(cmd_vel);
        r.sleep();
    }

    ros::Time started_orient_time = ros::Time::now();
    while (orient_effort_ > 0.1 && (ros::Time::now() - started_orient_time) < ros::Duration(rotate_time_)) {
        ros::spinOnce();
        ROS_INFO_STREAM_THROTTLE(0.4, "Aligning to wall, we aren't rotated correctly");
        orientation_command_pub_.publish(msg);
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0; // green button states
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = orient_effort_;
        cmd_vel_pub_.publish(cmd_vel);
        r.sleep();
    }

    if (started_moving_elevator && placing_ac.getState().isDone() && goal->auto_place) {
        ROS_INFO_STREAM("Full auto placing");
        place(node, game_piece);
    }
    else {
        ROS_INFO_STREAM("Finished aligned to goal");
    }
    result_.success = true;
    as_.setSucceeded(result_);

    moved_ = false; // should end down. TODO replace this with elev height
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "align_and_place_grid");

  AlignAndPlaceGridAction alignAndPlaceGrid("align_and_place_grid");
  ros::spin();

  return 0;
}