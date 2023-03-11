#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <talon_state_msgs/TalonState.h>
#include <behavior_actions/Fourber2023Action.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <std_msgs/Float64.h>
#include <controllers_2023_msgs/FourBarSrv.h>
#include <iostream>
#include <atomic>
#include <behaviors/game_pieces_2023.h>

#define FourberINFO(x) ROS_INFO_STREAM("2023_fourber_server : " << x)
#define FourberWARN(x) ROS_WARN_STREAM("2023_fourber_server : " << x)
#define FourberERR(x) ROS_ERROR_STREAM("2023_fourber_server : " << x)

typedef behavior_actions::Fourber2023Goal fourber_ns;

/*
# Fourber =
uint8 CUBE=0
uint8 VERTICAL_CONE=1
uint8 BASE_TOWARDS_US_CONE=2
uint8 BASE_AWAY_US_CONE=3
uint8 piece

uint8 INTAKE_LOW=0 # should be just two places that we need to call saftey with: when intaking and when going above a certain height
uint8 HIGH=1
uint8 saftey_position # looks up config value on where to go based on this, ignored if saftey is false

---
#result
bool success # if we got there
---
#feedback
bool success # if we got there
*/

template <class T>
void load_param_helper(const ros::NodeHandle &nh, std::string name, T &result, T default_val)
{
    if (!nh.getParam(name, result))
    {
        FourberERR("Could not find" << name << ", defaulting to " << default_val);
        result = default_val;
    }
}


class FourberAction2023
{

    protected:

        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<behavior_actions::Fourber2023Action> as_;
        ros::NodeHandle nh_params_;

        double safety_high_max_angle_;
        double safety_high_min_angle_;

        double safety_mid_max_angle_;
        double safety_mid_min_angle_;

        double safety_low_max_angle_;
        double safety_low_min_angle_;

        ros::ServiceClient fourbar_srv_;
        std::string action_name_;

        ros::Subscriber fourbar_offset_sub_;
        ros::Subscriber talon_states_sub_;
        ros::Subscriber fourbar_state_sub_;
        std::map<PieceMode, double> game_piece_lookup_;

        double position_offset_ = 0;
        double position_tolerance_ = 0.02;
        double fourbar_cur_setpoint_;
        double fourbar_cur_position_;

        SafetyState safety_state_;
        std::mutex safety_state_lock_;
        double previous_setpoint_;

        std::string joint;

        size_t fourbar_master_idx = std::numeric_limits<size_t>::max();

    public:

        FourberAction2023(std::string name) :
            as_(nh_, name, boost::bind(&FourberAction2023::executeCB, this, _1), false),
            nh_params_(nh_, "fourber_server_2023"),
            action_name_(name)
        {

            const std::map<std::string, std::string> service_connection_header{{"tcp_nodelay", "1"}};
            // TODO check topic
            fourbar_srv_ = nh_.serviceClient<controllers_2023_msgs::FourBarSrv>("/frcrobot_jetson/four_bar_controller_2023/four_bar_service", false, service_connection_header);
            if (!fourbar_srv_.waitForExistence())
            {
                FourberERR("========Could not find fourbar service========");
            }
            fourbar_offset_sub_ = nh_.subscribe("/fourbar_position_offset", 1, &FourberAction2023::heightOffsetCallback, this);
            talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states", 1, &FourberAction2023::talonStateCallback, this);

            load_param_helper(nh_, "position_tolerance", position_tolerance_, 0.02);

            load_param_helper(nh_, "joint", joint, std::string("four_bar"));

            // default values are guesses
            double res = -1;
            // cube
            load_param_helper(nh_, "cube/intake", res, 0.0);
            game_piece_lookup_[PieceMode(fourber_ns::CUBE, fourber_ns::INTAKE)] = res;

            load_param_helper(nh_, "cube/low_node", res, 0.5);
            game_piece_lookup_[PieceMode(fourber_ns::CUBE, fourber_ns::LOW_NODE)] = res;

            load_param_helper(nh_, "cube/middle_node", res, 0.7);
            game_piece_lookup_[PieceMode(fourber_ns::CUBE, fourber_ns::MIDDLE_NODE)] = res;

            load_param_helper(nh_, "cube/high_node", res, 1.0);
            game_piece_lookup_[PieceMode(fourber_ns::CUBE, fourber_ns::HIGH_NODE)] = res;
            // vertical cone
            load_param_helper(nh_, "vertical_cone/intake", res, 0.0);
            game_piece_lookup_[PieceMode(fourber_ns::VERTICAL_CONE, fourber_ns::INTAKE)] = res;

            load_param_helper(nh_, "vertical_cone/low_node", res, 0.5);
            game_piece_lookup_[PieceMode(fourber_ns::VERTICAL_CONE, fourber_ns::LOW_NODE)] = res;

            load_param_helper(nh_, "vertical_cone/middle_node", res, 0.7);
            game_piece_lookup_[PieceMode(fourber_ns::VERTICAL_CONE, fourber_ns::MIDDLE_NODE)] = res;

            load_param_helper(nh_, "vertical_cone/high_node", res, 1.0);
            game_piece_lookup_[PieceMode(fourber_ns::VERTICAL_CONE, fourber_ns::HIGH_NODE)] = res;
            // cone with base toward us
            load_param_helper(nh_, "base_towards_us_cone/intake", res, 0.0);
            game_piece_lookup_[PieceMode(fourber_ns::BASE_TOWARDS_US_CONE, fourber_ns::INTAKE)] = res;

            load_param_helper(nh_, "base_towards_us_cone/low_node", res, 0.5);
            game_piece_lookup_[PieceMode(fourber_ns::BASE_TOWARDS_US_CONE, fourber_ns::LOW_NODE)] = res;

            load_param_helper(nh_, "base_towards_us_cone/middle_node", res, 0.7);
            game_piece_lookup_[PieceMode(fourber_ns::BASE_TOWARDS_US_CONE, fourber_ns::MIDDLE_NODE)] = res;

            load_param_helper(nh_, "base_towards_us_cone/high_node", res, 1.0);
            game_piece_lookup_[PieceMode(fourber_ns::BASE_TOWARDS_US_CONE, fourber_ns::HIGH_NODE)] = res;
            // cone with base away from us
            load_param_helper(nh_, "base_away_us_cone/intake", res, 0.0);
            game_piece_lookup_[PieceMode(fourber_ns::BASE_AWAY_US_CONE, fourber_ns::INTAKE)] = res;
            load_param_helper(nh_, "base_away_us_cone/low_node", res, 0.5);
            game_piece_lookup_[PieceMode(fourber_ns::BASE_AWAY_US_CONE, fourber_ns::LOW_NODE)] = res;
            load_param_helper(nh_, "base_away_us_cone/middle_node", res, 0.7);
            game_piece_lookup_[PieceMode(fourber_ns::BASE_AWAY_US_CONE, fourber_ns::MIDDLE_NODE)] = res;
            load_param_helper(nh_, "base_away_us_cone/high_node", res, 1.0);
            game_piece_lookup_[PieceMode(fourber_ns::BASE_AWAY_US_CONE, fourber_ns::HIGH_NODE)] = res;

            load_param_helper(nh_, "safety_high/max_angle", res, 0.4);
            safety_high_max_angle_ = res;
            load_param_helper(nh_, "safety_high/min_angle", res, 0.4);
            safety_high_min_angle_ = res;

            load_param_helper(nh_, "safety_mid/max_angle", res, 0.4);
            safety_mid_max_angle_ = res;
            load_param_helper(nh_, "safety_mid/min_angle", res, 0.4);
            safety_mid_min_angle_ = res;

            load_param_helper(nh_, "safety_intake/max_angle", res, 0.4);
            safety_low_max_angle_ = res;
            load_param_helper(nh_, "safety_intake/min_angle", res, 0.4);
            safety_low_min_angle_ = res;

            as_.start();
            FourberINFO("Started Fourber Action server");
        }

        ~FourberAction2023(void)
        {
        }

        void publishFailure(std::string msg = "", uint8_t error_code = 0)
        {
            behavior_actions::Fourber2023Feedback feedback;
            behavior_actions::Fourber2023Result result;
            feedback.success = false;
            result.success = false;
            result.error_code = error_code;
            result.message = msg;
            as_.publishFeedback(feedback);
            as_.setAborted(result);
        }

        void publishSuccess()
        {
            behavior_actions::Fourber2023Feedback feedback;
            behavior_actions::Fourber2023Result result;
            feedback.success = true;
            result.success = true;
            as_.publishFeedback(feedback);
            as_.setSucceeded(result);
        }

        // blocks until fourbar is at the correct position
        bool waitForFourbar(const controllers_2023_msgs::FourBarSrv &srv)
        {
            // NOTE we can't check the talon position against the position passed into the four bar controller
            // because it changes that linear position into an angular position.
            // Check talon.set_point against talon.position instead

            // ^ is a good note, but now that they both use angular position it should be fine to use that
            ros::Rate r = ros::Rate(10);

            FourberINFO(srv.request.angle << " " << fourbar_cur_position_ << " " << fourbar_cur_setpoint_);

            while (true)
            {
                ros::spinOnce();
                ROS_INFO_STREAM_THROTTLE(1, "fourber_server_2023: Waiting for fourbar, " << fourbar_cur_position_ << " vs " << fourbar_cur_setpoint_);

                // essentially just keep fourbar where it is now
                if (as_.isPreemptRequested() || !ros::ok())
                {
                    FourberWARN("Fourber Preemepted");
                    controllers_2023_msgs::FourBarSrv last_req;
                    last_req.request.angle = fourbar_cur_setpoint_; // shouldn't this set to current position instead of setpoint?
                    if (!fourbar_srv_.call(last_req))
                    {
                        FourberERR("Could not set fourbar to the current setpoint!");
                    }
                    return false;
                }

                if (fabs(fourbar_cur_position_ - fourbar_cur_setpoint_) <= position_tolerance_)
                {
                    FourberINFO("Four bar reached position! " << fourbar_cur_position_ << " vs " << fourbar_cur_setpoint_);
                    break;
                }
                r.sleep();
            }

            return true;
        }

        // min distance is the minimum distance the forbar must be extended to not cause problems
        void safetyBoundsAndCallService()
        {
            safety_state_lock_.lock();
            double max_angle = safety_state_.max_angle;
            double min_angle = safety_state_.min_angle;
            safety_state_lock_.unlock();

            controllers_2023_msgs::FourBarSrv safety_req;

            // wanting to go to safe position and already at a safe position
            if (std::clamp(fourbar_cur_setpoint_, min_angle, max_angle) == fourbar_cur_setpoint_ && 
                std::clamp(fourbar_cur_position_, min_angle, max_angle) == fourbar_cur_position_)
            {
                publishSuccess();
                // nothing to do
                return;
            }
                        
            safety_req.request.angle = std::clamp(fourbar_cur_setpoint_, min_angle, max_angle);

            if (!fourbar_srv_.call(safety_req))
            {
                FourberERR("Unable to call fourbar service in saftey code");
                publishFailure("Unable to call fourbar service in safety code: " + std::to_string(safety_req.request.angle));
                return;
            }

            if (!waitForFourbar(safety_req))
            {
                FourberERR("Failed calling fourber with message 'safety_req'");
                publishFailure("Failed calling fourber with message " + std::to_string(safety_req.request.angle));
                return;
            }
            else
            {
                FourberINFO("Already in safe position");
                publishSuccess();
                return;
            }
        }

        void executeCB(const behavior_actions::Fourber2023GoalConstPtr &goal)
        {
            ros::spinOnce();
            FourberINFO("Called with " << goal->safety_positions.size() << " safety_positions");

            bool safety_set = false;

            if (goal->safety_positions.size() != 0) {
                // reset safety to add the new positions.
                // so that if we call it with [intake] then [mid] we get [mid] after the second call instead of [intake, mid]
                // because if we didn't do this we would have to do [intake], [none], [mid] (which is unsafe, the four bar moves)
                std::unique_lock<std::mutex> uLock(safety_state_lock_);
                safety_state_.max_angle = std::numeric_limits<double>::max();
                safety_state_.min_angle = std::numeric_limits<double>::min();

                previous_setpoint_ = fourbar_cur_setpoint_;
                FourberINFO("previous setpoint = " << previous_setpoint_); // currently broken because no talon state reading

                for (uint8_t position : goal->safety_positions) {
                    // let's say safety zones are [0,1] and [-0.25,0.75]
                    // desired outcome is what both allow which is [0,0.75]
                    // = [max(0, -0.25), min(1, 0.75)]
                    if (position == fourber_ns::SAFETY_HIGH)
                    {
                        safety_set = true;
                        FourberINFO("High");
                        safety_state_.max_angle = std::min(safety_state_.max_angle, safety_high_max_angle_);
                        safety_state_.min_angle = std::max(safety_state_.min_angle, safety_high_min_angle_);
                    }

                    else if (position == fourber_ns::SAFETY_MID)
                    {
                        safety_set = true;
                        FourberINFO("Mid");
                        safety_state_.max_angle = std::min(safety_state_.max_angle, safety_mid_max_angle_);
                        safety_state_.min_angle = std::max(safety_state_.min_angle, safety_mid_min_angle_);
                    }

                    else if (position == fourber_ns::SAFETY_INTAKE_LOW)
                    {
                        safety_set = true;
                        FourberINFO("Low");
                        safety_state_.max_angle = std::min(safety_state_.max_angle, safety_low_max_angle_);
                        safety_state_.min_angle = std::max(safety_state_.min_angle, safety_low_min_angle_);
                    }

                    else if (position == fourber_ns::SAFETY_TO_NO_SAFETY) {
                        safety_state_.max_angle = std::numeric_limits<double>::max();
                        safety_state_.min_angle = std::numeric_limits<double>::min();
                        controllers_2023_msgs::FourBarSrv go_to_previous_req;
                        go_to_previous_req.request.angle = previous_setpoint_;

                        if (!fourbar_srv_.call(go_to_previous_req))
                        {
                            FourberERR("Failed calling fourber service with message 'go_to_previous_req'");
                            publishFailure("Failed calling fourber service with message " + std::to_string(go_to_previous_req.request.angle));
                            return;
                        }

                        if (!waitForFourbar(go_to_previous_req))
                        {
                            publishFailure("Failed waiting for fourber service with message " + std::to_string(go_to_previous_req.request.angle));
                            return;
                        }

                        publishSuccess();
                        return;
                    }

                    FourberINFO("Current safety: " << safety_state_.min_angle << " to " << safety_state_.max_angle << ". Called with " << std::to_string(goal->piece) << " to " << std::to_string(goal->mode));
                }
            }

            if (safety_set) {
                safetyBoundsAndCallService();
                return;
            }
            
            // this code is reached when safety_positions.size() == 0
            // (aka being called by anything except the elevator)

            // select piece, nice synatax makes loading params worth it
            PieceMode lookup = PieceMode(goal->piece, goal->mode);
            double req_position;
            if (lookup.isValid()) {
                req_position = game_piece_lookup_[lookup];
            }
            else {
                FourberERR("Failed game piece lookup, ignoring message");
                publishFailure("Failed game piece lookup");
                return;
            }

            // apply offset
            req_position += position_offset_;
            if (position_offset_ != 0)
            {
                FourberWARN("Offset of " << position_offset_);
            }

            // for movements when inside a safe zone
            // SAFETY_TO_NO_SAFETY sets both to maximum and minimum doubles so this is fine
            // do we want to throw an error or just clamp?
            safety_state_lock_.lock();
            if (req_position < safety_state_.min_angle || req_position > safety_state_.max_angle) {
                FourberERR("Commanded position of " << req_position << " is unsafe! Not moving four bar and throwing an error");
                publishFailure("Unsafe position", behavior_actions::Fourber2023Result::UNSAFE_ANGLE);
                safety_state_lock_.unlock();
                return;
            }
            safety_state_lock_.unlock();

            FourberINFO("FourbERing a " << piece_to_string[goal->piece] << " to the position " << mode_to_string[goal->mode] << " and the FOURBAR to the position=" << req_position << " meters");

            // we know that saftey is set to none
            controllers_2023_msgs::FourBarSrv req;
            req.request.angle = req_position;

            if (!fourbar_srv_.call(req))   // somehow fourbar has failed, set status and abort to pass error up
            {
                FourberERR("Failed to moving fourbar :(");
                publishFailure("Failed to move fourbar to " + std::to_string(req.request.angle));
                return;
            }

            // failed
            if (!waitForFourbar(req))
            {
                publishFailure("Failed to wait for fourbar to " + std::to_string(req.request.angle));
                return;
            }

            ros::spinOnce();
            FourberINFO("Succeeded moving Fourbar!");
            publishSuccess();
            return;
        }

        void heightOffsetCallback(const std_msgs::Float64 position_offset_msg)
        {
            position_offset_ = position_offset_msg.data;
        }

        // "borrowed" from 2019 climb server
        void talonStateCallback(const talon_state_msgs::TalonState &talon_state)
        {
            // fourbar_master_idx == max of size_t at the start
            if (fourbar_master_idx == std::numeric_limits<size_t>::max())
            {
                for (size_t i = 0; i < talon_state.name.size(); i++)
                {
                    if (talon_state.name[i] == "four_bar")
                    {
                        fourbar_master_idx = i;
                        break;
                    }
                }
            }
            if (!(fourbar_master_idx == std::numeric_limits<size_t>::max())) 
            {
                fourbar_cur_position_ = talon_state.position[fourbar_master_idx];
                fourbar_cur_setpoint_ = talon_state.set_point[fourbar_master_idx];
            }
            else {
                FourberERR("Can not find talon with name = " << "four_bar");
            }
        }
}; // FourberAction2023


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fourber_server_2023");
    FourberAction2023 fourber("fourber_server_2023");
    ros::spin();
    return 0;
}
