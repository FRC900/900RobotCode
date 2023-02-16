#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <actionlib/client/simple_client_goal_state.h>

#include <behavior_actions/Elevater2023Action.h>
#include <behavior_actions/Fourber2023Action.h>

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <std_msgs/Float64.h>
#include <controllers_2023_msgs/ElevatorSrv.h>
#include <talon_state_msgs/TalonState.h>
#include <iostream>
#include <behaviors/game_pieces_2023.h>

#define ElevaterINFO(x) ROS_INFO_STREAM("2023_elevater_server : " << x)
#define ElevaterERR(x) ROS_ERROR_STREAM("2023_elevater_server : " << x)

typedef behavior_actions::Elevater2023Goal elevater_ns;

/*
uint8 INTAKE=0
uint8 LOW_NODE=1
uint8 MIDDLE_NODE=2
uint8 HIGH_NODE=3
uint8 mode

uint8 CUBE=0
uint8 VERTICAL_CONE=1
uint8 BASE_TOWARDS_US_CONE=2
uint8 BASE_AWAY_US_CONE=3
uint8 piece
*/
//constexpr std::array<const char *, 4> mode_to_string {"INTAKE", "LOW_NODE", "MIDDLE_NODE", "HIGH_NODE"};
//constexpr std::array<const char *, 4> piece_to_string {"CUBE", "VERTICAL_CONE", "BASE_TOWARDS_US_CONE", "BASE_AWAY_US_CONE"};

template <class T>
void load_param_helper(const ros::NodeHandle &nh, std::string name, T &result, T default_val)
{
    if (!nh.getParam(name, result))
    {
        ElevaterERR("Could not find " << name << ", defaulting to " << default_val);
        result = default_val;
    }
}

class ElevaterAction2023
{

    protected:

        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<behavior_actions::Elevater2023Action> as_;
        actionlib::SimpleActionClient<behavior_actions::Fourber2023Action> ac_fourber_;
        ros::NodeHandle nh_params_;

        ros::ServiceClient elevator_srv_;
        std::string action_name_;

        // lookup is [enum for piece][enum for level]
        std::map<PieceMode, double> game_piece_lookup_;

        ddynamic_reconfigure::DDynamicReconfigure ddr_;
        ros::Subscriber elevator_offset_sub_;
        ros::Subscriber talon_states_sub_;

        double elev_cur_position_;
        double position_offset_ = 0;
        double position_tolerance_ = 0.02;

        // only need to avoid certain points above and below
        double safety_high_position_min_;
        double safety_high_position_max_;

        double safety_intake_position_min_;
        double safety_intake_position_max_;

        size_t elevater_master_idx;
        SafteyState fourber_safety_state_; // feels bad to also hold fourber state here but it is needed to be able to properly send transition state vs no safety

    public:

        ElevaterAction2023(std::string name) :
            as_(nh_, name, boost::bind(&ElevaterAction2023::executeCB, this, _1), false),
            ac_fourber_("/elevator/fourber_server_2023", true),
            nh_params_(nh_, "elevater_server_2023"),
            action_name_(name),
            ddr_(nh_params_)
        {
            // time to load all 16 params!
            // the idea here is that the customization is good, and if values are the same than thats great
            // current mech designs do have us going to diffrent heights based on what we have, and this is essentially the lookup so that every node doesn't have to do this
            // we will never (hopefully) need more posibilites than this, so we are prepared for everything mech could come up with
            // also if we for certain pieces/heights we can change height if we notice something off
            elevater_master_idx = std::numeric_limits<size_t>::max();

            load_param_helper(nh_, "position_tolerance", position_tolerance_, 0.02);
            // default values are guesses
            double res = -1;
            // cube
            load_param_helper(nh_, "cube/intake", res, 0.0);
            //game_piece_lookup_[elevater_ns::CUBE, elevater_ns::INTAKE] = res;
            game_piece_lookup_[PieceMode(elevater_ns::CUBE, elevater_ns::INTAKE)] = res;
            load_param_helper(nh_, "cube/low_node", res, 0.5);
            game_piece_lookup_[PieceMode(elevater_ns::CUBE, elevater_ns::LOW_NODE)] = res;
            load_param_helper(nh_, "cube/middle_node", res, 0.7);
            game_piece_lookup_[PieceMode(elevater_ns::CUBE, elevater_ns::MIDDLE_NODE)] = res;
            load_param_helper(nh_, "cube/high_node", res, 1.0);
            game_piece_lookup_[PieceMode(elevater_ns::CUBE, elevater_ns::HIGH_NODE)] = res;
            // vertical cone
            load_param_helper(nh_, "vertical_cone/intake", res, 0.0);
            game_piece_lookup_[PieceMode(elevater_ns::VERTICAL_CONE, elevater_ns::INTAKE)] = res;
            load_param_helper(nh_, "vertical_cone/low_node", res, 0.5);
            game_piece_lookup_[PieceMode(elevater_ns::VERTICAL_CONE, elevater_ns::LOW_NODE)] = res;
            load_param_helper(nh_, "vertical_cone/middle_node", res, 0.7);
            game_piece_lookup_[PieceMode(elevater_ns::VERTICAL_CONE, elevater_ns::MIDDLE_NODE)] = res;
            load_param_helper(nh_, "vertical_cone/high_node", res, 0.5);
            game_piece_lookup_[PieceMode(elevater_ns::VERTICAL_CONE, elevater_ns::HIGH_NODE)] = res;
            // cone with base toward us
            load_param_helper(nh_, "base_towards_us_cone/intake", res, 0.0);
            game_piece_lookup_[PieceMode(elevater_ns::BASE_TOWARDS_US_CONE, elevater_ns::INTAKE)] = res;
            load_param_helper(nh_, "base_towards_us_cone/low_node", res, 0.5);
            game_piece_lookup_[PieceMode(elevater_ns::BASE_TOWARDS_US_CONE, elevater_ns::LOW_NODE)] = res;
            load_param_helper(nh_, "base_towards_us_cone/middle_node", res, 0.7);
            game_piece_lookup_[PieceMode(elevater_ns::BASE_TOWARDS_US_CONE, elevater_ns::MIDDLE_NODE)] = res;
            load_param_helper(nh_, "base_towards_us_cone/high_node", res, 1.0);
            game_piece_lookup_[PieceMode(elevater_ns::BASE_TOWARDS_US_CONE, elevater_ns::HIGH_NODE)] = res;
            // cone with base away from us
            load_param_helper(nh_, "base_away_us_cone/intake", res, 0.0);
            game_piece_lookup_[PieceMode(elevater_ns::BASE_AWAY_US_CONE, elevater_ns::INTAKE)] = res;
            load_param_helper(nh_, "base_away_us_cone/low_node", res, 0.5);
            game_piece_lookup_[PieceMode(elevater_ns::BASE_AWAY_US_CONE, elevater_ns::LOW_NODE)] = res;
            load_param_helper(nh_, "base_away_us_cone/middle_node", res, 0.7);
            game_piece_lookup_[PieceMode(elevater_ns::BASE_AWAY_US_CONE, elevater_ns::MIDDLE_NODE)] = res;
            load_param_helper(nh_, "base_away_us_cone/high_node", res, 1.0);
            game_piece_lookup_[PieceMode(elevater_ns::BASE_AWAY_US_CONE, elevater_ns::HIGH_NODE)] = res;


            load_param_helper(nh_, "saftey_high_min", safety_high_position_min_, 2.0);
            load_param_helper(nh_, "saftey_high_max", safety_high_position_max_, 2.5);

            load_param_helper(nh_, "safety_intake_min", safety_intake_position_min_, 0.0);
            load_param_helper(nh_, "safety_intake_max", safety_intake_position_max_, 0.5);

            ElevaterINFO("Game Piece params");

            const std::map<std::string, std::string> service_connection_header{{"tcp_nodelay", "1"}};
            // TODO check topic
            elevator_srv_ = nh_.serviceClient<controllers_2023_msgs::ElevatorSrv>("/frcrobot_jetson/elevator_controller_2023/elevator_service", false, service_connection_header);
            if (!elevator_srv_.waitForExistence(ros::Duration(5)))
            {
                ElevaterERR("=======Could not find elevator service========");
            }
            elevator_offset_sub_ = nh_.subscribe("/elevator_position_offset", 1, &ElevaterAction2023::heightOffsetCallback, this);
            talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states", 1, &ElevaterAction2023::talonStateCallback, this);

            ddr_.registerVariable<double>("safety_high_position", &game_piece_lookup_[PieceMode(elevater_ns::CUBE, elevater_ns::INTAKE)], "If elevator high requester greater than this, saftey mode called for fourbar", 0, 4);
            ddr_.registerVariable<double>("safety_low_position", &game_piece_lookup_[PieceMode(elevater_ns::CUBE, elevater_ns::INTAKE)], "If elevator height requester less than this, saftey low mode called for fourbar", 0, 4);

            ddr_.registerVariable<double>("CUBE_intake", &game_piece_lookup_[PieceMode(elevater_ns::CUBE, elevater_ns::INTAKE)], "", 0, 4);
            ddr_.registerVariable<double>("CUBE_low_node", &game_piece_lookup_[PieceMode(elevater_ns::CUBE, elevater_ns::LOW_NODE)], "", 0, 4);
            ddr_.registerVariable<double>("CUBE_middle_node", &game_piece_lookup_[PieceMode(elevater_ns::CUBE, elevater_ns::MIDDLE_NODE)], "", 0, 4);
            ddr_.registerVariable<double>("CUBE_high_node", &game_piece_lookup_[PieceMode(elevater_ns::CUBE, elevater_ns::HIGH_NODE)], "", 0, 4);

            ddr_.registerVariable<double>("VERTICAL_CONE_intake", &game_piece_lookup_[PieceMode(elevater_ns::CUBE, elevater_ns::INTAKE)], "", 0, 4);
            ddr_.registerVariable<double>("VERTICAL_CONE_low_node", &game_piece_lookup_[PieceMode(elevater_ns::VERTICAL_CONE, elevater_ns::LOW_NODE)], "", 0, 4);
            ddr_.registerVariable<double>("VERTICAL_CONE_middle_node", &game_piece_lookup_[PieceMode(elevater_ns::VERTICAL_CONE, elevater_ns::MIDDLE_NODE)], "", 0, 4);
            ddr_.registerVariable<double>("VERTICAL_CONE_high_node", &game_piece_lookup_[PieceMode(elevater_ns::VERTICAL_CONE, elevater_ns::HIGH_NODE)], "", 0, 4);

            ddr_.registerVariable<double>("BASE_TOWARDS_US_CONE_intake", &game_piece_lookup_[PieceMode(elevater_ns::BASE_TOWARDS_US_CONE, elevater_ns::INTAKE)], "", 0, 4);
            ddr_.registerVariable<double>("BASE_TOWARDS_US_CONE_low_node", &game_piece_lookup_[PieceMode(elevater_ns::BASE_TOWARDS_US_CONE, elevater_ns::LOW_NODE)], "", 0, 4);
            ddr_.registerVariable<double>("BASE_TOWARDS_US_CONE_middle_node", &game_piece_lookup_[PieceMode(elevater_ns::BASE_TOWARDS_US_CONE, elevater_ns::MIDDLE_NODE)], "", 0, 4);
            ddr_.registerVariable<double>("BASE_TOWARDS_US_CONE_high_node", &game_piece_lookup_[PieceMode(elevater_ns::BASE_TOWARDS_US_CONE, elevater_ns::HIGH_NODE)], "", 0, 4);

            ddr_.registerVariable<double>("BASE_AWAY_US_CONE_intake", &game_piece_lookup_[PieceMode(elevater_ns::BASE_AWAY_US_CONE, elevater_ns::INTAKE)], "", 0, 4);
            ddr_.registerVariable<double>("BASE_AWAY_US_CONE_low_node", &game_piece_lookup_[PieceMode(elevater_ns::BASE_AWAY_US_CONE, elevater_ns::LOW_NODE)], "", 0, 4);
            ddr_.registerVariable<double>("BASE_AWAY_US_CONE_middle_node", &game_piece_lookup_[PieceMode(elevater_ns::BASE_AWAY_US_CONE, elevater_ns::MIDDLE_NODE)], "", 0, 4);
            ddr_.registerVariable<double>("BASE_AWAY_US_CONE_high_node", &game_piece_lookup_[PieceMode(elevater_ns::BASE_AWAY_US_CONE, elevater_ns::HIGH_NODE)], "", 0, 4);

            ddr_.publishServicesTopics();
            as_.start();
            ElevaterINFO("Started Elevater Action server");
        }

        ~ElevaterAction2023(void)
        {
        }

        /*
        void print_map()
        {
            for (const auto &elem : game_piece_lookup_)
            {
                std::cout << elem.first << "\n";
                for (const auto &sub_elem : elem.second)
                {
                    std::cout << sub_elem.first << " " << sub_elem.second << "\n";
                }
                std::cout << "\n\n";
            }
        }
        */
        void publishFailure()
        {
            behavior_actions::Elevater2023Feedback feedback;
            behavior_actions::Elevater2023Result result;
            feedback.success = false;
            result.success = false;
            as_.publishFeedback(feedback);
            as_.setAborted(result);
        }

        void publishSuccess()
        {
            behavior_actions::Elevater2023Feedback feedback;
            behavior_actions::Elevater2023Result result;
            feedback.success = true;
            result.success = true;
            as_.publishFeedback(feedback);
            as_.setSucceeded(result);
        }

        void executeCB(const behavior_actions::Elevater2023GoalConstPtr &goal)
        {
            ros::spinOnce();
            // select piece, nice synatax makes loading params worth it
            PieceMode lookup = PieceMode(goal->piece, goal->mode);
            double req_position;
            if (lookup.isValid()) {
                req_position = game_piece_lookup_[lookup];
            }
            else {
                ElevaterERR("Failed game piece lookup");
                publishFailure();
                return;
            }

            // apply offset
            req_position += position_offset_;
            assert(req_position >= 0); // probably done in elevator server also
            behavior_actions::Fourber2023Goal fourber_goal;

            // high and low range checks
            if (safety_high_position_min_ <= req_position && req_position <= safety_high_position_max_)
            {
                fourber_goal.safety_position = behavior_actions::Fourber2023Goal::SAFETY_HIGH;
                fourber_safety_state_ = SafteyState::SAFTEY_HIGH;
            }
            else if (safety_intake_position_min_ <= req_position && req_position <= safety_intake_position_max_)
            {
                fourber_goal.safety_position = behavior_actions::Fourber2023Goal::SAFETY_INTAKE_LOW;
                fourber_safety_state_ = SafteyState::SAFTEY_LOW;
            }
            // not going within eaither of those, so if the safety state is set, we can trainsition to it being unset
            else if (fourber_safety_state_ == SafteyState::SAFTEY_HIGH || fourber_safety_state_ == SafteyState::SAFTEY_LOW)
            {
                fourber_goal.safety_position = behavior_actions::Fourber2023Goal::SAFETY_TO_NO_SAFETY;
                fourber_safety_state_ = SafteyState::NONE;
            }
            else
            {
                fourber_goal.safety_position = behavior_actions::Fourber2023Goal::NO_SAFETY;
            }

            // have a meaningful message to send
            bool fourber_success = false;
            if (!(fourber_goal.safety_position == behavior_actions::Fourber2023Goal::NO_SAFETY))
            {
                auto fourbar_result = ac_fourber_.sendGoalAndWait(fourber_goal, ros::Duration(5), ros::Duration(3));
                if (!(fourbar_result == actionlib::SimpleClientGoalState::SUCCEEDED))
                {
                    ElevaterERR("Fourber actionlib called from elevater has failed. Unable to safely move elevator. Aborting");
                    publishFailure();
                    return;
                }
            }

            ElevaterINFO("Moving a " << piece_to_string[goal->piece] << " to the position " << mode_to_string[goal->mode] << " and the ELEVATOR to the position=" << req_position << " meters");

            behavior_actions::Elevater2023Feedback feedback;
            behavior_actions::Elevater2023Result result;

            controllers_2023_msgs::ElevatorSrv req;
            req.request.position = req_position;

            if (!elevator_srv_.call(req))   // somehow elevator has failed, set status and abort to pass error up
            {
                ElevaterERR("Failed to moving elevator :(");
                publishFailure();
                return;
            }

            ros::Rate r = ros::Rate(10);
            while (true)
            {
                ros::spinOnce();
                ElevaterINFO("Moving elevator");

                // if we are preempted, we probably didn't want to move the elevator to the requested location, so stop it by requesting current position
                // hopefully saves a few seconds in a match
                if (as_.isPreemptRequested() || !ros::ok())
                {
                    req.request.position = elev_cur_position_;
                    if (!elevator_srv_.call(req))
                    {
                        ElevaterERR("Failed to set elevator setpoint to current height during preempt!");
                    }
                    result.success = false;
                    as_.setPreempted(result);
                    return;
                }

                if (fabs(elev_cur_position_ - req_position) <= position_tolerance_)
                {
                    ElevaterINFO("Elevator reached position!");
                    break;
                }
                r.sleep();
            }

            ElevaterINFO("Succeeded moving elevator!");
            publishSuccess();
            // print_map();
            ros::spinOnce();
            return;
        }

        void heightOffsetCallback(const std_msgs::Float64 speed_offset_msg)
        {
            position_offset_ = speed_offset_msg.data;
        }

        // "borrowed" from 2019 climb server
        void talonStateCallback(const talon_state_msgs::TalonState &talon_state)
        {
            // fourbar_master_idx == max of size_t at the start
            if (elevater_master_idx >= talon_state.name.size()) // could maybe just check for > 0
            {
                for (size_t i = 0; i < talon_state.name.size(); i++)
                {
                    if (talon_state.name[i] == "elevater_master")
                    {
                        elevater_master_idx = i;
                        break;
                    }
                }
                ElevaterERR("Can not find talong with name = " << "elevater_master");
            }
            if (!(elevater_master_idx == std::numeric_limits<size_t>::max())) 
            {
                elev_cur_position_ = talon_state.position[elevater_master_idx];
            }
        }

}; // ElevaterAction2023

int main(int argc, char **argv)
{
    ros::init(argc, argv, "elevater_server_2023");
    ElevaterAction2023 elevater("elevater_server_2023");
    ros::spin();
    return 0;
}
