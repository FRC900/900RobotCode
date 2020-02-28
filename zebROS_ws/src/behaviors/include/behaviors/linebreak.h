#pragma once

#include <ros/ros.h>
#include <atomic>
#include <sensor_msgs/JointState.h>
#include <string>

class Linebreak {
	private:
		std::string name_;
		size_t idx_; //index of this linebreak in the joint_states message
		int true_count_;
		int false_count_;
		int debounce_iterations_;

	public: //all need to be std::atomic because code running asynchronously might want to access it
		std::atomic<bool> triggered_;

		//pulse detection stuff
		std::atomic<bool> prev_state_;
		std::atomic<int> prev_toggle_; //-1 for falling edge, 0 initially, 1 for rising edge
		std::atomic<bool> rising_edge_happened_; //neither of these set to false unless resetPulseDetection() is called. We need this to be persistent so we can detect a rising edge even if it's followed by lots of toggles afterwards
		std::atomic<bool> falling_edge_happened_;
		std::atomic<bool> pulsed_; //if rising edge followed by falling edge

		//called every time the joint state subscriber callback is run
		bool update(const sensor_msgs::JointState &joint_state)
		{
			//set idx if it hasn't already been set
			if ( idx_ >= joint_state.name.size() ) //idx_ is infinitely big before it's set
			{
				for (size_t i = 0; i < joint_state.name.size(); i++)
				{
					if (joint_state.name[i] == name_){
						idx_ = i;
					}
				}
				//if the index wasn't set, couldn't find it
				if ( idx_ >= joint_state.name.size() ) {
					ROS_ERROR_STREAM("Linebreak named " << name_ << " not found in joint_states");
					true_count_ = 0;
					false_count_ = 0;
					return false;
				}
			}

			//update linebreak state
			prev_state_ = triggered_.load();

			if (joint_state.position[idx_] != 0) { //if linebreak true
				true_count_ += 1;
				false_count_ = 0;
			}
			else { //if linebreak false
				true_count_ = 0;
				false_count_ += 1;
			}

			if (true_count_ > debounce_iterations_){
				triggered_ = true;
			}
			else if (false_count_ > debounce_iterations_){
				triggered_ = false;
			}

			//do pulse detection stuff
			if(prev_state_ == false && triggered_ == true){
				rising_edge_happened_ = true;
				prev_toggle_ = 1;
			}
			else if(prev_state_ == true && triggered_ == false){
				//if rising edge had happened before the falling edge happened, we got a pulse
				if(prev_toggle_ == 1){ //1 means rising edge
					pulsed_ = true;
				}

				falling_edge_happened_ = true;
				prev_toggle_ = -1;
			}


			return true;
		}

		void resetPulseDetection()
		{
			prev_toggle_ = 0;
			rising_edge_happened_ = false;
			falling_edge_happened_ = false;
			pulsed_ = false;
		}

		Linebreak(std::string name) //name as found in joint_states
		{
			//general stuff
			name_ = name;
			idx_ = std::numeric_limits<size_t>::max(); //bigger than any number. Will be this until we actually set it
			true_count_ = 0;
			false_count_ = 0;
			triggered_ = false;

			//search for debounce iterations param and get it
			ros::NodeHandle nh;
			if(!nh.getParam("/actionlib_params/linebreak_debounce_iterations", debounce_iterations_))
			{
				ROS_ERROR("Couldn't find param /actionlib_params/linebreak_debounce_iterations. Using default of 3.");
				debounce_iterations_ = 3;
			}

			//pulse detection stuff
			prev_state_ = false;
			prev_toggle_ = 0;
			rising_edge_happened_ = false;
			falling_edge_happened_ = false;
			pulsed_ = false;
		}
};
