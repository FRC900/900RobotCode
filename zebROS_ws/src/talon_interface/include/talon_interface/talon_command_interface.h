#pragma once

#include <cassert>
#include <string>
#include <talon_interface/talon_state_interface.h>

namespace hardware_interface
{
	
	// Class to buffer data needed to set the state of the
	// Talon.  This should (eventually) include anything
	// which might be set during runtime.  Config data
	// which is set only once at startup can be handled
	// in the hardware manager constructor/init rather than through
	// this interface.
	// Various controller code will set the member vars of 
	// this class depending on the needs of the motor 
	// being controlled
	// Each pass through write() in the hardware interface
	// will use this to re-configure (if necessary) and then
	// update the setpoint on the associated Talon.
	// The hardware_controller is responsible for keeping
	// a master array of these classes - 1 entry per
	// physical Talon controller in the robot
	class TalonHWCommand
	{
		public:
			TalonHWCommand(void) :
				command_(0.),
				command_changed_(false),
				mode_(TalonMode_Uninitialized),
				mode_changed_(false),
				pidf_slot_(0),
				pidf_slot_changed_(false),
				iaccum_(0.0),
				iaccum_changed_(false),
				invert_(false),
				sensor_phase_(false),
				invert_changed_(false),
				neutral_mode_(NeutralMode_Uninitialized),
				neutral_mode_changed_(false),
				neutral_output_(false),
				encoder_feedback_(FeedbackDevice_Uninitialized),
				encoder_tick_per_rotation_(0),
				
				//output shaping
				closed_loop_ramp_(0),
				open_loop_ramp_(0),
				peak_output_forward_(100.),
				peak_output_reverse_(100.),
				nominal_output_forward_(100.),
				nominal_output_reverse_(100.),
				neutral_deadband_(0.),
				outputShapingChanged_(false),

				// voltage compensation
				voltage_compensation_saturation_(0),
				voltage_measurement_filter_(0),
				voltage_compensation_enable_(false),
				voltage_compensation_changed_(false),

				// current limiting
				current_limit_peak_amps_(0),
				current_limit_peak_msec_(0),
				current_limit_continuous_amps_(0),
				current_limit_enable_(false),
				current_limit_changed_(false)
			{
				for (int slot = 0; slot < 2; slot++)
				{
					p_[slot] = 0.0;
					i_[slot] = 0.0;
					d_[slot] = 0.0;
					f_[slot] = 0.0;
					i_zone_[slot] = 0;
					allowable_closed_loop_error_[slot] = 0;
					max_integral_accumulator_[slot] = 0;

					pidf_changed_[slot] = true;
				}
			}
			// This gets the requested setpoint, not the
			// status actually read from the controller
			// Need to think about which makes the most
			// sense to query...
			bool get(double &command)
			{
				command = command_;
				if (!command_changed_)
					return false;
				command_changed_ = false;
				return true;
			}

			TalonMode getMode(void) const {return mode_;}

			void setP(double oldP, int index){
				if ((index < 0) || ((size_t)index >= (sizeof(p_) / sizeof(p_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::setP()");
					return;
				}
				pidf_changed_[index] = true;
				p_[index] = oldP;}
			double getP(int index) const {
				if ((index < 0) || ((size_t)index >= (sizeof(p_) / sizeof(p_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::getP()");
					return 0.0;
				}
				return p_[index];
			}

			void setI(double ii, int index){
				if ((index < 0) || ((size_t)index >= (sizeof(i_) / sizeof(i_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::setI()");
					return;
				}
				pidf_changed_[index] = true;
				i_[index] = ii;}
			double getI(int index) const {
				if ((index < 0) || ((size_t)index >= (sizeof(i_) / sizeof(i_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::getI()");
					return 0.0;
				}
				return i_[index];
			}

			void setD(double dd, int index){
				if ((index < 0) || ((size_t)index >= (sizeof(d_) / sizeof(d_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::setD()");
					return;
				}
				pidf_changed_[index] = true;
				d_[index] = dd;}
			double getD(int index) const {
				if ((index < 0) || ((size_t)index >= (sizeof(d_) / sizeof(d_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::getD()");
					return 0.0;
				}
				return d_[index];
			}

			void setF(double ff, int index){
				if ((index < 0) || ((size_t)index >= (sizeof(f_) / sizeof(f_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::setF()");
					return;
				}
				pidf_changed_[index] = true;
				f_[index] = ff;}
			double getF(int index){
				if ((index < 0) || ((size_t)index >= (sizeof(f_) / sizeof(f_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::getF()");
					return 0.0;
				}
				return f_[index];
			}

			void setIZ(int oldIZ, int index){
				if ((index < 0) || ((size_t)index >= (sizeof(i_zone_) / sizeof(i_zone_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::setIZ()");
					return;
				}
				pidf_changed_[index] = true;
				i_zone_[index] = oldIZ;}
			int getIZ(int index) const {
				if ((index < 0) || ((size_t)index >= (sizeof(i_zone_) / sizeof(i_zone_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::getIZ()");
					return 0.0;
				}
				return i_zone_[index];
			}

			void setAllowableClosedloopError(int allowable_closed_loop_error, int index) {
				if ((index < 0) || ((size_t)index >= (sizeof(allowable_closed_loop_error_) / sizeof(allowable_closed_loop_error_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::setAllowableClosedLoopError()");
					return;
				}
				pidf_changed_[index] = true;
				allowable_closed_loop_error_[index] = allowable_closed_loop_error;
			}
			int getAllowableClosedloopError(int index) const {
				if ((index < 0) || ((size_t)index >= (sizeof(allowable_closed_loop_error_) / sizeof(allowable_closed_loop_error_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::getAllowableClosedLoopErrro()");
					return 0;
				}
				return allowable_closed_loop_error_[index];
			}
			void setMaxIntegralAccumulator(int max_integral_accumulator, int index) {
				if ((index < 0) || ((size_t)index >= (sizeof(max_integral_accumulator_) / sizeof(max_integral_accumulator_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::setAllowableClosedLoopError()");
					return;
				}
				pidf_changed_[index] = true;
				max_integral_accumulator_[index] = max_integral_accumulator;
			}
			int getMaxIntegralAccumulator(int index) const {
				if ((index < 0) || ((size_t)index >= (sizeof(max_integral_accumulator_) / sizeof(max_integral_accumulator_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::getAllowableClosedLoopErrro()");
					return 0.0;
				}
				return max_integral_accumulator_[index];
			}
			
			void setPID(double oldP, double oldI, double oldD, int index){
				if ((index < 0) || ((size_t)index >= (sizeof(p_) / sizeof(p_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::setPID()");
					return;
				}
				pidf_changed_[index] = true;
				p_[index] = oldP;i_[index] =oldI;d_[index]=oldD;}
			void setPID(double oldP, double oldI, double oldD, double oldF, int index){
				if ((index < 0) || ((size_t)index >= (sizeof(p_) / sizeof(p_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::setPIF()");
					return;
				}
				pidf_changed_[index] = true;
				p_[index]=oldP;i_[index]=oldI;d_[index]=oldD;f_[index]=oldF;}

			void setIntegralAccumulator(double iaccum)
			{
				iaccum_ = iaccum;
				iaccum_changed_ = true;
			}
			double getIntegralAccumulator(void) const { return iaccum_; }
			bool integralAccumulatorChanged(double &iaccum)
			{
				iaccum = iaccum_;
				if (!iaccum_changed_)
					return false;
				iaccum_changed_ = true;
				return true;
			}

			void set(double command) {command_changed_ = true; command_ = command;}
			void setMode(TalonMode mode)
			{
				if ((mode <= TalonMode_Uninitialized) || (mode >= TalonMode_Last))
				{
					ROS_WARN("Invalid mode passed to TalonHWCommand::setMode()");
					return;
				}
				mode_         = mode;
				mode_changed_ = true;
				this->set(0); // ??? Clear out setpoint for old mode
			}

			void setNeutralMode(NeutralMode neutral_mode)
			{
				if (neutral_mode == NeutralMode_Uninitialized)
					return;
				else if ((neutral_mode < NeutralMode_Uninitialized) || 
						 (neutral_mode >= NeutralMode_Last))
				{
					ROS_WARN("Invalid neutral_mode passed to TalonHWCommand::setNeutralMode()");
					return;
				}
				neutral_mode_         = neutral_mode;
				neutral_mode_changed_ = true;
			}
			bool getNeutralMode(void)
			{
				return neutral_mode_;
			}

			void setNeutralOutput(void)
			{
				neutral_output_ = true;
			}

			bool slotChanged(int &newpidfSlot)
			{
				newpidfSlot = pidf_slot_;
				if (!pidf_slot_changed_)
					return false;
				pidf_slot_changed_ = false;
				return true;
			}
			bool pidfChanged(double &p, double &i, double &d, double &f, int &iz, int &allowable_closed_loop_error, double &max_integral_accumulator, int index){
				if ((index < 0) || ((size_t)index >= (sizeof(p_) / sizeof(p_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::pidfChanged()");
					return false;
				}
				p = p_[index];
				i = i_[index];
				d = d_[index];
				f = f_[index];
				iz = i_zone_[index];
				allowable_closed_loop_error = allowable_closed_loop_error_[index];
				max_integral_accumulator = max_integral_accumulator_[index];
				if (!pidf_changed_[index])
					return false;
				pidf_changed_[index] = false;
				return true;
			}

			// Check  to see if mode changed since last call
			// If so, return true and set mode to new desired
			// talon mode
			// If mode hasn't changed, return false
			// Goal here is to prevent writes to the CAN
			// bus to repeatedly set the mode to the same value. 
			// Instead, only send a setMode to a given Talon if 
			// the mode has actually changed.
			bool newMode(TalonMode &mode)
			{
				mode = mode_;
				if (!mode_changed_)
					return false;
				mode_changed_ = false;
				return true;
			}

			void setPidfSlot(int npidf_slot){pidf_slot_ = npidf_slot;pidf_slot_changed_ = true;}
			int getPidfSlot(void)const{return pidf_slot_;}

			void setInvert(bool invert) {invert_ = invert; invert_changed_ = true;}
			void setSensorPhase(bool invert) {sensor_phase_ = invert; invert_changed_ = true;}
			bool invertChanged(bool &invert, bool &sensor_phase)
			{
				invert = invert_;
				sensor_phase = sensor_phase_;
				if (!invert_changed_)
					return false;
				invert_changed_ = false;
				return true;
			}

			bool neutralModeChanged(NeutralMode &neutral_mode)
			{
				neutral_mode = neutral_mode_;
				if (!neutral_mode_changed_)
					return false;
				neutral_mode_changed_ = false;
				return true;
			}
			
			//output shaping
			bool outputShapingChanged(double &closed_loop_ramp,
					double &open_loop_ramp,
					double &peak_output_forward,
					double &peak_output_reverse,
					double &nominal_output_forward,
					double &nominal_output_reverse,
					double &neutral_deadband)
			{
				closed_loop_ramp = closed_loop_ramp_;
				open_loop_ramp = open_loop_ramp_;
				peak_output_forward = peak_output_forward_;
				peak_output_reverse = peak_output_reverse_;
				nominal_output_forward = nominal_output_forward_;
				nominal_output_reverse = nominal_output_reverse_;
				neutral_deadband = neutral_deadband_;
				if (!outputShapingChanged_)
					return false;
				outputShapingChanged_ = false;
				return true;
			}

			// Set motor controller to neutral output
			// This should be a one-shot ... only
			// write it to the motor controller once
			bool neutralOutputChanged(void)
			{
				if (!neutral_output_)
					return false;
				neutral_output_ = false;
				return true;
			}

			FeedbackDevice getEncoderFeedback(void) const {return encoder_feedback_;}
			void setEncoderFeedback(FeedbackDevice encoder_feedback)
			{
				if ((encoder_feedback >= FeedbackDevice_Uninitialized) &&
				    (encoder_feedback <  FeedbackDevice_Last) )
					encoder_feedback_ = encoder_feedback;
				else
					ROS_WARN_STREAM("Invalid feedback device requested");
			}
			int getEncoderTickPerRotation(void) 	const {return encoder_tick_per_rotation_;}
			void setEncoderTickPerRotation(int encoder_tick_per_rotation) {encoder_tick_per_rotation_ = encoder_tick_per_rotation;}

			//output shaping
			void setClosedloopRamp(double closed_loop_ramp) {
				if (closed_loop_ramp_ != closed_loop_ramp) {
					closed_loop_ramp_ = closed_loop_ramp;
					outputShapingChanged_ = true;
				}
			}
			double getClosedloopRamp(void) const {return closed_loop_ramp_;}
			void setOpenloopRamp(double open_loop_ramp) {
				if (open_loop_ramp_ != open_loop_ramp) {
					open_loop_ramp_ = open_loop_ramp;
					outputShapingChanged_ = true;
				}
			}
			double getOpenloopRamp(void) const {return open_loop_ramp_;}

			void setPeakOutputForward(double peak_output_forward)
			{
				if (peak_output_forward != peak_output_forward_)
				{
					peak_output_forward_ = peak_output_forward;
					outputShapingChanged_ = true;
				}
			}
			double getPeakOutputForward(void) const {return peak_output_forward_;}

			void setPeakOutputReverse(double peak_output_reverse)
			{
				if (peak_output_reverse != peak_output_reverse_)
				{
					peak_output_reverse_ = peak_output_reverse;
					outputShapingChanged_ = true;
				}
			}
			double getPeakOutputReverse(void) const {return peak_output_reverse_;}

			void setNominalOutputForward(double nominal_output_forward)
			{
				if (nominal_output_forward != nominal_output_forward_)
				{
					nominal_output_forward_ = nominal_output_forward;
					outputShapingChanged_ = true;
				}
			}
			double getNominalOutputForward(void) const {return nominal_output_forward_;}

			void setNominalOutputReverse(double nominal_output_reverse)
			{
				if (nominal_output_reverse != nominal_output_reverse_)
				{
					nominal_output_reverse_ = nominal_output_reverse;
					outputShapingChanged_ = true;
				}
			}
			double getNominalOutputReverse(void) const {return nominal_output_reverse_;}

			void setNeutralDeadband(double neutral_deadband)
			{
				if (neutral_deadband != neutral_deadband_)
				{
					neutral_deadband_ = neutral_deadband;
					outputShapingChanged_ = true;
				}
			}
			double getNeutralDeadband(void) const {return neutral_deadband_;}

			void setVoltageCompensationSaturation(double voltage)
			{
				if (voltage != voltage_compensation_saturation_)
				{
					voltage_compensation_saturation_ = voltage;
					voltage_compensation_changed_ = true;
				}
			}
			double getVoltageCompensationSaturation(void) const { return voltage_compensation_saturation_;}

			void setVoltageMeasurementFilter(int filterWindowSamples)
			{
				if (filterWindowSamples != voltage_measurement_filter_)
				{
					voltage_measurement_filter_ = filterWindowSamples;
					voltage_compensation_changed_ = true;
				}
			}
			int getVoltageMeasurementFilter(void) const {return voltage_compensation_saturation_;}

			void setVoltageCompensationEnable(bool enable)
			{
				if (enable != voltage_compensation_enable_)
				{
					voltage_compensation_enable_ = enable;
					voltage_compensation_changed_ = true;
				}
			}

			bool getEnableVoltageCompenation(void) const {return voltage_compensation_enable_;} 

			bool VoltageCompensationChanged(double & voltage_compensation_saturation,
					int &voltage_measurement_filter,
					bool &voltage_compensation_enable)
			{
				voltage_compensation_saturation_ = voltage_compensation_saturation;
				voltage_measurement_filter_ = voltage_measurement_filter;
				voltage_compensation_enable_ = voltage_compensation_enable;
				if (voltage_compensation_changed_)
				{
					voltage_compensation_changed_ = false;
					return true;
				}
				return false;
			}

			// current limits
			void setPeakCurrentLimit(int amps)
			{
				current_limit_peak_amps_ = amps;
				current_limit_changed_ = true;
			}
			int getPeakCurrentLimit(void) const
			{
				return current_limit_peak_amps_;
			}

			void setPeakCurrentDuration(int msec)
			{
				current_limit_peak_msec_ = msec;
				current_limit_changed_ = true;
			}
			int getPeakCurrentDuration(void) const
			{
				return current_limit_peak_msec_;
			}
			void setContinuousCurrentLimit(int amps)
			{
				current_limit_continuous_amps_ = amps;
				current_limit_changed_ = true;
			}
			int getContinuousCurrentLimit(void) const
			{
				return current_limit_continuous_amps_;
			}
			void setCurrentLimitEnable(bool enable)
			{
				current_limit_enable_ = enable;
				current_limit_changed_ = true;
			}
			bool getCurrentLimitEnable(void) const
			{
				return current_limit_enable_;
			}

			bool currentLimitChanged(int &peak_amps, int &peak_msec, int &continuous_amps, bool &enable)
			{
				peak_amps = current_limit_peak_amps_;
				peak_msec = current_limit_peak_msec_;
				continuous_amps = current_limit_continuous_amps_;
				enable = current_limit_enable_;
				if (!current_limit_changed_)
					return false;
				current_limit_changed_ = true;
				return true;
			}

		private:
			double    command_; // motor setpoint - % vbus, velocity, position, etc
			bool      command_changed_;
			TalonMode mode_;         // talon mode - % vbus, close loop, motion profile, etc
			bool      mode_changed_; // set if mode needs to be updated on the talon hw
			//RG: shouldn't there be a variable for the peak voltage limits?
			int       pidf_slot_; // index 0 or 1 of the active PIDF slot
			bool      pidf_slot_changed_; // set to true to trigger a write to PIDF select on Talon
			double     iaccum_;
			bool      iaccum_changed_;

			bool      invert_;
			bool      sensor_phase_;
			bool      invert_changed_;

			NeutralMode neutral_mode_;
			bool        neutral_mode_changed_;
			bool        neutral_output_;

			FeedbackDevice encoder_feedback_;
			int encoder_tick_per_rotation_;

			//output shaping
			double closed_loop_ramp_;
			double open_loop_ramp_;
			double peak_output_forward_;
			double peak_output_reverse_;
			double nominal_output_forward_;
			double nominal_output_reverse_;
			double neutral_deadband_;
			bool outputShapingChanged_;

			double voltage_compensation_saturation_;
			int   voltage_measurement_filter_;
			bool  voltage_compensation_enable_;
			bool  voltage_compensation_changed_;

			int current_limit_peak_amps_;
			int current_limit_peak_msec_;
			int current_limit_continuous_amps_;
			bool current_limit_enable_;
			bool current_limit_changed_;

			// 2 entries in the Talon HW for each of these settings
			double p_[2];
			double i_[2];
			int    i_zone_[2];
			double d_[2];
			double f_[2];
			int    allowable_closed_loop_error_[2];
			double max_integral_accumulator_[2];
			bool   pidf_changed_[2];
	};

	// Handle - used by each controller to get, by name of the
	// corresponding joint, an interface with which to send commands
	// to a Talon
	class TalonCommandHandle: public TalonStateHandle
	{
		public:
			TalonCommandHandle(void) : 
				TalonStateHandle(), 
				cmd_(0)
			{
			}

			TalonCommandHandle(const TalonStateHandle &js, TalonHWCommand *cmd) : 
				TalonStateHandle(js), 
				cmd_(cmd)
			{
				if (!cmd_)
					throw HardwareInterfaceException("Cannot create Talon handle '" + js.getName() + "'. command pointer is null.");
			}

			// Operator which allows access to methods from
			// the TalonHWCommand member var associated with this
			// handle
			// Note that we could create separate methods in
			// the handle class for every method in the HWState
			// class, e.g.
			//     double getFoo(void) const {assert(_state); return state_->getFoo();}
			// but if each of them just pass things unchanged between
			// the calling code and the HWState method there's no
			// harm in making a single method to do so rather than
			// dozens of getFoo() one-line methods
			// 
			TalonHWCommand * operator->() {assert(cmd_); return cmd_;}

			// Get a pointer to the HW state associated with
			// this Talon.  Since CommandHandle is derived
			// from StateHandle, there's a state embedded
			// in each instance of a CommandHandle. Use
			// this method to access it.
			//
			// handle->state()->getCANID();
			//
			const TalonHWState * state(void) const { return TalonStateHandle::operator->(); }

		private:
			TalonHWCommand *cmd_;
	};

	// Use ClaimResources here since we only want 1 controller
	// to be able to access a given Talon at any particular time
	class TalonCommandInterface : public HardwareResourceManager<TalonCommandHandle, ClaimResources> {};
}
