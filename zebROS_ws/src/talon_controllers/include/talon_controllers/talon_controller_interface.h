#ifndef INC_TALON_CONTROLLER_INTERFACE__
#define INC_TALON_CONTROLLER_INTERFACE__

#include <atomic>
#include <thread>

#include <dynamic_reconfigure/server.h>

#include "ctre_interfaces/talon_command_interface.h"

namespace XmlRpc
{
	class XmlRpcValue;
}

namespace talon_controllers
{

class TalonConfigConfig;
// Create a wrapper class for each Talon mode.  For the basic controllers
// this isn't really helpful since the code could just be included in the
// controller itself.  But consider a more complex controller, for example
// swerve. The swerve controller runs a number of wheels, and each wheel
// has both position and velocity.  The wheel class might create a
// TalonPosisionPIDControllerInterface member var for the position motor and
// also a TalonVelocityPIDControllerInterface member var for the velocity.
// And since it will be creating one per wheel, it makes sense to wrap the common
// init code into a class rather than duplicate it for each wheel. Another
// controller - say a shooter wheel - could also use this same code to
// create a talon handle to access that motor
//

// Class which provides a common set of code for reading
// parameters for motor controllers from yaml / command line
// ROS params.  Not all of these values will be needed for all
// modes - specific controller interfaces will use what
// they do need and ignore the rest.
// The idea here is that code using a particular CI would
// call readParams(), modify any parameters which are specific
// to the controller, and then call init using the specificed
// parameters. This will handle the common case where most
// code using the CI will want to use the default names of settings
// but also allow customization
class TalonCIParams
{
	public:
		// Initialize with relatively sane defaults
		// for all parameters
		TalonCIParams(void);

		// Update params set by a dynamic reconfig config
		// Also pass in current params for ones which aren't
		// dynamically reconfigurable - pass them through
		// to the new one
		TalonCIParams(const TalonCIParams &params, const TalonConfigConfig &config);

		// Copy from internal state to TalonConfigConfig state
		TalonConfigConfig toConfig(void) const;

		// Read a joint name from the given nodehandle's params
		bool readJointName(const ros::NodeHandle &n);

		bool readConversion(const ros::NodeHandle &n);

		bool readTalonFXSensorConfig(const ros::NodeHandle &n);

		// Read a joint name from the given nodehandle's params
		bool readNeutralMode(const ros::NodeHandle &n);

		bool readFeedbackType(const ros::NodeHandle &n);

		bool readInverts(const ros::NodeHandle &n);

		bool readCloseLoopParams(const ros::NodeHandle &n);

		bool readOutputShaping(const ros::NodeHandle &n);
		bool readVoltageCompensation(const ros::NodeHandle &n);

		bool readVelocitySignalConditioning(const ros::NodeHandle &n);

		bool readLimitSwitches(const ros::NodeHandle &n);

		bool readSoftLimits(const ros::NodeHandle &n);

		bool readCurrentLimits(const ros::NodeHandle &n);

		bool readSupplyCurrentLimits(const ros::NodeHandle &n);

		bool readStatorCurrentLimits(const ros::NodeHandle &n);

		bool readMotionControl(const ros::NodeHandle &n);

		bool readStatusFramePeriods(const ros::NodeHandle &n);

		bool readControlFramePeriods(const ros::NodeHandle &n);

		bool readTalonThread(const ros::NodeHandle &n);

		std::string joint_name_;
		std::array<double, hardware_interface::TALON_PIDF_SLOTS> p_;
		std::array<double, hardware_interface::TALON_PIDF_SLOTS> i_;
		std::array<double, hardware_interface::TALON_PIDF_SLOTS> d_;
		std::array<double, hardware_interface::TALON_PIDF_SLOTS> f_;
		std::array<double, hardware_interface::TALON_PIDF_SLOTS> izone_;
		std::array<double, hardware_interface::TALON_PIDF_SLOTS> allowable_closed_loop_error_;
		std::array<double, hardware_interface::TALON_PIDF_SLOTS> max_integral_accumulator_;
		std::array<double, hardware_interface::TALON_PIDF_SLOTS> closed_loop_peak_output_;
		std::array<int, hardware_interface::TALON_PIDF_SLOTS>    closed_loop_period_;
		int    pidf_slot_;
		bool   aux_pid_polarity_;
		hardware_interface::DemandType demand1_type_;
		double demand1_value_;
		bool   invert_output_;
		bool   sensor_phase_;
		hardware_interface::NeutralMode neutral_mode_;
		hardware_interface::FeedbackDevice feedback_type_;
		double feedback_coefficient_;
		hardware_interface::RemoteFeedbackDevice remote_feedback_type_;
		int    ticks_per_rotation_;
		std::array<int, 2>                                    remote_feedback_device_ids_;
		std::array<hardware_interface::RemoteSensorSource, 2> remote_feedback_filters_;
		std::array<hardware_interface::FeedbackDevice, hardware_interface::SensorTerm_Last> sensor_terms_;
		double closed_loop_ramp_;
		double open_loop_ramp_;
		double peak_output_forward_;
		double peak_output_reverse_;
		double nominal_output_forward_;
		double nominal_output_reverse_;
		double neutral_deadband_;
		double voltage_compensation_saturation_;
		int    voltage_measurement_filter_;
		bool   voltage_compensation_enable_;
		hardware_interface::VelocityMeasurementPeriod velocity_measurement_period_;
		int    velocity_measurement_window_;

		hardware_interface::LimitSwitchSource limit_switch_local_forward_source_;
		hardware_interface::LimitSwitchNormal limit_switch_local_forward_normal_;
		hardware_interface::LimitSwitchSource limit_switch_local_reverse_source_;
		hardware_interface::LimitSwitchNormal limit_switch_local_reverse_normal_;
		hardware_interface::RemoteLimitSwitchSource limit_switch_remote_forward_source_;
		hardware_interface::LimitSwitchNormal limit_switch_remote_forward_normal_;
		unsigned int                          limit_switch_remote_forward_id_;
		hardware_interface::RemoteLimitSwitchSource limit_switch_remote_reverse_source_;
		hardware_interface::LimitSwitchNormal limit_switch_remote_reverse_normal_;
		unsigned int                          limit_switch_remote_reverse_id_;
		bool                                  clear_position_on_limit_f_{false};
		bool                                  clear_position_on_limit_r_{false};

		double softlimit_forward_threshold_;
		bool   softlimit_forward_enable_;
		double softlimit_reverse_threshold_;
		bool   softlimit_reverse_enable_;
		bool   override_limit_switches_enable_;
		int    current_limit_peak_amps_;
		int    current_limit_peak_msec_;
		int    current_limit_continuous_amps_;
		bool   current_limit_enable_;

		// TalonFX / Falcon500 only
		double supply_current_limit_;
		double supply_current_trigger_threshold_current_;
		double supply_current_trigger_threshold_time_;
		bool   supply_current_limit_enable_;

		double stator_current_limit_;
		double stator_current_trigger_threshold_current_;
		double stator_current_trigger_threshold_time_;
		bool   stator_current_limit_enable_;

		double motion_cruise_velocity_;
		double motion_acceleration_;
		int    motion_s_curve_strength_;
		int    motion_profile_trajectory_period_;
		std::array<int, hardware_interface::Status_Last> status_frame_periods_;
		std::array<int, hardware_interface::Control_Last> control_frame_periods_;

		double conversion_factor_;

		// TalonFX / Falcon500 specific
		hardware_interface::MotorCommutation motor_commutation_;
		hardware_interface::AbsoluteSensorRange absolute_sensor_range_;
		hardware_interface::SensorInitializationStrategy sensor_initialization_strategy_;

		bool enable_read_thread_;

	private:
		// Read a double named <param_type> from the array/map
		// in params
		bool findFloatParam(const std::string &param_name, XmlRpc::XmlRpcValue &params, double &val) const;

		// Read an integer named <param_name> from the array/map
		// in params
		bool findIntParam(const std::string &param_name, XmlRpc::XmlRpcValue &params, int &val) const;

		bool stringToLimitSwitchSource(const std::string &str,
									   hardware_interface::LimitSwitchSource &limit_switch_source) const;
		bool stringToRemoteLimitSwitchSource(const std::string &str,
											 hardware_interface::RemoteLimitSwitchSource &limit_switch_source) const;
		bool stringToLimitSwitchNormal(const std::string &str,
									   hardware_interface::LimitSwitchNormal &limit_switch_normal) const;

		bool stringToFeedbackDevice(const std::string &str,
				hardware_interface::FeedbackDevice &feedback_device) const;

		bool stringToRemoteSensorSource(const std::string &str,
				hardware_interface::RemoteSensorSource &source) const;
};

// Base class for controller interface types. This class
// will be the least restrictive - allow users to swtich modes,
// reprogram any config values, and so on.
// Derived classes will be more specialized - they'll only allow
// a specific Talon mode and disable code which doesn't apply
// to that mode
class TalonControllerInterface
{
	public:
		TalonControllerInterface(void);
		TalonControllerInterface(const TalonControllerInterface &other) = delete;
		TalonControllerInterface(TalonControllerInterface &&other) noexcept = delete;

		virtual ~TalonControllerInterface();

		TalonControllerInterface& operator=(const TalonControllerInterface &other) = delete;
		TalonControllerInterface& operator=(TalonControllerInterface &&other) noexcept = delete;

		// Standardize format for reading params for
		// motor controller
		virtual bool readParams(ros::NodeHandle &n, TalonCIParams &params);

		// Read params from config file and use them to
		// initialize the Talon hardware
		virtual bool initWithNode(hardware_interface::TalonCommandInterface *tci,
								  hardware_interface::TalonStateInterface * /*tsi*/,
								  ros::NodeHandle &n);

		// Same as above, except pass in an array
		// of node handles. First entry is set up as the leader
		// talon and the rest are set in follower mode to follow
		// the leader
		virtual bool initWithNode(hardware_interface::TalonCommandInterface *tci,
								  hardware_interface::TalonStateInterface *tsi,
								  std::vector<ros::NodeHandle> &n);

		// Init with XmlRpcValue instead of NodeHandle - XmlRpcValue
		// will be either a string or an array of strings of joints
		// to load
		virtual bool initWithNode(hardware_interface::TalonCommandInterface *tci,
								  hardware_interface::TalonStateInterface *tsi,
								  ros::NodeHandle &controller_nh,
								  XmlRpc::XmlRpcValue param);

		// Single-motor init using a named talon
		virtual bool initWithNode(hardware_interface::TalonCommandInterface *tci,
								  hardware_interface::TalonStateInterface *tsi,
								  ros::NodeHandle &controller_nh,
								  const std::string &talon_name);
		virtual bool initWithNode(hardware_interface::TalonCommandInterface *tci,
								  hardware_interface::TalonStateInterface *tsi,
								  ros::NodeHandle &controller_nh,
								  const char *talon_name);
#if 0
		// Allow users of the ControllerInterface to get
		// a copy of the parameters currently set for the
		// Talon.  They can then modify them at will and
		// call initWithParams to reprogram the Talon.
		// Hopefully this won't be needed all that often ...
		// the goal should be to provide a simpler interface
		// for commonly used operations
		virtual TalonCIParams getParams(void) const;

		// Initialize Talon hardware with the settings in params
		virtual bool initWithParams(hardware_interface::TalonCommandInterface *tci,
									const TalonCIParams &params);
#endif

		void callback(talon_controllers::TalonConfigConfig &config, uint32_t /*level*/);

		// Set the setpoint for the motor controller
		virtual void setCommand(const double command);

		// Set the mode of the motor controller
		virtual void setMode(hardware_interface::TalonMode mode);

		// Set the mode of the motor controller
		hardware_interface::TalonMode getMode(void) const;


		// Set the mode of the motor controller
		void setNeutralMode(hardware_interface::NeutralMode neutral_mode);

		// Pick the config slot (0 or 1) for PIDF/IZone values
		virtual bool setPIDFSlot(int slot);

		virtual bool setP(double newP, int slot);

		virtual void setIntegralAccumulator(double iaccum);

		virtual void setOverrideSoftLimitsEnable(bool enable);

		virtual void setPeakCurrentLimit(int amps);

		virtual void setPeakCurrentDuration(int msec);

		virtual void setContinuousCurrentLimit(int amps);

		virtual void setCurrentLimitEnable(bool enable);

		virtual void setForwardSoftLimitThreshold(double threshold);

		virtual void setForwardSoftLimitEnable(bool enable);

		virtual void setReverseSoftLimitThreshold(double threshold);

		virtual void setReverseSoftLimitEnable(bool enable);

		virtual void setSelectedSensorPosition(double position);

		virtual void clearStickyFaults(void);

		virtual void setMotionCruiseVelocity(double velocity);

		virtual void setMotionAcceleration(double acceleration);

		virtual void setMotionSCurveStrength(unsigned int s_curve_strength);

		virtual double getMotionCruiseVelocity(void);

		virtual double getMotionAcceleration(void);

		virtual double getMotionSCurveStrength(void);

		virtual void setStatusFramePeriod(hardware_interface::StatusFrame status_frame, uint8_t period);

		virtual void setControlFramePeriod(hardware_interface::ControlFrame control_frame, uint8_t period);

		virtual void setMotionProfileTrajectoryPeriod(int msec);

		virtual void clearMotionProfileTrajectories(void);

		virtual void clearMotionProfileHasUnderrun(void);

		virtual void pushMotionProfileTrajectory(const hardware_interface::TrajectoryPoint &traj_pt);

		double getPosition(void) const;

		double getSpeed(void) const;

		double getOutputCurrent(void) const;

		double getStatorCurrent(void) const;

		double getSupplyCurrent(void) const;

		bool getForwardLimitSwitch(void) const;
		bool getReverseLimitSwitch(void) const;

		void setPeakOutputForward(double peak);

		void setPeakOutputReverse(double peak);

		void setClosedloopRamp(double closed_loop_ramp);

		void setOpenloopRamp(double open_loop_ramp);

		void setDemand1Type(hardware_interface::DemandType demand1_type);

		void setDemand1Value(double demand1_value);

	protected:
		TalonCIParams                                        params_;
		hardware_interface::TalonCommandHandle               talon_;
		std::shared_ptr<boost::recursive_mutex>              srv_mutex_;

		// Variables for a thread which updates the dynamic reconfigure server
		// when vars are updated from calls to the interafce. This keeps the
		// values in the dynamic reconfigure GUI in sync with the values set
		// via functions called in this interface
		std::atomic_flag                                     srv_update_thread_flag_;
		std::atomic<bool>                                    srv_update_thread_active_;
		std::thread                                          srv_update_thread_;

		// List of follower talons associated with the master
		// listed above
		std::vector<hardware_interface::TalonCommandHandle>  follower_talons_;

		// Used to set initial (and only) talon
		// mode for FixedMode derived classes
		virtual bool setInitialMode(void);

	private :
		virtual bool init(hardware_interface::TalonCommandInterface *tci,
							ros::NodeHandle &n,
							hardware_interface::TalonCommandHandle &talon,
							std::shared_ptr<boost::recursive_mutex> srv_mutex,
							bool follower);

		// If dynamic reconfigure is running then update
		// the reported config there with the new internal state
		// Trigger a write of the current CIParams to the DDR server. This needs
		// to happen if the values have been updated via the interface code.
		// The meaning of the flag - set == no new updates, cleared == data has
		// been updated via code and the reconfigure gui needs that new data sent
		// to it to stay in sync
		// This call is on the control loop update path and needs to be quick.
		// atomic_flags are lock free, so clearing it should be a fast operation
		// which never blocks.  Thus, with any luck it won't slow down the
		// control loop by any significant amount.
		void syncDynamicReconfigure();

		// Loop forever, waiting for requests from the main thread
		// to update values from this class to the DDR server.
		void srvUpdateThread(std::shared_ptr<dynamic_reconfigure::Server<TalonConfigConfig>> srv);

		// Use data in params to actually set up Talon
		// hardware. Make this a separate method outside of
		// init() so that dynamic reconfigure callback can write
		// values using this method at any time
		// Technically, this writes to the talon command buffers
		// (via the talon_ handle), which doesn't actually write
		// to hardware. But writing to those buffers queues up an
		// actual write to the hardware in write() in the hardware
		// interface
		virtual void writeParamsToHW(const TalonCIParams &params,
									 hardware_interface::TalonCommandHandle &talon,
									 bool update_params = true);
		static constexpr double double_value_epsilon = 0.0001;
};

// A derived class which disables mode switching. Any
// single-mode CI class should derive from this class
template<typename hardware_interface::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
class TalonFixedModeControllerInterface : public TalonControllerInterface
{
	public:
		TalonFixedModeControllerInterface(void) : TalonControllerInterface() {}
		TalonFixedModeControllerInterface(const TalonFixedModeControllerInterface &other) = delete;
		TalonFixedModeControllerInterface(TalonFixedModeControllerInterface &&other) noexcept = delete;

		~TalonFixedModeControllerInterface() override;

		TalonFixedModeControllerInterface& operator=(const TalonFixedModeControllerInterface &other) = delete;
		TalonFixedModeControllerInterface& operator=(TalonFixedModeControllerInterface &&other) noexcept = delete;
	protected:
		// Disable changing mode for controllers derived from this class
		void setMode(hardware_interface::TalonMode /*mode*/) override;
		bool setInitialMode(void) override;
};

extern const char PERCENT_OUTPUT_NAME[];
extern const char PERCENT_OUTPUT_NAME[];
extern const char CLOSE_LOOP_POSITION_NAME[];
extern const char CLOSE_LOOP_VELOCITY_NAME[];
extern const char CLOSE_LOOP_CURRENT_NAME[];
extern const char MOTION_PROFILE_NAME[];
extern const char MOTION_MAGIC_NAME[];

using TalonPercentOutputControllerInterface = TalonFixedModeControllerInterface<hardware_interface::TalonMode::TalonMode_PercentOutput, PERCENT_OUTPUT_NAME>;

class TalonFollowerControllerInterface : public TalonControllerInterface
{
	public:
		TalonFollowerControllerInterface(void) : TalonControllerInterface() {}
		TalonFollowerControllerInterface(const TalonFollowerControllerInterface &other) = delete;
		TalonFollowerControllerInterface(TalonFollowerControllerInterface &&other) noexcept = delete;

		~TalonFollowerControllerInterface() override;

		TalonFollowerControllerInterface& operator=(const TalonFollowerControllerInterface &other) = delete;
		TalonFollowerControllerInterface& operator=(TalonFollowerControllerInterface &&other) noexcept = delete;

		bool initWithNode(hardware_interface::TalonCommandInterface *tci,
						  hardware_interface::TalonStateInterface   *tsi,
						  ros::NodeHandle &n) override;

		// Disable changing mode for controllers derived from this class
		void setMode(hardware_interface::TalonMode /*mode*/) override;

		// Maybe disable the setPIDFSlot call since that makes
		// no sense for a non-PID controller mode?
		void setCommand(const double /*command*/) override;
};

using TalonPositionCloseLoopControllerInterface = TalonFixedModeControllerInterface<hardware_interface::TalonMode::TalonMode_Position, CLOSE_LOOP_POSITION_NAME>;
using TalonVelocityCloseLoopControllerInterface = TalonFixedModeControllerInterface<hardware_interface::TalonMode::TalonMode_Velocity, CLOSE_LOOP_VELOCITY_NAME>;
using TalonCurrentCloseLoopControllerInterface = TalonFixedModeControllerInterface<hardware_interface::TalonMode::TalonMode_Current, CLOSE_LOOP_CURRENT_NAME>;
using TalonMotionProfileControllerInterface = TalonFixedModeControllerInterface<hardware_interface::TalonMode::TalonMode_MotionProfile, MOTION_PROFILE_NAME>;
using TalonMotionMagicCloseLoopControllerInterface = TalonFixedModeControllerInterface<hardware_interface::TalonMode::TalonMode_MotionMagic, MOTION_MAGIC_NAME>;

} // namespace

#endif