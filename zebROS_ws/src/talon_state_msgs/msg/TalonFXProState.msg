Header    header

string[]  name
string[]  control_mode
float64[] control_output
float64[] control_position
float64[] control_velocity
float64[] control_acceleration
float64[] control_jerk
bool[]    control_enable_foc
bool[]    control_override_brake_dur_neutral
float64[] control_max_abs_duty_cycle
float64[] control_deadband
float64[] control_feedforward
int8[]    control_slot
float64[] control_differential_position
int8[]    control_differential_slot
uint8[]   version_major
uint8[]   version_minor
uint8[]   version_bugfix
uint8[]   version_build
bool[]    forward_limit
bool[]    reverse_limit
string[]  applied_rotor_polarity

float64[] duty_cycle
float64[] torque_current
float64[] stator_current
float64[] supply_current
float64[] supply_voltage
float64[] device_temp
float64[] processor_temp

float64[] rotor_velocity
float64[] rotor_position
float64[] velocity
float64[] position

bool[]    motion_magic_is_running

bool[]    device_enable

string[]  differential_control_mode
float64[] differential_average_velocity
float64[] differential_average_position
float64[] differential_difference_velocity
float64[] differential_difference_position

string[]  bridge_output_value

bool[]    fault_hardware
bool[]    fault_proctemp
bool[]    fault_devicetemp
bool[]    fault_undervoltage
bool[]    fault_bootduringenable
bool[]    fault_bridgebrownout
bool[]    fault_unlicensedfeatureinuse
bool[]    fault_remotesensorreset
bool[]    fault_missingdifferentialfx
bool[]    fault_remotesensorposoverflow
bool[]    fault_oversupplyv
bool[]    fault_unstablesupplyv
bool[]    fault_reversehardlimit
bool[]    fault_forwardhardlimit
bool[]    fault_reversesoftlimit
bool[]    fault_forwardsoftlimit
bool[]    fault_remotesensordatainvalid
bool[]    fault_fusedsensoroutofsync
bool[]    fault_statorcurrlimit
bool[]    fault_supplycurrlimit
         
bool[]    sticky_fault_hardware
bool[]    sticky_fault_proctemp
bool[]    sticky_fault_devicetemp
bool[]    sticky_fault_undervoltage
bool[]    sticky_fault_bootduringenable
bool[]    sticky_fault_bridgebrownout
bool[]    sticky_fault_unlicensedfeatureinuse
bool[]    sticky_fault_remotesensorreset
bool[]    sticky_fault_missingdifferentialfx
bool[]    sticky_fault_remotesensorposoverflow
bool[]    sticky_fault_oversupplyv
bool[]    sticky_fault_unstablesupplyv
bool[]    sticky_fault_reversehardlimit
bool[]    sticky_fault_forwardhardlimit
bool[]    sticky_fault_reversesoftlimit
bool[]    sticky_fault_forwardsoftlimit
bool[]    sticky_fault_remotesensordatainvalid
bool[]    sticky_fault_fusedsensoroutofsync
bool[]    sticky_fault_statorcurrlimit
bool[]    sticky_fault_supplycurrlimit

float64[] closed_loop_proportional_output
float64[] closed_loop_integrated_output
float64[] closed_loop_feed_forward
float64[] closed_loop_derivative_output
float64[] closed_loop_output
float64[] closed_loop_reference
float64[] closed_loop_reference_slope
float64[] closed_loop_error

float64[] differential_output
float64[] differential_closed_loop_proportional_output
float64[] differential_closed_loop_integrated_output
float64[] differential_closed_loop_feed_forward
float64[] differential_closed_loop_derivative_output
float64[] differential_closed_loop_output
float64[] differential_closed_loop_reference
float64[] differential_closed_loop_reference_slope
float64[] differential_closed_loop_error