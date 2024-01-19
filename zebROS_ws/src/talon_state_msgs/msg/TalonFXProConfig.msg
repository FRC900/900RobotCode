Header header

string[]  name
uint8[]   can_id
float64[] kp0
float64[] ki0
float64[] kd0
float64[] ks0
float64[] kv0
float64[] ka0
float64[] kg0
string[]  gravity_type0
float64[] kp1
float64[] ki1
float64[] kd1
float64[] ks1
float64[] kv1
float64[] ka1
float64[] kg1
string[]  gravity_type1
float64[] kp2
float64[] ki2
float64[] kd2
float64[] ks2
float64[] kv2
float64[] ka2
float64[] kg2
string[]  gravity_type2
string[]  invert
string[]  neutral_mode
float64[] duty_cycle_neutral_deadband
float64[] peak_forward_duty_cycle
float64[] peak_reverse_duty_cycle

float64[] stator_current_limit
bool[]    stator_current_limit_enable

float64[] supply_current_limit
bool[]    supply_current_limit_enable
float64[] supply_current_threshold
float64[] supply_time_threshold

float64[] supply_voltage_time_constant
float64[] peak_forward_voltage
float64[] peak_reverse_voltage

float64[] peak_forward_torque_current
float64[] peak_reverse_torque_current
float64[] torque_neutral_deadband

float64[] feedback_rotor_offset
float64[] sensor_to_mechanism_ratio
float64[] rotor_to_sensor_ratio
string[]  feedback_sensor_source
uint8[]   feedback_remote_sensor_id

string[]  differential_sensor_source
int8[]    differential_talonfx_sensor_id
int8[]    differential_remote_sensor_id

float64[] peak_differential_duty_cycle
float64[] peak_differential_voltage
float64[] peak_differential_torque_current

float64[] duty_cycle_open_loop_ramp_period
float64[] voltage_open_loop_ramp_period
float64[] torque_open_loop_ramp_period

float64[] duty_cycle_closed_loop_ramp_period
float64[] voltage_closed_loop_ramp_period
float64[] torque_closed_loop_ramp_period

string[]  forward_limit_type
bool[]    forward_limit_autoset_position_enable
float64[] forward_limit_autoset_position_value
bool[]    forward_limit_enable
string[]  forward_limit_source
uint8[]   forward_limit_remote_sensor_id

string[]  reverse_limit_type
bool[]    reverse_limit_autoset_position_enable
float64[] reverse_limit_autoset_position_value
bool[]    reverse_limit_enable
string[]  reverse_limit_source
uint8[]   reverse_limit_remote_sensor_id

bool[]    beep_on_boot

bool[]    softlimit_forward_enable
bool[]    softlimit_reverse_enable
float64[] softlimit_forward_threshold
float64[] softlimit_reverse_threshold

float64[] motion_magic_cruise_velocity
float64[] motion_magic_acceleration
float64[] motion_magic_jerk
float64[] motion_magic_expo_kV
float64[] motion_magic_expo_kA

bool[]    continuous_wrap
bool[]    enable_read_thread
