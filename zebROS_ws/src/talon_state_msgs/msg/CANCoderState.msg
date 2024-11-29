Header    header
string[]  name
int8[]    device_number

string[]  sensor_direction
float64[] magnet_offset
float64[] absolute_sensor_discontinuity_point

float64[] conversion_factor

int8[]    version_major
int8[]    version_minor
int8[]    version_bugfix
int8[]    version_build
float64[] velocity
float64[] position
float64[] absolute_position
float64[] unfiltered_velocity
float64[] position_since_boot
float64[] supply_voltage
string[] magnet_health

bool[]    fault_hardware
bool[]    fault_undervolage
bool[]    fault_boot_during_enable
bool[]    fault_unlicensed_feature_in_use
bool[]    fault_bad_magnet
          
bool[]    sticky_fault_hardware
bool[]    sticky_fault_undervolage
bool[]    sticky_fault_boot_during_enable
bool[]    sticky_fault_unlicensed_feature_in_use
bool[]    sticky_fault_bad_magnet