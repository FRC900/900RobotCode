Header header

string eventName
int8 matchType
int8 matchNumber
int8 replayNumber

float64 matchTimeRemaining
int8 ALLIANCE_COLOR_UNKNOWN = -1
int8 ALLIANCE_COLOR_RED = 0
int8 ALLIANCE_COLOR_BLUE = 1
int8 allianceColor
int8 driverStationLocation
uint8[] gameSpecificData

bool Enabled
bool Disabled
bool Autonomous
bool Test
bool FMSAttached
bool DSAttached
bool OperatorControl
bool EStopped

float64 BatteryVoltage

string getMatchTimeStatus
string getAllianceStationStatus
string getVinVoltageStatus
