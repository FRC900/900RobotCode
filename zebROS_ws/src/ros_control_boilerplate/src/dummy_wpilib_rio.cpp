#include "hal/DriverStationTypes.h"
// Used on non-Rio targets to keep robot status in sync
// with actual DS control word. Happens automatically for
// the Rio, so this is a no-op
extern "C"
{
void HALSIM_SetControlWord(HAL_ControlWord /*controlword*/)
{
}

}
#include <string>
void HAL_SetCANBusString(const std::string &/*bus*/) { }

