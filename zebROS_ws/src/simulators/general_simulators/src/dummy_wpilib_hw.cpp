// Functions referenced by various WPILIB code used on non-Rio targets but not actually
// called. These functions let us build wpilib-dependent code on x86 and Jetson targets
// by stubbing out functions which aren't actually used
#include <hal/Types.h>
#include <HALInitializer.h>
extern "C" {

double HAL_GetVinVoltage(int32_t* status)
{
	//ROS_ERROR("Called HAL_GetVinVoltage() on unsupported platform");
	*status = 0;
	return 12.; /* Just to be safe, fake a reasonable voltage */
}

} /// extern "C"
