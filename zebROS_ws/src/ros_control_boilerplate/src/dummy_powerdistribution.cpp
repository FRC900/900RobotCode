
// WPILIB HAL on the Rio provides functions to access the unique
// set of functions for each power distribution option.  WPI sim
// only simulates the common features described in PowerDistribution.cpp
// and doesn't even provide sim versions of the HAL_REV_*PDH* and HAL_CTRE_*PDP*
// functions. Implement those functions here, using PowerDistribution sim
// code where available and stubs for the remaining functions. This means
// we won't have a high-fidelity simulation but at least it will allow
// us to use common code between hw and sim interfaces
//
//

#include "hal/PowerDistribution.h"
#include "REVPDH.h"

extern "C"
{

HAL_REVPDHHandle HAL_InitializeREVPDH(int32_t module,
                                       const char* allocationLocation,
                                       int32_t* status)
{
	return HAL_InitializePowerDistribution(module, HAL_PowerDistributionType_kRev, allocationLocation, status);
}

int32_t HAL_GetREVPDHModuleNumber(HAL_REVPDHHandle handle, int32_t* status)
{
	return HAL_GetPowerDistributionModuleNumber(handle, status);
}

HAL_Bool HAL_CheckREVPDHModuleNumber(int32_t module)
{
	return HAL_CheckPowerDistributionModule(module, HAL_PowerDistributionType_kRev);
}

double HAL_GetREVPDHChannelCurrent(HAL_REVPDHHandle handle, int32_t channel,
                                    int32_t* status)
{
	return HAL_GetPowerDistributionChannelCurrent(handle, channel, status);
}

uint16_t HAL_GetREVPDHTotalCurrent(HAL_REVPDHHandle handle, int32_t* status)
{
	return HAL_GetPowerDistributionTotalCurrent(handle, status);
}

void HAL_SetREVPDHSwitchableChannel(HAL_REVPDHHandle handle, HAL_Bool enabled,
                                     int32_t* status)
{
	HAL_SetPowerDistributionSwitchableChannel(handle, enabled, status);
}

HAL_Bool HAL_GetREVPDHSwitchableChannelState(HAL_REVPDHHandle handle,
                                              int32_t* status)
{
	return HAL_GetPowerDistributionSwitchableChannel(handle, status);
}

double HAL_GetREVPDHVoltage(HAL_REVPDHHandle handle, int32_t* status)
{
	return HAL_GetPowerDistributionVoltage(handle, status);
}

void HAL_GetREVPDHFaults(HAL_REVPDHHandle handle,
                         HAL_PowerDistributionFaults* faults, int32_t* status)
{
	return HAL_GetPowerDistributionFaults(handle, faults, status);
}

void HAL_GetREVPDHStickyFaults(HAL_REVPDHHandle handle,
                         HAL_PowerDistributionStickyFaults* faults, int32_t* status)
{
	return HAL_GetPowerDistributionStickyFaults(handle, faults, status);
}

void HAL_GetREVPDHStickyFaults(HAL_REVPDHHandle handle,
                               HAL_PowerDistributionStickyFaults* stickyFaults,
                               int32_t* status);
#if 0
HAL_Bool HAL_REV_IsPDHEnabled(HAL_REVPDHHandle /*handle*/, int32_t* status)
{
	*status = 0;
	return true;
}
#endif

void HAL_GetREVPDHVersion(HAL_REVPDHHandle /*handle*/,
                          HAL_PowerDistributionVersion* version,
                          int32_t* status)
{
	*status = 0;
	version->firmwareMajor = 900;
	version->firmwareMinor = 900;
	version->firmwareFix = 900;
	version->hardwareMajor = 900;
	version->hardwareMinor = 900;
	version->uniqueId = 900;

}

void HAL_ClearREVPDHStickyFaults(HAL_REVPDHHandle handle, int32_t* status)
{
	HAL_ClearPowerDistributionStickyFaults(handle, status);
}

#if 0
void HAL_REV_IdentifyPDH(HAL_REVPDHHandle /*handle*/, int32_t* status)
{
	status = 0;
}
#endif

HAL_REVPDHHandle HAL_InitializePDP(int32_t module,
                                   const char* allocationLocation,
                                   int32_t* status)
{
	return HAL_InitializePowerDistribution(module, HAL_PowerDistributionType_kCTRE, allocationLocation, status);
}

HAL_Bool HAL_CheckPDPModule(int32_t module)
{
	return HAL_CheckPowerDistributionModule(module, HAL_PowerDistributionType_kCTRE);
}

double HAL_GetPDPTemperature(HAL_PDPHandle handle, int32_t* status)
{
	return HAL_GetPowerDistributionTemperature(handle, status);
}

double HAL_GetPDPVoltage(HAL_PDPHandle handle, int32_t* status)
{
	return HAL_GetPowerDistributionVoltage(handle, status);
}

double HAL_GetPDPChannelCurrent(HAL_PDPHandle handle, int32_t channel,
                                int32_t* status)
{
	return HAL_GetPowerDistributionChannelCurrent(handle, channel, status);
}

double HAL_GetPDPTotalCurrent(HAL_PDPHandle handle, int32_t* status)
{
	return HAL_GetPowerDistributionTotalCurrent(handle, status);
}

double HAL_GetPDPTotalPower(HAL_PDPHandle /*handle*/, int32_t* status)
{
	*status = 0;
	return 0;
}

double HAL_GetPDPTotalEnergy(HAL_PDPHandle /*handle*/, int32_t* status)
{
	*status = 0;
	return 0;
}

void HAL_ResetPDPTotalEnergy(HAL_PDPHandle /*handle*/, int32_t* status)
{
	*status = 0;
}

void HAL_ClearPDPStickyFaults(HAL_PDPHandle handle, int32_t* status)
{
	HAL_ClearPowerDistributionStickyFaults(handle, status);
}


} // extern "C"
