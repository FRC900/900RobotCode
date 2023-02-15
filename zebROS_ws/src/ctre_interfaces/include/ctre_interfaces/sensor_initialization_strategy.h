#pragma once

namespace hardware_interface
{
enum SensorInitializationStrategy
{
	/**
	 * On boot up, set position to zero.
	 */
	BootToZero,
	/**
	 * On boot up, sync to the Absolute Position signal.  The Absolute position signal will be signed according to the selected Absolute Sensor Range.
	 */
	BootToAbsolutePosition,
};
}
