#pragma once

namespace hardware_interface
{
enum AbsoluteSensorRange
{
	/**
	 * Express the absolute position as an unsigned value.
	 * E.g. [0,+1) rotations or [0,360) deg.
	 */
	Unsigned_0_to_360,
	/**
	 * Express the absolute position as an signed value.
	 * E.g. [-0.5,+0.5) rotations or [-180,+180) deg.
	 */
	Signed_PlusMinus180,
};
}
