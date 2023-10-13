#ifndef RUMBLE_COMMAND_INTERFACE_INC_
#define RUMBLE_COMMAND_INTERFACE_INC_

#include "frc_interfaces/rumble_state_interface.h"
#include "state_handle/command_handle.h"

namespace hardware_interface
{

class RumbleHWCommand
{
public:
	void setLeft(const unsigned int left)
	{
		if (left != left_)
		{
			left_ = left;
			changed_ = true;
		}
	}
	void setRight(const unsigned int right)
	{
		if (right != right_)
		{
			right_ = right;
			changed_ = true;
		}
	}
	unsigned int getLeft(void) const { return left_; }
	unsigned int getRight(void) const { return right_; }

	bool changed(unsigned int &left, unsigned int &right)
	{
		left = left_;
		right = right_;
		const bool ret = changed_;
		changed_ = false;
		return ret;
	}

	void reset(void)
	{
		changed_ = true;
	}

private:
	unsigned int left_{0};
	unsigned int right_{0};
	unsigned int changed_{true};
};

// Create a handle pointing to a type RumbleHWCommand / RumbleHWState pair
using RumbleCommandHandle = CommandHandle<RumbleHWCommand, RumbleHWState, RumbleStateHandle>;

// Use ClaimResources here since we only want 1 controller
// to be able to access a given Rumble at any particular time
class RumbleCommandInterface : public HardwareResourceManager<RumbleCommandHandle, ClaimResources> {};

} // namespace hardware_interface

#endif

