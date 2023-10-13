#ifndef RUMBLE_STATE_INTERFACE_INC_
#define RUMBLE_STATE_INTERFACE_INC_

#include <hardware_interface/internal/hardware_resource_manager.h>
#include "state_handle/command_handle.h"

namespace hardware_interface
{

class RumbleHWState
{
public:
	void setLeft(const unsigned int left) { left_ = left; }
	void setRight(const unsigned int right) { right_ = right; }
	unsigned int getLeft(void) const { return left_; }
	unsigned int getRight(void) const { return right_; }

private:
	unsigned int left_{0};
	unsigned int right_{0};
};

// Create a handle pointing to a type RumbleHWState / RumbleHWState pair
using RumbleStateHandle = StateHandle<const RumbleHWState>;
using RumbleWritableStateHandle =  StateHandle<RumbleHWState>;

class RumbleStateInterface : public HardwareResourceManager<RumbleStateHandle> {};
class RemoteRumbleStateInterface : public HardwareResourceManager<RumbleWritableStateHandle, ClaimResources> {};
} // namespace hardware_interface

#endif

