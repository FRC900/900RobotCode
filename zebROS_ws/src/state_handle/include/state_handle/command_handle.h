#pragma once

#include "state_handle/state_handle.h"

// Handle - used by each controller to get, by name of the
// corresponding joint, an interface with which to send commands
// to hardware
namespace hardware_interface
{
template <class HWCommand, class HWState, class StateHandle>
class CommandHandle: public StateHandle
{
	public:
		CommandHandle(void) :
			StateHandle(),
			cmd_(0)
		{
		}

		CommandHandle(const StateHandle &js, HWCommand *cmd) :
			StateHandle(js),
			cmd_(cmd)
		{
			if (!cmd_)
				throw HardwareInterfaceException("Cannot create CommandHandle '" + js.getName() + "'. command pointer is null.");
		}

		// Operator which allows access to methods from
		// the HWCommand member var associated with this
		// handle
		// Note that we could create separate methods in
		// the handle class for every method in the HWCommand
		// class, e.g.
		//     double getFoo(void) const {assert(_state); return state_->getFoo();}
		// but if each of them just pass things unchanged between
		// the calling code and the HWCommand method there's no
		// harm in making a single method to do so rather than
		// dozens of getFoo() one-line methods
		//
		HWCommand *operator->()
		{
			assert(cmd_);
			return cmd_;
		}

		// Get a pointer to the HW state associated with
		// this command.  Since CommandHandle is derived
		// from StateHandle, there's a state embedded
		// in each instance of a CommandHandle. Use
		// this method to access it.
		//
		// handle->state()->getCANID();
		//
		const HWState *state(void) const
		{
			return StateHandle::operator->();
		}

	private:
		HWCommand *cmd_;
};

} // namespace
