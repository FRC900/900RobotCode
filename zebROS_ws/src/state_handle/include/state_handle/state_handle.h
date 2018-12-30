#pragma once

#include <hardware_interface/hardware_interface.h>
#include <string>

// Handle - used by each controller to get, by name of the
// corresponding joint, an interface with which to get state
// info about a hardware type
namespace hardware_interface
{
template <class T>
class StateHandle
{
	public:
		StateHandle(void) :
			state_(nullptr)
		{}

		// Initialize the base TalonStateHandle with pointers
		// from the state data object.
		StateHandle(const std::string &name, T *state) :
			name_(name),
			state_(state)
		{
			if (!state)
				throw HardwareInterfaceException("Cannot create state handle '" + name + "'. state pointer is null.");
		}
		std::string getName(void) const
		{
			return name_;
		}

		// Operator which allows access to methods from
		// the <class T> member var associated with this
		// handle
		// Note that we could create separate methods in
		// the handle class for every method in the HWState
		// class, e.g.
		//     double getFoo(void) const {assert(_state); return state_->getFoo();}
		// but if each of them just pass things unchanged between
		// the calling code and the HWState method there's no
		// harm in making a single method to do so rather than
		// dozens of getFoo() one-line methods
		T *operator->() const
		{
			assert(state_);
			return state_;
		}

	private:
		std::string  name_;
		T           *state_;
};

}
