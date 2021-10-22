#ifndef INC_JOYSTICK_INTERFACE_H_
#define INC_JOYSTICK_INTERFACE_H_

#include <string>
#include <vector>

#include <ros/console.h>

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>

namespace hardware_interface
{

class JoystickState
{
	public:
		JoystickState(const char *name, size_t id)
			: id_(id)
			, name_(name)
		{
		}
		JoystickState(const JoystickState &other)
			: id_(other.id_)
			, name_(other.name_)
			, axises_(other.axises_)
			, buttons_(other.buttons_)
			, povs_(other.povs_)
		{
		}

		const JoystickState &operator=(const JoystickState &other)
		{
			if (this != &other)
			{
				axises_  = other.axises_;
				buttons_ = other.buttons_;
				povs_    = other.povs_;
			}
			return *this;
		}

		size_t getId(void) const      { return id_; }

		void   clearAxises(void)      { axises_.clear();  }
		void   clearButtons(void)     { buttons_.clear(); }
		void   clearPOVs(void)        { povs_.clear();    }
		void   clear(void)            { clearAxises(); clearButtons(); clearPOVs(); }

		void   addAxis(float axis)    { axises_.push_back(axis);    }
		void   addButton(bool button) { buttons_.push_back(button); }
		void   addPOV(int pov)        { povs_.push_back(pov);       }

		bool setAxis(size_t index, float axis)
		{
			if (index >= axises_.size())
				ROS_ERROR_STREAM("Error setting axis for JoystickState "
						<< name_ << " : index out of bounds. Index = "
						<< index << " vector size = " << axises_.size());
				return false;
			axises_[index] = axis;
			return true;
		}
		bool setButton(size_t index, bool button)
		{
			if (index >= buttons_.size())
				ROS_ERROR_STREAM("Error setting button for JoystickState "
						<< name_ << " : index out of bounds. Index = "
						<< index << " vector size = " << buttons_.size());
				return false;
			buttons_[index] = button;
			return true;
		}

		bool setPOV(size_t index, int pov)
		{
			if (index >= povs_.size())
				ROS_ERROR_STREAM("Error setting pov for JoystickState "
						<< name_ << " : index out of bounds. Index = "
						<< index << " vector size = " << povs_.size());
				return false;
			povs_[index] = pov;
			return true;
		}

		const std::vector<float>& getAxises()  const { return axises_;  }
		const std::vector<bool>&  getButtons() const { return buttons_; }
		const std::vector<int>&   getPOVs()    const { return povs_;    }

		size_t getAxisCount(void)              const { return axises_.size();  };
		size_t getButtonCount(void)            const { return buttons_.size(); };
		size_t getPOVCount(void)               const { return povs_.size();    };

		// For these, don't flag an error, just return 0/false
		// That gives a reasonable default when an
		// axis / button / pov is missing
		bool getAxis(size_t index) const
		{
			if (index >= axises_.size())
				return 0.0;
			return axises_[index];
		}
		bool getButton(size_t index) const
		{
			if (index >= buttons_.size())
				return false;
			return buttons_[index];
		}
		int getPOV(size_t index) const
		{
			if (index >= povs_.size())
				return -1; // Invalid direction, should result in all directions == false
			return povs_[index];
		}

	private:
		const size_t       id_;
		const std::string  name_;
		std::vector<float> axises_;
		std::vector<bool>  buttons_;
		std::vector<int>   povs_;
};

typedef StateHandle<const JoystickState> JoystickStateHandle;
typedef StateHandle<JoystickState>       JoystickWritableStateHandle;

class JoystickStateInterface       : public HardwareResourceManager<JoystickStateHandle> {};
class RemoteJoystickIStatenterface : public HardwareResourceManager<JoystickWritableStateHandle, ClaimResources> {};

} // namespace hardware_interface
#endif
