#ifndef CAN_BUS_STATUS_INTERFACE_INC__
#define CAN_BUS_STATUS_INTERFACE_INC__

#include <string>
#include <cstdint>

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>

namespace hardware_interface::can_bus_status
{

//holds match data
class CANBusStatusHWState
{
public:
	explicit CANBusStatusHWState(const std::string &name)
		: name_(name)
	{
	}
	CANBusStatusHWState(const CANBusStatusHWState &other) = delete;
	CANBusStatusHWState(CANBusStatusHWState &&other) noexcept = delete;
	~CANBusStatusHWState() = default;

	CANBusStatusHWState &operator=(const CANBusStatusHWState &other) = delete;
	CANBusStatusHWState &operator=(CANBusStatusHWState &&) = delete;

	const std::string &getName(void) const {return name_; }

	void setBusUtilization(const float bus_utilization) { bus_utilization_ = bus_utilization; }
	float getBusUtilization(void) const { return bus_utilization_; }

	void setBusOffCount(const uint32_t bus_off_count) { bus_off_count_ = bus_off_count; }
	uint32_t getBusOffCount(void) const { return bus_off_count_; }

	void setTXFullCount(const uint32_t tx_full_count) { tx_full_count_ = tx_full_count; }
	uint32_t getTXFullCount(void) const { return tx_full_count_; }

	void setREC(const uint32_t rec) { rec_ = rec; }	
	uint32_t getREC(void) const { return rec_;}

	void setTEC(const uint32_t tec) { tec_ = tec; }	
	uint32_t getTEC(void) const { return tec_;}

	void setIsNetworkFD(const bool is_network_fd) { is_network_fd_ = is_network_fd; }
	bool getIsNetworkFD(void) const { return is_network_fd_; }

private:
    const std::string name_;
    float bus_utilization_{0};
    uint32_t bus_off_count_{0};
    uint32_t tx_full_count_{0};
    uint32_t rec_{0};
    uint32_t tec_{0};
    bool     is_network_fd_{false};
};

using CANBusStatusStateHandle = StateHandle<const CANBusStatusHWState>;
using CANBusStatusStateWritableHandle = StateHandle<CANBusStatusHWState>;

class CANBusStatusStateInterface       : public HardwareResourceManager<CANBusStatusStateHandle> {};
class RemoteCANBusStatusStateInterface : public HardwareResourceManager<CANBusStatusStateWritableHandle, ClaimResources> {};

}
#endif
