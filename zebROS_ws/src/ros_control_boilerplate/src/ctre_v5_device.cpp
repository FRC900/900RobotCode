#include "ros/console.h"
#include "ros/node_handle.h"
#include "hal/DriverStation.h"

#include "ros_control_boilerplate/ctre_v5_device.h"

static std::optional<std::string> phoenixErrorCodeToString(const ctre::phoenix::ErrorCode error_code);

CTREV5Device::CTREV5Device(const std::string &name_space,
                           const std::string &device_type,
                           const std::string &joint_name,
                           const int id)
    : device_type_{device_type}
    , name_{joint_name}
    , id_{id}
{
    ros::NodeHandle root_nh(name_space);
    ros::NodeHandle param_nh(root_nh, "generic_hw_control_loop"); // TODO : this shouldn't be hard-coded?
    int can_config_count_limit = can_config_count_limit_;
    if (!param_nh.param("talon_config_count_limit", can_config_count_limit, can_config_count_limit))
    {
        ROS_ERROR("Failed to read talon_config_count_limit in frc_robot_interface");
    }
    else
    {
        can_config_count_limit_ = can_config_count_limit;
    }
}

CTREV5Device::~CTREV5Device() = default;

bool CTREV5Device::safeCall(ctre::phoenix::ErrorCode error_code, const std::string &method_name)
{
    const auto error_string = phoenixErrorCodeToString(error_code);
    if (!error_string)
    {
        can_error_count_ = 0;
        can_error_sent_ = false;
        return true;
    }

	ROS_ERROR_STREAM("Error : " << device_type_ << " name = " << name_ << " id = " << id_ << " calling " << method_name << " : " << *error_string);
	can_error_count_++;
	if ((can_error_count_ > 1000) && !can_error_sent_)
	{
		HAL_SendError(true, -1, false, "CTREV5 safeCall - too many CAN bus errors!", "", "", true);
        can_error_count_ = 0;
		can_error_sent_ = true;
	}

    return false;
}

bool CTREV5Device::safeConfigCall(ctre::phoenix::ErrorCode error_code, const std::string &method_name)
{
	can_config_count_ += 1;
	if (can_config_count_ > can_config_count_limit_)
	{
		ROS_INFO_STREAM(device_type_ << " config call count for " << name_ << " this iteration reached on function " << method_name);
		return false;
	}
	return safeCall(error_code, method_name);
}

void CTREV5Device::resetCanConfigCount(void)
{
    can_config_count_ = 0;
}

std::string CTREV5Device::getName(void) const
{
    return name_;
}

int CTREV5Device::getId(void) const
{
    return id_;
}

static std::optional<std::string> phoenixErrorCodeToString(const ctre::phoenix::ErrorCode error_code)
{
    switch (error_code)
    {
    case ctre::phoenix::OK:
        return std::nullopt;

    case ctre::phoenix::CAN_MSG_STALE:
        return std::string("CAN_MSG_STALE/CAN_TX_FULL/TxFailed");

    case ctre::phoenix::InvalidParamValue:
        return std::string("InvalidParamValue/CAN_INVALID_PARAM");

    case ctre::phoenix::RxTimeout:
        return std::string("RxTimeout/CAN_MSG_NOT_FOUND");

    case ctre::phoenix::TxTimeout:
        return std::string("TxTimeout/CAN_NO_MORE_TX_JOBS");

    case ctre::phoenix::UnexpectedArbId:
        return std::string("UnexpectedArbId/CAN_NO_SESSIONS_AVAIL");

    case ctre::phoenix::BufferFull:
        return std::string("BufferFull");

    case ctre::phoenix::CAN_OVERFLOW:
        return std::string("CAN_OVERFLOW");

    case ctre::phoenix::SensorNotPresent:
        return std::string("SensorNotPresent");

    case ctre::phoenix::FirmwareTooOld:
        return std::string("FirmwareTooOld");

    case ctre::phoenix::CouldNotChangePeriod:
        return std::string("CouldNotChangePeriod");

    case ctre::phoenix::BufferFailure:
        return std::string("BufferFailure");

    case ctre::phoenix::FirwmwareNonFRC:
        return std::string("FirwmwareNonFRC");

    case ctre::phoenix::GENERAL_ERROR:
        return std::string("GENERAL_ERROR");

    case ctre::phoenix::SIG_NOT_UPDATED:
        return std::string("SIG_NOT_UPDATED");

    case ctre::phoenix::NotAllPIDValuesUpdated:
        return std::string("NotAllPIDValuesUpdated");

    case ctre::phoenix::GEN_PORT_ERROR:
        return std::string("GEN_PORT_ERROR");

    case ctre::phoenix::PORT_MODULE_TYPE_MISMATCH:
        return std::string("PORT_MODULE_TYPE_MISMATCH");

    case ctre::phoenix::GEN_MODULE_ERROR:
        return std::string("GEN_MODULE_ERROR");

    case ctre::phoenix::MODULE_NOT_INIT_SET_ERROR:
        return std::string("MODULE_NOT_INIT_SET_ERROR");

    case ctre::phoenix::MODULE_NOT_INIT_GET_ERROR:
        return std::string("MODULE_NOT_INIT_GET_ERROR");

    case ctre::phoenix::WheelRadiusTooSmall:
        return std::string("WheelRadiusTooSmall");

    case ctre::phoenix::TicksPerRevZero:
        return std::string("TicksPerRevZero");

    case ctre::phoenix::DistanceBetweenWheelsTooSmall:
        return std::string("DistanceBetweenWheelsTooSmall");

    case ctre::phoenix::GainsAreNotSet:
        return std::string("GainsAreNotSet");

    case ctre::phoenix::WrongRemoteLimitSwitchSource:
        return std::string("WrongRemoteLimitSwitchSource");

    case ctre::phoenix::DoubleVoltageCompensatingWPI:
        return std::string("DoubleVoltageCompensatingWPI");

    case ctre::phoenix::CANdleAnimSlotOutOfBounds:
        return std::string("CANdleAnimSlotOutOfBounds");

    case ctre::phoenix::IncompatibleMode:
        return std::string("IncompatibleMode");

    case ctre::phoenix::InvalidHandle:
        return std::string("InvalidHandle");

    case ctre::phoenix::FeatureRequiresHigherFirm:
        return std::string("FeatureRequiresHigherFirm");

    case ctre::phoenix::TalonFeatureRequiresHigherFirm:
        return std::string("TalonFeatureRequiresHigherFirm");

    case ctre::phoenix::ConfigFactoryDefaultRequiresHigherFirm:
        return std::string("ConfigFactoryDefaultRequiresHigherFirm");

    case ctre::phoenix::ConfigMotionSCurveRequiresHigherFirm:
        return std::string("ConfigMotionSCurveRequiresHigherFirm");

    case ctre::phoenix::CANdleAnimationsRequireHigherFirm:
        return std::string("CANdleAnimationsRequireHigherFirm");

    case ctre::phoenix::TalonFXFirmwarePreVBatDetect:
        return std::string("TalonFXFirmwarePreVBatDetect");

    case ctre::phoenix::LibraryCouldNotBeLoaded:
        return std::string("LibraryCouldNotBeLoaded");

    case ctre::phoenix::MissingRoutineInLibrary:
        return std::string("MissingRoutineInLibrary");

    case ctre::phoenix::ResourceNotAvailable:
        return std::string("ResourceNotAvailable");

    case ctre::phoenix::PulseWidthSensorNotPresent:
        return std::string("PulseWidthSensorNotPresent");

    case ctre::phoenix::GeneralWarning:
        return std::string("GeneralWarning");

    case ctre::phoenix::FeatureNotSupported:
        return std::string("FeatureNotSupported");

    case ctre::phoenix::NotImplemented:
        return std::string("NotImplemented");

    case ctre::phoenix::FirmVersionCouldNotBeRetrieved:
        return std::string("FirmVersionCouldNotBeRetrieved");

    case ctre::phoenix::FeaturesNotAvailableYet:
        return std::string("FeaturesNotAvailableYet");

    case ctre::phoenix::ControlModeNotValid:
        return std::string("ControlModeNotValid");

    case ctre::phoenix::ControlModeNotSupportedYet:
        return std::string("ConrolModeNotSupportedYet");

    case ctre::phoenix::CascadedPIDNotSupporteYet:
        return std::string("CascadedPIDNotSupporteYet/AuxiliaryPIDNotSupportedYet");

    case ctre::phoenix::RemoteSensorsNotSupportedYet:
        return std::string("RemoteSensorsNotSupportedYet");

    case ctre::phoenix::MotProfFirmThreshold:
        return std::string("MotProfFirmThreshold");

    case ctre::phoenix::MotProfFirmThreshold2:
        return std::string("MotProfFirmThreshold2");

    case ctre::phoenix::MusicFileNotFound:
        return std::string("MusicFileNotFound");

    case ctre::phoenix::MusicFileWrongSize:
        return std::string("MusicFileWrongSize");

    case ctre::phoenix::MusicFileTooNew:
        return std::string("MusicFileTooNew");

    case ctre::phoenix::MusicFileInvalid:
        return std::string("MusicFileInvalid");

    case ctre::phoenix::InvalidOrchestraAction:
        return std::string("InvalidOrchestraAction");

    case ctre::phoenix::MusicFileTooOld:
        return std::string("MusicFileTooOld");

    case ctre::phoenix::MusicInterrupted:
        return std::string("MusicInterrupted");

    case ctre::phoenix::MusicNotSupported:
        return std::string("MusicNotSupported");

    case ctre::phoenix::kInvalidGuid:
        return std::string("kInvalidGuid");

    case ctre::phoenix::kInvalidClass:
        return std::string("kInvalidClass");

    case ctre::phoenix::kInvalidProtocol:
        return std::string("kInvalidProtocol");

    case ctre::phoenix::kInvalidPath:
        return std::string("kInvalidPath");

    case ctre::phoenix::kGeneralWinUsbError:
        return std::string("kGeneralWinUsbError");

    case ctre::phoenix::kFailedSetup:
        return std::string("kFailedSetup");

    case ctre::phoenix::kListenFailed:
        return std::string("kListenFailed");

    case ctre::phoenix::kSendFailed:
        return std::string("kSendFailed");

    case ctre::phoenix::kReceiveFailed:
        return std::string("kReceiveFailed");

    case ctre::phoenix::kInvalidRespFormat:
        return std::string("kInvalidRespFormat");

    case ctre::phoenix::kWinUsbInitFailed:
        return std::string("kWinUsbInitFailed");

    case ctre::phoenix::kWinUsbQueryFailed:
        return std::string("kWinUsbQueryFailed");

    case ctre::phoenix::kWinUsbGeneralError:
        return std::string("kWinUsbGeneralError");

    case ctre::phoenix::kAccessDenied:
        return std::string("kAccessDenied");

    case ctre::phoenix::kFirmwareInvalidResponse:
        return std::string("kFirmwareInvalidResponse");

    case ctre::phoenix::SimDeviceNotFound:
        return std::string("SimDeviceNotFound");

    case ctre::phoenix::SimPhysicsTypeNotSupported:
        return std::string("SimPhysicsTypeNotSupported");

    case ctre::phoenix::SimDeviceAlreadyExists:
        return std::string("SimDeviceAlreadyExists");

    default:
    {
        std::stringstream s;
        s << "Unknown Talon error " << error_code;
        return std::string(s.str());
    }
    }
}