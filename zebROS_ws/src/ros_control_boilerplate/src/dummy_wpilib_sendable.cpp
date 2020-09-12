// Code to stub out various Senable implementations. These should quietly do nothing.
// The plan is bypassing the default WPIlib code will let us get rid of a lot of other
// unused functions - networktables, etc.
//
#include "frc/smartdashboard/SendableRegistry.h"

namespace frc {

struct SendableRegistry::Impl
{
	bool foo;
};

SendableRegistry& SendableRegistry::GetInstance()
{
	static SendableRegistry s;
	return s;
}

#if 0
void SendableRegistry::Add(Sendable* sendable, const wpi::Twine& name)
{
}

void SendableRegistry::Add(Sendable* sendable, const wpi::Twine& moduleType, int channel)
{
}

void SendableRegistry::Add(Sendable* sendable, const wpi::Twine& moduleType, int moduleNumber, int channel)
{
}

void SendableRegistry::Add(Sendable* sendable, const wpi::Twine& subsystem, const wpi::Twine& name)
{
}

void AddLW(Sendable* sendable, const wpi::Twine& name);

#endif
void SendableRegistry::AddLW(Sendable* sendable, const wpi::Twine& moduleType, int channel)
{
}
#if 0

void AddLW(Sendable* sendable, const wpi::Twine& moduleType, int moduleNumber,
		 int channel);

void AddLW(Sendable* sendable, const wpi::Twine& subsystem,
		 const wpi::Twine& name);

void AddChild(Sendable* parent, Sendable* child);

void AddChild(Sendable* parent, void* child);

#endif
bool SendableRegistry::Remove(Sendable* /*sendable*/)
{
return true;
}
#if 0

void Move(Sendable* to, Sendable* from);

bool Contains(const Sendable* sendable) const

std::string GetName(const Sendable* sendable) const

void SetName(Sendable* sendable, const wpi::Twine& name);

void SetName(Sendable* sendable, const wpi::Twine& moduleType, int channel);

void SetName(Sendable* sendable, const wpi::Twine& moduleType,
		   int moduleNumber, int channel);

void SetName(Sendable* sendable, const wpi::Twine& subsystem,
		   const wpi::Twine& name);

std::string GetSubsystem(const Sendable* sendable) const

void SetSubsystem(Sendable* sendable, const wpi::Twine& subsystem);

int GetDataHandle();

std::shared_ptr<void> SetData(Sendable* sendable, int handle,
							std::shared_ptr<void> data);

std::shared_ptr<void> GetData(Sendable* sendable, int handle);

void EnableLiveWindow(Sendable* sendable);

void DisableLiveWindow(Sendable* sendable);

UID GetUniqueId(Sendable* sendable);

Sendable* GetSendable(UID uid);

void Publish(UID sendableUid, std::shared_ptr<NetworkTable> table);

void Update(UID sendableUid);

void ForeachLiveWindow(
  int dataHandle,
  wpi::function_ref<void(CallbackData& cbdata)> callback) const

#endif
SendableRegistry::SendableRegistry()
{
}

}  // namespace frc

#include "frc/smartdashboard/SendableBase.h"
namespace frc
{

	SendableBase::SendableBase(bool)
	{
	}
};

#include "ntcore_cpp.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

namespace nt
{

bool SetEntryValue(NT_Entry /*entry*/, std::shared_ptr<Value> /*value*/)
{
	return true;
}
void SetEntryTypeValue(NT_Entry /*entry*/, std::shared_ptr<Value> /*value*/)
{
}
NT_Inst GetDefaultInstance()
{
	static NT_Inst nti;
	return nti;
}

Value::Value()
{
}
Value::Value(NT_Type /*type*/, uint64_t /*time*/, const private_init&)
{
}
Value::~Value()
{
}

NetworkTable::NetworkTable(NT_Inst /*inst*/, wpi::Twine const & /*path*/, NetworkTable::private_init const & /*pi*/)
{
}

NetworkTable::~NetworkTable()
{
}

bool NetworkTable::ContainsKey(const Twine& /*key*/) const
{
	return true;
}

bool NetworkTable::ContainsSubTable(const Twine& /*key*/) const
{
	return true;
}
std::shared_ptr<NetworkTable> NetworkTable::GetSubTable(const Twine& /*key*/) const
{
	NetworkTable::private_init pi;
	return std::make_shared<NetworkTable>(0, "", pi);
}

std::vector<std::string> NetworkTable::GetKeys(int /*types*/) const
{
	return std::vector<std::string>();
}

std::vector<std::string> NetworkTable::GetSubTables() const
{
	return std::vector<std::string>();
}

NetworkTableEntry NetworkTable::GetEntry(const Twine& /*key*/) const
{
	return NetworkTableEntry();
}

void NetworkTable::SetPersistent(StringRef /*key*/)
{
}

void NetworkTable::ClearPersistent(StringRef /*key*/)
{
}

bool NetworkTable::IsPersistent(StringRef /*key*/) const
{
	return false;
}
void NetworkTable::SetFlags(StringRef /*key*/, unsigned int /*flags*/)
{
}

void NetworkTable::ClearFlags(StringRef /*key*/, unsigned int /*flags*/)
{
}

unsigned int NetworkTable::GetFlags(StringRef /*key*/) const
{
	return 0UL;
}

void NetworkTable::Delete(const Twine& /*key*/)
{
}

bool NetworkTable::PutNumber(StringRef /*key*/, double /*value*/)
{
	return true;
}

bool NetworkTable::SetDefaultNumber(StringRef /*key*/, double /*defaultValue*/)
{
	return true;
}

double NetworkTable::GetNumber(StringRef /*key*/, double /*defaultValue*/) const
{
	return 0.0;
}

bool NetworkTable::PutString(StringRef /*key*/, StringRef /*value*/)
{
	return true;
}

bool NetworkTable::SetDefaultString(StringRef /*key*/, StringRef /*defaultValue*/)
{
	return true;
}

std::string NetworkTable::GetString(StringRef /*key*/, StringRef /*defaultValue*/) const
{
	return std::string();
}

bool NetworkTable::PutBoolean(StringRef /*key*/, bool /*value*/)
{
	return true;
}

bool NetworkTable::SetDefaultBoolean(StringRef /*key*/, bool /*defaultValue*/)
{
	return true;
}

bool NetworkTable::GetBoolean(StringRef /*key*/, bool /*defaultValue*/) const
{
	return true;
}

bool NetworkTable::PutBooleanArray(StringRef /*key*/, ArrayRef<int> /*value*/)
{
	return true;
}

bool NetworkTable::SetDefaultBooleanArray(StringRef /*key*/,
		ArrayRef<int> /*defaultValue*/)
{
	return true;
}

std::vector<int> NetworkTable::GetBooleanArray(StringRef /*key*/,
		ArrayRef<int> /*defaultValue*/) const
{
	return std::vector<int>();
}

bool NetworkTable::PutNumberArray(StringRef /*key*/, ArrayRef<double> /*value*/)
{
	return true;
}

bool NetworkTable::SetDefaultNumberArray(StringRef /*key*/,
		ArrayRef<double> /*defaultValue*/)
{
	return true;
}

std::vector<double> NetworkTable::GetNumberArray(
		StringRef /*key*/, ArrayRef<double> /*defaultValue*/) const
{
	return std::vector<double>();
}

bool NetworkTable::PutStringArray(StringRef /*key*/, ArrayRef<std::string> /*value*/)
{
	return true;
}

bool NetworkTable::SetDefaultStringArray(StringRef /*key*/,
		ArrayRef<std::string> /*defaultValue*/)
{
	return true;
}

std::vector<std::string> NetworkTable::GetStringArray(
		StringRef /*key*/, ArrayRef<std::string> /*defaultValue*/) const
{
	return std::vector<std::string>();
}

bool NetworkTable::PutRaw(StringRef /*key*/, StringRef /*value*/)
{
	return true;
}

bool NetworkTable::SetDefaultRaw(StringRef /*key*/, StringRef /*defaultvalue*/)
{
	return true;
}

std::string NetworkTable::GetRaw(StringRef /*key*/, StringRef /*defaultvalue*/) const
{
	return std::string();
}

bool NetworkTable::PutValue(const Twine& /*key*/, std::shared_ptr<Value> /*value*/)
{
	return true;
}

bool NetworkTable::SetDefaultValue(const Twine& /*key*/,
		std::shared_ptr<Value> /*defaultValue*/)
{
	return true;
}

std::shared_ptr<Value> NetworkTable::GetValue(const Twine& /*key*/) const
{
	return std::make_shared<Value>();
}

StringRef NetworkTable::GetPath() const
{
	return StringRef();
}

/*
const char* NetworkTable::SaveEntries(const Twine& filename) const
{
	return nullptr;
}

const char* LoadEntries(
		const Twine& filename,
		std::function<void(size_t line, const char* msg)> warn)
{
	return nullptr;
}
*/
void NetworkTable::AddTableListener(ITableListener* /*listener*/)
{
}

void NetworkTable::AddTableListener(ITableListener* /*listener*/,
		bool /*immediateNotify*/)
{
}

void NetworkTable::AddTableListenerEx(ITableListener* /*listener*/,
		unsigned int /*flags*/)
{
}

void NetworkTable::AddTableListener(StringRef /*key*/, ITableListener* /*listener*/,
		bool /*immediateNotify*/)
{
}

void NetworkTable::AddTableListenerEx(StringRef /*key*/, ITableListener* /*listener*/,
		unsigned int /*flags*/)
{
}

void NetworkTable::AddSubTableListener(ITableListener* /*listener*/)
{
}

void NetworkTable::AddSubTableListener(ITableListener* /*listener*/, bool /*localNotify*/)
{
}

void NetworkTable::RemoveTableListener(NT_EntryListener /*listener*/)
{
}

void NetworkTable::RemoveTableListener(ITableListener* /*listener*/)
{
}


std::shared_ptr<NetworkTable> NetworkTableInstance::GetTable(const Twine& /*key*/) const
{
	NetworkTable::private_init pi;
	return std::make_shared<NetworkTable>(0, "", pi);
}

};




