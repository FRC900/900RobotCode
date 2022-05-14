#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"
namespace nt
{
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

NetworkTableEntry NetworkTable::GetEntry(std::string_view /*key*/) const
{
	return NetworkTableEntry();
}
std::shared_ptr<NetworkTable> NetworkTableInstance::GetTable(std::string_view /*key*/) const
{
	return std::make_shared<NetworkTable>(0, "", NetworkTable::private_init{});
}
void SetEntryTypeValue(NT_Entry /*entry*/, std::shared_ptr<Value> /*value*/)
{
}
bool SetEntryValue(NT_Entry /*entry*/, std::shared_ptr<Value> /*value*/)
{
	return true;
}
}

