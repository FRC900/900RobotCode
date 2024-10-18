#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

namespace nt
{
NT_Inst GetDefaultInstance()
{
	static NT_Inst nti;
	return nti;
}

std::shared_ptr<NetworkTable> NetworkTableInstance::GetTable(std::string_view /*key*/) const
{
	return std::make_shared<NetworkTable>(0, "", NetworkTable::private_init{});
}

Topic NetworkTable::GetTopic(std::string_view) const {
  return Topic{};
}

#if 0
void Release(NT_Handle)
{
}
#endif

bool SetBoolean(unsigned int, bool, long)
{
	return true;
}
bool SetInteger(unsigned int, long, long)
{
	return true;
}
bool SetString(unsigned int, std::string_view, long)
{
	return true;
}

NT_Publisher Publish(NT_Topic , NT_Type , std::string_view ,
                     nt::PubSubOptions const & ) {
  return {};
}
NT_Publisher NT_PublishEx(NT_Topic topic, NT_Type type,
						  const struct WPI_String *typeStr,
						  const struct WPI_String *properties,
						  const struct NT_PubSubOptions *options)
{
	return {};
}
NT_Publisher PublishEx(NT_Topic topic, NT_Type type, std::string_view typeStr,
					   const wpi::json &properties,
					   const PubSubOptions &options)
{
	return {};
}
}