#include <stdexcept>
#include <xmlrpcpp/XmlRpcValue.h>

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
static void readStringCommon(const XmlRpc::XmlRpcValue &joint_params,
                             const std::string &key,
                             std::string &value,
                             const std::string &joint_name = "")
{
    const XmlRpc::XmlRpcValue &xml_val = joint_params[key];
    if (!xml_val.valid() || xml_val.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
        throw std::runtime_error("An invalid joint key \"" + key + "\" was specified (expecting a string)" + (joint_name.size() > 0 ? " for joint " + joint_name : ""));
    }
    value = static_cast<std::string>(xml_val);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void readStringRequired(const XmlRpc::XmlRpcValue &joint_params,
                        const std::string &key,
                        std::string &value,
                        const std::string &joint_name = "")
{
    if (!joint_params.hasMember(key))
    {
        throw std::runtime_error("Joint key \"" + key + "\" was not specified" + (joint_name.size() > 0 ? " for joint " + joint_name : ""));
    }
    readStringCommon(joint_params, key, value, joint_name);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool readStringOptional(const XmlRpc::XmlRpcValue &joint_params,
                        const std::string &key,
                        std::string &value,
                        const std::string &joint_name = "")
{

    if (!joint_params.hasMember(key))
    {
        return false;
    }
    readStringCommon(joint_params, key, value, joint_name);
    return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
static void readBooleanCommon(const XmlRpc::XmlRpcValue &joint_params,
                             const std::string &key,
                             bool &value,
                             const std::string &joint_name = "")
{
    const XmlRpc::XmlRpcValue &xml_val = joint_params[key];
    if (!xml_val.valid() || xml_val.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
    {
        throw std::runtime_error("An invalid joint key \"" + key + "\" was specified (expecting a boolean)" + (joint_name.size() > 0 ? " for joint " + joint_name : ""));
    }
    value = static_cast<bool>(xml_val);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void readBooleanRequired(const XmlRpc::XmlRpcValue &joint_params,
                        const std::string &key,
                        bool &value,
                        const std::string &joint_name = "")
{
    if (!joint_params.hasMember(key))
    {
        throw std::runtime_error("Joint key \"" + key + "\" was not specified" + (joint_name.size() > 0 ? " for joint " + joint_name : ""));
    }
    readBooleanCommon(joint_params, key, value, joint_name);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool readBooleanOptional(const XmlRpc::XmlRpcValue &joint_params,
                         const std::string &key,
                         bool &value,
                         const std::string &joint_name = "")
{

    if (!joint_params.hasMember(key))
    {
        return false;
    }
    readBooleanCommon(joint_params, key, value, joint_name);
    return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
static void readIntCommon(const XmlRpc::XmlRpcValue &joint_params,
                          const std::string &key,
                          int &value,
                          const std::string &joint_name = "")
{
    const XmlRpc::XmlRpcValue &xml_val = joint_params[key];
    if (!xml_val.valid() || xml_val.getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
        throw std::runtime_error("An invalid joint key \"" + key + "\" was specified (expecting an int )" + (joint_name.size() > 0 ? " for joint " + joint_name : ""));
    }
    value = static_cast<int>(xml_val);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void readIntRequired(const XmlRpc::XmlRpcValue &joint_params,
                     const std::string &key,
                     int &value,
                     const std::string &joint_name = "")
{
    if (!joint_params.hasMember(key))
    {
        throw std::runtime_error("Joint key \"" + key + "\" was not specified" + (joint_name.size() > 0 ? " for joint " + joint_name : ""));
    }
    readIntCommon(joint_params, key, value, joint_name);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool readIntOptional(const XmlRpc::XmlRpcValue &joint_params,
                     const std::string &key,
                     int &value,
                     const std::string &joint_name = "")
{

    if (!joint_params.hasMember(key))
    {
        return false;
    }
    readIntCommon(joint_params, key, value, joint_name);
    return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
static void readDoubleCommon(const XmlRpc::XmlRpcValue &joint_params,
                             const std::string &key,
                             double &value,
                             const std::string &joint_name = "")
{
    if (const XmlRpc::XmlRpcValue &xml_val = joint_params[key]; xml_val.valid())
    {
        if (xml_val.getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
            value = static_cast<int>(xml_val);
            return;
        }
        if (xml_val.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        {
            value = static_cast<double>(xml_val);
            return;
        }
    }
    throw std::runtime_error("An invalid joint key \"" + key + "\" was specified (expecting an int or double )" + (joint_name.size() > 0 ? " for joint " + joint_name : ""));
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void readDoubleRequired(const XmlRpc::XmlRpcValue &joint_params,
                     const std::string &key,
                     double &value,
                     const std::string &joint_name = "")
{
    if (!joint_params.hasMember(key))
    {
        throw std::runtime_error("Joint key \"" + key + "\" was not specified" + (joint_name.size() > 0 ? " for joint " + joint_name : ""));
    }
    readDoubleCommon(joint_params, key, value, joint_name);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool readDoubleOptional(const XmlRpc::XmlRpcValue &joint_params,
                     const std::string &key,
                     double &value,
                     const std::string &joint_name = "")
{

    if (!joint_params.hasMember(key))
    {
        return false;
    }
    readDoubleCommon(joint_params, key, value, joint_name);
    return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
static void readArrayCommon(const XmlRpc::XmlRpcValue &joint_params,
                             const std::string &key,
                             XmlRpc::XmlRpcValue &value,
                             const std::string &joint_name = "")
{
    value = joint_params[key];
    if (!value.valid() || value.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        throw std::runtime_error("An invalid joint key \"" + key + "\" was specified (expecting a string)" + (joint_name.size() > 0 ? " for joint " + joint_name : ""));
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void readArrayRequired(const XmlRpc::XmlRpcValue &joint_params,
                        const std::string &key,
                        XmlRpc::XmlRpcValue &value,
                        const std::string &joint_name = "")
{
    if (!joint_params.hasMember(key))
    {
        throw std::runtime_error("Joint key \"" + key + "\" was not specified" + (joint_name.size() > 0 ? " for joint " + joint_name : ""));
    }
    readArrayCommon(joint_params, key, value, joint_name);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void readJointLocalParams(const XmlRpc::XmlRpcValue &joint_params,
                          const std::string &joint_name,
                          const bool local,
                          const bool saw_local_keyword,
                          bool &local_update,
                          bool &local_hardware)
{
	local_update = local;
	if (joint_params.hasMember("local_update"))
	{
		if (saw_local_keyword)
        {
			throw std::runtime_error("local can't be combined with local_update in joint " + joint_name);
        }
        readBooleanCommon(joint_params, "local_update", local_update, joint_name);
	}
	local_hardware = local;
	if (joint_params.hasMember("local_hardware"))
	{
		if (saw_local_keyword)
        {
			throw std::runtime_error("local can't be combined with local_hardware in joint " + joint_name);
        }
        readBooleanCommon(joint_params, "local_hardware", local_hardware, joint_name);
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void readJointLocalReadWriteParams(const XmlRpc::XmlRpcValue &joint_params,
                                   const std::string &joint_name,
                                   const bool local,
                                   const bool saw_local_keyword,
                                   bool &local_read,
                                   bool &local_write)
{
    local_read = local;
    if (joint_params.hasMember("local_read"))
    {
        if (saw_local_keyword)
        {
			throw std::runtime_error("local can't be combined with local_read in joint " + joint_name);
        }
        readBooleanCommon(joint_params, "local_read", local_read, joint_name);
    }
    local_write = local;
	if (joint_params.hasMember("local_write"))
	{
		if (saw_local_keyword)
        {
			throw std::runtime_error("local can't be combined with local_write in joint " + joint_name);
        }
        readBooleanCommon(joint_params, "local_write", local_write, joint_name);
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#include "frc/PneumaticsModuleType.h"
void readSolenoidModuleType(const XmlRpc::XmlRpcValue &joint_params,
                            const bool local_hardware,
                            frc::PneumaticsModuleType &module_type,
                            const std::string &joint_name)
{
    std::string module_string;
    const bool has_module_type = readStringOptional(joint_params, "module_type", module_string, joint_name);
    if (!local_hardware && has_module_type)
    {
        throw std::runtime_error("A module_type was specified for non-local hardware for joint " + joint_name);
    }
    if (local_hardware)
    {
        if (!has_module_type)
        {
            throw std::runtime_error("A module_type was not specified for joint " + joint_name);
        }
        if (module_string == "ctrepcm")
        {
            module_type = frc::PneumaticsModuleType::CTREPCM;
        }
        else if (module_string == "revph")
        {
            module_type = frc::PneumaticsModuleType::REVPH;
        }
        else
        {
            throw std::runtime_error("Unknown module_type for " + joint_name + ", expecting \"ctrepcm\" or \"revph\"");
        }
    }
}