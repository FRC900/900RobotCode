#ifndef INC_PARAM_READ__
#define INC_PARAM_READ__

#include <string>
#include "xmlrpcpp/XmlRpcValue.h"

static bool readStringParam(const std::string &param_name, XmlRpc::XmlRpcValue &params, std::string &val)
{
    if (!params.hasMember(param_name))
        return false;
    XmlRpc::XmlRpcValue &param = params[param_name];
    if (!param.valid())
        throw std::runtime_error(param_name + " was not a valid string type");
    if (param.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
        val = static_cast<std::string>(param);
        return true;
    }
    throw std::runtime_error("A non-string value was read for" + param_name);

    return false;
}

static bool readIntParam(const std::string &param_name, XmlRpc::XmlRpcValue &params, int &val)
{
    if (!params.hasMember(param_name))
        return false;
    XmlRpc::XmlRpcValue &param = params[param_name];
    if (!param.valid())
        throw std::runtime_error(param_name + " was not a valid int type");
    if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        val = static_cast<int>(param);
        return true;
    }
    else
        throw std::runtime_error("A non-double value was read for" + param_name);

    return false;
}


static bool readFloatParam(const std::string &param_name, XmlRpc::XmlRpcValue &params, double &val)
{
    if (!params.hasMember(param_name))
        return false;
    XmlRpc::XmlRpcValue &param = params[param_name];
    if (!param.valid())
        throw std::runtime_error(param_name + " was not a valid double type");
    if (param.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
        val = static_cast<double>(param);
        return true;
    }
    else if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        val = static_cast<int>(param);
        return true;
    }
    else
        throw std::runtime_error("A non-double value was read for" + param_name);

    return false;
}

static bool extractFloatVal(XmlRpc::XmlRpcValue &param, double &val)
{
    if (!param.valid())
        throw std::runtime_error("val was not a valid double type");
    if (param.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
        val = static_cast<double>(param);
        return true;
    }
    else if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        val = static_cast<int>(param);
        return true;
    }
    else
        throw std::runtime_error("A non-double value was read for value");

    return false;
}

static bool readBoolParam(const std::string &param_name, XmlRpc::XmlRpcValue &params, bool &val)
{
    if (!params.hasMember(param_name))
        return false;
    XmlRpc::XmlRpcValue &param = params[param_name];
    if (!param.valid())
        throw std::runtime_error(param_name + " was not a valid bool type");
    if (param.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
    {
        val = static_cast<bool>(param);
        return true;
    }
    else
        throw std::runtime_error("A non-bool value was read for" + param_name);

    return false;
}

#endif