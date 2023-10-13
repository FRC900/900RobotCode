#ifndef INC_READ_CONFIG_UTILS__
#define INC_READ_CONFIG_UTILS__

#include <xmlrpcpp/XmlRpcValue.h>
#include "frc/PneumaticsModuleType.h"

void readStringRequired(const XmlRpc::XmlRpcValue &joint_params,
                        const std::string &key,
                        std::string &value,
                        const std::string &joint_name = "");
bool readStringOptional(const XmlRpc::XmlRpcValue &joint_params,
                        const std::string &key,
                        std::string &value,
                        const std::string &joint_name = "");
void readBooleanRequired(const XmlRpc::XmlRpcValue &joint_params,
                        const std::string &key,
                        bool &value,
                        const std::string &joint_name = "");
bool readBooleanOptional(const XmlRpc::XmlRpcValue &joint_params,
                        const std::string &key,
                        bool &value,
                        const std::string &joint_name = "");
void readIntRequired(const XmlRpc::XmlRpcValue &joint_params,
                        const std::string &key,
                        int &value,
                        const std::string &joint_name = "");
bool readIntOptional(const XmlRpc::XmlRpcValue &joint_params,
                        const std::string &key,
                        int &value,
                        const std::string &joint_name = "");
void readDoubleRequired(const XmlRpc::XmlRpcValue &joint_params,
                        const std::string &key,
                        double &value,
                        const std::string &joint_name = "");
bool readDoubleOptional(const XmlRpc::XmlRpcValue &joint_params,
                        const std::string &key,
                        double &value,
                        const std::string &joint_name = "");
void readArrayRequired(const XmlRpc::XmlRpcValue &joint_params,
                       const std::string &key,
                       XmlRpc::XmlRpcValue &value,
                       const std::string &joint_name = "");
bool readArrayOptional(const XmlRpc::XmlRpcValue &joint_params,
                       const std::string &key,
                       XmlRpc::XmlRpcValue &value,
                       const std::string &joint_name = "");
void readJointLocalParams(const XmlRpc::XmlRpcValue &joint_params,
                          const std::string &joint_name,
                          const bool local,
                          const bool saw_local_keyword,
                          bool &local_update,
                          bool &local_hardware);
void readJointLocalReadWriteParams(const XmlRpc::XmlRpcValue &joint_params,
                                   const std::string &joint_name,
                                   const bool local,
                                   const bool saw_local_keyword,
                                   bool &local_read,
                                   bool &local_write);
void readSolenoidModuleType(const XmlRpc::XmlRpcValue &joint_params,
                            const bool local_hardware,
                            frc::PneumaticsModuleType &module_type,
                            const std::string &joint_name);

#endif