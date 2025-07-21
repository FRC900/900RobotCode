#ifndef CTRE_V5_DEVICE_INC__
#define CTRE_V5_DEVICE_INC__

#include <atomic>
#include <string>

#include "ctre/phoenix/ErrorCode.h"

class CTREV5Device
{
public:
    CTREV5Device(const std::string &name_space,
                 const std::string &device_type,
                 const std::string &joint_name,
                 const int id);
    CTREV5Device(const CTREV5Device &) = delete;
    CTREV5Device(CTREV5Device &&other) noexcept = delete;
    virtual ~CTREV5Device();

    CTREV5Device &operator=(const CTREV5Device &) = delete;
    CTREV5Device &operator=(CTREV5Device &&) noexcept = delete;

    static void resetCanConfigCount(void);
    static void setCanConfigCountLimit(const size_t can_config_count_limit);

    std::string getName(void) const;
    int getId(void) const;

protected:
    bool safeCall(ctre::phoenix::ErrorCode error_code, const std::string &method_name) const;
    bool safeConfigCall(ctre::phoenix::ErrorCode error_code, const std::string &method_name) const;

private:
    const std::string device_type_;
    const std::string name_;
    const int id_;
    // These are used in both the main and the read threads, so they 
    // have to be atomic vars to avoid undefined behavior
    static inline std::atomic<size_t> can_error_count_{0};
    static inline std::atomic<bool> can_error_sent_{false};
    static inline int can_config_count_{0};
    static inline int can_config_count_limit_{10};
};

#endif