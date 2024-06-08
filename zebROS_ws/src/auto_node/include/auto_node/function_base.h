// Pure virtual base class that all auto functions must inherit from.
// This provides a common interface for all functions, and allows for
// easy creation of new functions by simply inheriting from this class.
// The methods in the class are set up to be called when a function
// is started, which is used to [re]initialzed things needed for each
// execution of that function.  Update is called repeatedly until the
// function is_finished returns true.  Finish is called when the function
// is_finished returns true, and is used to do any final actions - stopping
// a motor that was moving, preempting actionlib calls, etc.
// wait_for_completion is a flag that can be set to true if the function
// should block until the function is_finished returns true.  This is useful
// for allowing functions to run in parallel.
#ifndef FUNCTION_BASE_INC__
#define FUNCTION_BASE_INC__
#include <optional>
#include <string>
#include <xmlrpcpp/XmlRpcValue.h>

class FunctionBase {
public:
    explicit FunctionBase(const std::string &name,
                          const bool wait_for_completion) :
        name_(name), wait_for_completion_(wait_for_completion) {}
    virtual ~FunctionBase() = default;
    virtual bool start(void)   { return true; }
    virtual bool finish(void)  { return true; }
    virtual bool update(void)  { return true; }

    virtual bool preempt(void) { return true;}

    virtual bool is_finished(void) = 0;
    bool wait_for_completion(void) const
    {
        return wait_for_completion_;
    }
protected:
    // Mainly useful for making error messages more informative
    // Make this public if it turns out that it's useful for
    // the top-level code to report on the name of the function
    // being executed.
    std::string get_name(void) const
    {
        return name_;
    }

private:
    std::string name_;
    bool wait_for_completion_{true};
};


// Create a derived class from FunctionBase with methods for actionlib
// feedback and result handling.  This will likely be template-heavy,
// similar to the wait for action funciton in the previous auto_node.



#endif