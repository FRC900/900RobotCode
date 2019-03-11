// Code to implement a dynamic reconfigure server. This allows parameters to
// be changed on the fly using rqt_reconfigure.  This node creates a dynamic
// reconfigure server and then initializes it from params loaded from YAML
// config files.
#include <dynamic_reconfigure/server.h>

template <class T>
class DynamicReconfigureWrapper
{
	public:
		DynamicReconfigureWrapper()
			: srv_(nullptr)
		    , srv_mutex_(nullptr)
		{
		}

		void init(const ros::NodeHandle &nh,
				const typename dynamic_reconfigure::Server<T>::CallbackType &callback)
		{
			// Create the server and set up a callback function
			// This callback is responsible for copying from a
			// passed-in config object into local copies of the
			// variables
			srv_mutex_ = std::make_shared<boost::recursive_mutex>();
			srv_ = std::make_shared<dynamic_reconfigure::Server<T>>(*srv_mutex_, nh);
			srv_->setCallback(callback);

			// Create a config object, load values from the param
			// server, use those to initialize the dyanmic reconfigure
			// values
			T current_param_server_config;
			current_param_server_config.__fromServer__(nh);
			srv_->updateConfig(current_param_server_config);
		}

		void updateConfig(const T &config)
		{
			if (srv_)
				srv_->updateConfig(config);
		}

	protected:
		std::shared_ptr<dynamic_reconfigure::Server<T>> srv_;
		std::shared_ptr<boost::recursive_mutex>         srv_mutex_;
};
