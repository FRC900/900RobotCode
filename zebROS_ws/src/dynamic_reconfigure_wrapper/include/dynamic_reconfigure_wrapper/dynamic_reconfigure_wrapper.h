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
		    , config_ptr_(nullptr)
		{
		}

		~DynamicReconfigureWrapper()
		{
		}

		DynamicReconfigureWrapper(const ros::NodeHandle &nh, T& loaded_config)
		{
			init(nh, loaded_config);
		}

		void init(const ros::NodeHandle &nh, T& config)
		{
			// Create the server and set up a callback function
			// This callback is responsible for copying from a
			// passed-in config object into local copies of the
			// variables
			// Fist call to setCallback calls the callback which
			// sets config to the defaults from the .cfg file
			// We don't want those values, so use a temp config
			// to hold them and throw them out before setting the
			// actual values passed in
			T tmp_config;
			config_ptr_ = &tmp_config;
			srv_mutex_ = std::make_shared<boost::recursive_mutex>();
			srv_ = std::make_shared<dynamic_reconfigure::Server<T>>(*srv_mutex_, nh);
			srv_->setCallback(boost::bind(&DynamicReconfigureWrapper<T>::callback, this, _1, _2));
			config_ptr_ = &config;

			updateConfig(config);
		}

		void updateConfig(const T &config)
		{
			if (srv_)
				srv_->updateConfig(config);
		}

	protected:
		std::shared_ptr<dynamic_reconfigure::Server<T>> srv_;
		std::shared_ptr<boost::recursive_mutex>         srv_mutex_;
		T                                              *config_ptr_;

		// Generic callback function copies config from GUI into local config
		void callback(T &config, uint32_t level)
		{
			(void)level;
			if (config_ptr_)
				*config_ptr_ = config;
		}

};
