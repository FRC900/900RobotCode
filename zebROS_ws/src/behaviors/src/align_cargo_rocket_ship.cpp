#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_srvs/Empty.h"
#include "behaviors/AlignGoal.h"
#include "behaviors/AlignAction.h"
#include "behaviors/ElevatorAction.h"
#include "behaviors/enumerated_elevator_indices.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"

#include "behaviors/base_align_server.h"

class AlignCargoRocketShip:public BaseAlignAction {
	public:
		void x_error_cb(const std_msgs::Bool &msg) {
			x_aligned_ = msg.data;
		}
};

int main(int argc, char ** argv)
{
	return 0;
}
