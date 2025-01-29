#!/usr/bin/env python3

import rospy
import actionlib

from behavior_actions.msg import Placing2025Goal, Placing2025Feedback, Placing2025Result, Placing2025Action
# from behavior_actions.msg import Elevater2025Goal, Elevater2025Feedback, Elevater2025Result, Elevater2025Action
from behavior_actions.msg import Roller2025Goal, Roller2025Feedback, Roller2025Result, Roller2025Action
from controllers_2025_msgs.srv import ElevatorSrv, ElevatorSrvRequest, ElevatorSrvResponse

class PlacingServer(object):
    def __init__(self, name):
        self.__action_name = name
        self.result = Placing2025Result()
        self.goal = Placing2025Goal()
        self.feedback = Placing2025Feedback()
        # self.elevater_client = actionlib.SimpleActionClient('Elevater', Elevater2025Action)
        # rospy.loginfo('Waiting for Elevater action server')
        # self.elevater_client.wait_for_server()
        self.roller_client = actionlib.SimpleActionClient('roller_server_2025', Roller2025Action)
        rospy.loginfo('Waiting for Roller action server')
        self.roller_client.wait_for_server()


    def execute_cb(self, goal : Placing2025Goal):

        if placing_goal.setup_only: rospy.loginfo('Placing Server - Setup Mode') # Tell the user that the robot is in setup mode
        
        placing_goal = Placing2025Goal()
        # elevater_result = Elevater2025Result()
        # elevater_goal = Elevater2025Goal()
        # elevater_goal.mode = placing_goal.level
        roller_goal = Roller2025Goal()
        roller_goal.mode = 1 # Set the roller goal to OUTTAKE
        rospy.wait_for_service('elevator_service')
        elevator_goal = placing_goal.level
        try:
            elevator_srv = rospy.ServiceProxy('elevator_service', ElevatorSrv)
            if elevator_goal == 3: # L1
                elevator_srv(10.45) ### THESE ARE ARBITRARY VALUES FOR TESTING, REPLACE WITH ACTUAL VALUES
            elif elevator_goal == 2: # L2
                elevator_srv(7.45)
            elif elevator_goal == 1: # L3
                elevator_srv(4.45)
            elif elevator_goal == 0: # L4
                elevator_srv(1.45)
            elevator_success = True
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            elevator_success = False
            
        # def elevater_done_cb(state, result):
        #     rospy.loginfo('Elevater action finished')
        # self.elevater_client.send_goal(elevater_goal.mode, elevater_done_cb)
        # rospy.loginfo('Elevater action sent')

        if placing_goal.setup_only:
            rospy.loginfo('Setup Complete')
            self.result.success = True
            self.server.set_succeeded(self.result)
            return
        
        def roller_done_cb(state, result):
            rospy.loginfo('Roller action finished')
        
        if elevator_success:
            rospy.loginfo('Elevater service succeeded, now placing')
            self.roller_client.send_goal(roller_goal.mode, roller_done_cb)
            rospy.loginfo('Roller action sent')

        self.result.success = True
        rospy.loginfo("2025_placing_server: succeeded")
        self.server.set_succeeded(self.result)

if __name__ == '__main__':
    rospy.init_node('2025_placing_server')
    server = PlacingServer(rospy.get_name())
    rospy.spin()