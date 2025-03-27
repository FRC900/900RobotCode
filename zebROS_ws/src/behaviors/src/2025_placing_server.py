#!/usr/bin/env python3

import rospy
import actionlib

from behavior_actions.msg import Placing2025Goal, Placing2025Feedback, Placing2025Result, Placing2025Action
from behavior_actions.msg import Elevater2025Goal, Elevater2025Feedback, Elevater2025Result, Elevater2025Action
from behavior_actions.msg import Roller2025Goal, Roller2025Feedback, Roller2025Result, Roller2025Action
from geometry_msgs.msg import Twist

class PlacingServer(object):
    DRIVE_BACK_SPEED = 0.5
    DRIVE_BACK_TIME = 0.25

    def __init__(self, name):
        self.__action_name = name
        self.result = Placing2025Result()
        self.feedback = Placing2025Feedback()
        self.elevater_client = actionlib.SimpleActionClient('/elevater/elevater_server_2025', Elevater2025Action)
        rospy.loginfo('Waiting for Elevater action server')
        self.elevater_client.wait_for_server()
        self.roller_client = actionlib.SimpleActionClient('/roller/roller_server_2025', Roller2025Action)
        rospy.loginfo('Waiting for Roller action server')
        self.roller_client.wait_for_server()
        rospy.loginfo('Found Elevater and Roller server')
        self.cmd_vel_pub = rospy.Publisher("/placing/cmd_vel", Twist, tcp_nodelay=True, queue_size=1)
        self.server = actionlib.SimpleActionServer(name, Placing2025Action, execute_cb=self.execute_cb, auto_start = False)
        self.server.start()


    def execute_cb(self, goal : Placing2025Goal):
        r = rospy.Rate(60)

        if goal.setup_only: rospy.loginfo('Placing Server - Setup Mode') # Tell the user that the robot is in setup mode
        
        elevater_goal = Elevater2025Goal()
        elevater_goal.mode = goal.level
        roller_goal = Roller2025Goal()
        roller_goal.mode = roller_goal.OUTTAKE # Set the roller goal to OUTTAKE
       
        def elevater_done_cb(state, result):
            nonlocal elevator_done
            nonlocal elevator_success
            rospy.loginfo('Elevater action finished')
            elevator_done = True
            elevator_success = result.success

        # Raise elevator
        self.elevater_client.send_goal(elevater_goal, done_cb=elevater_done_cb)

        elevator_done = False
        elevator_success = False
        rospy.loginfo('Elevater action sent')

        while not elevator_done and not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                self.elevater_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.server.set_preempted()
                return
            r.sleep()

        if not elevator_success:
            rospy.loginfo('Failure in elevater server (raising)')
            self.result.success = False
            self.server.set_aborted(self.result)
            return
        
        # If we're setting up, we're done
        if goal.setup_only:
            rospy.loginfo('Setup Complete')
            self.result.success = True
            self.server.set_succeeded(self.result)
            return
        
        # Place
        roller_done = False
        roller_success = False
        def roller_done_cb(state, result):
            nonlocal roller_done
            nonlocal roller_success
            rospy.loginfo('Roller action finished')
            roller_done = True
            roller_success = True

        rospy.loginfo('Elevater service succeeded, now placing')
        self.roller_client.send_goal(roller_goal, roller_done_cb)
        rospy.loginfo('Roller action sent')

        while not roller_done and not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                self.roller_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.server.set_preempted()
                return
            r.sleep()

        if not roller_success:
            rospy.loginfo('Failure in roller server')
            self.result.success = False
            self.server.set_aborted(self.result)
            return
        
        self.result.success = True
        rospy.loginfo("placing_server_2025: succeeded")
        self.server.set_succeeded(self.result)

        if not goal.dont_drive_back:
            rospy.loginfo("placing_server_2025: driving back")
            twist = Twist()
            twist.linear.x = -abs(self.DRIVE_BACK_SPEED)
            start = rospy.Time.now()
            while (rospy.Time.now() - start) < rospy.Duration(self.DRIVE_BACK_TIME) and not rospy.is_shutdown():
                self.cmd_vel_pub.publish(twist)
            self.cmd_vel_pub.publish(Twist()) # send zero when done
        
        # Lower elevator
        self.elevater_client.send_goal(Elevater2025Goal(mode=Elevater2025Goal.INTAKE), done_cb=elevater_done_cb)
        
        elevator_done = False
        elevator_success = False
        rospy.loginfo('Elevater lowering action sent')

        while not elevator_done and not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                self.elevater_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.server.set_preempted()
                return
            r.sleep()
        
        if not elevator_success:
            rospy.loginfo('Failure in elevater server (lowering)')
            self.result.success = False
            self.server.set_aborted(self.result)
            return


if __name__ == '__main__':
    rospy.init_node('placing_server_2025')
    server = PlacingServer(rospy.get_name())
    rospy.spin()