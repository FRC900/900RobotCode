#!/usr/bin/env python3

import rospy
import actionlib

import math

from behavior_actions.msg import AlignAndPlace2025Action, AlignAndPlace2025Goal, AlignAndPlace2025Result, AlignAndPlace2025Feedback
from behavior_actions.msg import AlignToReef2025Action, AlignToReef2025Goal, AlignToReef2025Result, AlignToReef2025Feedback

from talon_controller_msgs.srv import Command, CommandRequest

class Intaker2025ActionServer(object):
    # create messages that are used to publish feedback/result
    _result = AlignAndPlace2025Result()
    _feedback = AlignAndPlace2025Feedback()

    def __init__(self, name):
        self.aligning_client = rospy.SimpleActionClient("/aligner", AlignAndPlace2025Action)
        self.aligning_client.wait_for_server()
        self.placing_client = rospy.SimpleActionClient()
        self.placing_client.wait_for_server()

        self.distance = 10
        self.aligning_success = True
        self.placing_distance = rospy.get_param("placing_distance")

        self._action_name = name

        self._as = actionlib.SimpleActionServer(self._action_name, AlignAndPlace2025Action, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def aligning_feedback_cb(self, feedback: AlignToReef2025Feedback):
        self.distance = math.sqrt(feedback.x_error ** 2 + feedback.y_error ** 2)
    
    def aligning_done_cb(self, result: AlignToReef2025Result):
        if result.success:
            self.aligning_success = True
            self._feedback.stage = self._feedback.PLACING
        else:
            self.aligning_success = False

    def execute_cb(self, goal):
        r = rospy.Rate(250)

        self._feedback.stage = self._feedback.ALIGNING
        self._as.publish_feedback(self._feedback)

        self.aligning_success = True
        aligning_goal = AlignAndPlace2025Goal(pipe=goal.pipe)
        self.aligning_client.send_goal(aligning_goal, feedback_cb=self.aligning_feedback_cb, done_cb=aligning_done_cb)
        rospy.loginfo("2025_alignandplaceing_server: aligning")

        while self.distance > self.placing_distance and (not rospy.is_shutdown()):
            if self._as.is_preempt_requested():
                self.aligning_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self._as.set_preempted()
                self._result.success = False
                return
            if not self.aligning_success:
                rospy.logerr("2025_alignandplaceing_server: failure in aligning server")
                self._result.success = False
                return
            r.sleep()
        
        if self._feedback.stage == self._feedback.ALIGNING:
            self._feedback.stage = self._feedback.ALIGNING_PLACING
        
        # Call placing server
        # While placing server is placing:
            # Usual preempt check
            # If aligner fails:
                # Cancel placing goal
                # self._result.success = False :(
            # If placing fails:
                # Cancel aligner goal
                # self._result.success = False :(
        # Publish feedback about being done? (I put this in bc its in the prog gen task but...)
        
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)

    def callback(self, data):
        if self.switch_name in data.name:
            self.switch = data.position[data.name.index(self.switch_name)]
            #rospy.loginfo(f"Found {self.switch_name} with value {self.switch}")
        else:
            rospy.logwarn_throttle(1.0, f'2025_intaker_server: {self.switch_name} not found')
            pass

if __name__ == '__main__':
    rospy.init_node('intaker_server_2025')
    
    server = Intaker2025ActionServer(rospy.get_name())
    rospy.spin()