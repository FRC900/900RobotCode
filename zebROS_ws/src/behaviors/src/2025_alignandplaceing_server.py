#!/usr/bin/env python3

import rospy
import actionlib

import math

from behavior_actions.msg import AlignAndPlace2025Action, AlignAndPlace2025Goal, AlignAndPlace2025Result, AlignAndPlace2025Feedback
from behavior_actions.msg import AlignToReef2025Action, AlignToReef2025Goal, AlignToReef2025Result, AlignToReef2025Feedback
from behavior_actions.msg import Placing2025Action, Placing2025Goal, Placing2025Result, Placing2025Feedback

class AlignAndPlaceServer(object):
    # create messages that are used to publish feedback/result
    _result = AlignAndPlace2025Result()
    _feedback = AlignAndPlace2025Feedback()

    def __init__(self, name):
        self.aligning_client = actionlib.SimpleActionClient("/align_to_reef_single_tag/align_to_reef_single_tag", AlignToReef2025Action)
        self.aligning_client.wait_for_server()
        self.placing_client = actionlib.SimpleActionClient("/placing_server_2025", Placing2025Action)
        self.placing_client.wait_for_server()

        self.placing_distance = rospy.get_param("placing_distance")

        self._action_name = name

        self._as = actionlib.SimpleActionServer(self._action_name, AlignAndPlace2025Action, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal: AlignAndPlace2025Goal):
        r = rospy.Rate(250)

        self._feedback.stage = self._feedback.ALIGNING
        self._as.publish_feedback(self._feedback)

        def aligning_feedback_cb(feedback: AlignToReef2025Feedback):
            nonlocal distance
            distance = math.sqrt(feedback.x_error ** 2 + feedback.y_error ** 2)
        
        def aligning_done_cb(state, result: AlignToReef2025Result):
            nonlocal aligning_success
            nonlocal aligning_done
            aligning_success = result.success
            aligning_done = True
            if aligning_success:
                self._feedback.stage = self._feedback.PLACING
                self._as.publish_feedback(self._feedback)

        # Call aligning server
        distance = 69
        aligning_success = True
        aligning_done = False

        aligning_goal = AlignToReef2025Goal(pipe = goal.pipe)
        self.aligning_client.send_goal(aligning_goal, feedback_cb=aligning_feedback_cb, done_cb=aligning_done_cb)
        rospy.loginfo("2025_alignandplaceing_server: aligning")

        while distance > self.placing_distance and not aligning_done and (not rospy.is_shutdown()):
            if self._as.is_preempt_requested():
                self.aligning_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self._as.set_preempted()
                self._result.success = False
                return
            if not aligning_success:
                rospy.logerr("2025_alignandplaceing_server: error in aligning server")
                self._result.success = False
                self._as.set_aborted(self._result)
                return
            r.sleep()
        
        if self._feedback.stage == self._feedback.ALIGNING:
            self._feedback.stage = self._feedback.ALIGNING_PLACING
            self._as.publish_feedback(self._feedback)
        
        def placing_done_cb(state, result: Placing2025Result):
            nonlocal placing_success
            nonlocal placing_done
            placing_success = result.success
            placing_done = True

        # Once we're within some distance, call placing server w/ setup_only = True
        placing_success = True
        placing_done = False

        placing_goal = Placing2025Goal()
        placing_goal.level = goal.level
        placing_goal.setup_only = True
        self.placing_client.send_goal(placing_goal, done_cb=placing_done_cb)
        rospy.loginfo("2025_alignandplaceing_server: setting up placing")

        while ((not aligning_done) or (not placing_done)) and (not rospy.is_shutdown()):
            if self._as.is_preempt_requested():
                self.aligning_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.placing_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self._as.set_preempted()
                self._result.success = False
                return
            if not aligning_success:
                rospy.logerr("2025_alignandplaceing_server: error in aligning server")
                self._result.success = False
                self._as.set_aborted(self._result)
                return
            if not placing_success:
                rospy.logerr("2025_alignandplaceing_server: error in placing server (setup_only = True)")
                self._result.success = False
                self._as.set_aborted(self._result)
                return
            r.sleep()
        
        if not aligning_success:
            rospy.logerr("2025_alignandplaceing_server: error in aligning server")
            self._result.success = False
            self._as.set_aborted(self._result)
            return
        if not placing_success:
            rospy.logerr("2025_alignandplaceing_server: error in placing server (setup_only = True)")
            self._result.success = False
            self._as.set_aborted(self._result)
            return
        
        # Once aligning is finished, call placing server w/ setup_only = False
        placing_success = True
        placing_done = False

        placing_goal = Placing2025Goal()
        placing_goal.level = goal.level
        placing_goal.setup_only = False
        self.placing_client.send_goal(placing_goal, done_cb=placing_done_cb)
        rospy.loginfo("2025_alignandplaceing_server: placing")

        while (not placing_done) and (not rospy.is_shutdown()):
            if self._as.is_preempt_requested():
                self.aligning_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.placing_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self._as.set_preempted()
                self._result.success = False
                return
            r.sleep()
        
        if not placing_success:
            rospy.logerr("2025_alignandplaceing_server: error in placing server (setup_only = False)")
            self._result.success = False
            self._as.set_aborted(self._result)
            return

        self.aligning_client.cancel_goals_at_and_before_time(rospy.Time.now())
        
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('alignandplaceing_server_2025')
    
    server = AlignAndPlaceServer(rospy.get_name())
    rospy.spin()