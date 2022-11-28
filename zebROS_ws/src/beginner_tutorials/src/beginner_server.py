#!/usr/bin/env python3

import rospy

import actionlib
import beginner_tutorials.msg


class FibonacciAction(object):
    def __init__(self):
        self._feedback = beginner_tutorials.msg.beginnerFeedback()
        self._result = beginner_tutorials.msg.beginnerResult()
        self._as = actionlib.SimpleActionServer("fibonacci", beginner_tutorials.msg.beginnerAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # append the seeds for the fibonacci sequence
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)

        # publish info to the console for the user
        rospy.loginfo('Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))

        # start executing the action
        for i in range(1, goal.order):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('The goal has been cancelled/preempted')
                self._as.set_preempted()
                success = False
                # we end the calculation of the Fibonacci sequence
                break
            self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()

        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('Succeeded calculating the Fibonacci of order %i' % goal.order)
            self._as.set_succeeded(self._result)
    
if __name__ == '__main__':
    rospy.init_node('fibonacci')
    FibonacciAction()
    rospy.spin()