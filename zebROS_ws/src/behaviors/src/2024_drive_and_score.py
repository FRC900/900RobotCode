#! /usr/bin/env python3

import actionlib
import rospy
import math
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import math

from behavior_actions.msg import DriveAndScore2024Action, DriveAndScore2024Goal, DriveAndScore2024Feedback, DriveAndScore2024Result 
from behavior_actions.msg import Clawster2024Action, Clawster2024Feedback, Clawster2024Result, Clawster2024Goal
from behavior_actions.msg import Arm2024Action, Arm2024Feedback, Arm2024Result, Arm2024Goal
import behavior_actions.msg
from tf.transformations import euler_from_quaternion # may look like tf1 but is actually tf2
from frc_msgs.msg import MatchSpecificData
import std_srvs.srv
import angles
import numpy

# Used for anytime we need to drive to a position and then score, (amp/trap) as opposed to shooting where we just need to align (2024_align_to_speaker_server.py and other)

class DriveAndScore:
    # create messages that are used to publish feedback/result
    _feedback = DriveAndScore2024Feedback()
    _result = DriveAndScore2024Result()

    def __init__(self, name):   
        self.valid_samples = 0
        self._action_name = name
        # get us to right spot
        self.align_client = actionlib.SimpleActionClient('/align_to_trap/align_to_trap_2024', behavior_actions.msg.AlignToTrap2024Action)
        rospy.loginfo("2024_intaking_server: waiting for shooter pivot server")
        self.align_client.wait_for_server()
        # shoot trap
        self.shooting_client = actionlib.SimpleActionClient('/shooting/shooting_server_2024', behavior_actions.msg.Shooting2024Action)
        rospy.loginfo("2024_intaking_server: waiting for shooting server")
        self.shooting_client.wait_for_server()
        
        self.arm_client = actionlib.SimpleActionClient('/arm/move_arm_server_2024', Arm2024Action)
        rospy.loginfo("2024_intaking_server: waiting for arm server")
        self.arm_client.wait_for_server()

        self.clawster_client = actionlib.SimpleActionClient('/clawster/clawster_server_2024', Clawster2024Action)
        rospy.loginfo("2024_intaking_server: waiting for clawster server")
        self.clawster_client.wait_for_server()
        self.align_done = False
        self.arm_done = False
        self.shooting_done = False 
        self._as = actionlib.SimpleActionServer(self._action_name, DriveAndScore2024Action, execute_cb=self.score_cb, auto_start = False)
        self._as.start()
    
    def align_done_cb(self, status, result):
        rospy.loginfo("2024 drive and score, align server done")
        self.align_done = True

    def arm_done_cb(self, status, result):
        rospy.loginfo("2024 drive and score, arm server done")
        self.arm_done = True
    
    def shooting_done_cb(self, status, result):
        rospy.loginfo("2024 drive and score, shooting server done")
        self.shooting_done = True

    def score_cb(self, goal: DriveAndScore2024Goal):
        self.feed_forward = True
        success = True
        self.align_done = False
        self.arm_done = False
        self.shooting_done = False

        rospy.loginfo(f"Drive and score 2024 - called with goal {goal}")
        r = rospy.Rate(60.0)
        align_goal = behavior_actions.msg.AlignToTrap2024Goal()

        if goal.destination == goal.AMP:
            rospy.loginfo(f"Drive and score 2024 - going amp")
            align_goal.destination = align_goal.AMP
            self.align_client.send_goal(align_goal, done_cb=self.align_done_cb)
            # also start moving arm
            arm_goal = Arm2024Goal() 
            arm_goal.path = arm_goal.AMP
            self.arm_client.send_goal(arm_goal, done_cb=self.arm_done_cb)
        elif goal.destination == goal.TRAP:
            rospy.loginfo(f"Drive and score 2024 - going trap")
            align_goal.destination = align_goal.TRAP
            self.align_client.send_goal(align_goal, done_cb=self.align_done_cb)
            # for this case we just wait until we are done and then send shooting

        # for telling if we hvae 
        shooting = False
        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                self.arm_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.align_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.shooting_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.clawster_client.cancel_goals_at_and_before_time(rospy.Time.now())
                success = False
                break
            rospy.loginfo_throttle(0.1, f"2024_align_and_SCORE: waiting on {'aligning' if not self.align_done else ''} {'arm' if not self.arm_done and goal.destination == goal.AMP else ''} {'shooting' if not self.shooting_done  and goal.destination == goal.TRAP else ''}")

            # should be good to outake
            if goal.destination == goal.AMP and self.align_done and self.arm_done:
                rospy.loginfo("Drive and score 2024 - align and arm done")
                # want to outtake from claw 
                clawster_goal = Clawster2024Goal()
                clawster_goal.mode = clawster_goal.OUTTAKE
                clawster_goal.destination = clawster_goal.CLAW
                self.clawster_client.send_goal(clawster_goal)
                # we have fired the note at this point so we are done
                self._result.success = True
                self._as.set_succeeded(self._result)
                return

            if goal.destination == goal.TRAP and self.align_done and not shooting:
                rospy.loginfo("2024 drive and score - trap aligned, shooting")
                # need to shoot 
                shooting = True
                shooting_goal = behavior_actions.msg.Shooting2024Goal()
                shooting_goal.mode = shooting_goal.TRAP
                self.shooting_client.send_goal(shooting_goal, done_cb=self.shooting_done_cb)
            
            if goal.destination == goal.TRAP and self.shooting_done:
                # have sent note so we are done
                self._result.success = True
                self._as.set_succeeded(self._result)
                rospy.loginfo('%s: Succeeded Trap (hopefully)' % self._action_name)
                return

            self._as.publish_feedback(self._feedback)
            r.sleep()

        if success:
            r.sleep()
            self._result.success = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('drive_and_score')
    server = DriveAndScore(rospy.get_name())
    rospy.spin()