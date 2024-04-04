#! /usr/bin/env python3

import rospy
from behavior_actions.msg import Intaking2024Action, Intaking2024Goal, Intaking2024Feedback, Intaking2024Result
from behavior_actions.msg import DriveToObjectAction, DriveToObjectGoal, DriveToObjectFeedback, DriveToObjectResult
from behavior_actions.msg import DriveObjectIntake2024Action, DriveObjectIntake2024Goal, DriveObjectIntake2024Feedback, DriveObjectIntake2024Result

from talon_controller_msgs.srv import Command, CommandRequest, CommandResponse
# Brings in the SimpleActionClient
import actionlib
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class DriveObjectIntakeServer(object):
    # create messages that are used to publish feedback/result
    feedback = DriveObjectIntake2024Feedback()
    result = DriveObjectIntake2024Result()

    def __init__(self, name):
        self.action_name = name
        
        self.intaking_client = actionlib.SimpleActionClient('/intaking/intaking_server_2024', Intaking2024Action)
        rospy.loginfo("2024_drive_object_intake: waiting for intaking server")
        self.intaking_client.wait_for_server()

        self.drive_to_object_client = actionlib.SimpleActionClient('/drive_to_object/drive_to_object', DriveToObjectAction)
        rospy.loginfo("2024_drive_object_intake: waiting for drive to object server")
        self.drive_to_object_client.wait_for_server()

        self.distance_away_ = rospy.get_param("distance_away")
        self.tolerance_ = rospy.get_param("drive_object_tolerance")
        self.note_name_ = rospy.get_param("note_str") # maybe it changes for some other reason
        self.drive_timeout_ = rospy.get_param("drive_timeout")
        self.intake_timeout_ = rospy.get_param("intake_timeout")
        self.drive_back_time_ = rospy.get_param("drive_back_time")
        self.drive_back_speed_ = rospy.get_param("drive_back_speed")
        self.max_angle_away_ = rospy.get_param("max_angle_away")
        self.cmd_vel_pub = rospy.Publisher("/auto_note_align/cmd_vel", Twist, queue_size=1, tcp_nodelay=True)
        self.intake_server_done = False
        self.intake_server_success = False
        self.server = actionlib.SimpleActionServer(self.action_name, DriveObjectIntake2024Action, execute_cb=self.execute_cb, auto_start = False)
        rospy.loginfo(f"2024_drive_object_intake: started action server with parameters\nDistance away from note {self.distance_away_} \
                    tolerance: {self.tolerance_}, note id {self.note_name_}, drive timeout {self.drive_timeout_}, intake timeout {self.intake_timeout_}")
        self.server.start()
        
    

    def preempt_servers(self):
        rospy.logwarn("2024_drive_object_intake server: preempted")
        # stop drive to object
        self.drive_to_object_client.cancel_goals_at_and_before_time(rospy.Time.now())
        # stop intaking
        self.intaking_client.cancel_goals_at_and_before_time(rospy.Time.now())

    def intaking_result(self, state, intaking_result: Intaking2024Result):
        rospy.loginfo(f"Intaking server finished with {intaking_result.success} ")
        # *Think* no race condition because done is set after success
        self.intake_server_success = intaking_result.success
        self.intake_server_done = True

    def intaking_feedback(self, intaking_feedback: Intaking2024Feedback):
        if intaking_feedback.note_hit_intake and not self.has_hit_note:
            self.has_hit_note = True
            rospy.loginfo("Drive object intake: Note hit intake, cancelling drive to object")
            self.drive_to_object_client.cancel_goals_at_and_before_time(rospy.Time.now())

    def execute_cb(self, goal: DriveObjectIntake2024Goal):
        self.intake_server_done = False
        self.has_hit_note = False
        self.intake_server_success = False # set to true by intaking server when note hits intake (need to add current sensing)

        intaking_goal = Intaking2024Goal()
        intaking_goal.destination = goal.destination
        


        self.intaking_client.send_goal(intaking_goal, feedback_cb=self.intaking_feedback, done_cb=self.intaking_result)
        rospy.loginfo("2024_drive_object_intake: Sending intaking goal")
        
        drive_to_object_goal = DriveToObjectGoal()
        drive_to_object_goal.x_tolerance = self.tolerance_
        drive_to_object_goal.y_tolerance = self.tolerance_
        drive_to_object_goal.id = self.note_name_ 
        drive_to_object_goal.transform_to_drive = "intake"
        drive_to_object_goal.min_x_vel = 3.0 # this is faster than before
        drive_to_object_goal.min_y_vel = 0.5 # TURN THIS OFF
        drive_to_object_goal.use_y = True # use 
        drive_to_object_goal.override_goal_angle = True # used for aligning in y and rotating at the same time 
        drive_to_object_goal.fast_zone = 0.0
        drive_to_object_goal.min_y_pos = goal.min_y_pos # -5 for first blue
        drive_to_object_goal.max_y_pos = goal.max_y_pos # 0.1 for first blue
        drive_to_object_goal.max_angle = self.max_angle_away_

        drive_object_done = False
        
        def drive_object_feedback(drive_feedback: DriveToObjectFeedback):
            # forward up drive to object feedback
            self.feedback.angle_error = drive_feedback.angle_error
            self.feedback.x_error = drive_feedback.x_error
            self.feedback.y_error = drive_feedback.y_error
            rospy.loginfo_throttle(1, f"2024_drive_object_intake: feedback {self.feedback}")
            self.feedback.tracking_obj = drive_feedback.tracking_obj
            self.server.publish_feedback(self.feedback)
        
        def drive_object_result(state, drive_result: DriveToObjectResult):
            rospy.loginfo("DRIVE OBJECT DONE")
            nonlocal drive_object_done
            drive_object_done = True
            if not drive_result.success: 
                rospy.logwarn("Drive to object failed, Continuing Intake")
                # not a failure, but do need to let high up code know
                self.feedback.DRIVING_TO_NOTE_FAILED = True
                self.server.publish_feedback(self.feedback)

        self.drive_to_object_client.send_goal(drive_to_object_goal, done_cb=drive_object_result, feedback_cb=drive_object_feedback)

        start = rospy.Time.now()
    
        r = rospy.Rate(250)
        if goal.drive_timeout:
            rospy.loginfo(f"2024 drive object intake: setting drive timeout to {goal.drive_timeout}")
            drive_timeout = goal.drive_timeout 
        else:
            drive_timeout = self.drive_timeout_ 
            
        while not (self.has_hit_note or drive_object_done or rospy.is_shutdown() or (rospy.Time.now() - start).to_sec() > drive_timeout):
            if self.server.is_preempt_requested():
                self.preempt_servers() # preempts all actionlib servers
                self.server.set_preempted()
                return
            r.sleep()

        rospy.loginfo(f"2024_drive_object_intake: done, NOT WAITING for intake, time to drive {(rospy.Time.now() - start).to_sec()}")
        rospy.loginfo(f"Has hit note {self.has_hit_note} drive obj done {drive_object_done} time delta {(rospy.Time.now() - start).to_sec()} drive timeout {drive_timeout} start ")

        self.drive_to_object_client.cancel_goals_at_and_before_time(rospy.Time.now())
        self.result.success = True # always set to success so we don't crash auto node
        self.server.set_succeeded(self.result)

        # drive back at drive_back_speed for drive_back_time
        start = rospy.Time.now()
        twist = Twist()
        while (rospy.Time.now() - start).to_sec() < self.drive_back_time_:
            if self.server.is_preempt_requested():
                self.preempt_servers() # preempts all actionlib servers
                self.server.set_preempted()
                twist.linear.x = 0
                self.cmd_vel_pub.publish(twist)
                return
            twist.linear.x = -self.drive_back_speed_
            self.cmd_vel_pub.publish(twist)
            r.sleep()
       
if __name__ == '__main__':
    rospy.init_node('drive_to_and_intake_server_2024')
    server = DriveObjectIntakeServer(rospy.get_name())
    rospy.spin()