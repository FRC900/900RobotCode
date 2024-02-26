#! /usr/bin/env python3

import rospy
from behavior_actions.msg import Intaking2024Action, Intaking2024Goal, Intaking2024Feedback, Intaking2024Result
from behavior_actions.msg import DriveToObjectAction, DriveToObjectGoal, DriveToObjectFeedback, DriveToObjectResult
from behavior_actions.msg import DriveObjectIntake2024Action, DriveObjectIntake2024Goal, DriveObjectIntake2024Feedback, DriveObjectIntake2024Result

from talon_controller_msgs.srv import Command, CommandRequest, CommandResponse
# Brings in the SimpleActionClient
import actionlib
from std_msgs.msg import Float64

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
        self.timeout_ = rospy.get_param("timeout")
        self.intake_server_done = False
        self.intake_server_success = False
        self.server = actionlib.SimpleActionServer(self.action_name, DriveObjectIntake2024Action, execute_cb=self.execute_cb, auto_start = False)
        rospy.loginfo(f"2024_drive_object_intake: started action server with parameters\nDistance away from note {self.distance_away_} \
                    tolerance: {self.tolerance_} note id {self.note_name_} timeout {self.timeout_}")
        self.server.start()
        
    
    def preempt_servers(self):
        rospy.logwarn("2024_drive_object_intake server: preempted")
        # stop drive to object
        self.drive_to_object_client.cancel_goals_at_and_before_time(rospy.Time.now())
        # stop intaking
        self.intaking_client.cancel_goals_at_and_before_time(rospy.Time.now())

    def execute_cb(self, goal: DriveObjectIntake2024Goal):
        self.intake_server_done = False
        self.intake_server_success = False # set to true by intaking server when note hits intake (need to add current sensing)

        intaking_goal = Intaking2024Goal()
        intaking_goal.destination = goal.destination
        
        def intaking_result(state, intaking_result: Intaking2024Result):
            rospy.loginfo(f"Intaking server finished with {intaking_result.success} ")
            # *Think* no race condition because done is set after success
            self.intake_server_success = intaking_result.success
            self.intake_server_done = True

        self.intaking_client.send_goal(intaking_goal, done_cb=intaking_result)
        rospy.loginfo("2024_drive_object_intake: Sending intaking goal")
        
        drive_to_object_goal = DriveToObjectGoal()
        drive_to_object_goal.tolerance = self.tolerance_
        drive_to_object_goal.id = self.note_name_ 
        drive_to_object_goal.distance_away = self.distance_away_
        
        def drive_object_feedback(drive_feedback: DriveToObjectFeedback):
            # forward up drive to object feedback
            self.feedback.angle_error = drive_feedback.angle_error
            self.feedback.x_error = drive_feedback.x_error
            self.feedback.y_error = drive_feedback.y_error
            self.server.publish_feedback(self.feedback)
        
        def drive_object_result(state, drive_result: DriveToObjectResult):
            if not drive_result.success: 
                rospy.logwarn("Drive to object failed, Continuing Intake")
                # not a failure, but do need to let high up code know
                self.feedback.DRIVING_TO_NOTE_FAILED = True
                self.server.publish_feedback(self.feedback)

        self.drive_to_object_client.send_goal(drive_to_object_goal, done_cb=drive_object_result, feedback_cb=drive_object_feedback)

        start = rospy.Time.now()
        r = rospy.Rate(10)
        while not (self.intake_server_done or rospy.is_shutdown() or (rospy.Time.now() - start).to_sec() > self.timeout_):
            if self.server.is_preempt_requested():
                self.preempt_servers() # preempts all actionlib servers
                self.server.set_preempted()
                return
            r.sleep()
        
        rospy.loginfo(f"Intake server done {self.intake_server_done} result {self.intake_server_success} Past timeout: {(rospy.Time.now() - start).to_sec() > self.timeout_}")
        self.preempt_servers()
        self.result.success = self.intake_server_success # set above with intaking result
        self.server.set_succeeded(self.result)
       
if __name__ == '__main__':
    rospy.init_node('drive_to_and_intake_server_2024')
    server = DriveObjectIntakeServer(rospy.get_name())
    rospy.spin()