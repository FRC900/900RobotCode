#! /usr/bin/env python3

import rospy
from behavior_actions.msg import Intaking2024Action, Intaking2024Goal, Intaking2024Feedback, Intaking2024Result
from behavior_actions.msg import ShooterPivot2024Action, ShooterPivot2024Goal, ShooterPivot2024Feedback, ShooterPivot2024Result
from behavior_actions.msg import NoteDiverterAction, NoteDiverterFeedback, NoteDiverterResult, NoteDiverterGoal
from behavior_actions.msg import Clawster2024Action, Clawster2024Feedback, Clawster2024Result, Clawster2024Goal
from behavior_actions.msg import Arm2024Action, Arm2024Feedback, Arm2024Result, Arm2024Goal
from talon_controller_msgs.srv import Command, CommandRequest, CommandResponse
from talon_state_msgs.msg import TalonFXProState

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
# Brings in the SimpleActionClient
import actionlib
from std_msgs.msg import Float64

class Intaking2024Server(object):
    # create messages that are used to publish feedback/result
    feedback = Intaking2024Feedback()
    result = Intaking2024Result()

    def dyn_rec_callback(self, config, level):
        rospy.loginfo("Received reconf call: " + str(config))
        self.intaking_speed = config["intaking_speed"]
        self.current_threshold = config["current_threshold"]
        return config

    def __init__(self, name):
        self.action_name = name
        
        #self.arm_client = actionlib.SimpleActionClient('/arm/move_arm_server_2024', Arm2024Action)
        #rospy.loginfo("2024_intaking_server: waiting for arm server")
        #self.arm_client.wait_for_server()
        rospy.logerr("ARM CLIENT NOT INITALIZED BECAUSE IT DOESN'T EXIST")

        self.shooter_pivot_client = actionlib.SimpleActionClient('/shooter/set_shooter_pivot', ShooterPivot2024Action)
        rospy.loginfo("2024_intaking_server: waiting for shooter pivot server")
        self.shooter_pivot_client.wait_for_server()
        self.diverter_client = actionlib.SimpleActionClient('/diverter/diverter_server_2024', NoteDiverterAction)
        rospy.loginfo("2024_intaking_server: waiting for diverter server")
        self.diverter_client.wait_for_server()
        self.clawster_client = actionlib.SimpleActionClient('/clawster/clawster_server_2024', Clawster2024Action)
        rospy.loginfo("2024_intaking_server: waiting for clawster server")
        self.clawster_client.wait_for_server()
        rospy.loginfo(f"Clawster client {self.clawster_client}")

        self.pivot_position = 0
        self.pivot_index = None

        self.shooter_pos_sub = rospy.Subscriber("/frcrobot_jetson/talonfxpro_states", TalonFXProState, self.shooter_pivot_callback)
        self.intake_client = rospy.ServiceProxy("/frcrobot_jetson/intake_talonfxpro_controller/command", Command)

        ddynrec = DDynamicReconfigure("intaking_dyn_rec")
        ddynrec.add_variable("intaking_speed", "float/double variable", rospy.get_param("intaking_speed"), 0.0, 1.0)
        ddynrec.add_variable("current_threshold", "float/double variable", rospy.get_param("current_threshold"), 0.0, 200.0)
        ddynrec.start(self.dyn_rec_callback)

        self.intaking_speed = rospy.get_param("intaking_speed")
        self.intaking_timeout = rospy.get_param("intaking_timeout")
        self.safe_shooter_angle = rospy.get_param("safe_shooter_angle")

        self.current_threshold = rospy.get_param("current_threshold")
        
        self.intaking_current = 0.0
        self.intaking_talon_idx = None
        self.talonfxpro_sub = rospy.Subscriber('/frcrobot_jetson/talonfxpro_states', TalonFXProState, self.talonfxpro_states_cb)

        self.server = actionlib.SimpleActionServer(self.action_name, Intaking2024Action, execute_cb=self.execute_cb, auto_start = False)
        self.server.start()
            
    def shooter_pivot_callback(self, data):
        if self.pivot_index is None:
            for i in range(len(data.name)):
                if (data.name[i] == "shooter_pivot_motionmagic_joint"): 
                    self.pivot_position = data.position[i]
                    self.pivot_index = i
                    break
        else:
            self.pivot_position = data.position[self.pivot_index]

    def talonfxpro_states_cb(self, states: TalonFXProState):
        rospy.loginfo_throttle(5, "Intaking node recived talonfx pro states")
        if (self.intaking_talon_idx == None):
            for i in range(len(states.name)):
                #rospy.loginfo(data.name[i])
                if (states.name[i] == "intake"): 
                    self.intaking_current = states.torque_current[i]
                    self.intaking_talon_idx = i
                    break
        else:
            self.intaking_current = states.torque_current[self.intaking_talon_idx]
 
    def execute_cb(self, goal: Intaking2024Goal):
        r = rospy.Rate(60)

        self.feedback.state = self.feedback.SHOOTERPIVOTING
        self.server.publish_feedback(self.feedback)

        if self.pivot_position > self.safe_shooter_angle:
            pivot_goal = ShooterPivot2024Goal()
            pivot_goal.pivot_position = self.safe_shooter_angle
            self.shooter_pivot_client.send_goal(pivot_goal)
            while self.pivot_position > self.safe_shooter_angle:
                if self.server.is_preempt_requested():
                    rospy.loginfo("2024_intaking_server: preempted")
                    self.shooter_pivot_client.cancel_goals_at_and_before_time(rospy.Time.now())
                    self.server.set_preempted()
                    return
                r.sleep()
        
            self.shooter_pivot_client.cancel_goals_at_and_before_time(rospy.Time.now())

        self.feedback.state = self.feedback.DIVERTING
        self.server.publish_feedback(self.feedback)

        diverter_goal = NoteDiverterGoal()
        if goal.destination == goal.SHOOTER:
            diverter_goal.mode = diverter_goal.TO_SHOOTER 
        elif goal.destination == goal.CLAW:
            diverter_goal.mode = diverter_goal.TO_CLAW
        rospy.loginfo("2024_intaking_server: running diverter")
        self.diverter_client.send_goal(diverter_goal)

        self.feedback.state = self.feedback.CLAWSTERING
        self.server.publish_feedback(self.feedback)

        clawster_goal = Clawster2024Goal()
        clawster_goal.mode = clawster_goal.INTAKE
        clawster_done = False
        clawster_result: Clawster2024Result = None
        def clawster_done_cb(state, result):
            nonlocal clawster_done, clawster_result
            clawster_done = True
            clawster_result = result
        
        if goal.destination == goal.SHOOTER:
            clawster_goal.destination = clawster_goal.PRESHOOTER
            rospy.loginfo("2024_intaking_server: running preshooter")
        elif goal.destination == goal.CLAW:
            rospy.loginfo("2024_intaking_server: running claw")
            clawster_goal.destination = clawster_goal.CLAW

        self.clawster_client.send_goal(clawster_goal, done_cb=clawster_done_cb)

        self.feedback.state = self.feedback.INTAKING
        self.server.publish_feedback(self.feedback)

        intake_srv = CommandRequest()
        intake_srv.command = self.intaking_speed
        self.intake_client.call(intake_srv)

        start = rospy.Time.now()
        # if run until preempt want to just go for the entire auto
        while goal.run_until_preempt or (not (clawster_done or rospy.is_shutdown() or (rospy.Time.now() - start).to_sec() > self.intaking_timeout)):
            rospy.loginfo_throttle(0.5, f"2024_intaking_server: waiting for {'preshooter' if goal.destination == goal.SHOOTER else 'claw'}")
            if self.intaking_current > self.current_threshold:
                rospy.logwarn(f"Intaking current = {self.intaking_current} is above {self.current_threshold}")
                self.feedback.note_hit_intake = True
                self.server.publish_feedback(self.feedback)
                
            if self.server.is_preempt_requested():
                rospy.loginfo("2024_intaking_server: preempted")

                # stop intake
                intake_srv = CommandRequest()
                intake_srv.command = 0.0
                self.intake_client.call(intake_srv)

                # stop diverter
                self.diverter_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.clawster_client.cancel_goals_at_and_before_time(rospy.Time.now())
                # stop preshooter and claw
                self.server.set_preempted()
                return
            r.sleep()
        
        # stop intake
        intake_srv = CommandRequest()
        intake_srv.command = 0.0
        self.intake_client.call(intake_srv)

        # stop diverter
        self.diverter_client.cancel_goals_at_and_before_time(rospy.Time.now())
        # stop preshooter and claw
        self.clawster_client.cancel_goals_at_and_before_time(rospy.Time.now())

        if clawster_done:
            if clawster_result.has_game_piece:
                rospy.loginfo("2024_intaking_server: succeeded")
                self.result.success = True
                self.server.set_succeeded(self.result)
                return
            else:
                rospy.loginfo("2024_intaking_server: clawster done, but no game piece??")
                self.result.success = False
                self.server.set_succeeded(self.result)
                return

        if (rospy.Time.now() - start).to_sec() > self.intaking_timeout:
            rospy.loginfo("2024_intaking_server: timed out")
            self.result.success = False
            self.server.set_succeeded(self.result)
            return

        rospy.loginfo(f"2024_intaking_server: we haven't succeeded, so we've failed. is rospy shutdown? {rospy.is_shutdown()}")
        self.result.success = False
        self.server.set_succeeded(self.result)
        

       
if __name__ == '__main__':
    rospy.init_node('intaking_server_2024')
    server = Intaking2024Server(rospy.get_name())
    rospy.spin()