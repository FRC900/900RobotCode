#! /usr/bin/env python3

import rospy
from behavior_actions.msg import Intaking2025Action, Intaking2025Goal, Intaking2025Feedback, Intaking2025Result
from talon_controller_msgs.srv import Command, CommandRequest, CommandResponse
from talon_state_msgs.msg import TalonFXProState
from sensor_msgs.msg import JointState

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
# Brings in the SimpleActionClient
import actionlib
from std_msgs.msg import Float64

SIM = False

class Intaking2025Server(object):
    # create messages that are used to publish feedback/result
    feedback = Intaking2025Feedback()
    result = Intaking2025Result()

    def dyn_rec_callback(self, config, level):
        rospy.loginfo("Received reconf call: " + str(config))
        self.intaking_speed = config["intaking_speed"]
        self.current_threshold = config["current_threshold"]
        return config

    def __init__(self, name):
        self.action_name = name

        self.pivot_position = 0
        self.pivot_index = None

        self.intake_client = rospy.ServiceProxy("/frcrobot_jetson/intake_talonfxpro_controller/command", Command)

        ddynrec = DDynamicReconfigure("intaking_dyn_rec")
        ddynrec.add_variable("intaking_speed", "float/double variable", rospy.get_param("intaking_speed"), 0.0, 13.0)
        ddynrec.add_variable("current_threshold", "float/double variable", rospy.get_param("current_threshold"), 0.0, 200.0)
        ddynrec.start(self.dyn_rec_callback)

        self.intaking_speed = rospy.get_param("intaking_speed")
        self.intaking_timeout = rospy.get_param("intaking_timeout")

        self.current_threshold = rospy.get_param("current_threshold")

        self.outtaking_speed = rospy.get_param("outtaking_speed")
        self.outtaking_time = rospy.get_param("outtaking_time")

        self.backwards_time = rospy.get_param("backwards_time")
        
        self.intaking_current = 0.0
        self.intaking_talon_idx = None
        self.talonfxpro_sub = rospy.Subscriber('/frcrobot_jetson/talonfxpro_states', TalonFXProState, self.talonfxpro_states_cb)

        self.joint_state_sub = rospy.Subscriber("/frcrobot_rio/joint_states", JointState, callback=self.rio_callback, tcp_nodelay=True)
        self.diverter_switch = False
        self.run_intake_backwards = None
        self.server = actionlib.SimpleActionServer(self.action_name, Intaking2025Action, execute_cb=self.execute_cb, auto_start = False)
        self.server.start()

    def rio_callback(self, data):
        # check diverter_switch
        if "diverter_limit_switch" in data.name:
            if self.feedback.note_hit_intake != data.position[data.name.index("diverter_limit_switch")]:
                rospy.loginfo(f'Intaking, note hit (or left) diverter self.feedback.note_hit_intake {self.feedback.note_hit_intake} data.position[data.name.index("diverter_limit_switch")] {data.position[data.name.index("diverter_limit_switch")]}')
                self.feedback.note_hit_intake = bool(data.position[data.name.index("diverter_limit_switch")])
                self.server.publish_feedback(self.feedback)
        else:
            rospy.logwarn_throttle(1.0, f'2025_intaking_server: diverter_limit_switch not found')
            pass
            
    def talonfxpro_states_cb(self, states: TalonFXProState):
        #rospy.loginfo_throttle(5, "Intaking node recived talonfx pro states")
        if (self.intaking_talon_idx == None):
            for i in range(len(states.name)):
                #rospy.loginfo(data.name[i])
                if (states.name[i] == "intake"): 
                    self.intaking_current = states.torque_current[i]
                    self.intaking_talon_idx = i
                    break
        else:
            self.intaking_current = states.torque_current[self.intaking_talon_idx]
    

    def execute_cb(self, goal: Intaking2025Goal):
        r = rospy.Rate(60)

        self.run_intake_backwards = None

        # self.feedback.have_note = False

        if SIM:
            rospy.loginfo(f"2024_intaking_server: SIMULATION MODE, goal = {goal}")
            import time
            time.sleep(2)
            # say that a note hit the intake
            self.feedback.note_hit_intake = True
            self.server.publish_feedback(self.feedback)
            # loop for 0.5 seconds and return if preempted
            start = time.time()
            while time.time() - start < 0.5:
                if self.server.is_preempt_requested() or rospy.is_shutdown():
                    rospy.loginfo("2024_intaking_server: preempted")
                    self.server.set_preempted()
                    return
                r.sleep()
            self.result.success = True
            self.server.set_succeeded(self.result)
            return

        if goal.destination == goal.OUTTAKE:
            intake_srv = CommandRequest()
            intake_srv.command = -self.outtaking_speed
            self.intake_client.call(intake_srv)

            start = rospy.Time.now()

            while goal.run_until_preempt or (not (rospy.is_shutdown() or (rospy.Time.now() - start).to_sec() > self.outtaking_time)):
                rospy.loginfo_throttle(0.5, f"2024_intaking_server: outtaking")
                    
                if self.server.is_preempt_requested():
                    rospy.loginfo("2024_intaking_server: preempted")

                    # stop intake
                    intake_srv = CommandRequest()
                    intake_srv.command = 0.0
                    self.intake_client.call(intake_srv)

                    self.server.set_preempted()
                    return
                r.sleep()
            
            # stop intake
            intake_srv = CommandRequest()
            intake_srv.command = 0.0
            self.intake_client.call(intake_srv)

            return

        self.feedback.state = self.feedback.SHOOTERPIVOTING
        self.server.publish_feedback(self.feedback)

        self.feedback.state = self.feedback.INTAKING
        self.server.publish_feedback(self.feedback)

        intake_srv = CommandRequest()
        intake_srv.command = self.intaking_speed
        orig_command = intake_srv.command
        self.intake_client.call(intake_srv)

        backwards_start_time = None

        start = rospy.Time.now()
        # if run until preempt want to just go for the entire auto
        while goal.run_until_preempt or (not (clawster_done or rospy.is_shutdown() or (rospy.Time.now() - start).to_sec() > self.intaking_timeout)):
            rospy.loginfo_throttle(0.5, f"2024_intaking_server: waiting for {'preshooter' if goal.destination == goal.SHOOTER else 'claw'}")
            if self.run_intake_backwards is not None and rospy.Time.now() > self.run_intake_backwards:
                rospy.loginfo("Running backwards!")
                intake_srv.command = -self.outtaking_speed
                self.intake_client.call(intake_srv)
                self.run_intake_backwards = None
                backwards_start_time = rospy.Time.now()
            if backwards_start_time is not None and rospy.Time.now() > backwards_start_time + rospy.Duration(self.backwards_time):
                rospy.loginfo("Running forward again!")
                intake_srv.command = orig_command
                self.intake_client.call(intake_srv)
                backwards_start_time = None
            if self.intaking_current > self.current_threshold:
                rospy.logwarn(f"Intaking current = {self.intaking_current} is above {self.current_threshold}")
                # self.feedback.note_hit_intake = True
                self.server.publish_feedback(self.feedback)
            # if clawster_done:
            #     self.feedback.have_note = True
            #     self.server.publish_feedback(self.feedback)
            
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
    server = Intaking2025Server(rospy.get_name())
    rospy.spin()