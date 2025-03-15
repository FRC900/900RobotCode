#!/usr/bin/env python3
import rospy
import actionlib

from behavior_actions.msg import Elevater2025Action, Elevater2025Goal, Elevater2025Feedback, Elevater2025Result
from talon_state_msgs.msg import TalonFXProState
from controllers_2025_msgs.srv import ElevatorSrv, ElevatorSrvRequest
from sensor_msgs.msg import JointState
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

class Elevater2025ActionServer(object):

    feedback = Elevater2025Feedback()
    result = Elevater2025Result()

    def __init__(self, name):
        self.elevator_client = rospy.ServiceProxy(f"/frcrobot_jetson/{rospy.get_param('controller_name')}/elevator_service", ElevatorSrv)
        self.talonfxpro_states_sub = rospy.Subscriber("/frcrobot_jetson/talonfxpro_states", TalonFXProState, self.talonfxpro_states_cb, tcp_nodelay=True)

        self.elevator_joint_name = rospy.get_param("joint_name")
        self.elevator_idx = None
        self.current_position = -1

        self.intake_pos = rospy.get_param("intake")
        self.L1_pos = rospy.get_param("L1")
        self.L2_pos = rospy.get_param("L2")
        self.L3_pos = rospy.get_param("L3")
        self.L4_pos = rospy.get_param("L4")

        ddynrec = DDynamicReconfigure("elevater_dyn_rec")
        ddynrec.add_variable("intake_pos", "float/double variable", self.intake_pos, 0.0, 2.0)
        ddynrec.add_variable("L1_pos", "float/double variable", self.L1_pos, 0.0, 2.0)
        ddynrec.add_variable("L2_pos", "float/double variable", self.L2_pos, 0.0, 2.0)
        ddynrec.add_variable("L3_pos", "float/double variable", self.L3_pos, 0.0, 2.0)
        ddynrec.add_variable("L4_pos", "float/double variable", self.L4_pos, 0.0, 2.0)
        ddynrec.start(self.dyn_rec_callback)

        self.tolerance = rospy.get_param("tolerance")

        self.avoid_elevator_switch_name = rospy.get_param("avoid_elevator_switch")
        self.roller_limit_switch_name = rospy.get_param("outtake_switch")
        
        self.avoid_elevator_switch_val = 0
        self.roller_limit_switch_val = 0
        self.joint_state_sub = rospy.Subscriber("/frcrobot_rio/joint_states", JointState, callback=self.rio_callback, tcp_nodelay=True)

        self.server = actionlib.SimpleActionServer(name, Elevater2025Action, execute_cb=self.execute_cb, auto_start = False)
        self.server.start()

    def dyn_rec_callback(self, config, level):
        rospy.loginfo("Received reconf call: " + str(config))
        self.intake_pos = config["intake_pos"]
        self.L1_pos = config["L1_pos"]
        self.L2_pos = config["L2_pos"]
        self.L3_pos = config["L3_pos"]
        self.L4_pos = config["L4_pos"]

        return config

    def rio_callback(self, data):
        if self.avoid_elevator_switch_name in data.name and self.roller_limit_switch_name in data.name:
            self.avoid_elevator_switch_val = data.position[data.name.index(self.avoid_elevator_switch_name)]
            self.roller_limit_switch_val = data.position[data.name.index(self.roller_limit_switch_name)]
        else:
            rospy.logwarn_throttle(1.0, f'2025_elevater_server: switch not found')

    def talonfxpro_states_cb(self, states: TalonFXProState):
        if self.elevator_idx == None:
            for i in range(len(states.name)):
                if states.name[i] == self.elevator_joint_name:
                    self.current_position = states.position[i]
                    self.elevator_idx = i
                    break
        else:
            self.current_position = states.position[self.elevator_idx]
    
    def safe_to_send_elevator(self) -> bool:
        # return (not self.avoid_elevator_switch_val)
        return True

    def execute_cb(self, goal: Elevater2025Goal):
        r = rospy.Rate(50)

        target_pos_str = None
        target_pos = None

        if goal.mode == goal.INTAKE:
            target_pos_str = "INTAKE"
            target_pos = self.intake_pos
        elif goal.mode == goal.L1:
            target_pos_str = "L1"
            target_pos = self.L1_pos
        elif goal.mode == goal.L2:
            target_pos_str = "L2"
            target_pos = self.L2_pos
        elif goal.mode == goal.L3:
            target_pos_str = "L3"
            target_pos = self.L3_pos
        elif goal.mode == goal.L4:
            target_pos_str = "L4"
            target_pos = self.L4_pos
        else:
            rospy.logerr(f"2025_elevater_server: INVALID MODE {goal.mode}")
            self.result.success = False
            self.server.set_aborted(self.result)
            return

        # means we are going up, so limit switches must be in correct configuration (i.e off on avoid elevator and on roller)
        rospy.loginfo(f"Roller limit switch state {self.roller_limit_switch_val} avoid state {self.avoid_elevator_switch_val}")
        rospy.loginfo(f"Safe to run elevator {self.safe_to_send_elevator()}")
        if goal.mode != goal.INTAKE:
            start = rospy.Time.now()
            while not ((rospy.Time.now() - start).to_sec() > 1.0):
                if self.safe_to_send_elevator():
                    rospy.loginfo("Limit switches correct!, Bringing elevator up")
                    break
                else:
                    rospy.logwarn_throttle(0.5, "Not bringing elevator up because limit switches not correct! Waiting up to 1 second")
                r.sleep()
            if not self.safe_to_send_elevator():
                rospy.logerr("Elevator timeout failed, check limit switches, exiting")
                self.result.success = False
                self.server.set_aborted(self.result)
                return

        rospy.loginfo(f"2025_elevater_server: going to position {target_pos_str} = {target_pos} m")
        req = ElevatorSrvRequest()
        req.position = target_pos
        self.elevator_client.call(req)

        while not rospy.is_shutdown():
            self.feedback.current_height = self.current_position
            self.feedback.target_height = target_pos
            self.server.publish_feedback(self.feedback)
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                rospy.logwarn("2025_elevater_server: preempted!")
                req.position = self.current_position
                self.elevator_client.call(req)
                self.result.success = False
                self.server.set_aborted(self.result)
                return
            if abs(self.current_position - target_pos) < self.tolerance:
                rospy.loginfo("2025_elevater_server: succeeded (yay!)")
                self.result.success = True
                self.server.set_succeeded(self.result)
                return
            r.sleep()
        
        # if we exit this loop, rospy has presumably shutdown.
        self.server.set_succeded(self.result) # /shrug

if __name__ == '__main__':
    rospy.init_node('elevater_server_2025')
    
    server = Elevater2025ActionServer(rospy.get_name())
    rospy.spin()
