#!/usr/bin/env python3
import rospy
import actionlib

from behavior_actions.msg import Elevater2025Action, Elevater2025Goal, Elevater2025Feedback, Elevater2025Result
from talon_state_msgs.msg import TalonFXProState
from controllers_2025_msgs.srv import ElevatorSrv, ElevatorSrvRequest

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

        self.tolerance = rospy.get_param("tolerance")

        self.server = actionlib.SimpleActionServer(name, Elevater2025Action, execute_cb=self.execute_cb, auto_start = False)
        self.server.start()

    def talonfxpro_states_cb(self, states: TalonFXProState):
        if self.elevator_idx == None:
            for i in range(len(states.name)):
                if states.name[i] == self.elevator_joint_name:
                    self.current_position = states.position[i]
                    self.elevator_idx = i
                    break
        else:
            self.current_position = states.position[self.elevator_idx]

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