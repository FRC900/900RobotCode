#!/usr/bin/env python3

import rospy
from behavior_actions.msg import Roller2025Action, Roller2025Goal, Roller2025Feedback, Roller2025Result
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
# from talon_controller_msgs.srv import Command, CommandRequest, CommandResponse
from controllers_2025_msgs.srv import RollerSrv, RollerSrvRequest
# from talon_state_msgs.msg import TalonFXProState
from frc_msgs.srv import RumbleCommand, RumbleCommandRequest
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
import actionlib
import time

class Roller2025ActionServer(object):
    # create messages that are used to publish feedback/result
    _result = Roller2025Result()

    def __init__(self, name):
        self.roller_client = rospy.ServiceProxy(f"/frcrobot_jetson/{rospy.get_param('controller_name')}/roller_service", RollerSrv)
        self.rumble_client = rospy.ServiceProxy( "/frcrobot_rio/rumble_controller/command", RumbleCommand)
        self._action_name = name
        '''
            roller_speed: 0.4
            controller_name: "roller_controller"
            switch_name: "roller_limit_switch"
        '''
        self.switch = 0
        self.avoid_elevator_switch = 0
        self.switch_name = rospy.get_param('switch_name')
        self.avoid_elevator_switch_name = rospy.get_param('avoid_elevator_switch_name')
        self.intake_roller_speed = rospy.get_param('intake_roller_speed')
        self.l1_roller_speed = rospy.get_param('l1_roller_speed')
        self.l2_roller_speed = rospy.get_param('l2_roller_speed')
        self.l3_roller_speed = rospy.get_param('l3_roller_speed')
        self.l4_roller_speed = rospy.get_param('l4_roller_speed')
        self.slow_roller_speed = rospy.get_param('slow_roller_speed')
        self.time_to_wait = rospy.get_param("time_to_wait")
        self.switch_sub = rospy.Subscriber("/frcrobot_rio/joint_states", JointState, self.callback, tcp_nodelay=True)

        self.last_touched_diverter = rospy.Time()

        ddynrec = DDynamicReconfigure("roller_dyn_rec") 
        ddynrec.add_variable("intake_roller_speed", "float/double variable", self.intake_roller_speed, 0.0, 4.0)
        ddynrec.add_variable("l1_roller_speed", "float/double variable", self.l1_roller_speed, 0.0, 4.0)
        ddynrec.add_variable("l2_roller_speed", "float/double variable", self.l2_roller_speed, 0.0, 4.0)
        ddynrec.add_variable("l3_roller_speed", "float/double variable", self.l3_roller_speed, 0.0, 4.0)
        ddynrec.add_variable("l4_roller_speed", "float/double variable", self.l4_roller_speed, 0.0, 4.0)

        ddynrec.start(self.dyn_rec_callback)

        rospy.loginfo(f"intake roller speed: {self.intake_roller_speed}, 2025_roller_server: switch name: {self.switch_name}, avoid elevator switch name: {self.avoid_elevator_switch_name}, L1 roller speed: {self.l1_roller_speed}, L2 roller speed: {self.l2_roller_speed}, L3 roller speed: {self.l3_roller_speed}, L4 roller speed: {self.l4_roller_speed}")

        self._as = actionlib.SimpleActionServer(self._action_name, Roller2025Action, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def dyn_rec_callback(self, config, level):
        rospy.loginfo("Received reconf call: " + str(config))
        self.intake_roller_speed = config["intake_roller_speed"]
        self.l1_roller_speed = config["l1_roller_speed"]
        self.l2_roller_speed = config["l2_roller_speed"]
        self.l3_roller_speed = config["l3_roller_speed"]
        self.l4_roller_speed = config["l4_roller_speed"]

        return config
    
    def execute_cb(self, goal):
        pct_out = Float64()
        success = True
        r = rospy.Rate(50)
        
        if goal.mode == goal.INTAKE:
            rospy.loginfo("2025_roller_server: Roller intaking!")
            pct_out.data = self.intake_roller_speed
            self.roller_client.call(RollerSrvRequest(pct_out.data))

            # wait for first switch, then slow down speed
            while self.avoid_elevator_switch == 0 and not rospy.is_shutdown():
                if self._as.is_preempt_requested():
                    self._as.set_preempted()
                    rospy.logwarn("2025_roller_server: Preempted!")
                    pct_out.data = 0
                    self.roller_client.call(RollerSrvRequest(pct_out.data))
                    success = False
                    return
                rospy.loginfo_throttle(1, f"Waiting for avoid elevator to become 0 (first one) {self.avoid_elevator_switch}, center switch {self.switch}")
                r.sleep()
            
            rospy.loginfo(f"2025_roller_server: first switch hit, slowing down to {self.slow_roller_speed}")

            pct_out.data = self.slow_roller_speed
            self.roller_client.call(RollerSrvRequest(pct_out.data))

            # make sure we get it
            while self.switch == 0 and not rospy.is_shutdown():
                if self._as.is_preempt_requested():
                    self._as.set_preempted()
                    rospy.logwarn("2025_roller_server: Preempted!")
                    pct_out.data = 0
                    self.roller_client.call(RollerSrvRequest(pct_out.data))
                    success = False
                    return
                rospy.loginfo_throttle(1, f"Waiting for center swith to become 0 {self.avoid_elevator_switch}, center switch {self.switch}")
                r.sleep()

            # wait for first switch to be off so we don't intersect elevator
            while self.avoid_elevator_switch == 1 and not rospy.is_shutdown():
                if self._as.is_preempt_requested():
                    self._as.set_preempted()
                    rospy.logwarn("2025_roller_server: Preempted!")
                    pct_out.data = 0
                    self.roller_client.call(RollerSrvRequest(pct_out.data))
                    success = False
                    return
                rospy.loginfo_throttle(1, f"Waiting for elevator avoid switch avoid switch second one {self.avoid_elevator_switch}, center switch {self.switch}")
                r.sleep()

            pct_out.data = 0
            self.roller_client.call(RollerSrvRequest(pct_out.data))
            if success:
                self._result.success = True
            else:
                self._result.success = False

        else:
            rospy.loginfo("2025_roller_server: Roller outtaking!")
            if goal.mode == goal.L4:
                pct_out.data = self.l4_roller_speed
            elif goal.mode == goal.L3:
                pct_out.data = self.l3_roller_speed
            elif goal.mode == goal.L2:
                pct_out.data = self.l2_roller_speed
            elif goal.mode == goal.L1:
                pct_out.data = self.l1_roller_speed
            self.roller_client.call(RollerSrvRequest(pct_out.data))
            while self.switch == 1 and not rospy.is_shutdown():
                if self._as.is_preempt_requested():
                    self._as.set_preempted()
                    rospy.logwarn("2025_roller_server: Preempted!")
                    pct_out.data = 0
                    self.roller_client.call(RollerSrvRequest(pct_out.data))
                    success = False
                    return
                r.sleep()
            time.sleep(self.time_to_wait)
            pct_out.data = 0
            self.roller_client.call(RollerSrvRequest(pct_out.data))
            if success:
                self._result.success = False
            else:
                self._result.success = True

        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)

        # haptic feedback on intake/outtake
        self.rumble_client.call(RumbleCommandRequest(65535, 65535))

        time.sleep(0.25)

        self.rumble_client.call(RumbleCommandRequest(0, 0))

    def callback(self, data):
        if self.switch_name in data.name:
            self.switch = data.position[data.name.index(self.switch_name)]
            #rospy.loginfo(f"Found {self.switch_name} with value {self.switch}")
        else:
            rospy.logwarn_throttle(1.0, f"2025_roller_server: {self.switch_name} not found")
            pass

        if self.avoid_elevator_switch_name in data.name:
            self.avoid_elevator_switch = data.position[data.name.index(self.avoid_elevator_switch_name)]
        else:
            rospy.logwarn_throttle(1.0, f"2025_roller_server: {self.avoid_elevator_switch_name} not found")

if __name__ == '__main__':
    rospy.init_node('roller_server_2025')
    server = Roller2025ActionServer(rospy.get_name())
    rospy.spin()