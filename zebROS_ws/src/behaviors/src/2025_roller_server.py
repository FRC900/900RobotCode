#!/usr/bin/env python3

import rospy
from behavior_actions.msg import Roller2025Action, Roller2025Goal, Roller2025Feedback, Roller2025Result
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from talon_controller_msgs.srv import Command, CommandRequest, CommandResponse
from talon_state_msgs.msg import TalonFXProState
import actionlib

class Roller2025ActionServer(object):
    # create messages that are used to publish feedback/result
    _result = Roller2025Result()

    def __init__(self, name):
        self.roller_client = rospy.ServiceProxy(f"/frcrobot_jetson/{rospy.get_param('controller_name')}/command", Command)
        self._action_name = name
        '''
            roller_speed: 0.4
            controller_name: "roller_controller"
            switch_name: "roller_limit_switch"
        '''
        self.switch = 0
        self.switch_name = rospy.get_param('switch_name')
        self.roller_speed = rospy.get_param('roller_speed')

        self.switch_sub = rospy.Subscriber("/frcrobot_rio/joint_states", JointState, self.callback, tcp_nodelay=True)

        self.last_touched_diverter = rospy.Time()

        rospy.loginfo(f"2025_roller_server: switch name: {self.switch_name}, roller speed: {self.roller_speed}")

        self._as = actionlib.SimpleActionServer(self._action_name, Roller2025Action, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        pct_out = Float64()
        success = True
        r = rospy.Rate(250)
        
        if goal.mode == goal.INTAKE:
            rospy.loginfo("2025_roller_server: Roller intaking!")
            pct_out.data = self.roller_speed
            self.roller_client.call(CommandRequest(pct_out.data))
            while self.switch == 0 and not rospy.is_shutdown():
                if self._as.is_preempt_requested():
                    self._as.set_preempted()
                    rospy.logwarn("2025_roller_server: Preempted!")
                    pct_out.data = 0
                    self.roller_client.call(CommandRequest(pct_out.data))
                    success = False
                    return
                r.sleep()

            pct_out.data = 0
            self.roller_client.call(CommandRequest(pct_out.data))
            if success:
                self._result.success = True
            else:
                self._result.success = False

        else:
            rospy.loginfo("2025_roller_server: Roller outtaking!")
            pct_out.data = self.roller_speed
            self.roller_client.call(CommandRequest(pct_out.data))
            while self.switch == 1 and not rospy.is_shutdown():
                if self._as.is_preempt_requested():
                    self._as.set_preempted()
                    rospy.logwarn("2025_roller_server: Preempted!")
                    pct_out.data = 0
                    self.roller_client.call(CommandRequest(pct_out.data))
                    success = False
                    return
                r.sleep()

            pct_out.data = 0
            self.roller_client.call(CommandRequest(pct_out.data))
            if success:
                self._result.success = False
            else:
                self._result.success = True

        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)

    def callback(self, data):
        if self.switch_name in data.name:
            self.switch = data.position[data.name.index(self.switch_name)]
            #rospy.loginfo(f"Found {self.switch_name} with value {self.switch}")
        else:
            rospy.logwarn_throttle(1.0, f'2025_roller_server: {self.switch_name} not found')
            pass

if __name__ == '__main__':
    rospy.init_node('roller_server_2025')
    server = Roller2025ActionServer(rospy.get_name())
    rospy.spin()