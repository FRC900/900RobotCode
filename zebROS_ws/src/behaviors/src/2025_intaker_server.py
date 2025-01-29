#!/usr/bin/env python3

import rospy
import actionlib

from behavior_actions.msg import Intaker2025Result, Intaker2025Action
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from talon_controller_msgs.srv import Command, CommandRequest

class Intaker2025ActionServer(object):
    # create messages that are used to publish feedback/result
    _result = Intaker2025Result()

    def __init__(self, name):
        self.intake_client = rospy.ServiceProxy(f"/frcrobot_jetson/{rospy.get_param('controller_name')}/command", Command)
        self._action_name = name
        '''
            intake_speed: 0.4
            controller_name: "intake_controller"
            switch_name: "intake_limit_switch"
        '''
        self.switch = 0
        self.switch_name = rospy.get_param('switch_name')
        self.intake_speed = rospy.get_param('intake_speed')

        self.switch_sub = rospy.Subscriber("/frcrobot_rio/joint_states", JointState, self.callback, tcp_nodelay=True)

        self.last_touched_diverter = rospy.Time()

        rospy.loginfo(f"2025_intaker_server: switch name: {self.switch_name}, intake speed: {self.intake_speed}")

        self._as = actionlib.SimpleActionServer(self._action_name, Intaker2025Action, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        pct_out = Float64()
        success = True
        r = rospy.Rate(50)

        rospy.loginfo("2025_intaker_server: intaking!")
        pct_out.data = self.intake_speed
        self.intake_client.call(CommandRequest(pct_out.data))
        while self.switch == 0 and not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                rospy.logwarn("2025_intaker_server: Preempted!")
                pct_out.data = 0
                self.intake_client.call(CommandRequest(pct_out.data))
                success = False
                return
            r.sleep()

        pct_out.data = 0
        self.intake_client.call(CommandRequest(pct_out.data))
        if success:
            self._result.success = True
        else:
            self._result.success = False
        
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)

    def callback(self, data):
        if self.switch_name in data.name:
            self.switch = data.position[data.name.index(self.switch_name)]
            #rospy.loginfo(f"Found {self.switch_name} with value {self.switch}")
        else:
            rospy.logwarn_throttle(1.0, f'2025_intaker_server: {self.switch_name} not found')
            pass

if __name__ == '__main__':
    rospy.init_node('intaker_server_2025')
    
    server = Intaker2025ActionServer(rospy.get_name())
    rospy.spin()