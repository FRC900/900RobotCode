#!/usr/bin/env python3

import rospy
import actionlib

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

from behavior_actions.msg import Clawster2024Result, Clawster2024Goal, Clawster2024Action
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class Clawster2024ActionServer(object):
    # create messages that are used to publish feedback/result
    _result = Clawster2024Result()

    def __init__(self, name):
        self.claw_pub = rospy.Publisher(f"/frcrobot_jetson/{rospy.get_param('controller_name')}/command", Float64, queue_size=1)
        self.switch_sub = rospy.Subscriber("/frcrobot_rio/joint_states", JointState, callback)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, Clawster2024Action, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal: Clawster2024Goal):
        pct_out = Float64()
        success = True
        r = rospy.Rate(10)
        if goal.mode == goal.INTAKE:
            pct_out.data = intake_speed
            self.claw_pub.publish(pct_out)
            while switch == 0 and (not rospy.is_shutdown()):
                if self._as.is_preempt_requested():
                    self._as.set_preempted()
                    success = False
                    break
                r.sleep()

            pct_out.data = 0
            self.claw_pub.publish(pct_out)
            if success:
                self._result.has_game_piece = True
            else:
                self._result.has_game_piece = False

        else:
            pct_out.data = outtake_speed
            self.claw_pub.publish(pct_out)
            while switch != 0 and (not rospy.is_shutdown()):
                if self._as.is_preempt_requested():
                    self._as.set_preempted()
                    success = False
                    break
                r.sleep()

            rospy.Rate(delay).sleep()
            pct_out.data = 0
            self.claw_pub.publish(pct_out)
            if success:
                self._result.has_game_piece = False
            else:
                self._result.has_game_piece = True
        
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)

def callback(data):
    global switch

    if switch_name in data.name:
        switch = data.position[data.name.index(switch_name)]  # Or whatever actually says when it's pressed
    else:
        rospy.loginfo(f'Warning: {switch_name} not found')

def dyn_rec_callback(config, level):
    rospy.loginfo("Received reconf call: " + str(config))
    global intake_speed
    global outtake_speed
    global delay
    intake_speed = config["intake_speed"]
    outtake_speed = config["outtake_speed"]
    delay = config["delay"]
    return config

if __name__ == '__main__':
    rospy.init_node('clawster_server_2024')
    switch_name = rospy.get_param('switch_name')

    ddynrec = DDynamicReconfigure(f"{rospy.get_name()}_dyn_rec")
    ddynrec.add_variable("intake_speed", "float/double variable", rospy.get_param("intake_speed"), -1.0, 1.0)
    ddynrec.add_variable("outtake_speed", "float/double variable", rospy.get_param("outtake_speed"), -1.0, 1.0)
    ddynrec.add_variable("delay", "float/double variable", rospy.get_param("outtake_stop_delay"), 0.0, 2.0)
    ddynrec.start(dyn_rec_callback)
    
    server = Clawster2024ActionServer(rospy.get_name())
    rospy.spin()