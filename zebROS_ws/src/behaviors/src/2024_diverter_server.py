#!/usr/bin/env python3

import rospy
import actionlib

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

from behavior_actions.msg import NoteDiverterResult, NoteDiverterGoal, NoteDiverterAction
from std_msgs.msg import Float64
from talon_controller_msgs.srv import Command, CommandRequest, CommandResponse

class NoteDiverterActionServer(object):
    # create messages that are used to publish feedback/result
    _result = NoteDiverterResult()

    def __init__(self, name):
        self.diverter_client = rospy.ServiceProxy("/frcrobot_jetson/note_diverter_controller/command", Command)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, NoteDiverterAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal: NoteDiverterGoal):
        r = rospy.Rate(60)

        # Diverter topic
        diverter_srv = CommandRequest()
        if goal.mode == goal.TO_CLAW:
            diverter_srv.command = diverter_speed
        else:
            diverter_srv.command = -1 * diverter_speed # Reverses(?)
        self.diverter_client.call(diverter_srv)

        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.loginfo("2024_diverter_server: preempted")

                # stop diverter
                diverter_srv.command = 0
                self.diverter_client.call(diverter_srv)

                self._as.set_preempted()

                return
            r.sleep()

        self._result.success = True
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)

def dyn_rec_callback(config, level):
    rospy.loginfo("Received reconf call: " + str(config))
    global diverter_speed
    diverter_speed = config["diverter_speed"]
    return config

if __name__ == '__main__':
    rospy.init_node('diverter_server_2024')
    
    ddynrec = DDynamicReconfigure("diverter_dyn_rec")
    ddynrec.add_variable("diverter_speed", "float/double variable", rospy.get_param("note_diverter_speed"), 0.0, 13.0)
    ddynrec.start(dyn_rec_callback)

    server = NoteDiverterActionServer(rospy.get_name())
    rospy.spin()