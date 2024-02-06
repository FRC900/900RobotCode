#!/usr/bin/env python3

import rospy
import actionlib

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

from behavior_actions.msg import NoteDiverterResult, NoteDiverterGoal, NoteDiverterAction
from std_msgs.msg import Float64

class NoteDiverterActionServer(object):
    # create messages that are used to publish feedback/result
    _result = NoteDiverterResult()

    def __init__(self, name):
        self.conveyor_pub = rospy.Publisher("/frcrobot_jetson/note_conveyor_controller/command", Float64, queue_size=1)
        self.diverter_pub = rospy.Publisher("/frcrobot_jetson/note_diverter_controller/command", Float64, queue_size=1)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, NoteDiverterAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal: NoteDiverterGoal):
        # Conveyer topic
        conveyor_pct = Float64()
        if goal.mode == goal.OFF:
            conveyor_pct.data = 0
        else:
            conveyor_pct.data = conveyor_speed
        self.conveyor_pub.publish(conveyor_pct)

        # Diverter topic
        diverter_pct = Float64()
        if goal.mode == goal.OFF:
            diverter_pct.data = 0
        elif goal.mode == goal.TO_CLAW:
            diverter_pct.data = diverter_speed
        else:
            diverter_pct.data = -1 * diverter_speed # Reverses(?)
        self.diverter_pub.publish(diverter_pct)

        self._result.success = True
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)

def dyn_rec_callback(config, level):
    rospy.loginfo("Received reconf call: " + str(config))
    global conveyor_speed
    global diverter_speed
    conveyor_speed = config["conveyor_speed"]
    diverter_speed = config["diverter_speed"]
    return config

if __name__ == '__main__':
    rospy.init_node('diverter_server_2024')
    
    ddynrec = DDynamicReconfigure("diverter_dyn_rec")
    ddynrec.add_variable("conveyor_speed", "float/double variable", rospy.get_param("note_conveyor_speed"), 0.0, 13.0)
    ddynrec.add_variable("diverter_speed", "float/double variable", rospy.get_param("note_diverter_speed"), 0.0, 13.0)
    ddynrec.start(dyn_rec_callback)

    server = NoteDiverterActionServer(rospy.get_name())
    rospy.spin()