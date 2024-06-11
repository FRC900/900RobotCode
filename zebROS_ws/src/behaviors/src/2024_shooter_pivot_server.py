#! /usr/bin/env python3

import rospy
import actionlib
# import std_msgs.msg

from talon_state_msgs.msg import TalonFXProState
from controllers_2024_msgs.srv import ShooterPivotSrv, ShooterPivotSrvRequest
from behavior_actions.msg import ShooterPivot2024Action, ShooterPivot2024Goal, ShooterPivot2024Feedback, ShooterPivot2024Result

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure


class ShooterPivotServer2024:
    _result = ShooterPivot2024Result()
    _feedback = ShooterPivot2024Feedback()

    motion_magic_value_index = None
    motion_magic_value = 1.0

    vel = 0.0

    control_pos = 0.0

    def callback(self, data):
        if (self.motion_magic_value_index == None):
            for i in range(len(data.name)):
                #rospy.loginfo(data.name[i])
                if (data.name[i] == "shooter_pivot_motionmagic_joint"): 
                    self.motion_magic_value = data.position[i]
                    self.vel = data.rotor_velocity[i] / 35.56
                    self.motion_magic_value_index = i
                    self.control_pos = data.control_position[i]
                    break
        else:
            self.motion_magic_value = data.position[self.motion_magic_value_index]
            self.vel = data.velocity[self.motion_magic_value_index]
            self.control_pos = data.control_position[self.motion_magic_value_index]

        if (abs(self.motion_magic_value - self.control_pos) < self.tolerance) and (abs(self.vel) < self.velocity_tolerance):
                self.valid_samples += 1
        else:
            self.valid_samples = 0

    def __init__(self):

        self.server = actionlib.SimpleActionServer('set_shooter_pivot', ShooterPivot2024Action, self.execute_cb, auto_start = False)

        rospy.Subscriber('/frcrobot_jetson/talonfxpro_states', TalonFXProState, self.callback)
        #subscribing here so that we can figure out the actual speed of said motor at a given time, talonfxpro_states gives us these values
        #maybe we subscibe to the voltage velocity controller instead of hte fx pro

        self.shooter_pivot_client = rospy.ServiceProxy("/frcrobot_jetson/shooter_pivot_controller/shooter_pivot_service", ShooterPivotSrv)

        ddynrec = DDynamicReconfigure("pivot_dyn_rec")
        ddynrec.add_variable("pivot_tolerance", "float/double variable", rospy.get_param("pivot_tolerance"), 0.0, 0.2)
        ddynrec.add_variable("pivot_velocity_tolerance", "float/double variable", rospy.get_param("pivot_velocity_tolerance"), 0.0, 0.2)
        ddynrec.start(self.dyn_rec_callback)

        self.tolerance = rospy.get_param("pivot_tolerance")
        self.velocity_tolerance = rospy.get_param("pivot_velocity_tolerance")

        self.num_samples_required = rospy.get_param("num_samples_required")

        # debouncing!
        self.valid_samples = 0
   
        self.server.start()

    def dyn_rec_callback(self, config, level):
        rospy.loginfo("Received reconf call: " + str(config))
        self.tolerance = config["pivot_tolerance"]
        self.velocity_tolerance = config["pivot_velocity_tolerance"]
        return config

    def execute_cb(self, goal):
        r = rospy.Rate(50)

        initial_motion_magic_value = self.motion_magic_value 
        self.shooter_pivot_client.call(ShooterPivotSrvRequest(goal.pivot_position))
        self._feedback.percent_complete = 0.0
        self._feedback.is_at_pivot_position = False
        self.server.publish_feedback(self._feedback)
        self.valid_samples = 0
        r.sleep()
        while True:
            if rospy.is_shutdown():
                break

            if (goal.pivot_position - initial_motion_magic_value) == 0.0: #either a double or a int i'm not entirely sure
                self._feedback.percent_complete = 100.0
            else:
                self._feedback.percent_complete = (((self.motion_magic_value - initial_motion_magic_value) / (goal.pivot_position - initial_motion_magic_value))) * 100
            self.server.publish_feedback(self._feedback)

            if self.server.is_preempt_requested():
                self.shooter_pivot_client.call(ShooterPivotSrvRequest(self.motion_magic_value))
                self.server.set_preempted()
                break
            
            if self.valid_samples >= self.num_samples_required:
                self.valid_samples = 0
                self._result.success = True
                self._feedback.percent_complete = 100.0
                self._feedback.is_at_pivot_position = True
                self.server.publish_feedback(self._feedback)
                self.server.set_succeeded(self._result)
                break
            
            r.sleep()
        

if __name__ == '__main__':
    rospy.init_node('shooter_pivot_server_2024')

    server = ShooterPivotServer2024()
    rospy.spin()
