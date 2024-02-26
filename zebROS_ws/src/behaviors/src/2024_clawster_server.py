#!/usr/bin/env python3

# CONTORLS PRESHOOTER AND CLAW

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
        self.switch_sub = rospy.Subscriber("/frcrobot_rio/joint_states", JointState, self.callback)
        self._action_name = name
        '''
            intake_speed: 0.4
            outtake_speed: -0.5
            outtake_stop_delay: 0.5
            controller_name: "clawster_controller"
            switch_name: "claw_limit_switch"
        '''
        self.outtake_delay = rospy.get_param("outtake_delay")
        # claw params
        self.claw_switch = 0
        self.claw_switch_name = rospy.get_param('claw/switch_name')
        self.claw_intake_speed = rospy.get_param('claw/intake_speed')
        self.claw_outtake_speed = rospy.get_param('claw/outtake_speed')

        # preshooter params 
        self.preshooter_switch = 0
        self.preshooter_switch_name = rospy.get_param('preshooter/switch_name')
        self.preshooter_intake_speed = rospy.get_param('preshooter/intake_speed')
        self.preshooter_outtake_speed = rospy.get_param('preshooter/outtake_speed')

        rospy.loginfo(f"Claw: switch name {self.claw_switch_name} claw intake speed {self.claw_intake_speed}")
        rospy.loginfo(f"Preshooter: switch name {self.preshooter_switch_name} claw intake speed {self.preshooter_intake_speed}")

        self._as = actionlib.SimpleActionServer(self._action_name, Clawster2024Action, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def get_current_switch(self, goal: Clawster2024Goal):
        if goal.destination == goal.PRESHOOTER:
            return self.preshooter_switch
        elif goal.destination == goal.CLAW:
            return self.claw_switch
        else:
            rospy.logerr("GOAL NOT SET CORRECTLY?")

    def execute_cb(self, goal: Clawster2024Goal):
        pct_out = Float64()
        success = True
        r = rospy.Rate(10)
        if goal.destination == goal.PRESHOOTER:
            rospy.loginfo("Clawster called with PRESHOOTER")
            intake_speed = self.preshooter_intake_speed
            outtake_speed = self.preshooter_outtake_speed
        elif goal.destination == goal.CLAW:
            rospy.loginfo("Clawster called with CLAW")
            intake_speed = self.preshooter_intake_speed
            outtake_speed = self.preshooter_outtake_speed

        rospy.loginfo(f"Clawster: limit switch set to {self.get_current_switch(goal)}")
        if goal.mode == goal.INTAKE:
            rospy.loginfo("Clawster_server: Intaking!")
            pct_out.data = intake_speed
            self.claw_pub.publish(pct_out)
            while self.get_current_switch(goal) == 0 and (not rospy.is_shutdown()):
                if self._as.is_preempt_requested():
                    self._as.set_preempted()
                    rospy.logwarn("Clawster_server: Preempted!")
                    pct_out.data = 0
                    self.claw_pub.publish(pct_out)
                    success = False
                    return
                r.sleep()

            pct_out.data = 0
            self.claw_pub.publish(pct_out)
            if success:
                self._result.has_game_piece = True
            else:
                self._result.has_game_piece = False
        else:
            rospy.loginfo("Clawster_server: Outtaking/Shooting!")
            pct_out.data = outtake_speed
            self.claw_pub.publish(pct_out)
            while self.get_current_switch(goal) != 0 and (not rospy.is_shutdown()):
                if self._as.is_preempt_requested():
                    self._as.set_preempted()
                    rospy.logwarn("Clawster_server: Preempted!")
                    pct_out.data = 0
                    self.claw_pub.publish(pct_out)
                    success = False
                    return
                r.sleep()

            rospy.Rate(self.outtake_delay).sleep()
            pct_out.data = 0
            self.claw_pub.publish(pct_out)
            if success:
                self._result.has_game_piece = False
            else:
                self._result.has_game_piece = True
        
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)

    def callback(self, data):
        # check claw switch
        if self.claw_switch_name in data.name:
            self.claw_switch = data.position[data.name.index(self.claw_switch_name)]
            #rospy.loginfo(f"Found {self.claw_switch_name} with value {self.claw_switch}")
        else:
            rospy.logwarn_throttle(1.0, f'2024_clawster_server: {self.claw_switch_name} not found')
            pass

        # check preshooter switch
        if self.preshooter_switch_name in data.name:
            self.preshooter_switch = data.position[data.name.index(self.preshooter_switch_name)] 
            #rospy.loginfo(f"Found {self.preshooter_switch_name} with value {self.preshooter_switch}")
        else:
            rospy.logwarn_throttle(1.0, f'2024_clawster_server: {self.preshooter_switch_name} not found')
            pass


# its nice to have but for speed of working its being taken out
#def dyn_rec_callback(config, level):
#    rospy.loginfo("Received reconf call: " + str(config))
#    global intake_speed
#    global outtake_speed
#    global delay
#    intake_speed = config["intake_speed"]
#    outtake_speed = config["outtake_speed"]
#    delay = config["delay"]
#    return config

if __name__ == '__main__':
    rospy.init_node('clawster_server_2024')
    #switch_name = rospy.get_param('switch_name')

    #ddynrec = DDynamicReconfigure(f"{rospy.get_name()}_dyn_rec")
    #ddynrec.add_variable("intake_speed", "float/double variable", rospy.get_param("intake_speed"), -1.0, 1.0)
    #ddynrec.add_variable("outtake_speed", "float/double variable", rospy.get_param("outtake_speed"), -1.0, 1.0)
    #ddynrec.add_variable("delay", "float/double variable", rospy.get_param("outtake_stop_delay"), 0.0, 2.0)
    #ddynrec.start(dyn_rec_callback)
    
    server = Clawster2024ActionServer(rospy.get_name())
    rospy.spin()