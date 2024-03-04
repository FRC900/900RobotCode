#!/usr/bin/env python3

import rospy
import actionlib
import time
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

from behavior_actions.msg import Shooting2024Goal, Shooting2024Feedback, Shooting2024Result, Shooting2024Action
from behavior_actions.msg import Shooter2024Goal, Shooter2024Feedback, Shooter2024Result, Shooter2024Action
from behavior_actions.msg import ShooterPivot2024Goal, ShooterPivot2024Feedback, ShooterPivot2024Result, ShooterPivot2024Action
from behavior_actions.msg import Clawster2024Goal, Clawster2024Feedback, Clawster2024Result, Clawster2024Action
from std_msgs.msg import Float64
from interpolating_map import InterpolatingMap

class ShootingServer(object):

    def __init__(self, name):
        self.action_name = name
        self.result = Shooting2024Result()
        self.feedback = Shooting2024Feedback()
        self.shooter_client = actionlib.SimpleActionClient('/shooter/set_shooter_speed', Shooter2024Action)
        rospy.loginfo("2024_shooting_server: waiting for shooter")
        # this will block forever if something lower level fails to come up
        self.shooter_client.wait_for_server()
        self.pivot_client = actionlib.SimpleActionClient('/shooter/set_shooter_pivot', ShooterPivot2024Action)
        rospy.loginfo("2024_shooting_server: waiting for pivot")
        self.pivot_client.wait_for_server()
        # The preshooter and claw are very similar (drive motor until limit switch pressed). They'll probably be the same server.
        self.preshooter_client = actionlib.SimpleActionClient('/clawster/clawster_server_2024', Clawster2024Action)
        rospy.loginfo("2024_shooting_server: waiting for clawster")
        self.preshooter_client.wait_for_server()

        # speeds_map: [[distance: [top_left_speed, top_right_speed, bottom_left_speed, bottom_right_speed]], ...]
        speeds_map_param = rospy.get_param("speeds_map")

        self.top_left_map = InterpolatingMap()
        self.top_left_map.container = {l[0]: l[1][0] for l in speeds_map_param}

        self.top_right_map = InterpolatingMap()
        self.top_right_map.container = {l[0]: l[1][1] for l in speeds_map_param}

        self.bottom_left_map = InterpolatingMap()
        self.bottom_left_map.container = {l[0]: l[1][2] for l in speeds_map_param}

        self.bottom_right_map = InterpolatingMap()
        self.bottom_right_map.container = {l[0]: l[1][3] for l in speeds_map_param}

        self.angle_map = InterpolatingMap()
        self.angle_map.container = {l[0]: l[1] for l in rospy.get_param("angle_map")}

        # Amp (constant speeds and angle)
        self.amp_top_left_speed = rospy.get_param("amp_top_left_speed")
        self.amp_top_right_speed = rospy.get_param("amp_top_right_speed")
        self.amp_bottom_left_speed = rospy.get_param("amp_bottom_left_speed")
        self.amp_bottom_right_speed = rospy.get_param("amp_bottom_right_speed")
        self.amp_pivot_position = rospy.get_param("amp_pivot_position")

        ddynrec = DDynamicReconfigure("shooting_dyn_rec")
        ddynrec.add_variable("subwoofer_top_left_speed", "float/double variable", rospy.get_param("subwoofer_top_left_speed"), 0.0, 1000.0)
        ddynrec.add_variable("subwoofer_top_right_speed", "float/double variable", rospy.get_param("subwoofer_top_right_speed"), 0.0, 1000.0)
        ddynrec.add_variable("subwoofer_bottom_left_speed", "float/double variable", rospy.get_param("subwoofer_bottom_left_speed"), 0.0, 1000.0)
        ddynrec.add_variable("subwoofer_bottom_right_speed", "float/double variable", rospy.get_param("subwoofer_bottom_right_speed"), 0.0, 1000.0)
        ddynrec.add_variable("subwoofer_pivot_position", "float/double variable", rospy.get_param("subwoofer_pivot_position"), 0.5, 1.5)
        ddynrec.start(self.dyn_rec_callback)

        # Subwoofer (constant speeds and angle)
        self.subwoofer_top_left_speed = rospy.get_param("subwoofer_top_left_speed")
        self.subwoofer_top_right_speed = rospy.get_param("subwoofer_top_right_speed")
        self.subwoofer_bottom_left_speed = rospy.get_param("subwoofer_bottom_left_speed")
        self.subwoofer_bottom_right_speed = rospy.get_param("subwoofer_bottom_right_speed")
        self.subwoofer_pivot_position = rospy.get_param("subwoofer_pivot_position")
        self.subwoofer_reconfigured = False

        self.delay_after_shooting = rospy.get_param("delay_after_shooting")

        self.server = actionlib.SimpleActionServer(self.action_name, Shooting2024Action, execute_cb=self.execute_cb, auto_start = False)
        self.server.start()

    def dyn_rec_callback(self, config, level):
        rospy.loginfo("Received reconf call: " + str(config))
        self.subwoofer_top_left_speed = config["subwoofer_top_left_speed"]
        self.subwoofer_top_right_speed = config["subwoofer_top_right_speed"]
        self.subwoofer_bottom_left_speed = config["subwoofer_bottom_left_speed"]
        self.subwoofer_bottom_right_speed = config["subwoofer_bottom_right_speed"]
        self.subwoofer_pivot_position = config["subwoofer_pivot_position"]
        self.subwoofer_reconfigured = True
        return config

    def execute_cb(self, goal: Shooting2024Goal):
        if goal.cancel_movement:
            rospy.logwarn("2024_shooting_server: CANCELING SPIN UP")
            self.shooter_client.cancel_goals_at_and_before_time(rospy.Time.now())
            self.pivot_client.cancel_goals_at_and_before_time(rospy.Time.now())
            self.result.success = True
            self.server.set_succeeded(self.result)
            return
        
        self.feedback.current_stage = self.feedback.SPINNING
        self.server.publish_feedback(self.feedback)

        shooter_goal = Shooter2024Goal()
        pivot_angle = None # default

        if goal.mode == goal.SUBWOOFER:
            shooter_goal.top_left_speed = self.subwoofer_top_left_speed
            shooter_goal.top_right_speed = self.subwoofer_top_right_speed
            shooter_goal.bottom_left_speed = self.subwoofer_bottom_left_speed
            shooter_goal.bottom_right_speed = self.subwoofer_bottom_right_speed
            pivot_angle = self.subwoofer_pivot_position

            rospy.loginfo(f"2024_shooting_server: spinning up for subwoofer angle")
        
        elif goal.mode == goal.AMP:
            shooter_goal.top_left_speed = self.amp_top_left_speed
            shooter_goal.top_right_speed = self.amp_top_right_speed
            shooter_goal.bottom_left_speed = self.amp_bottom_left_speed
            shooter_goal.bottom_right_speed = self.amp_bottom_right_speed
            pivot_angle = self.amp_pivot_position

            rospy.loginfo(f"2024_shooting_server: spinning up for amp")
        elif goal.mode == goal.PODIUM:
            shooter_goal.top_left_speed = 500
            shooter_goal.top_right_speed = 500
            shooter_goal.bottom_left_speed = 500
            shooter_goal.bottom_right_speed = 500
            pivot_angle = 0.58

            rospy.loginfo(f"2024_shooting_server: spinning up for amp")
        else:
            # Look up speed and angle to send to shooter and pivot server
            shooter_goal.top_left_speed = self.top_left_map[goal.distance]
            shooter_goal.top_right_speed = self.top_right_map[goal.distance]
            shooter_goal.bottom_left_speed = self.bottom_left_map[goal.distance]
            shooter_goal.bottom_right_speed = self.bottom_right_map[goal.distance]
            pivot_angle = self.angle_map[goal.distance]

            rospy.loginfo(f"2024_shooting_server: spinning up to distance {goal.distance}")

        shooter_done = False
        def shooter_feedback_cb(feedback: Shooter2024Feedback):
            nonlocal shooter_done
            shooter_done = feedback.is_shooting_at_speed
        self.shooter_client.send_goal(shooter_goal, feedback_cb=shooter_feedback_cb)

        rospy.loginfo("2024_shooting_server: pivoting")

        self.feedback.current_stage = self.feedback.PIVOTING
        self.server.publish_feedback(self.feedback)

        pivot_goal = ShooterPivot2024Goal()
        pivot_goal.pivot_position = pivot_angle

        pivot_done = False
        def pivot_done_cb(state, result):
            nonlocal pivot_done
            pivot_done = True
        self.pivot_client.send_goal(pivot_goal, done_cb=pivot_done_cb)
        rospy.loginfo("Sleeping for 0.25")
        time.sleep(0.25)
        r = rospy.Rate(60.0)

        while not (shooter_done and pivot_done):
            rospy.loginfo_throttle(0.5, "2024_shooting_server: waiting for shooter and pivot")
            if self.server.is_preempt_requested():
                rospy.loginfo("2024_shooting_server: preempted")

                # ensure shooter turned off
                self.shooter_client.cancel_goals_at_and_before_time(rospy.Time.now())

                # ensure pivot stopped
                self.pivot_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.server.set_preempted()
                return
            
            r.sleep()

        if not goal.setup_only:
            rospy.loginfo("2024_shooting_server: shooting")

            self.feedback.current_stage = self.feedback.SHOOTING
            self.server.publish_feedback(self.feedback)

            preshooter_goal = Clawster2024Goal()
            preshooter_goal.mode = preshooter_goal.OUTTAKE
            preshooter_goal.destination = preshooter_goal.PRESHOOTER

            preshooter_done = False
            def preshooter_done_cb(state, result):
                nonlocal preshooter_done
                preshooter_done = True
                rospy.logwarn("2024_shooting_server: PRESHOOTER CB DONE")

            self.preshooter_client.send_goal(preshooter_goal, done_cb=preshooter_done_cb)

            while not preshooter_done and not rospy.is_shutdown():
                rospy.loginfo_throttle(0.5, "2024_shooting_server: waiting for preshooter")
                if self.server.is_preempt_requested():
                    rospy.loginfo("2024_shooting_server: preempted")

                    # ensure shooter turned off
                    self.shooter_client.cancel_goals_at_and_before_time(rospy.Time.now())

                    # ensure pivot at good position
                    pivot_goal.pivot_position = 0.5
                    self.pivot_client.send_goal(pivot_goal)
                    # self.pivot_client.cancel_goals_at_and_before_time(rospy.Time.now())

                    # stop preshooter
                    self.preshooter_client.cancel_goals_at_and_before_time(rospy.Time.now())

                    self.server.set_preempted()

                    return
                r.sleep()

            pivot_goal.pivot_position = 0.5
            self.pivot_client.send_goal(pivot_goal)

            rospy.loginfo("2024_shooting_server: +5 points hopefully")
        
        if not goal.leave_spinning:
            rospy.loginfo("2024_shooting_server: spinning down")
            rospy.sleep(rospy.Duration(self.delay_after_shooting))

            # ensure shooter turned off
            self.shooter_client.cancel_goals_at_and_before_time(rospy.Time.now())

            # ensure pivot stopped
            # self.pivot_client.cancel_goals_at_and_before_time(rospy.Time.now())

        # stop preshooter
        self.preshooter_client.cancel_goals_at_and_before_time(rospy.Time.now())

        self.result.success = True
        rospy.loginfo("2024_shooting_server: succeeded")
        self.server.set_succeeded(self.result)

if __name__ == '__main__':
    rospy.init_node('shooting_server_2024')
    
    server = ShootingServer(rospy.get_name())
    rospy.spin()