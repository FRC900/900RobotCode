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
from geometry_msgs.msg import Twist

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

        # used for driving back after amp shot
        self.cmd_vel_pub = rospy.Publisher("/auto_note_align/cmd_vel", Twist, queue_size=1, tcp_nodelay=True)

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

        #  "shooting_dyn_rec " not appearing at dnyamic reconfigure ???
        ddynrec = DDynamicReconfigure("shooting_dyn_rec")
        ddynrec.add_variable("subwoofer_top_left_speed", "float/double variable", rospy.get_param("subwoofer_top_left_speed"), 0.0, 1000.0)
        ddynrec.add_variable("subwoofer_top_right_speed", "float/double variable", rospy.get_param("subwoofer_top_right_speed"), 0.0, 1000.0)
        ddynrec.add_variable("subwoofer_bottom_left_speed", "float/double variable", rospy.get_param("subwoofer_bottom_left_speed"), 0.0, 1000.0)
        ddynrec.add_variable("subwoofer_bottom_right_speed", "float/double variable", rospy.get_param("subwoofer_bottom_right_speed"), 0.0, 1000.0)
        ddynrec.add_variable("subwoofer_pivot_position", "float/double variable", rospy.get_param("subwoofer_pivot_position"), 0.45, 3.0)

        ddynrec.add_variable("amp_top_left_speed", "float/double variable", rospy.get_param("amp_top_left_speed"), 0.0, 1000.0)
        ddynrec.add_variable("amp_top_right_speed", "float/double variable", rospy.get_param("amp_top_right_speed"), 0.0, 1000.0)
        ddynrec.add_variable("amp_bottom_left_speed", "float/double variable", rospy.get_param("amp_bottom_left_speed"), 0.0, 1000.0)
        ddynrec.add_variable("amp_bottom_right_speed", "float/double variable", rospy.get_param("amp_bottom_right_speed"), 0.0, 1000.0)
        ddynrec.add_variable("amp_pivot_position", "float/double variable", rospy.get_param("amp_pivot_position"), 0.45, 3.0)

        ddynrec.add_variable("trap_top_left_speed", "float/double variable", rospy.get_param("trap_top_left_speed"), 0.0, 1000.0)
        ddynrec.add_variable("trap_top_right_speed", "float/double variable", rospy.get_param("trap_top_right_speed"), 0.0, 1000.0)
        ddynrec.add_variable("trap_bottom_left_speed", "float/double variable", rospy.get_param("trap_bottom_left_speed"), 0.0, 1000.0)
        ddynrec.add_variable("trap_bottom_right_speed", "float/double variable", rospy.get_param("trap_bottom_right_speed"), 0.0, 1000.0)
        ddynrec.add_variable("trap_pivot_position", "float/double variable", rospy.get_param("trap_pivot_position"), 0.45, 3.0)
        
        #lob_pass configs
        ddynrec.add_variable("lob_top_left_speed", "float/double variable", rospy.get_param("lob_top_left_speed"), 0.0, 1000.0)
        ddynrec.add_variable("lob_top_right_speed", "float/double variable", rospy.get_param("lob_top_right_speed"), 0.0, 1000.0)
        ddynrec.add_variable("lob_bottom_left_speed", "float/double variable", rospy.get_param("lob_bottom_left_speed"), 0.0, 1000.0)
        ddynrec.add_variable("lob_bottom_right_speed", "float/double variable", rospy.get_param("lob_bottom_right_speed"), 0.0, 1000.0)
        ddynrec.add_variable("lob_pivot_position", "float/double variable", rospy.get_param("lob_pivot_position"), 0.45, 3.0)

        #shot_pass configs
        ddynrec.add_variable("shot_top_left_speed", "float/double variable", rospy.get_param("shot_top_left_speed"), 0.0, 1000.0)
        ddynrec.add_variable("shot_top_right_speed", "float/double variable", rospy.get_param("shot_top_right_speed"), 0.0, 1000.0)
        ddynrec.add_variable("shot_bottom_left_speed", "float/double variable", rospy.get_param("shot_bottom_left_speed"), 0.0, 1000.0)
        ddynrec.add_variable("shot_bottom_right_speed", "float/double variable", rospy.get_param("shot_bottom_right_speed"), 0.0, 1000.0)
        ddynrec.add_variable("shot_pivot_position", "float/double variable", rospy.get_param("shot_pivot_position"), 0.45, 3.0)

        ddynrec.start(self.dyn_rec_callback)

        # Subwoofer (constant speeds and angle)
        self.subwoofer_top_left_speed = rospy.get_param("subwoofer_top_left_speed")
        self.subwoofer_top_right_speed = rospy.get_param("subwoofer_top_right_speed")
        self.subwoofer_bottom_left_speed = rospy.get_param("subwoofer_bottom_left_speed")
        self.subwoofer_bottom_right_speed = rospy.get_param("subwoofer_bottom_right_speed")
        self.subwoofer_pivot_position = rospy.get_param("subwoofer_pivot_position")

        # Amp (constant speeds and angle)
        self.amp_top_left_speed = rospy.get_param("amp_top_left_speed")
        self.amp_top_right_speed = rospy.get_param("amp_top_right_speed")
        self.amp_bottom_left_speed = rospy.get_param("amp_bottom_left_speed")
        self.amp_bottom_right_speed = rospy.get_param("amp_bottom_right_speed")
        self.amp_pivot_position = rospy.get_param("amp_pivot_position")
        self.amp_drive_back_time = rospy.get_param("amp_drive_back_time")
        self.amp_drive_back_speed = rospy.get_param("amp_drive_back_speed")

        # Trap (constant speeds and angle)
        self.trap_top_left_speed = rospy.get_param("trap_top_left_speed")
        self.trap_top_right_speed = rospy.get_param("trap_top_right_speed")
        self.trap_bottom_left_speed = rospy.get_param("trap_bottom_left_speed")
        self.trap_bottom_right_speed = rospy.get_param("trap_bottom_right_speed")
        self.trap_pivot_position = rospy.get_param("trap_pivot_position")

        #lob_pass 
        self.lob_top_left_speed = rospy.get_param("lob_top_left_speed")
        self.lob_top_right_speed = rospy.get_param("lob_top_right_speed")
        self.lob_bottom_left_speed = rospy.get_param("lob_bottom_left_speed")
        self.lob_bottom_right_speed = rospy.get_param("lob_bottom_right_speed")
        self.lob_pivot_position = rospy.get_param("lob_pivot_position")

       #shot pass 
        self.shot_top_left_speed = rospy.get_param("shot_top_left_speed")  
        self.shot_top_right_speed = rospy.get_param("shot_top_right_speed")
        self.shot_bottom_left_speed = rospy.get_param("shot_bottom_left_speed")
        self.shot_bottom_right_speed = rospy.get_param("shot_bottom_right_speed")
        self.shot_pivot_position = rospy.get_param("shot_pivot_position")

        self.delay_after_shooting = rospy.get_param("delay_after_shooting")

        self.server = actionlib.SimpleActionServer(self.action_name, Shooting2024Action, execute_cb=self.execute_cb, auto_start = False)
        self.server.start()

        rospy.loginfo("2024_shooting_server: initialized")

    def dyn_rec_callback(self, config, level):
        rospy.loginfo("Received reconf call: " + str(config))
        self.subwoofer_top_left_speed = config["subwoofer_top_left_speed"]
        self.subwoofer_top_right_speed = config["subwoofer_top_right_speed"]
        self.subwoofer_bottom_left_speed = config["subwoofer_bottom_left_speed"]
        self.subwoofer_bottom_right_speed = config["subwoofer_bottom_right_speed"]
        self.subwoofer_pivot_position = config["subwoofer_pivot_position"]

        self.amp_top_left_speed = config["amp_top_left_speed"]
        self.amp_top_right_speed = config["amp_top_right_speed"]
        self.amp_bottom_left_speed = config["amp_bottom_left_speed"]
        self.amp_bottom_right_speed = config["amp_bottom_right_speed"]
        self.amp_pivot_position = config["amp_pivot_position"]

        self.trap_top_left_speed = config["trap_top_left_speed"]
        self.trap_top_right_speed = config["trap_top_right_speed"]
        self.trap_bottom_left_speed = config["trap_bottom_left_speed"]
        self.trap_bottom_right_speed = config["trap_bottom_right_speed"]
        self.trap_pivot_position = config["trap_pivot_position"]

        self.lob_top_left_speed = config["lob_top_left_speed"]
        self.lob_top_right_speed = config["lob_top_right_speed"]
        self.lob_bottom_left_speed = config["lob_bottom_left_speed"]
        self.lob_bottom_right_speed = config["lob_bottom_right_speed"]
        self.lob_pivot_position = config["lob_pivot_position"]

        self.shot_top_left_speed = config["shot_top_left_speed"]
        self.shot_top_right_speed = config["shot_top_right_speed"]
        self.shot_bottom_left_speed = config["shot_bottom_left_speed"]
        self.shot_bottom_right_speed = config["shot_bottom_right_speed"]
        self.shot_pivot_position = config["shot_pivot_position"]
        return config

    def execute_cb(self, goal: Shooting2024Goal):
        if goal.cancel_movement:
            rospy.logwarn("2024_shooting_server: CANCELING SPIN UP")
            shooter_goal = Shooter2024Goal()
            shooter_goal.top_left_speed = 0.0
            shooter_goal.top_right_speed = 0.0
            shooter_goal.bottom_left_speed = 0.0
            shooter_goal.bottom_right_speed = 0.0
            shooter_goal.leave_spinning = False
            self.shooter_client.send_goal(shooter_goal)
            time.sleep(0.25)
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
        elif goal.mode == goal.TRAP:
            shooter_goal.top_left_speed = self.trap_top_left_speed
            shooter_goal.top_right_speed = self.trap_top_right_speed
            shooter_goal.bottom_left_speed = self.trap_bottom_left_speed
            shooter_goal.bottom_right_speed = self.trap_bottom_right_speed
            pivot_angle = self.trap_pivot_position

            rospy.loginfo(f"2024_shooting_server: spinning up for trap")
        elif goal.mode == goal.PODIUM:
            shooter_goal.top_left_speed = 500
            shooter_goal.top_right_speed = 500
            shooter_goal.bottom_left_speed = 500
            shooter_goal.bottom_right_speed = 500
            pivot_angle = 0.58

            rospy.loginfo(f"2024_shooting_server: spinning up for podium")

        elif goal.mode == goal.LOB_PASS:
            shooter_goal.top_left_speed = self.lob_top_left_speed
            shooter_goal.top_right_speed = self.lob_top_right_speed
            shooter_goal.bottom_left_speed = self.lob_bottom_left_speed
            shooter_goal.bottom_right_speed = self.lob_bottom_right_speed
            pivot_angle = self.lob_pivot_position
            rospy.loginfo(f"2024_shooting_server: spinning up for LOB_PASS")
            
        elif goal.mode == goal.SHOT_PASS:
            shooter_goal.top_left_speed = self.shot_top_left_speed
            shooter_goal.top_right_speed = self.shot_top_right_speed
            shooter_goal.bottom_left_speed = self.shot_bottom_left_speed
            shooter_goal.bottom_right_speed = self.shot_bottom_right_speed
            pivot_angle = self.shot_pivot_position
            rospy.loginfo(f"2024_shooting_server: spinning up for SHOT_PASS")

        else:
            # Look up speed and angle to send to shooter and pivot server
            shooter_goal.top_left_speed = self.top_left_map[goal.distance]
            shooter_goal.top_right_speed = self.top_right_map[goal.distance]
            shooter_goal.bottom_left_speed = self.bottom_left_map[goal.distance]
            shooter_goal.bottom_right_speed = self.bottom_right_map[goal.distance]
            pivot_angle = self.angle_map[goal.distance]

            rospy.loginfo(f"2024_shooting_server: spinning up to distance {goal.distance}")

        shooter_done = False
        # def shooter_feedback_cb(feedback: Shooter2024Feedback):
        #     nonlocal shooter_done
        #     rospy.loginfo(f"2024_shooting_server: shooter feedback is {feedback.is_shooting_at_speed}")
        #     shooter_done = feedback.is_shooting_at_speed

        def shooter_done_cb(state, result):
            nonlocal shooter_done
            rospy.loginfo(f"2024_shooting_server: shooter DONE called")
            shooter_done = True
        
        shooter_goal.leave_spinning = goal.leave_spinning
        rospy.loginfo(f"2024_shooting_server: sending shooter goal {shooter_goal}")
        self.shooter_client.send_goal(shooter_goal, done_cb=shooter_done_cb) # there was a feedback callback here too, but I believe it contributes to a race condition
        # what likely happens is feedback that we're not done yet is sent, then the result that we are done is sent as well as the feedback that we are done
        # we then receive that we are done (setting shooter_done=True) slightly *before* the second to last feedback message (saying we're not done) is received
        # this leads to shooter_done being set to False. we then don't receive the final feedback message, because SimpleActionClient ignores feedback sent after
        # the goal is done (source: https://github.com/ros/actionlib/blob/23acb6e7364dda9380f823e03284536574764c6a/actionlib/src/actionlib/action_client.py#L416)

        rospy.loginfo(f"2024_shooting_server: pivoting to angle {pivot_angle}")

        self.feedback.current_stage = self.feedback.PIVOTING
        self.server.publish_feedback(self.feedback)

        pivot_goal = ShooterPivot2024Goal()
        pivot_goal.pivot_position = pivot_angle

        pivot_done = False
        def pivot_done_cb(state, result):
            nonlocal pivot_done
            pivot_done = True
        self.pivot_client.send_goal(pivot_goal, done_cb=pivot_done_cb)
        # rospy.loginfo("Sleeping for 0.25")
        # time.sleep(0.25)
        r = rospy.Rate(60.0)

        while not (shooter_done and pivot_done):
            rospy.loginfo_throttle(0.5, f"2024_shooting_server: waiting for {'shooter' if not shooter_done else ''} and {'pivot' if not pivot_done else ''}")
            if self.server.is_preempt_requested():
                rospy.loginfo("2024_shooting_server: preempted")

                # ensure shooter turned off
                self.shooter_client.cancel_goals_at_and_before_time(rospy.Time.now())

                # ensure pivot stopped
                self.pivot_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.server.set_preempted()
                return
            
            r.sleep()

        self.feedback.current_stage = self.feedback.SHOOTING
        self.server.publish_feedback(self.feedback)
        if not goal.setup_only:
            rospy.loginfo("2024_shooting_server: shooting")

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
                    rospy.loginfo("2024_shooting_server: preempted preshooter")

                    # ensure shooter turned off
                    self.shooter_client.cancel_goals_at_and_before_time(rospy.Time.now())

                    # ensure pivot at good position
                    pivot_goal.pivot_position = 0.5
                    self.pivot_client.send_goal(pivot_goal)
                    # self.pivot_client.cancel_goals_at_and_before_time(rospy.Time.now())

                    # stop preshooter
                    self.preshooter_client.cancel_goals_at_and_before_time(rospy.Time.now())

                    self.server.set_preempted()
                    if goal.mode == goal.AMP:
                        rospy.loginfo("DRIVING BACK")
                        cmd_vel_msg = Twist()
                        start = rospy.Time.now()

                        while (rospy.Time.now() - start) < rospy.Duration(self.amp_drive_back_time):
                            cmd_vel_msg.angular.x = 0
                            cmd_vel_msg.angular.y = 0
                            cmd_vel_msg.angular.z = 0
                            cmd_vel_msg.linear.x = self.amp_drive_back_speed
                            cmd_vel_msg.linear.y = 0
                            cmd_vel_msg.linear.z = 0
                            self.cmd_vel_pub.publish(cmd_vel_msg)
                            rospy.loginfo("Publishing cmd _vel ")
                            r.sleep()
                        
                        cmd_vel_msg.angular.x = 0
                        cmd_vel_msg.angular.y = 0
                        cmd_vel_msg.angular.z = 0
                        cmd_vel_msg.linear.x = 0
                        cmd_vel_msg.linear.y = 0
                        cmd_vel_msg.linear.z = 0
                        self.cmd_vel_pub.publish(cmd_vel_msg)
                    return
                r.sleep()

            if goal.mode == goal.AMP:
                rospy.loginfo("AMP DRIVE BACK")
                cmd_vel_msg = Twist()
                start = rospy.Time.now()

                while (rospy.Time.now() - start) < rospy.Duration(self.amp_drive_back_time):
                    cmd_vel_msg.angular.x = 0
                    cmd_vel_msg.angular.y = 0
                    cmd_vel_msg.angular.z = 0
                    cmd_vel_msg.linear.x = self.amp_drive_back_speed
                    cmd_vel_msg.linear.y = 0
                    cmd_vel_msg.linear.z = 0
                    self.cmd_vel_pub.publish(cmd_vel_msg)
                    rospy.loginfo("Publishing cmd _vel ")
                    r.sleep()
                
                cmd_vel_msg.angular.x = 0
                cmd_vel_msg.angular.y = 0
                cmd_vel_msg.angular.z = 0
                cmd_vel_msg.linear.x = 0
                cmd_vel_msg.linear.y = 0
                cmd_vel_msg.linear.z = 0
                self.cmd_vel_pub.publish(cmd_vel_msg)

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