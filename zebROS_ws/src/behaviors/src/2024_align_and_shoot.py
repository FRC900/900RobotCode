#!/usr/bin/env python3

import rospy
import actionlib

from behavior_actions.msg import AlignAndShoot2024Goal, AlignAndShoot2024Result, AlignAndShoot2024Feedback, AlignAndShoot2024Action   
from behavior_actions.msg import AlignToSpeaker2024Goal, AlignToSpeaker2024Result, AlignToSpeaker2024Feedback, AlignToSpeaker2024Action
from behavior_actions.msg import Shooting2024Goal, Shooting2024Feedback, Shooting2024Result, Shooting2024Action
from behavior_actions.msg import AutoAlignSpeaker
import geometry_msgs.msg
from frc_msgs.msg import MatchSpecificData
import std_srvs.srv

#from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from std_msgs.msg import Header

from sensor_msgs.msg import JointState

from geometry_msgs.msg import TwistStamped

import tf2_ros, time, math

class AlignAndShoot:
    def __init__(self, name):
        self.action_name = name
        self.result = AlignAndShoot2024Result()
        self.feedback = AlignAndShoot2024Feedback()
        
        self.preshooter_switch = 0
        self.align_to_speaker_client = actionlib.SimpleActionClient('/align_to_speaker/align_to_speaker_2024', AlignToSpeaker2024Action) #figure out the name for the server thingy namespace
        rospy.loginfo("2024_align_and_shoot: waiting for align to speaker server")
        self.align_to_speaker_client.wait_for_server()
        self.shooting_client = actionlib.SimpleActionClient('/shooting/shooting_server_2024', Shooting2024Action) #figure out waht the name or the serverthingy name space
        rospy.loginfo("2024_align_and_shoot: waiting for shooting server")
        # self.shooting_client.wait_for_server()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.dist_sub = rospy.Subscriber("/speaker_align/dist_and_ang", AutoAlignSpeaker, self.distance_and_angle_callback, tcp_nodelay=True, queue_size=1) #use this to find the distance that we are from the spaerk thing
        self.dist_value = 1.5

        if self.dist_sub.get_num_connections() > 0: rospy.loginfo("2024_align_and_shoot: distance and angle topic being published to")
        
        self.server = actionlib.SimpleActionServer(self.action_name, AlignAndShoot2024Action, execute_cb=self.execute_cb, auto_start = False)
        rospy.loginfo("2024_align_and_shoot: starting server")
        
        self.last_relocalize_sub = rospy.Subscriber("/last_relocalize", Header, self.relocalized_cb, tcp_nodelay=True, queue_size=1)
        self.joint_state_sub = rospy.Subscriber("/frcrobot_rio/joint_states", JointState, callback=self.rio_callback)

        self.enable_continuous_autoalign_server = rospy.Service("enable_autoalign", std_srvs.srv.SetBool, self.enable_cb)
        self.enable_continuous_autoalign = False

        self.dont_send_shooting_goal = False
        
        self.localization_timeout = rospy.get_param("localization_timeout")

        self.only_shooter_during_continuous_autoalign = rospy.get_param("only_shooter_during_continuous_autoalign")

        self.aligning = False

        self.alliance_color = MatchSpecificData.ALLIANCE_COLOR_UNKNOWN
        self.is_teleop_and_enabled = False
        self.match_data_sub = rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, callback=self.match_data_cb, tcp_nodelay=True, queue_size=1)

        self.linear_vel_threshold = 0.2
        self.angular_vel_threshold = 0.2
        self.stopped = False
        self.cmd_vel_sub = rospy.Subscriber("/frcrobot_jetson/swerve_drive_controller/cmd_vel_out", TwistStamped, self.cmd_vel_cb)

        self.last_relocalized = rospy.Time()
        self.align_to_speaker_done = False
        self.shooting_done = False
        self.server.start()
        rospy.loginfo("2024_align_and_shoot: starting timer")
        rospy.loginfo("2024_align_and_shoot: server started")

    def cmd_vel_cb(self, msg: TwistStamped):
        self.stopped = math.hypot(msg.twist.linear.x, msg.twist.linear.y) <= self.linear_vel_threshold and msg.twist.angular.z <= self.angular_vel_threshold

    def match_data_cb(self, msg: MatchSpecificData):
        self.alliance_color = msg.allianceColor
        if not (msg.Enabled and not msg.Autonomous) and self.is_teleop_and_enabled:
            self.preempt()
        self.is_teleop_and_enabled = msg.Enabled and not msg.Autonomous

    def enable_cb(self, req: std_srvs.srv.SetBoolRequest):
        if self.enable_continuous_autoalign and not req.data:
            self.preempt()
        self.enable_continuous_autoalign = req.data
        rospy.loginfo(f"enable continouous autoalign service called with {req.data}")
        return std_srvs.srv.SetBoolResponse(success=True,message="")

    def relocalized_cb(self, msg: Header):
        rospy.loginfo_throttle(0.5, "2024_align_and_shoot : relocalized")
        self.last_relocalized = msg.stamp

    def distance_and_angle_callback(self, msg: AutoAlignSpeaker):
        self.dist_value = msg.distance
    
    def align_to_speaker_feedback_cb(self, feedback: AlignToSpeaker2024Feedback):
        self.align_to_speaker_done = feedback.aligned
        # rospy.loginfo("2024 Align and shoot: Align to speaker")
    
    def shooting_feedback_cb(self, feedback: Shooting2024Feedback):
        self.shooting_done = (feedback.current_stage == feedback.SHOOTING)
        rospy.logwarn("2024 Align and shoot: Shooting done!")
    
    def rio_callback(self, data):
        # check diverter_switch
        if "preshooter_limit_switch" in data.name:
            # if this is in auto, this sees that the note left (we shot), cancels shooter movement, and we're sad in the auto node
            if self.preshooter_switch and not data.position[data.name.index("preshooter_limit_switch")] and self.is_teleop_and_enabled:
                self.preempt()
            self.preshooter_switch = data.position[data.name.index("preshooter_limit_switch")]
        else:
            rospy.logwarn_throttle(1.0, f'2024_align_and_shoot: preshooter_limit_switch not found')
            pass

    def preempt(self):
        self.align_to_speaker_client.cancel_goals_at_and_before_time(rospy.Time.now())
        shooting_goal = Shooting2024Goal()
        shooting_goal.cancel_movement = True
        self.shooting_client.send_goal(shooting_goal)
        self.aligning = False

    def execute_cb(self, goal: AlignAndShoot2024Goal):
        self.align_to_speaker_done = False

        r = rospy.Rate(60)
        align_to_speaker_goal = AlignToSpeaker2024Goal()
        shooting_goal = Shooting2024Goal()

        rospy.loginfo("2024_align_and_shoot: spinning up shooter")
        self.dont_send_shooting_goal = True
        shooting_goal.mode = shooting_goal.SPEAKER
        shooting_goal.distance = self.dist_value #sets the dist value for goal ditsance with resepct ot hte calblack
        shooting_goal.setup_only = True
        shooting_goal.only_shooter_setup = True
        shooting_goal.leave_spinning = True

        self.shooting_client.send_goal(shooting_goal)

        if not self.aligning:
            align_to_speaker_goal.align_forever = goal.align_forever
            align_to_speaker_goal.offsetting = goal.offsetting
            self.aligning = goal.align_forever
            self.align_to_speaker_client.send_goal(align_to_speaker_goal, feedback_cb=self.align_to_speaker_feedback_cb)
            rospy.loginfo("2024_align_and_shoot: align goal sent")

        relocalized_recently = True

        # We want to run this loop while rospy is not shutdown, and:
        # we have not relocalized recently OR we have not aligned to speaker OR we are not done shooting
        while (not self.align_to_speaker_done) or (not self.stopped) and not rospy.is_shutdown():
            #relocalized_recently = (rospy.Time.now() - self.last_relocalized) < rospy.Duration(self.localization_timeout)
            rospy.loginfo_throttle(0.1, f"2024_align_and_shoot: aligning waiting on {'speaker' if not self.align_to_speaker_done else ''} {'localization' if not relocalized_recently else ''} {'stopping' if not self.stopped else ''}")
            if self.server.is_preempt_requested():
                rospy.loginfo("2024_align_and_shoot: preempted")
                self.align_to_speaker_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.shooting_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.server.set_preempted()
                return
            r.sleep()

        rospy.loginfo("2024_align_and_shoot: done aligning, about to shoot")
        self.dont_send_shooting_goal = True
        shooting_goal.mode = shooting_goal.SPEAKER
        shooting_goal.distance = self.dist_value #sets the dist value for goal ditsance with resepct ot hte calblack
        shooting_goal.setup_only = False
        shooting_goal.leave_spinning = False
        shooting_done = False

        def shooting_done_cb(state, result):
            nonlocal shooting_done
            shooting_done = True
        self.shooting_client.send_goal(shooting_goal, done_cb=shooting_done_cb)
        rospy.loginfo("2024_align_and_shoot: sent shooting client goal")

        while not shooting_done and not rospy.is_shutdown():
            rospy.loginfo_throttle(0.5, "2024_align_and_shoot: waiting for shooting server to finish")
            if self.server.is_preempt_requested():
                rospy.loginfo("2024_align_and_shoot: preempted")
                self.align_to_speaker_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.shooting_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.server.set_preempted()
                self.dont_send_shooting_goal = False
                return
            r.sleep()

        rospy.loginfo("2024_align_and_shoot: succeeded")
        self.result.success = True
        self.server.set_succeeded(self.result)
        time.sleep(0.5)
        self.dont_send_shooting_goal = False


if __name__ == '__main__':
    rospy.init_node('align_and_shoot_2024')
    
    server = AlignAndShoot(rospy.get_name())
    rospy.spin()