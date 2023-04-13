#! /usr/bin/env python3

# 2023 balancing server, runs PID on the pitch angle from the IMU (ZED) and 0s it

# imports for actionlib server and messages
import rospy
import actionlib
import behavior_actions.msg
import sensor_msgs.msg
import std_msgs.msg
import math
import geometry_msgs.msg._Quaternion
from tf.transformations import euler_from_quaternion # may look like tf1 but is actually tf2
import geometry_msgs.msg

def handle_param_load(name):
    try:
        res = rospy.get_param(name)
    except rospy.ROSException as e:
        rospy.logerr(f"Unable to load param with name={name}")
        return False
    return res

class Balancer:
    # create messages that are used to publish feedback/result
    _feedback = behavior_actions.msg.Balancer2023Feedback()
    _result = behavior_actions.msg.Balancer2023Result()

    def __init__(self, name):
        self.debug = False
        self._action_name = name

        self._as = actionlib.SimpleActionServer(self._action_name, behavior_actions.msg.Balancer2023Action,
                                                execute_cb=self.balancer_callback, auto_start=False)
        self._as.start()
        rospy.loginfo("Finished started actionlib server ")
        res = handle_param_load("~angle_threshold")
        self.angle_threshold = res if res else math.radians(2) 


        # load params, don't care about topics after we have the subs/pubs
        res = handle_param_load("setpoint_topic") 
        setpoint_pub_topic = res if res else "position_balance_pid/pitch_cmd_pub"

        res = handle_param_load("topic_from_plant")
        pitch_state_pub_topic = res if res else "position_balance_pid/pitch_state_pub"

        res = handle_param_load("topic_from_controller")
        x_command_sub_topic = res if res else "position_balance_pid/x_command"

        res = handle_param_load("command_timeout") # not used right now, not sure what it should be used for? Maybe delay 
        self.command_timeout = res if res else 0.5 # sec
        
        res = handle_param_load("enable_topic")
        enable_topic = res if res else "x_position_balance_pid/pid_enable"

        res = handle_param_load("imu_sub_topic")
        imu_sub_topic = res if res else "/imu/zeroed_imu"
        rospy.loginfo(f"Using imu topic {imu_sub_topic}")

        self.PID_enabled = False
        self.desired_pitch = 0.0 # just in case we want to change it later, add offset or something
        self.time_to_balamced = 2 # how many seconds of being within the angle threshold to be considered balanced 
        self.first_balance_time = None
        self.current_pitch = -999
        self.current_pitch_time = -1000
        self.pitch_offset = 0
        # should just be publishing 0.0 because thats the angle we want    
        # feeling like python has strict types with all these types I have to type out
        self.pub_setpoint = rospy.Publisher(setpoint_pub_topic, std_msgs.msg.Float64, queue_size=1)
        # publishes our current pitch from imu
        self.pub_pitch_state = rospy.Publisher(pitch_state_pub_topic, std_msgs.msg.Float64, queue_size=1)
        # result of pid node, tells us what X command to give to motors
        self.sub_x_command = rospy.Subscriber(x_command_sub_topic, std_msgs.msg.Float64, self.x_cmd_cb)
        self.pub_PID_enable = rospy.Publisher(enable_topic, std_msgs.msg.Bool, queue_size=1)
        
        self.cmd_vel_topic = "/balance_position/balancer_server/swerve_drive_controller/cmd_vel"
        self.pub_cmd_vel = rospy.Publisher(self.cmd_vel_topic, geometry_msgs.msg.Twist, queue_size=1)
        self.sub_imu = rospy.Subscriber(imu_sub_topic, sensor_msgs.msg.Imu, self.imu_callback)

        self.sub_effort = rospy.Subscriber("/teleop/orient_strafing/control_effort", std_msgs.msg.Float64, self.robot_orientation_effort_callback)
        self.state = rospy.Subscriber("/teleop/orient_strafing/setpoint", std_msgs.msg.Float64, self.state_callback)
        self.current_orient_effort = 0
        self.current_orient_setpoint = 0
        self.current_yaw = 0
        self.multipler = 1
        rospy.loginfo("Finished initalizing 2023_balancer_server")
    
    def state_callback(self, msg):
        self.current_orient_setpoint = msg.data

    def robot_orientation_effort_callback(self, msg):
        self.current_orient_effort = msg.data

    # painful to fill out all of the fields with 0, saves some code repetition
    def make_zero_twist(self):
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0 
        return msg

    def x_cmd_cb(self, x_command: std_msgs.msg.Float64):
        if self.debug:
            rospy.loginfo_throttle(1, f"X_cmd_callback with {x_command}") 
        # send to motors, @TODO
        msg = self.make_zero_twist()
        msg.linear.x = float(x_command.data) if abs(math.degrees(self.current_pitch)) > 1.5 else 0
        msg.angular.z = float(self.current_orient_effort)
        self.pub_cmd_vel.publish(msg)

    def imu_callback(self, imu_msg):
        q = imu_msg.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w]) 
        #roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        self.current_yaw = yaw
        self.current_pitch = pitch * self.multipler
        self.current_pitch_time = imu_msg.header.stamp.secs
        if self.debug:
            #rospy.loginfo_throttle(0.5, f"Balancer server - Pitch in degrees {round(pitch*(180/math.pi), 4)}\nBalancer server - offset {round(self.pitch_offset, 4)}")
            pass
        self.pub_pitch_state.publish(pitch - self.pitch_offset)

    def preempt(self):
        rospy.logwarn(f"Balancer server preempt called!")
        self.pitch_offset = 0
        msg = std_msgs.msg.Bool()
        msg.data = False
        self.pub_PID_enable.publish(msg)
        # publish last zero just in case
        twist_msg = self.make_zero_twist()
        self.pub_cmd_vel.publish(twist_msg)

    def balancer_callback(self, goal):
        if not goal.towards_charging_station:
            self.multipler = -1
        else:
            self.multipler = 1
        rospy.loginfo(f"Balancer Actionlib called with goal {goal}")
        self.pitch_offset = goal.angle_offset
        rospy.loginfo(f"Pitch offset={self.pitch_offset}")
        msg = std_msgs.msg.Bool()
        msg.data = True
        self.pub_PID_enable.publish(msg)
        
        msgF = std_msgs.msg.Float64()
        msgF.data = self.desired_pitch
        self.pub_setpoint.publish(msgF) # say we want pitch to be 0
        r = rospy.Rate(100)
        while True:
            # I think subscribers should update without spinOnce... it doesn't exist in python
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.logwarn('%s: Preempted' % self._action_name)
                self.preempt() # stop pid
                self._as.set_preempted()
                self._as.publish_feedback(self._feedback)
                return
            
            error = abs(self.current_pitch - self.desired_pitch)
            self._feedback.error = error
            stable_and_balanced = False
            if error <= self.angle_threshold:
                now = rospy.Time.now()
                if self.first_balance_time is None:
                    self.first_balance_time = now
                elif now - self.first_balance_time >= rospy.Duration.from_sec(self.time_to_balamced):
                    stable_and_balanced = True
            else:
                self.first_balance_time = None
            
            self._feedback.stable_and_balanced = stable_and_balanced
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('balancer')
    name = rospy.get_name()
    balancer_server = Balancer(name)
    rospy.spin()

