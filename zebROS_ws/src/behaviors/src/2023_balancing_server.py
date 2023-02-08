#! /usr/bin/env python3

# balancing server, controls the entire balancing process and keeps track of the state and eventually leads to robot being balanced
# assumes that the robot is already aligned with the charging station
# ALSO assumes that negative is up 



# imports for actionlib server and messages
import rospy
import actionlib
import behavior_actions.msg
import sensor_msgs.msg
import std_msgs.msg
import math
import geometry_msgs.msg._Quaternion
from enum import Enum
from tf.transformations import euler_from_quaternion # may look like tf1 but is actually tf2

class States(Enum):
    # we only actually care about two states, the one with no wheels and all the rest as one state
    NO_WEELS_ON = 0 # starting state, nothing touching charging station
    ONE_WHEEL_ON_RAMP = 1 # when first wheel gets on to the ramp
    ONE_WHEEL_ON_CENTER = 2 # when that same wheel gets to the middle, should be able to call PID node from here
    TWO_WHEELS_ON_CENTER = 3 # when we have both wheels on the center
    # no states after this because if we fall off the other side we have done something very wrong and need to restart

def handle_param_load(name):
    try:
        res = rospy.get_param(name)
    except rospy.ROSException as e:
        rospy.logerr(f"Unable to load param with name={name}")
        return False
    return res

class AutoBalancing:
    # create messages that are used to publish feedback/result
    _feedback = behavior_actions.msg.Balancing2023Feedback()
    _result = behavior_actions.msg.Balancing2023Result()

    def __init__(self, name):
        self.debug = False
        self._action_name = name

        self._as = actionlib.SimpleActionServer(self._action_name, behavior_actions.msg.Balancing2023Action,
                                                execute_cb=self.balancing_callback, auto_start=False)
        self._as.start()

        res = handle_param_load("imu_sub_topic")
        imu_sub_topic = res if res else "/imu/zeroed_imu"
        rospy.loginfo(f"Using imu topic {imu_sub_topic}")
        
        self.balancer_client = actionlib.SimpleActionClient("balancer_server", behavior_actions.msg.Balancer2023Action)
        self.balancer_client.wait_for_server(rospy.Duration(5)) # 5 sec timeout
        
        self.angle_to_add = math.radians(20)
        self.PID_enabled = False
        self.current_pitch = -999
        self.current_pitch_time = -1000
        self.state = States.NO_WEELS_ON 
        self.sub_imu = rospy.Subscriber(imu_sub_topic, sensor_msgs.msg.Imu, self.imu_callback)
        self.down_measurements = 0
        self.threshold = math.radians(0.5) # how much a measurement must change by to be counted
        
        rospy.loginfo("Finished initalizing balancing server")

    def imu_callback(self, imu_msg):
        rospy.logdebug("Imu callback")
        q = imu_msg.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w]) 
        #roll = euler[0]
        pitch = euler[1]
        #yaw = euler[2]
        self.current_pitch = pitch
        self.current_pitch_time = imu_msg.header.stamp.secs
        if self.debug:
            rospy.loginfo_throttle(1, f"Balancing server - Pitch in degrees {round(pitch*(180/math.pi), 4)}")

    def preempt(self):
        rospy.logwarn(f"Called preempt for balancING server")
        #self.balancer_client.cancel_all_goals()
        
        self.balancer_client.cancel_goal()
        self.state = States.NO_WEELS_ON
        #self.balancer_client.cancel_goals_at_and_before_time(rospy.Time.now())

    def balancing_callback(self, goal):
        rospy.loginfo(f"Auto Balancing Actionlib called with goal {goal}")
        # TODO, make sure we are aligned with 0 degrees (or 180 depending on how it works out)d
        
        goal_to_send = behavior_actions.msg.Balancer2023Goal()
        goal_to_send.angle_offset = self.angle_to_add
    
        self.balancer_client.send_goal(goal_to_send)
        
        # angle when we are on the ramp where balancer can take over = 0.259rad ~ 15 degrees
        # going to try and run pid to get to that and then run balancer? 
        r = rospy.Rate(100)
        while True:                
            # I think subscribers should update without spinOnce... it doesn't exist in python
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self.preempt() # stop pid
                self._as.set_preempted()
                break

            if abs(self.current_pitch) >= math.radians(10) and self.state == States.NO_WEELS_ON:
                rospy.logwarn("Recalled balancer with 0 offset=======================")
                self.balancer_client.cancel_all_goals()
                r.sleep()
                goal_to_send = behavior_actions.msg.Balancer2023Goal()
                goal_to_send.angle_offset = 0
                self.balancer_client.send_goal(goal_to_send)
                self.state = States.ONE_WHEEL_ON_RAMP

            # publish the feedback
            self._as.publish_feedback(self._feedback)
            r.sleep()

if __name__ == '__main__':
    rospy.logwarn("Initalizing balancing server")
    rospy.init_node('balancer')
    name = rospy.get_name()
    balancer_server = AutoBalancing(name)
    rospy.spin()

