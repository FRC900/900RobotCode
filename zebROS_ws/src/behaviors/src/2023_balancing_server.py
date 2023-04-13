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
import geometry_msgs.msg
from enum import Enum
from tf.transformations import euler_from_quaternion # may look like tf1 but is actually tf2
import std_srvs.srv
import angles
import time


class States(Enum):
    # we only actually care about two states, the one with no wheels and all the rest as one state
    LEFT_SIDE_OF_STATION = -4
    ON_LEFT_SIDE_STATION = -3
    RIGHT_SIDE_STATION_DOWN = -2
    ON_GROUND_RIGHT_SIDE = -1
    NO_WEELS_ON = 0 # starting state, nothing touching charging station
    ONE_WHEEL_ON_RAMP = 1 # when first wheel gets on to the ramp
    ONE_WHEEL_ON_CENTER = 2 # when that same wheel gets to the middle, should be able to call PID node from here
    TWO_WHEELS_ON_CENTER = 3 # when we have both wheels on the center
    AFTER_TWO_WHEELS = 4
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

        # res = handle_param_load("imu_sub_topic")
        imu_sub_topic = "/imu/zeroed_imu"
        rospy.loginfo(f"Using imu topic {imu_sub_topic}")
        
        self.balancer_client = actionlib.SimpleActionClient("balancer_server", behavior_actions.msg.Balancer2023Action)
        self.balancer_client.wait_for_server(rospy.Duration(5)) # 5 sec timeout
        #rospy.wait_for_service('/frcrobot_jetson/swerve_drive_controller/brake', rospy.Duration(5))
        self.brake_srv = rospy.ServiceProxy('/frcrobot_jetson/swerve_drive_controller/brake', std_srvs.srv.Empty)

        cmd_vel_topic = "/balance_position/balancer_server/swerve_drive_controller/cmd_vel"
        self.pub_cmd_vel = rospy.Publisher(cmd_vel_topic, geometry_msgs.msg.Twist, queue_size=1)

        pub_orient_topic = "/teleop/orientation_command"
        self.pub_orient_command = rospy.Publisher(pub_orient_topic, std_msgs.msg.Float64, queue_size=1)

        self.PID_enabled = False
        self.current_pitch = -1000
        # used for determining if to snap to 0 or 180
        self.current_yaw = -1000
        self.state = States.LEFT_SIDE_OF_STATION
        self.sub_imu = rospy.Subscriber(imu_sub_topic, sensor_msgs.msg.Imu, self.imu_callback)

        self.sub_effort = rospy.Subscriber("/teleop/orient_strafing/control_effort", std_msgs.msg.Float64, self.robot_orientation_effort_callback)
        self.current_orient_effort = 0
        rospy.loginfo("Finished initalizing balancing server")

    def robot_orientation_effort_callback(self, msg):
        self.current_orient_effort = msg.data

    def imu_callback(self, imu_msg):
        rospy.logdebug("Imu callback")
        q = imu_msg.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w]) 
        #roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        self.current_pitch = pitch
        self.current_yaw = yaw
        #rospy.loginfo_throttle(0.25, f"Balancing server - pitch in degrees {round(pitch*(180/math.pi), 4)}")
        if self.debug:
            rospy.loginfo_throttle(1, f"Balancing server - Pitch in degrees {round(pitch*(180/math.pi), 4)}")

    def preempt(self):
        rospy.logwarn(f"Called preempt for balancING server")
        #self.balancer_client.cancel_all_goals()
        cmd_vel_msg = geometry_msgs.msg.Twist()
        cmd_vel_msg.angular.x = 0 # todo, tune me
        cmd_vel_msg.angular.y = 0
        cmd_vel_msg.angular.z = 0
        cmd_vel_msg.linear.x = 0
        cmd_vel_msg.linear.y = 0
        cmd_vel_msg.linear.z = 0
        self.pub_cmd_vel.publish(cmd_vel_msg)
        self.balancer_client.cancel_goal()
        self.state = States.LEFT_SIDE_OF_STATION
        #self.balancer_client.cancel_goals_at_and_before_time(rospy.Time.now())

    # assumes that positive 
    def balancing_callback(self, goal):
        if not goal.towards_charging_station:
            multipler = -1.0
        else:
            multipler = 1.0
        
        rospy.loginfo(f"Auto Balancing Actionlib called with goal {goal}")
        # snap to 0 or 180, whichever is closer
        zero_dist = abs(angles.shortest_angular_distance(self.current_yaw, 0))
        pi_dist = abs(angles.shortest_angular_distance(self.current_yaw, math.pi))
        rospy.loginfo(f"Dist to 0 is {zero_dist} and pi_dist {pi_dist}")
        if zero_dist < pi_dist:
            if abs(zero_dist) > math.radians(20):
                rospy.logerr("Will not rotate more than 20 degrees! Exiting balance")
                self.preempt()
                self._as.set_preempted()
                return
            target_angle = 0
        else:
            if abs(pi_dist) > math.radians(20):
                rospy.logerr("Will not rotate more than 20 degrees! Exiting balance")
                self.preempt()
                self._as.set_preempted()
                return
            target_angle = math.pi
 
        r = rospy.Rate(100)
        start_time = rospy.Time.now()
        print(f"Target angle is {target_angle}=========x")

        # I think subscribers should update without spinOnce... it doesn't exist in python
        # check that preempt has not been requested by the client
        if self._as.is_preempt_requested() or rospy.is_shutdown():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self.preempt() # stop pid
            self._as.set_preempted()
            return
        
        orientation_msg = std_msgs.msg.Float64()
        orientation_msg.data = target_angle
        self.pub_orient_command.publish(orientation_msg)
        rospy.logerr(f"Multipler is {multipler}")

        while True:
            # finding out when we go up for first time
            if (((self.current_pitch <= -math.radians(13) and goal.towards_charging_station) or
                (self.current_pitch >= math.radians(13) and not goal.towards_charging_station))):
                rospy.logwarn(f"Found first going up! Angle = {math.degrees(self.current_pitch)}")
                break
            rospy.loginfo_throttle(2, "Sending cmd of 1")
            cmd_vel_msg = geometry_msgs.msg.Twist()
            cmd_vel_msg.angular.x = 0 # todo, tune me
            cmd_vel_msg.angular.y = 0
            cmd_vel_msg.angular.z = self.current_orient_effort
            cmd_vel_msg.linear.x = 0.9 * multipler
            cmd_vel_msg.linear.y = 0
            cmd_vel_msg.linear.z = 0
            self.pub_cmd_vel.publish(cmd_vel_msg)
            self.pub_orient_command.publish(orientation_msg)
            r.sleep()


        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time <= rospy.Duration(40)):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self.preempt() # stop pid
                self._as.set_preempted()
                return
            if (((self.current_pitch >= math.radians(9) and goal.towards_charging_station) or
        (self.current_pitch <= -math.radians(9) and not goal.towards_charging_station))):
                print(f"Found going down! Angle = {math.degrees(self.current_pitch)}")
                break
            cmd_vel_msg = geometry_msgs.msg.Twist()
            cmd_vel_msg.angular.x = 0 # todo, tune me
            cmd_vel_msg.angular.y = 0
            cmd_vel_msg.angular.z = self.current_orient_effort
            cmd_vel_msg.linear.x = 1.5 * multipler
            cmd_vel_msg.linear.y = 0
            cmd_vel_msg.linear.z = 0
            self.pub_cmd_vel.publish(cmd_vel_msg)
            self.pub_orient_command.publish(orientation_msg)
            r.sleep()

        # finding out when we go down
        start_time = rospy.Time.now()
        rospy.logwarn(f"Moving to stage 2============ Angle = {math.degrees(self.current_pitch)}")
        while (rospy.Time.now() - start_time <= rospy.Duration(30)):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self.preempt() # stop pid
                self._as.set_preempted()
                return
            if abs(self.current_pitch) <= math.radians(1):
                rospy.logwarn(f"Found floor! Angle = {math.degrees(self.current_pitch)}")
                break
            cmd_vel_msg = geometry_msgs.msg.Twist()
            cmd_vel_msg.angular.x = 0 # todo, tune me
            cmd_vel_msg.angular.y = 0
            cmd_vel_msg.angular.z = self.current_orient_effort
            cmd_vel_msg.linear.x = 1.5 * multipler
            cmd_vel_msg.linear.y = 0
            cmd_vel_msg.linear.z = 0
            self.pub_cmd_vel.publish(cmd_vel_msg)
            self.pub_orient_command.publish(orientation_msg)
            r.sleep()
        
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time <= rospy.Duration(0.2)):
            cmd_vel_msg = geometry_msgs.msg.Twist()
            cmd_vel_msg.angular.x = 0 # todo, tune me
            cmd_vel_msg.angular.y = 0
            cmd_vel_msg.angular.z = self.current_orient_effort
            cmd_vel_msg.linear.x = 0.9 * multipler
            cmd_vel_msg.linear.y = 0
            cmd_vel_msg.linear.z = 0
            self.pub_cmd_vel.publish(cmd_vel_msg)
            self.pub_orient_command.publish(orientation_msg) 
            r.sleep()

        # finding out when we are on the floor
        start_time = rospy.Time.now()
        rospy.logwarn("Moving to stage on the floor============")
        while (rospy.Time.now() - start_time <= rospy.Duration(30)):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self.preempt() # stop pid
                self._as.set_preempted()
                return
            rospy.loginfo_throttle(0.25, f"Angle = {math.degrees(self.current_pitch)}")
            # finding out when we are going back on the station
            if (((self.current_pitch >= math.radians(9) and goal.towards_charging_station) or
            (self.current_pitch <= -math.radians(9) and not goal.towards_charging_station))):
                rospy.logwarn(f"Found going up for second time Angle = {math.degrees(self.current_pitch)}")
                break    
        
            cmd_vel_msg = geometry_msgs.msg.Twist()
            cmd_vel_msg.angular.x = 0 # todo, tune me
            cmd_vel_msg.angular.y = 0
            cmd_vel_msg.angular.z = self.current_orient_effort
            cmd_vel_msg.linear.x = 0.9 * multipler * -1 
            cmd_vel_msg.linear.y = 0
            cmd_vel_msg.linear.z = 0
            self.pub_cmd_vel.publish(cmd_vel_msg)
            self.pub_orient_command.publish(orientation_msg)
            r.sleep()

        start_time = rospy.Time.now()
        rospy.logwarn("After moving to the floor")
        while (rospy.Time.now() - start_time <= rospy.Duration(1)):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self.preempt() # stop pid
                self._as.set_preempted()
                return
            cmd_vel_msg = geometry_msgs.msg.Twist()
            cmd_vel_msg.angular.x = 0 # todo, tune me
            cmd_vel_msg.angular.y = 0
            cmd_vel_msg.angular.z = self.current_orient_effort
            cmd_vel_msg.linear.x = 0.9 * multipler * -1
            cmd_vel_msg.linear.y = 0
            cmd_vel_msg.linear.z = 0
            self.pub_cmd_vel.publish(cmd_vel_msg)
            self.pub_orient_command.publish(orientation_msg)
            r.sleep()

        rospy.loginfo("Running PID!")
        goal_to_send = behavior_actions.msg.Balancer2023Goal()
        goal_to_send.angle_offset = 0
        goal_to_send.towards_charging_station = goal.towards_charging_station
        self.balancer_client.send_goal(goal_to_send)
        while not (-9 < math.degrees(self.current_pitch) < 9):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self.preempt() # stop pid
                self._as.set_preempted()
                return
            self.pub_orient_command.publish(orientation_msg)
            r.sleep()
            
        rospy.loginfo("Within tolerance, stopping and breaking!")
        self.balancer_client.cancel_all_goals()
        self.state = States.AFTER_TWO_WHEELS
        rospy.sleep(0.25)
    
        rospy.loginfo("running final PID!")
        goal_to_send = behavior_actions.msg.Balancer2023Goal()
        goal_to_send.angle_offset = 0
        goal_to_send.towards_charging_station = goal.towards_charging_station
        self.balancer_client.send_goal(goal_to_send)
        while True and not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self.preempt() # stop pid
                self._as.set_preempted()
                return
            self.pub_orient_command.publish(orientation_msg)
            r.sleep()
            
        # publish the feedback
        self._result.success = True
        self._as.publish_feedback(self._feedback)
        r.sleep()
        self._as.set_succeeded(self._result)
            

if __name__ == '__main__':
    rospy.logwarn("Initalizing balancing server")
    rospy.init_node('balancer')
    name = rospy.get_name()
    balancer_server = AutoBalancing(name)
    rospy.spin()

