#!/usr/bin/env python3

from abc import ABC, abstractmethod
import rospy
import actionlib
from behavior_actions.msg import SysIdAction, SysIdGoal, SysIdResult, SysIdFeedback
from std_srvs.srv import SetBool, SetBoolRequest
from ros_control_boilerplate.srv import SignalLoggerString, SignalLoggerStringRequest
from geometry_msgs.msg import Twist
from talon_controller_msgs.srv import Command, CommandRequest

# sysid mechanism - hide the implementation details of the mechanism
#   being controlled behind simple start / set_voltage / stop methods
# A 1-motor mechanism will just pass through publishing or service calls
#   to the motor controller
# More complex mechanisms (e.g. swerve drive) will have their own implementations
#   of start / set_voltage / stop which inlcude additional processing
class SysIdMechanism(ABC):
    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def set_voltage(self, voltage):
        pass

class SysIdSwerveMechanism(SysIdMechanism):
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/frcrobot_jetson/swerve_drive_controller/cmd_vel', Twist, queue_size=1)
        rospy.loginfo('SysIdSwerveMechanism : Waiting for swerve drive controller')
        self.voltage_out_client = rospy.ServiceProxy('/frcrobot_jetson/swerve_drive_controller/voltage_out_drive_mode', SetBool)
        self.voltage_out_client.wait_for_service()
        rospy.loginfo("SysIdSwerveMechanism : Found swerve_drive_controller")

    def start(self):
        self.voltage_out_client.call(SetBoolRequest(True))
        self.set_voltage(0)

    def stop(self):
        self.set_voltage(0)
        self.voltage_out_client.call(SetBoolRequest(False))

    def set_voltage(self, voltage):
        cmd_vel = Twist()
        cmd_vel.linear.x = voltage
        cmd_vel.linear.y = 0
        cmd_vel.linear.z = 0
        cmd_vel.angular.x = 0
        cmd_vel.angular.y = 0
        cmd_vel.angular.z = 0
        self.cmd_vel_pub.publish(cmd_vel)

class SysIdSimpleMechanism(SysIdMechanism):
    def __init__(self, command_service):
        rospy.loginfo(f'SysIdSwerveMechanism : Waiting for controller {command_service}')
        self.command_client = rospy.ServiceProxy(command_service, Command)
        self.command_client.wait_for_service()
        rospy.loginfo("SysIdSwerveMechanism : Found controller")

    def start(self):
        self.set_voltage(0)

    def stop(self):
        self.set_voltage(0)

    def set_voltage(self, voltage):
        self.motor_client.call(CommandRequest(voltage))

class SysIdServer(object):
    def __init__(self, name, mechanism):
        self.result = SysIdResult()
        self.feedback = SysIdFeedback()

        self.mechanism = mechanism

        self.static_hold_time = rospy.get_param('static_hold_time', 1.0) # seconds

        self.signal_logger_client = rospy.ServiceProxy('/frcrobot_jetson/write_string', SignalLoggerString)
        rospy.loginfo("Waiting for signal logger service")
        self.signal_logger_client.wait_for_service()
        rospy.loginfo("Found signal logger service")

        self.server = actionlib.SimpleActionServer(name, SysIdAction, execute_cb=self.execute_cb, auto_start = False)
        self.server.start()
        rospy.loginfo("Started SysId actionlib server")

    def state_enum_to_string(self, quasistatic, forward):
        if quasistatic:
            if forward:
                return "quasistatic-forward"
            else:
                return "quasistatic-reverse"
        else:
            if forward:
                return "dynamic-forward"
            else:
                return "dynamic-reverse"

    def execute_cb(self, goal : SysIdGoal):
        self.feedback.voltage_out = 0
        self.server.publish_feedback(self.feedback)

        r = rospy.Rate(20.0)

        if not self.signal_logger_client.call(SignalLoggerStringRequest("state", self.state_enum_to_string(goal.quasistatic, goal.forward))):
            rospy.loginfo("Signal logger client service call failed")

        r.sleep()
        self.mechanism.start()
        r.sleep()

        direction_multiplier = 1.0 if goal.forward else -1.0
        finished_time = None
        start_time = rospy.Time.now()

        while not rospy.is_shutdown() and not self.server.is_preempt_requested():
            if (goal.quasistatic):
                elapsed_sec = (rospy.Time.now() - start_time).to_sec()
                self.feedback.voltage_out = min(elapsed_sec * goal.ramp_rate, goal.voltage_target)
            else:
                self.feedback.voltage_out = goal.voltage_target
            self.feedback.voltage_out *= direction_multiplier

            if (self.feedback.voltage_out >= goal.voltage_target):
                if (finished_time is None):
                    finished_time = rospy.Time.now()
            else:
                finished_time = None

            if (finished_time is not None) and ((rospy.Time.now() - finished_time).to_sec() > self.static_hold_time):
                break

            self.mechanism.set_voltage(self.feedback.voltage_out)
            self.server.publish_feedback(self.feedback)
            r.sleep()

        self.mechanism.stop()
        self.feedback.voltage_out = 0
        self.server.publish_feedback(self.feedback)
        r.sleep()

        if self.server.is_preempt_requested():
            rospy.logwarn("SysId Server Preempted")
            self.result.success = False
            self.server.set_preempted()
        else:
            self.result.success = True
            self.server.set_succeeded(self.result)

        r.sleep()

if __name__ == '__main__':
    rospy.init_node('sysid_server')
    swerve_mechanism = SysIdSwerveMechanism()
    swerve_server = SysIdServer('sysid_swerve_server', swerve_mechanism)
    rospy.spin()