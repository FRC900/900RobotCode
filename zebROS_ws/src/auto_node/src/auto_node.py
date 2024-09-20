#!/usr/bin/python3
import signal
import sys

from typing import List
from dataclasses import dataclass, field

from ActionRunner import ActionRunner
from action import Action

import rospy
from behavior_actions.msg import AutoMode

from threading import RLock, Thread

class AutoNode():
    def __init__(self):
        super().__init__('autonomous_node')
        
        self.__auto_sub = NodeHandle.node_handle.create_subscription(msg_type=AutonomousSelection, topic="AutonomousSelection", callback=self.get_selected_auto, qos_profile=rclpy.qos.qos_profile_default)
        self.__selected_auto: str

        self.create_timer(timer_period_sec=1.0/50.0, callback=self.loop)

    def loop(self) -> None:
        """
        Periodic function for the autonomous node.
        """

        from autonomous_node.autos import init_auto_selection_map, AutonomousNames
        from autonomous_node.autos.AutoBase import AutoBase

        if rclpy.ok():
            robot_mode : RobotMode = FRCRobotUtils.robot_status.get_mode()

            if robot_mode == RobotMode.TELEOP:
                return

            if self.selected_autonomous_string is None:
                self.AutonomousConfiguration_options.selected_autonomous = "Invalid Auto"
            else:
                self.AutonomousConfiguration_options.selected_autonomous = self.selected_autonomous_string

            self.AutonomousConfiguration_publisher.publish(self.AutonomousConfiguration_options)

            try:
                if (self.__selected_auto is None or self.selected_autonomous_string != self.__prev_selected_autonomous_string) \
                        and AutonomousNames(self.selected_autonomous_string) in self.AutonomousSelection_MAP:
                    self.__selected_auto : AutoBase = self.AutonomousSelection_MAP[AutonomousNames(self.selected_autonomous_string)]
                elif AutonomousNames(self.selected_autonomous_string) not in self.AutonomousSelection_MAP:
                    self.__selected_auto = None
            except Exception as e:
                NodeHandle.node_handle.get_logger().error(f"Invalid auto string received! {self.selected_autonomous_string}", throttle_duration_sec=10)
                #NodeHandle.node_handle.get_logger().error(e.__str__()) #detailed debugging info
                self.__selected_auto = None

            if robot_mode == RobotMode.AUTONOMOUS:
                # Start the action on the transition from Disabled to Auto.
                if self.__prev_robot_mode == RobotMode.DISABLED:
                    self.get_logger().info(f"Robot in auto, prev disabled")
                    if self.__selected_auto is not None:
                        self.get_logger().info(f"Selected auto is not none. It is {self.__selected_auto_action}")
                        self.runner.start_action(self.__selected_auto_action)
                        self.__prev_selected_auto = None
            elif robot_mode == RobotMode.DISABLED:
                if self.__prev_robot_mode == RobotMode.AUTONOMOUS:
                    self.__selected_auto = None
                    self.runner.reset_action_list()

                if self.__selected_auto != self.__prev_selected_auto:
                    if self.__selected_auto != None:
                        self.get_logger().info(f"Recreating actions for {self.__selected_auto.display_name}")
                        self.__selected_auto_action = self.__selected_auto.get_action()
                        self.get_logger().info(f"Actions recreated for {self.__selected_auto.display_name}")
                    elif self.__prev_robot_mode == RobotMode.AUTONOMOUS:
                        pass
                    else:
                        NodeHandle.node_handle.get_logger().error(f"Selected auto is none! {self.selected_autonomous_string}", throttle_duration_sec=10)
                        
                if self.__selected_auto is not None:
                    self.__selected_auto.reset()

            self.__prev_robot_mode = robot_mode
            self.__prev_selected_auto = self.__selected_auto
            self.__prev_selected_autonomous_string = self.selected_autonomous_string

            if self.__prev_robot_mode == RobotMode.AUTONOMOUS:
                # Only loop in auto
                self.runner.loop(robot_mode)

    def get_selected_auto(self, msg : AutoMode) -> None:
        rospy.loginfo(f"Recived auto mode of - {msg.auto_mode}")
        self.selected_auto_id = msg.auto_mode

def signal_handler(sig, frame):
    rospy.logerr("Auto node recived SIGINT - shutting down")
    sys.exit(0)     

def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("auto_node")
    auto = AutoNode()


if __name__ == '__main__':
    main()
