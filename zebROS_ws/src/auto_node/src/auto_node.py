#!/usr/bin/python3
import signal
import sys

from typing import List
from action_runner import ActionRunner
from action import Action

import rospy
from behavior_actions.msg import AutoMode

from frc_utils.match_data_helper import RobotStatusHelper, Alliance, RobotMode



class AutoNode():
    def __init__(self):        
        self.__auto_sub = rospy.Subscriber("/auto/auto_mode", AutoMode, self.set_auto_id)
        self.__selected_auto_id: int
        self.__path_pub = rospy.Publisher("/auto/selected_path" )
        self.__robot_status = RobotStatusHelper() 
        self.create_timer(timer_period_sec=1.0/50.0, callback=self.loop)

    def loop(self) -> None:
        """
        Periodic function for the autonomous node.
        """

        if rospy.is_shutdown():
            rospy.logerr("Shutdown")
            exit(0)
            
        robot_mode : RobotMode = self.__robot_status.get_mode()

        if robot_mode == RobotMode.TELEOP:
            return

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

    def set_auto_id(self, msg : AutoMode) -> None:
        rospy.loginfo_throttle(3, f"Recived auto mode of - {msg.auto_mode}")
        self.__selected_auto_id = msg.auto_mode

def signal_handler(sig, frame):
    rospy.logerr("Auto node recived SIGINT - shutting down")
    sys.exit(0)     

def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("auto_node")
    auto = AutoNode()


if __name__ == '__main__':
    main()
