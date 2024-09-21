#!/usr/bin/python3
import signal
import sys

from typing import List
from action_runner import ActionRunner
from auto_base import AutoBase
from action import Action

import rospy
from behavior_actions.msg import AutoMode

import autos
from autos import AutonomousNames
from frc_utils.match_data_helper import RobotStatusHelper, Alliance, RobotMode

# number to str of auto name
IDS_TO_AUTO_NAME: dict[int, str] = {2: AutonomousNames.Test4Note}


class AutoNode():
    def __init__(self):        
        self.__selected_autonomous_string: str = ""
        self.__prev_selected_autonomous_string : str = ""

        self.runner = ActionRunner()

        self.__prev_robot_mode = RobotMode.DISABLED
        self.__selected_auto : AutoBase = None
        self.__prev_selected_auto : AutoBase = None
        self.__selected_auto_action : Action = None

        #self.__path_pub = rospy.Publisher("/auto/selected_path")
        self.__robot_status = RobotStatusHelper() 
        self.AUTO_NAME_TO_AUTOBASE = autos.init_auto_selection_map()
        self.timer = rospy.Timer(period=rospy.Duration(1.0/50.0), callback=self.loop)
        self.__auto_sub = rospy.Subscriber("/auto/auto_mode", AutoMode, self.set_auto_id)

    def loop(self, _) -> None:
        """
        Periodic function for the autonomous node.
        """
        rospy.loginfo_throttle(10, "Auto mode loop running")
        if rospy.is_shutdown():
            rospy.logerr("Shutdown")
            exit(0)
            
        robot_mode : RobotMode = self.__robot_status.get_mode()

        # never ever do anything in teleop. 
        if robot_mode == RobotMode.TELEOP:
            self.__selected_auto = None
            self.runner.reset_action_list()
            return

        # only way this happens is if we havent set a valid auto, otherwise it will just be the last valid selected
        if self.__selected_auto is None:                 
            rospy.logerr("No auto selected!")
        
        if robot_mode == RobotMode.AUTONOMOUS:
            # Start the action on the transition from Disabled to Auto.
            if self.__prev_robot_mode == RobotMode.DISABLED:
                rospy.loginfo(f"Auto node - Robot in auto, prev disabled")
                if self.__selected_auto is not None:
                    rospy.loginfo(f"Selected auto is NOT none. It is {self.__selected_auto_action} Running auto.")
                    self.runner.start_action(self.__selected_auto_action)
                    self.__prev_selected_auto = None
        elif robot_mode == RobotMode.DISABLED:
            if self.__prev_robot_mode == RobotMode.AUTONOMOUS:
                self.__selected_auto = None
                self.runner.reset_action_list()

            if self.__selected_auto != self.__prev_selected_auto:
                if self.__selected_auto != None:
                    rospy.loginfo(f"Recreating actions for {self.__selected_auto.display_name}")
                    self.__selected_auto_action = self.__selected_auto.get_action()
                    rospy.loginfo(f"Actions recreated for {self.__selected_auto.display_name}")
                elif self.__prev_robot_mode == RobotMode.AUTONOMOUS:
                    pass
                else:
                    rospy.logerr(f"Selected auto is none!", throttle_duration_sec=10)
                    
            if self.__selected_auto is not None:
                self.__selected_auto.reset()

        self.__prev_robot_mode = robot_mode
        self.__prev_selected_auto = self.__selected_auto

        if self.__prev_robot_mode == RobotMode.AUTONOMOUS:
            # Only loop in auto
            self.runner.loop(robot_mode)
    

    def set_auto_id(self, msg : AutoMode) -> None:
        try:
            self.__selected_auto = self.AUTO_NAME_TO_AUTOBASE[IDS_TO_AUTO_NAME[msg.auto_mode]] 
            rospy.loginfo_throttle(10, f"Recived auto mode of - {msg.auto_mode} mapped to {IDS_TO_AUTO_NAME[msg.auto_mode]}")
        except Exception as e:
            rospy.logerr(f"Unable to look up auto with id {msg.auto_mode}\n error of {e}")

def signal_handler(sig, frame):
    rospy.logerr("Auto node recived SIGINT - shutting down")
    sys.exit(0)     

def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("auto_node")
    auto = AutoNode()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()


if __name__ == '__main__':
    main()
