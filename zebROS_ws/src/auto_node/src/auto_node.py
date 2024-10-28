#!/usr/bin/python3
from __future__ import annotations

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
from path_loader import PathLoader
from frc_utils.match_data_helper import RobotStatusHelper, Alliance, RobotMode
from std_msgs.msg import String

# number to str of auto name, what /auto/auto_mode needs to send to run a given auto
IDS_TO_AUTO_NAME: dict[int, str] = {1: AutonomousNames.TestCmdVel, 
                                    2: AutonomousNames.Test4Note,
                                    3: AutonomousNames.TestCmdVelCircle}


class AutoNode():
    def __init__(self):        
        self.__selected_autonomous_string: str = ""
        self.__prev_selected_autonomous_string : str = ""

        self.runner = ActionRunner()

        self.__prev_robot_mode = RobotMode.DISABLED
        self.__selected_auto : AutoBase = None
        self.__prev_selected_auto : AutoBase = None
        self.__selected_auto_action : Action = None
        self.__path_loader : PathLoader = PathLoader()
        #self.__path_pub = rospy.Publisher("/auto/selected_path")
        self.__robot_status = RobotStatusHelper() 
        self.AUTO_NAME_TO_AUTOBASE = autos.init_auto_selection_map()
        self.timer = rospy.Timer(period=rospy.Duration(1.0/50.0), callback=self.loop)
        self.__auto_sub = rospy.Subscriber("/auto/auto_mode", AutoMode, self.set_auto_id)
        self.__auto_name_pub = rospy.Publisher("/auto/auto_name", String, queue_size=1)

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
        # this is ok because prematch we sit disabled and the FMS reports auto, so all the setup will happen prematch
        # i.e (disabled, auto) -> (enabled, auto) -> (disabled, auto) -> (enabled, teleop)
        # want to preempt when going from auto to teleop without a disable first (does not happen often)
        # dont want to preempt just generically in teleop because could lead to problems
        if robot_mode == RobotMode.TELEOP and self.__prev_robot_mode == RobotMode.AUTONOMOUS:
            self.__selected_auto = None
            self.runner.preempt_all_actions() # want to preempt from auto -> teleop/disabled but not otherwise
            self.runner.reset_action_list()
        if robot_mode == RobotMode.TELEOP:
            self.__selected_auto = None
            self.runner.reset_action_list()
            
        # only way this happens is if we havent set a valid auto, otherwise it will just be the last valid selected
        if self.__selected_auto is None:                
            err_str = "No auto selected!"
            if robot_mode == RobotMode.TELEOP:
                err_str += " Must be disabled (Currently in teleop)"
            rospy.logerr_throttle(1, err_str)
            self.__auto_name_pub.publish(String(err_str))
        else:
            self.__auto_name_pub.publish(String(self.__selected_auto.display_name))

        # need to stress test this a ton and make sure there are no edge cases 
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
                self.runner.preempt_all_actions() # also want to preempt from auto -> disable 
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
        robot_mode : RobotMode = self.__robot_status.get_mode()
        try:
            self.__selected_auto = self.AUTO_NAME_TO_AUTOBASE[IDS_TO_AUTO_NAME[msg.auto_mode]]
            if self.__selected_auto.expected_trajectory_count != 0:
                self.__path_loader.set_auto_name(IDS_TO_AUTO_NAME[msg.auto_mode]) # will load the path for the selected auto
            else:
                rospy.logwarn_throttle_identical(10, f"AUTO NODE NOT LOADING A PATH for {self.__selected_auto.display_name} because expected trajectory count is 0")
            
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
