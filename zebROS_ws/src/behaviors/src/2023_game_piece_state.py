#!/usr/bin/env python3
import rospy
import sensor_msgs.msg
import frc_msgs.msg
import std_msgs.msg
from message_filters import ApproximateTimeSynchronizer, Subscriber as SyncSubscriber
from behavior_actions.msg import *
import math
from enum import Enum

from dynamic_reconfigure.server import Server
from behaviors.cfg import GamePieceState2023Config

class GamePieceState:

    def __init__(self) -> None:
        self.state = GamePieceState2023.NONE
        self.offset_from_center = math.inf

        self.button_box_sub = rospy.Subscriber("/frcrobot_jetson/button_box_state_controller", frc_msgs.msg.ButtonBoxState, queue_size=1)
        
        self.terabees_sub = rospy.Subscriber("/intake/reader/state", IntakeState2023, callback=self.terabeeCB, queue_size=1)

        self.game_piece_pub = rospy.Publisher("game_piece_state", GamePieceState2023, queue_size=1)

        server = Server(GamePieceState2023Config, self.drCallback)

        # meters
        self.intake_width = rospy.get_param("intake_width")
        self.cube_threshold = rospy.get_param("cube_threshold")
        rospy.loginfo(f"game_piece_state_2023 : intake_width is {self.intake_width}, cube_threshold is {self.cube_threshold}")

        server.update_configuration({"intake_width": self.intake_width, "cube_threshold": self.cube_threshold})

    def publish_states(self):
        gpmsg = GamePieceState2023()
        gpmsg.header.frame_id = "intake"
        gpmsg.header.stamp = rospy.Time.now()
        gpmsg.offset_from_center = self.offset_from_center
        gpmsg.game_piece = self.state
        self.game_piece_pub.publish(gpmsg)

    def drCallback(self, config, level):
        self.intake_width = config.intake_width
        self.cube_threshold = config.cube_threshold
        return config

    def terabeeCB(self, msg: IntakeState2023):
        if msg.leftDistance != msg.leftDistance or msg.rightDistance != msg.rightDistance:
            self.offset_from_center = math.nan
            self.state = GamePieceState2023.NONE
            self.publish_states()
        else:
            # ex.       1
            # 0.........0
            # ..2...6....
            # ->2   4<---
            # left reading = 2
            # right reading = 4
            # both relative to left = 2, 6
            # average is 4
            # center of intake is 5 (10/2)
            # return center - average (positive is left) = 1

            width_of_object = max((self.intake_width - msg.rightDistance), msg.leftDistance) - min((self.intake_width - msg.rightDistance), msg.leftDistance)
            if width_of_object < self.cube_threshold:
                self.state = GamePieceState2023.BASE_TOWARDS_US_CONE # actually it's any cone (I think vertical looks the same as base towards) but we have so many cone states
            else:
                self.state = GamePieceState2023.CUBE

            self.offset_from_center = (self.intake_width / 2) - (((self.intake_width - msg.rightDistance) + msg.leftDistance) / 2)
            self.publish_states()

    def buttonBoxCB(self, button_box):
        pass
        
    
if __name__ == "__main__":
    print("started")
    rospy.init_node("game_piece_state_2023", anonymous=True)
    rospy.loginfo("Starting 2023 game piece state server!")
    game_piece_state = GamePieceState()
    rospy.spin()
    rospy.loginfo("Successfully started: 2023 game piece state server!")
