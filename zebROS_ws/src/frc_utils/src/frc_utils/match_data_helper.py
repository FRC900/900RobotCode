from enum import Enum
from frc_msgs.msg import MatchSpecificData
import rospy

# should match the message
class Alliance(Enum):
    RED = 0
    BLUE = 1
    UNKNOWN = -1

class RobotStatusHelper:
    def __init__(self):
        self.__match_data_msg: MatchSpecificData = None
        rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, self.__update, tcp_nodelay=True)

    def __update(self, msg: MatchSpecificData):
        self.__match_data_msg = msg

    def get_message(self) -> MatchSpecificData:
        return self.__match_data_msg

    def get_alliance(self) -> Alliance:
        return Alliance(self.__match_data_msg)

    def get_match_time(self) -> float:
        return self.__match_data_msg.matchTimeRemaining
    
    def is_connected(self) -> bool:
        return self.__match_data_msg.FMSAttached
    
    def enabled(self) -> bool:
        return self.__match_data_msg.Enabled
    
    def disabled(self) -> bool:
        return self.__match_data_msg.Disabled
    
    def is_autonomous(self) -> bool:
        return self.__match_data_msg.Autonomous
    
    def is_teleop(self) -> bool:
        return self.__match_data_msg.OperatorControl