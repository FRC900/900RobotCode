from enum import Enum
from frc_msgs.msg import MatchSpecificData
import rospy

# should match the message
class Alliance(Enum):
    RED = 0
    BLUE = 1
    UNKNOWN = -1

class RobotMode(Enum):
    DISABLED = 0
    TELEOP = 1
    AUTONOMOUS = 2
    TEST = 3

# @TODO don't crash when we haven't recived a message 
class RobotStatusHelper:
    def __init__(self):
        rospy.loginfo("Waiting 3 seconds for match data")
        self.__match_data_msg: MatchSpecificData = None
        rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, self.__update, tcp_nodelay=True)
        rospy.sleep(rospy.Duration(3)) # wait for some match data

    def __update(self, msg: MatchSpecificData):
        self.__match_data_msg = msg

    def get_message(self) -> MatchSpecificData:
        return self.__match_data_msg

    def get_alliance(self) -> Alliance:
        return Alliance(self.__match_data_msg.allianceColor)

    def get_mode(self) -> RobotMode:
        if self.disabled(): return RobotMode.DISABLED
        elif self.is_autonomous(): return RobotMode.AUTONOMOUS
        elif self.is_teleop(): return RobotMode.TELEOP
        elif self.is_test(): return RobotMode.TEST
        else: 
            rospy.logerr("FRC Match Data Helper - Unknown state")
            return RobotMode.DISABLED # if we don't know whats happening safest to say we are disabled 

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

    def is_test(self) -> bool:
        return self.__match_data_msg.Test