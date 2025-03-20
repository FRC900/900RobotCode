from action import Action
from datetime import datetime
import rospy

class WaitAction(Action):
    """An action that waits for the specified amount of time in seconds"""
    def __init__(self, time_to_wait_sec : float):
        self.__time_to_wait = time_to_wait_sec
        self.__start_time = rospy.Time.now()

    def start(self):
        print(f"Wait action called with time {self.__time_to_wait} seconds")
        self.__start_time = rospy.Time.now()

    def update(self):
        pass

    def done(self):
        pass

    def isFinished(self) -> bool:
        duration = (rospy.Time.now() - self.__start_time).to_sec()
        if duration > self.__time_to_wait:
            print("Wait action finished!")
            return True 
        else:
            return False

    def preempt(self):
        rospy.logwarn("Wait action preempted!!!!.... what did you want to happen")
        
