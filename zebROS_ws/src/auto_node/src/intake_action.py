from action import Action
from datetime import datetime
from typing import List
from subsystem import Subsystem

class IntakeAction(Action):
    """An action that waits for the specified amount of time in seconds"""
    def __init__(self, time_to_wait_sec : float):
        self.__time_to_wait = time_to_wait_sec
        self.__start_time = datetime.now()

    def start(self):
        print(f"start called with ttw {self.__time_to_wait}")
        self.__start_time = datetime.now()

    def update(self):
        pass

    def done(self):
        pass

    def isFinished(self) -> bool:
        duration = datetime.now() - self.__start_time
        if duration.total_seconds() > self.__time_to_wait:
            print("Wait action finished!")
            return True 

    def affectedSystems(self) -> List[Subsystem]:
        return [ Subsystem.INTAKE ]
