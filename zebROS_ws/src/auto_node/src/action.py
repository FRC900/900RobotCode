from abc import ABC, abstractmethod
from typing import List

class Action(ABC):
    """
    An abstract class that defines an action. Actions must be designed to be non-blocking
    """
    
    # checks if action is done 
    @abstractmethod
    def isFinished(self) -> bool:
        pass
    
    # called once at the start
    @abstractmethod
    def start(self):
        pass
    
    # called once each update loop
    @abstractmethod
    def update(self):
        pass
    
    # called when isFinished is true
    @abstractmethod
    def done(self):
        pass

    # must be implemented, will be called if auto is preempted for some reason. Should reset everything and make the robot not move
    @abstractmethod
    def preempt(self):
        pass 

    def __str__(self) -> str:
        return self.__class__.__name__
