from abc import ABC, abstractmethod
from typing import List
from subsystem import Subsystem

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
    
    # does not actually have much effect (i.e multiple actions taking same subsytem works fine)
    @abstractmethod
    def affectedSystems(self) -> List[Subsystem]:
        pass 

    def __str__(self) -> str:
        return self.__class__.__name__
