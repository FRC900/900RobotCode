from abc import ABC, abstractmethod
from typing import List
from subsystem import Subsystem

class Action(ABC):
    """
    An abstract class that defines an action. Actions must be designed to be non-blocking
    """
    @abstractmethod
    def isFinished(self) -> bool:
        pass

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def done(self):
        pass

    @abstractmethod
    def affectedSystems(self) -> List[Subsystem]:
        pass 

    def __str__(self) -> str:
        return self.__class__.__name__
