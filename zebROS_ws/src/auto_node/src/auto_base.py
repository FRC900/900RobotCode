from enum import Enum
import typing

from abc import ABC, abstractmethod
from series_action import SeriesAction

class AutoBase(ABC):
    """
    Base class for all autonomous definitions.
    """
    def __init__(self, display_name : str) -> None:
        self.display_name: str = display_name

    @abstractmethod
    def get_action(self) -> SeriesAction:
        """
        Abstract method for getting the autonomous action.
        """
        pass

    def get_unique_name(self) -> str:
        """
        Returns the unique autonomous name based on the filters and display name.
        """
        return f"{self.game_piece.name}_{self.start_position.name}_{self.display_name}"
    
    def reset(self):
        pass
        #self.trajectory_iterator.reset_iterator()
