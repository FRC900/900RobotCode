
# should be __init__.py in autos directory
from enum import Enum
from auto_base import AutoBase
# import all autos here
from Test4Note import Test4Note

class AutonomousNames(str, Enum):
    Test4Note = "Test4Note"

    def __str__(self) -> str:
        return str.__str__(self)


def init_auto_selection_map() -> dict[AutonomousNames, AutoBase]:
    """
    Returns an autonomous selection map, mapping auto names to auto programs.
    """
    return {
        AutonomousNames.Test4Note: Test4Note()
    }
