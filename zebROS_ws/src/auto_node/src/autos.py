
# should be __init__.py in autos directory
from __future__ import annotations
from enum import Enum
from auto_base import AutoBase
# import all autos here
from Test4Note import Test4Note
from TestCmdVel import TestCmdVel
from TestCmdVelCircle import TestCmdVelCircle
from Test2Coral import Test2Coral

class AutonomousNames(str, Enum):
    Test4Note = "2025_4_Note"
    TestCmdVel = "2025_cmd_vel_test"
    TestCmdVelCircle = "2024_cmd_vel_circle_test"
    Test2Coral = "2025_2_Coral"

    def __str__(self) -> str:
        return str.__str__(self)


def init_auto_selection_map() -> dict[AutonomousNames, AutoBase]:
    """
    Returns an autonomous selection map, mapping auto names to auto programs.
    """
    return {
        AutonomousNames.TestCmdVel: TestCmdVel(),
        AutonomousNames.Test4Note: Test4Note(),
        AutonomousNames.TestCmdVelCircle: TestCmdVelCircle(),
        AutonomousNames.Test2Coral: Test2Coral()
    }
