
# should be __init__.py in autos directory
from __future__ import annotations
from enum import Enum
from auto_base import AutoBase
# import all autos here
from Test4Note import Test4Note
from TestCmdVel import TestCmdVel
from TestCmdVelCircle import TestCmdVelCircle
from FourCoral import FourCoralNonProcessor
from FourCoral import FourCoralProcessor
from FourCoral import PushFourCoral
from TestDriveForwardBack import TestDriveForwardBack
from DriveForward import DriveForward
from OneCoralMid import OneCoralMidNonProcessor
from OneCoralMid import OneCoralMidProcessor

class AutonomousNames(str, Enum):
    Test4Note = "2025_4_Note"
    TestCmdVel = "2025_cmd_vel_test"
    TestCmdVelCircle = "2024_cmd_vel_circle_test"
    FourCoralNonProcessor = "2025_4_Coral"
    TestDriveForwardBack = "test_drive_forward_back"
    FourCoralProcessor = "2025_4_Coral_Processor_Side"
    DriveForward = "DriveForward"
    PushFourCoral = "2025_Push_4_Coral"
    OneCoralMidProcessor = "2025_2_Mid_Coral"
    OneCoralMidNonProcessor = "2025_2_Mid_Coral_NonProcessor"

    def __str__(self) -> str:
        return str.__str__(self)


def init_auto_selection_map() -> dict[AutonomousNames, AutoBase]:
    """
    Returns an autonomous selection map, mapping auto names to auto programs.
    """
    return {
        AutonomousNames.DriveForward: DriveForward(),
        AutonomousNames.Test4Note: Test4Note(),
        AutonomousNames.TestCmdVelCircle: TestCmdVelCircle(),
        AutonomousNames.FourCoralNonProcessor: FourCoralNonProcessor(),
        AutonomousNames.TestDriveForwardBack: TestDriveForwardBack(),
        AutonomousNames.FourCoralProcessor: FourCoralProcessor(),
        AutonomousNames.PushFourCoral: PushFourCoral(),
        AutonomousNames.OneCoralMidNonProcessor: OneCoralMidNonProcessor(),
        AutonomousNames.OneCoralMidProcessor: OneCoralMidProcessor()
    }
