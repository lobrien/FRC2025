import commands2
from wpilib import XboxController

from subsystems.drive import DriveSubsystem
from util.ntloggerutility import NTLoggerUtility
from constants.operatorinterfaceconstants import OperatorInterfaceConstants


class DriveWithJoysticks(commands2.Command):
    def __init__(self, drive: DriveSubsystem, controller: XboxController) -> None:
        super().__init__()
        self.drive = drive
        self.controller = controller
        self.addRequirements(drive)

        self.logger = NTLoggerUtility("DriveLogs")

    def execute(self) -> None:
        self.logger.info("Command", "DriveWithJoysticks executing")
        self.drive.drive_field_relative(
            -self.controller.getRightY() / OperatorInterfaceConstants.DRIVE_SLOWER,  # Pushing joystick forward produces a negative number.
            -self.controller.getRightX() / OperatorInterfaceConstants.DRIVE_SLOWER,  # Pushing joystick left produces a negative number.
            -self.controller.getLeftX() / OperatorInterfaceConstants.DRIVE_SLOWER
            # Pushing jouystick left produces a negative number, but we want + for CCW/turn left.
        )