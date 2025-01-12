import commands2
from util.ntloggerutility import NTLoggerUtility
from datetime import datetime
import wpilib

from subsystems.drive import DriveSubsystem

class RunAutonomous(commands2.Command):
    """
    Drives the robot clockwise forward for 2 seconds, then backwards for 2 seconds.
    """

    def __init__(self, drive : DriveSubsystem):
        """
        Create member variables for the command.

        Args:
            drive: A variable holding a reference to the robot's DriveSubsystem.
        """
        super().__init__()
        self.drive = drive
        self.logger = NTLoggerUtility("DriveLogs")
        self.addRequirements(drive)

    def getTimestamp(self):
        return datetime.now().strftime("%H:%M:%S")

    def initialize(self) -> None:
        """
        Called once when the Command is scheduled.

        Write's a timestamped message to the "Command" log in the `DriveLogs` table that the command has been created.

        Use command composition to run the `CircleCWForward` command, then the `CircleCWBack` command.
        """
        timestamp = self.getTimestamp()
        msg = f"[%{timestamp}] RunAutonomousCommand: CircleCWForward -> CircleCWBack"
        self.logger.info("Command", msg)

        cmd = (CircleCWForward(self.drive)
            .andThen(CircleCWBack(self.drive)))

# Behavior in commands. Simple states.
class CircleCWForward(commands2.Command):
    """
    Drives the robot forward in a clockwise circle for 2 seconds
    """
    def __init__(self, drive: DriveSubsystem):
        super().__init__()
        self.drive = drive
        self.logger = NTLoggerUtility("DriveLogs")
        self.addRequirements(drive)

        self.timer = wpilib.Timer()

        timestamp = self.getTimestamp()
        msg = f"[%{timestamp}] CircleCWForward: initialized"
        self.logger.info("Command", msg)

    def getTimestamp(self) -> str:
        return datetime.now().strftime("%H:%M:%S")

    def initialize(self) -> None:
        self.timer.start()

    def execute(self) -> None:
        """
        Called continuously while the command is scheduled. (This is such simple behavior it could be in `initialize`.)

        """
        self.drive.drive(1.0, -0.5)

    def isFinished(self) -> bool:
        """
        :return `True` when the command should end. In this case, after 2 seconds.

        """
        return self.timer.get() > 2.0

    def end(self, interrupted: bool) -> None:
        """
        Stops the robot.
        :param interrupted:
        """
        msg = f"[%{timestamp}] CircleCWForward: ended"
        self.logger.info("Command", msg)
        self.drive.drive(0, 0)


class CircleCWBack(commands2.Command):
    """
     Drives the robot backwart in a clockwise circle for 2 seconds
     """

    def __init__(self, drive: DriveSubsystem):
        super().__init__()
        self.drive = drive
        self.logger = NTLoggerUtility("DriveLogs")
        self.addRequirements(drive)

        self.timer = wpilib.Timer()

        timestamp = self.getTimestamp()
        msg = f"[%{timestamp}] CircleCWBackwardCommand: initialized"
        self.logger.info("Command", msg)

    def getTimestamp(self) -> str:
        return datetime.now().strftime("%H:%M:%S")

    def initialize(self) -> None:
        self.timer.start()

    def execute(self) -> None:
        """
        Called continuously while the command is scheduled. (This is such simple behavior it could be in `initialize`.)

        """
        self.drive.drive(1.0, -0.5)

    def isFinished(self) -> bool:
        """
        :return `True` when the command should end. In this case, after 2 seconds.

        """
        return self.timer.get() > 2.0

    def end(self, interrupted: bool) -> None:
        """
        Stops the robot.
        :param interrupted:
        """
        self.drive.drive(0, 0)
