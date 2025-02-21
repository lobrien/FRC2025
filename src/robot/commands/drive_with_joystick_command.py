import commands2

from constants.driveconstants import DriveConstants
from subsystems.drive_subsystem import DriveSubsystem


class DriveWithJoystickCommand(
    commands2.Command
):  # Class type command from the library command2
    def __init__(
        self, drive: DriveSubsystem, drive_percent_fn: callable
    ):  # What value this needs to run
        super().__init__()
        self.drive_subsystem = drive  # Varible drive_subsystem is drive from __init__()
        self.drive_percent_fn = drive_percent_fn  # Same concept as above

        self.addRequirements(drive)  # Requires this subsystem

    def execute(self):  # What actions it does
        x_speed, y_speed, rot_speed = (
            self.drive_percent_fn()
        )  # Set turn and drive speed to values got from function
        x_speed = x_speed * DriveConstants.MAX_SPEED_INCHES_PER_SECOND * -1
        y_speed = y_speed * DriveConstants.MAX_SPEED_INCHES_PER_SECOND * -1
        # TODO: Shouldn't we have a dedicated maximum rotation speed? Would it be in inches/sec or degrees/sec?
        rot_speed = rot_speed * DriveConstants.MAX_DEGREES_PER_SECOND * -1

        self.drive_subsystem.drive(
            x_speed_inches_per_second=y_speed,
            y_speed_inches_per_second=x_speed,
            rot_speed_degrees_per_second=rot_speed,
        )  # Give these values to drive function

    def isFinished(
        self,
    ):  # When something else happens then this is done, cause this is a default command
        return True
