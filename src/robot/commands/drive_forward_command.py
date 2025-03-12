import math

import wpilib
import commands2

from constants.new_types import inches_per_second, degrees_per_second
from subsystems.drive_subsystem import DriveSubsystem


# DriveForwardCommand Class
class DriveForwardCommand(commands2.Command):
    def __init__(
        self,
        drive_subsystem: DriveSubsystem,
        duration: float,
        speed_inches_per_second: inches_per_second = 12,
    ):
        super().__init__()
        self.drive_subsystem = drive_subsystem
        self.duration = duration  # How long to run for
        self.speed = speed_inches_per_second  # Default speed is 12 inches per second
        self.timer = wpilib.Timer()

        self.addRequirements(drive_subsystem)  # Requires this subsystem

    def initialize(self):  # Setting function
        self.timer.start()

    def execute(self):  # What actions it does
        # Step 1: Determine the robot's current heading, which we want to maintain
        current_rotation = self.drive_subsystem.get_heading_rotation2d()
        # Convert from Rotation2d to radians (which is the unit required by Python trig functions)
        current_heading = current_rotation.radians()

        # Step 2: Calculate the x and y components of the speed
        x_speed : inches_per_second = self.speed * math.cos(current_heading)
        y_speed : inches_per_second = self.speed * math.sin(current_heading)

        # Step 3: Drive the robot
        self.drive_subsystem.drive(
            x_speed_inches_per_second=x_speed,
            y_speed_inches_per_second=y_speed,
            rot_speed_degrees_per_second=degrees_per_second(0.0),
        )

    def isFinished(self) -> bool:
        if self.timer.hasElapsed(period=self.duration):
            return True
        else:
            return False

    def end(self, was_interrupted: bool):  # Stop driving
        self.drive_subsystem.drive(
            x_speed_inches_per_second=inches_per_second(0.0),
            y_speed_inches_per_second=inches_per_second(0.0),
            rot_speed_degrees_per_second=degrees_per_second(0.0),
        )
