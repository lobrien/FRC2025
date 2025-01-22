import commands2
import wpilib

# from commands import DriveForwardCommand
from subsystems import drive_subsystem

class RobotContainer:
    def __init__(self):
        self.drive_subsystem = drive_subsystem.DriveSubsystem()
        # self.forward_command = DriveForwardCommand(self.drive_subsystem, 2.0)  # Duration = 2s
        # self.autonomous_command = CommandGroup()  # Make into a group of commands
        # self.autonomous_command.addSequential(self.forward_command)  # Add drive forward command to auto

        self.drive_controller = wpilib.XboxController(0)

        self.drive_subsystem.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.drive_subsystem.drive(self.drive_controller.getLeftY(),self.drive_controller.getLeftX()),

                self.drive_subsystem
            )
        )
