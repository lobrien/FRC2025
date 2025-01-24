import commands2
import wpilib

from subsystems import drive_subsystem
from commands import drive_forward_command, drive_with_joystick_command

class RobotContainer:
    def __init__(self):
        self.drive_subsystem = drive_subsystem.DriveSubsystem()
        joystick_drive_command = drive_with_joystick_command.DriveWithJoystickCommand   #Created a command for driving with joystick values

        # forward_command = drive_forward_command.DriveForwardCommand(self.drive_subsystem, 2.0)  # Duration = 2s       
        # self.autonomous_command = commands2.SequentialCommandGroup  # Make into a group of commands
        # self.autonomous_command.addCommands(forward_command)  # Add drive forward command to auto
        # self.autonomous_command.addCommands(deadband_command)

        # self.teleop_command = commands2.SequentialCommandGroup  # Make into a group of commands

        self.drive_subsystem.setDefaultCommand(
            commands2.RunCommand(
                lambda: joystick_drive_command, #Switch out the function with the command

                self.drive_subsystem
            )
        )

    # def get_auto_commands(self):
    #     return self.autonomous_command
    
    # def get_teleop_commands(self):
    #     return self.teleop_command

    # def print_driver_joystick_values(self):
    #     wpilib.SmartDashboard.putString('Left Y', 'at: {:5.1f}'.format(self.drive_controller.getLeftY()))
    #     wpilib.SmartDashboard.putString('Left X', 'at: {:5.1f}'.format(self.drive_controller.getLeftX()))

