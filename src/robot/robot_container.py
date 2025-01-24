import commands2
import wpilib
from wpimath import applyDeadband

# from commands import DriveForwardCommand
from subsystems import drive_subsystem
from commands import drive_forward_command, set_deadband_and_print

class RobotContainer:
    def __init__(self):
        self.drive_subsystem = drive_subsystem.DriveSubsystem()
        forward_command = drive_forward_command.DriveForwardCommand(self.drive_subsystem, 2.0)  # Duration = 2s
        self.autonomous_command = commands2.SequentialCommandGroup  # Make into a group of commands
        self.autonomous_command.addCommands(forward_command)  # Add drive forward command to auto
        self.autonomous_command.addCommands(deadband_command)

        deadband_command = set_deadband_and_print.SetDeadbandAndPrint() 
        self.teleop_command = commands2.SequentialCommandGroup  # Make into a group of commands
        self.teleop_command.addCommands(deadband_command)  # Add setting deadband command to teleop

        self.drive_controller = wpilib.XboxController(0)

        self.drive_subsystem.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.drive_subsystem.drive(self.drive_controller.getLeftY(),self.drive_controller.getLeftX()),

                self.drive_subsystem
            )
        )

    def get_auto_commands(self):
        return self.autonomous_command
    
    def get_teleop_commands(self):
        return self.teleop_command
    
    def set_controller_deadbands(self):
        applyDeadband(self.drive_controller.getLeftY(), 0.5)
        applyDeadband(self.drive_controller.getLeftX(), 0.5) 

    def print_driver_joystick_values(self):
        wpilib.SmartDashboard.putString('Left Y', 'at: {:5.1f}'.format(self.drive_controller.getLeftY()))
        wpilib.SmartDashboard.putString('Left X', 'at: {:5.1f}'.format(self.drive_controller.getLeftX()))

