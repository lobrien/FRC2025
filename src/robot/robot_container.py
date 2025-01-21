import commands2

# from commands import DriveForwardCommand
from subsystems import drive_subsystem

class RobotContainer:
    def __init__(self):
        self.drive_subsystem = drive_subsystem.DriveSubsystem()
        # self.forward_command = DriveForwardCommand(self.drive_subsystem, 2.0)  # Duration = 2s
        # self.autonomous_command = CommandGroup()  # Make into a group of commands
        # self.autonomous_command.addSequential(self.forward_command)  # Add drive forward command to auto

        desired_rotations = self.drive_subsystem.get_cancoder(),

        self.drive_joystick = commands2.button.CommandXboxController(0)

        self.drive_subsystem.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.drive_subsystem.drive(self.drive_joystick.getLeftY(),self.drive_joystick.getLeftX()),

                self.drive_subsystem.turn_motor.set_control(self.drive_subsystem.position_request.with_position(desired_rotations)),
                
                self.drive_subsystem
            )
        )
