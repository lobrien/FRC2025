import commands2
import wpilib
from commands2.button import CommandXboxController
from wpimath import applyDeadband
from commands2 import RepeatCommand


from commands.print_something_command import PrintSomethingCommand
from constants.operatorinterfaceconstants import OperatorInterfaceConstants
from subsystems.drive_subsystem import DriveSubsystem
from commands.drive_forward_command import DriveForwardCommand
from commands.drive_with_joystick_command import DriveWithJoystickCommand

class RobotContainer:
    def __init__(self):
        self.drive_subsystem = DriveSubsystem() #Update the array when ready
        self.controller = CommandXboxController(OperatorInterfaceConstants.DRIVER_CONTROLLER_PORT)
        self.autonomous_command = DriveForwardCommand(self.drive_subsystem, 5)


        self.controller.a().onTrue(PrintSomethingCommand("WHEA A Button Pressed"))

        applyDeadband(value=self.controller.getLeftY(), deadband=0.1) #deleted the deadband command and set it here
        applyDeadband(value=self.controller.getLeftX(), deadband=0.1)

    def get_auto_command(self):
        return self.autonomous_command
    

