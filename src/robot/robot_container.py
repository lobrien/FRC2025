import commands2
import wpilib
from commands2.button import CommandXboxController

from commands.print_something_command import PrintSomethingCommand
from constants.operatorinterfaceconstants import OperatorInterfaceConstants
from subsystems import drive_subsystem
from commands import drive_forward_command, drive_with_joystick_command, print_something_command


class RobotContainer:
    def __init__(self):
        self.drive_subsystem = drive_subsystem.DriveSubsystem()
        self.controller = CommandXboxController(OperatorInterfaceConstants.DRIVER_CONTROLLER_PORT)
        self.autonomous_command = PrintSomethingCommand("WHEA Autonomous Command Executed")


        self.controller.a().onTrue(PrintSomethingCommand("WHEA A Button Pressed"))
    def get_auto_command(self):
        return self.autonomous_command
    

