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
        self.autonomous_command = DriveForwardCommand(self.drive_subsystem, duration=5)

        self.teleop_command = DriveWithJoystickCommand(self.drive_subsystem, driving_percent=self.get_drive_value_from_joystick) #The teleop command is the drive with joystick commmand which takes the drive subsystem and the getting drive value function


        self.controller.a().onTrue(PrintSomethingCommand("WHEA A Button Pressed"))

        self.drive_subsystem.setDefaultCommand(self.teleop_command) #Set the teleop command as the default for drive subsystem
    
    def get_auto_command(self):
        return self.autonomous_command
    
    def get_drive_value_from_joystick(self):
        turn_percent = self.controller.getLeftX()  #Get the joystick values 
        drive_percent = self.controller.getLeftY()

        turn_percent = applyDeadband(value=self.controller.getLeftY(), deadband=0.1) #Apply deadband to the values above            
        drive_percent = applyDeadband(value=self.controller.getLeftX(), deadband=0.1)

        return turn_percent, drive_percent  #Gives us these variables when we call this function
    

