import wpilib
import commands2
from wpimath import applyDeadband

from constants.operatorinterfaceconstants import OperatorInterfaceConstants
from subsystems.drive_subsystem import DriveSubsystem

class DriveWithJoystickCommand(commands2.Command): 
    def __init__(self, drive: DriveSubsystem):
        super().__init__()
        self.drive_subsystem = drive

    def initialize(self):  # Setting function
        self.drive_controller = wpilib.XboxController(OperatorInterfaceConstants.DRIVER_CONTROLLER_PORT) #implemented another constant for user interface 

        applyDeadband(self.drive_controller.getLeftY(), 0.5) #deleted the deadband command and set it here
        applyDeadband(self.drive_controller.getLeftX(), 0.5) 

    def execute(self):  # What actions it does
        self.drive_subsystem.drive(self.drive_controller.getLeftY(),self.drive_controller.getLeftX()), self.drive_subsystem #Place the function that gives the value to drive def
        
        #Do we need the self.drive_subsystem at the end?

