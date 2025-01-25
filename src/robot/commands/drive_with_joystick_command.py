import wpilib
import commands2

from constants.operatorinterfaceconstants import OperatorInterfaceConstants
from subsystems.drive_subsystem import DriveSubsystem

class DriveWithJoystickCommand(commands2.Command): 
    def __init__(self, drive: DriveSubsystem):
        super().__init__()
        self.drive_subsystem = drive

        self.addRequirements(drive)


    def execute(self):  # What actions it does        
        print("executing")

    def isFinished(self):
        return False

