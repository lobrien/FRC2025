import wpilib
import commands2

from constants.operatorinterfaceconstants import OperatorInterfaceConstants
from constants.driveconstants import DriveConstants
from subsystems.drive_subsystem import DriveSubsystem

class DriveWithJoystickCommand(commands2.Command): #Class type command from the libary command2
    def __init__(self, drive: DriveSubsystem, drive_percent_fn:callable): #What value this needs to run
        super().__init__()
        self.drive_subsystem = drive #Varible drive_subsystem is drive from __init__()
        self.drive_percent_fn = drive_percent_fn #Same concept as above

        self.addRequirements(drive) #Requires this subsystem

    def execute(self):  # What actions it does        
        x_speed, y_speed, rot_speed = self.drive_percent_fn() #Set turn and drive speed to values got from function
        # At this point, the values are ±1, and dimensionless/unitless.
        # TODO: drive_subsystem.drive() expects speeds in inches/sec and rotation in degrees/second,
        # but the most we can provide at the moment is 1 inch/second and 1 degree/second.
        x_speed = x_speed * DriveConstants.MAX_SPEED_INCHES_PER_SECOND * -1
        y_speed = y_speed * DriveConstants.MAX_SPEED_INCHES_PER_SECOND * -1
        rot_speed = rot_speed * DriveConstants.MAX_SPEED_INCHES_PER_SECOND * -1 /2
        # Joystick y axis is back/forward, but robot +x is forward, so swap x and y.
        # TODO: Rod thinks there needs to be negation in here, double check by 
        # hard-coding each parameter with known values.  We want to get this correct,
        # so we can also call drive() from position control loops.

        self.drive_subsystem.drive(x_speed_inches_per_second=y_speed, y_speed_inches_per_second=x_speed, rot_speed_degrees_per_second=rot_speed) #Give these values to drive function

    def isFinished(self): #When something else happends then this is done, cause this is a default command
        return True
    
    def resetGyro(self, drive_subsystem:DriveSubsystem):
        self.drive_subsystem = drive_subsystem
        
        self.drive_subsystem.gyro.set_yaw(0.0)

