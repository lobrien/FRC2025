import wpilib
import commands2

from subsystems.drive_subsystem import DriveSubsystem

# DriveForwardCommand Class
class DriveForwardCommand(commands2.Command): 
    def __init__(self, drive_subsystem:DriveSubsystem, duration:float, speed_inches_per_second:float = 12):
        super().__init__()
        self.drive_subsystem = drive_subsystem
        self.duration = duration # How long to run for
        self.speed = speed_inches_per_second  # Default speed is 12 inches per second
        self.timer = wpilib.Timer()

    def initialize(self):  # Setting function
        self.timer.start()

    def execute(self):  # What actions it does
        self.drive_subsystem.drive(x_speed_inches_per_second=self.speed, y_speed_inches_per_second=self.speed, rot_speed_degrees_per_second=0.0)

    def isFinished(self) -> bool:
        if self.timer.hasElapsed(period=self.duration):
            return True
        else:
            return False

    def end(self, was_interrupted:bool):  # Stop driving
        self.drive_subsystem.drive(x_speed_inches_per_second=0.0, y_speed_inches_per_second=0.0, rot_speed_degrees_per_second=0.0)

    
