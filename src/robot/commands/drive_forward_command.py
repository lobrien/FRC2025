import wpilib
import commands2

from subsystems.drive_subsystem import DriveSubsystem

# DriveForwardCommand Class
class DriveForwardCommand(commands2.Command): 
    def __init__(self, drive_subsystem:DriveSubsystem, duration:float, speed:float = 0.25):
        super().__init__()
        self.drive_subsystem = drive_subsystem
        self.duration = duration # How long to run for
        self.speed = speed  # Default speed is 0.5
        self.timer = wpilib.Timer()

    def initialize(self):  # Setting function
        self.timer.start()

    def execute(self):  # What actions it does
        self.drive_subsystem.drive(self.speed, self.speed)

    def isFinished(self) -> bool:
        if self.timer.hasElapsed(period=self.duration):
            return True
        else:
            return False

    def end(self, was_interupted:bool):  # Stop driving
        self.drive_subsystem.drive(0.0, 0.0)
