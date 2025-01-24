import wpilib
import commands2

# DriveForwardCommand Class
class DriveForwardCommand(commands2.Command): 
    def __init__(self, drive_subsystem, duration, speed = 0.5):
        self.drive_subsystem = drive_subsystem
        self.duration = duration
        self.speed = speed  # Default speed is 0.5
        self.timer = wpilib.Timer()

    def initialize(self):  # Setting function
        self.timer.reset()
        self.timer.start()  # Starting timer

    def execute(self):  # What actions it does
        self.drive_subsystem.drive(self.speed, self.speed)

    def isFinished(self):
        return self.timer.hasPeriodPassed(self.duration)

    def end(self):  # Stop driving
        self.drive_subsystem.drive(0.0, 0.0)