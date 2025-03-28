import commands2
from subsystems.drive_subsystem import DriveSubsystem

class SlowModeOffCommand(commands2.Command):
    """
    TODO:
    """

    def __init__(self, drive: DriveSubsystem):
        super().__init__()
        self.drive = drive
        self.addRequirements(drive)

    def execute(self):
        self.drive.slow_mode = False

    def isFinished(self) -> bool:
        return True