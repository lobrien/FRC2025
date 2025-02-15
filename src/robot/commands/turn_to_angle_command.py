import commands2

from subsystems.drive_subsystem import DriveSubsystem

#Command Class
class TurnToAngleCommand(commands2.Command): 
    def __init__(self, drive_subsystem:DriveSubsystem, should_set_fn:callable):
        super().__init__()
        self.drive_subsystem = drive_subsystem
        self.set_fn = should_set_fn() # What degree to turn to

        self.addRequirements(drive_subsystem)

    def initialize(self):  # Setting function
        pass

    def execute(self):  # What actions it does
        if self.set_fn:
            self.drive_subsystem.set_drive_angle(desired_angle_degrees=0)

    def isFinished(self) -> bool:
        return True

