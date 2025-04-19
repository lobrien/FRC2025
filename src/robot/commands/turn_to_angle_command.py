import commands2

from subsystems.drive_subsystem import DriveSubsystem

#Command Class  
    """
    Class used to set the angle that the robot used to turn
    gets variables from DriveSubsystem
    """
class TurnToAngleCommand(commands2.Command): 
    def __init__(self, drive_subsystem:DriveSubsystem, should_set_fn:callable):
        super().__init__()
        self.drive_subsystem = drive_subsystem
        self.set_fn = should_set_fn() # What degree to turn to

        self.addRequirements(drive_subsystem)

    """
    sets function
    """
    def initialize(self):  # Setting function
        pass

    """
    Executes code
    """
    def execute(self):  # What actions it does
        if self.set_fn:
            self.drive_subsystem.set_drive_angle(desired_angle_degrees=0)

    """
    makes sure code stops
    """
    def isFinished(self) -> bool:
        return True

