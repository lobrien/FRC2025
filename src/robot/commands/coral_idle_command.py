import commands2
from subsystems.coral_subsystem import CoralSubsystem

class CoralIdle(commands2.Command):
    """
    A command for when coral subsystem needs to be idle
    :param: coral The subsystem that works with coral
    """

    def __init__(self, coral: CoralSubsystem):
        super().__init__()
        self.coral = coral
        self.addRequirements(coral)

    def execute(self):
        # Move intake.
        self.coral.stop()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        # Since this is the default command, it should only end if it is interrupted.
        pass

