import commands2
from subsystems.algae_subsystem import AlgaeSubsystem

class AlgaeIdle(commands2.Command):
    """
    A command for when the algae needs to be idle
    :param: algae The subsystem that works with algae
    """

    def __init__(self, algae: AlgaeSubsystem):
        super().__init__()
        self.algae = algae
        self.addRequirements(algae)

    def execute(self):
        # Move intake.
        self.algae.stop()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        # Since this is the default command, it should only end if it is interrupted.
        pass

