import commands2
from subsystems.coral_subsystem import CoralSubsystem


class CoralIntake(commands2.Command):
    """
    A command for intaking the coral game pieces 
    :param: coral The subsystem that works with coral
    """

    def __init__(self, coral: CoralSubsystem):
        super().__init__()
        self.coral = coral
        self.addRequirements(coral)

    def execute(self):
        # Move intake.
        print("still not hammard")
        self.coral.intake()

    def isFinished(self) -> bool:
        # checking if the coral is loaded
        return self.coral.is_coral_loaded()
    
    def end(self, was_interupted):
        #stop after the coral is loaded
        self.coral.stop()
