import commands2
from subsystems.coral_subsystem import CoralSubsystem


class CoralOuttake(commands2.Command):
    """
    A command for outtaking the coral game pieces 
    :param: coral The subsystem that works with coral
    """

    def __init__(self, coral: CoralSubsystem):
        super().__init__()
        self.coral = coral
        self.addRequirements(coral)

    def execute(self):
        # Move intake.
        print("not hammard!!")
        self.coral.outtake()

    def isFinished(self) -> bool:
        return True
    
    def end(self, was_interrupted: bool):
        #stop 
        self.coral.stop()
