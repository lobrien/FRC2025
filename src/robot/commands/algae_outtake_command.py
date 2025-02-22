import commands2
from subsystems.algae_subsystem import AlgaeSubsystem

class AlgaeOuttake(commands2.CommandBase):
    """
    A command for outtaking the algae game pieces 
    :param: algae The subsystem that works with algae
    """

    def __init__(self, algae: AlgaeSubsystem):
        super().__init__()
        self.algae = algae
        self.addRequirements(algae)

    def execute(self):
        # Move wheels.
        self.algae.outtake()

    def isFinished(self) -> bool:
        return True
    
    def end(self, was_interrupted: bool):
        #stop 
        self.algae.stop()
