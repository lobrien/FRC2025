import commands2
from subsystems.algae_subsystem import AlgaeSubsystem

class AlgaeIntake(commands2.Command):
    """
    A command for intaking the algae game pieces 
    :param: algae The subsystem that works with algae
    """

    def __init__(self, algae: AlgaeSubsystem):
        super().__init__()
        self.algae = algae
        self.addRequirements(algae)

    def execute(self):
        # Move intake.
        self.algae.intake()

    def isFinished(self) -> bool:
        # checking if the algae is loaded
        return self.algae.is_algae_loaded()
    
    def end(self):
        #stop after the algae is loaded
        self.algae.stop()
