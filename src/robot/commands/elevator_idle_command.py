import commands2
from subsystems.elevator_subsystem import ElevatorSubsystem

class ElevatorIdle(commands2.Command):
    """
    A command for when elevator subsystem needs to be idle
    :param: elevator The subsystem that works with elevator
    """

    def __init__(self, elevator: ElevatorSubsystem):
        super().__init__()
        self.elevator = elevator
        self.addRequirements(elevator)

    def execute(self):
        # Stop elevator.
        self.elevator.elevator_stop()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        # Since this is the default command, it should only end if it is interrupted.
        pass

