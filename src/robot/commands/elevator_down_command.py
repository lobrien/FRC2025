import commands2
from subsystems.elevator_subsystem import ElevatorSubsystem

class ElevatorDown(commands2.Command):
    """
    Code used to determining how far the elevator can go down
    Uses elevator_init commands to allow the elevator to move
    """
    def __init__(self, elevator: ElevatorSubsystem):
        super().__init__()
        self.elevator = elevator
        self.addRequirements(elevator)

    """
    Executes the code
    """
    def execute(self):
        # Moves elevator down
        self.elevator.elevator_down()

    """
    Sets a boolean for if the elevator has reached its lower limit
    """
    def isFinished(self) -> bool:
        return self.elevator.lower_limit_reached()
    
    """
    Stops elevator if the lower limit has been reached
    """
    def end(self, was_interrupted: bool):
        #stop 
        self.elevator.elevator_stop()
