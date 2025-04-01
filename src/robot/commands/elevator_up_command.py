import commands2
from subsystems.elevator_subsystem import ElevatorSubsystem

class ElevatorUp(commands2.Command):
    """
    Code used to determine what happens when the elevator goes up
    Uses elevator_init commands to allow the elevator to move
    """
    def __init__(self, elevator: ElevatorSubsystem):
        super().__init__()
        self.elevator = elevator
        self.addRequirements(elevator)

    """
    Executes code
    """
    def execute(self):
        # Moves the elevator up
        self.elevator.elevator_up()

    """
    Creates a boolean to see if the elevator has reached its height limit
    """
    def isFinished(self) -> bool:
        return self.elevator.higher_limit_reached()
    
    """
    Stops code if elevator gets too high
    """
    def end(self, was_interrupted: bool):
        #stop 
        self.elevator.elevator_stop()
