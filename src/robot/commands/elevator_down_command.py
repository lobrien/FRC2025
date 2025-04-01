import commands2
from subsystems.elevator_subsystem import ElevatorSubsystem

class ElevatorDown(commands2.Command):
    """
    Elevator goes down when left joystick triggered down :
    """

    def __init__(self, elevator: ElevatorSubsystem):
        super().__init__()
        self.elevator = elevator
        self.addRequirements(elevator)

    def execute(self):
        # Move intake.
        self.elevator.elevator_down()

    def isFinished(self) -> bool:
        return self.elevator.lower_limit_reached()
    
    def end(self, was_interrupted: bool):
        #stop 
        self.elevator.elevator_stop()
