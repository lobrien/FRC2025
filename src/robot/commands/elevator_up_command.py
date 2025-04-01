import commands2
from subsystems.elevator_subsystem import ElevatorSubsystem

class ElevatorUp(commands2.Command):
    """
    Elevator moves up when left joystick is triggered up    
    """
    def __init__(self, elevator: ElevatorSubsystem):
        super().__init__()
        self.elevator = elevator
        self.addRequirements(elevator)

    def execute(self):
        # Move intake.
        self.elevator.elevator_up()

    def isFinished(self) -> bool:
        return self.elevator.higher_limit_reached()
    
    def end(self, was_interrupted: bool):
        #stop 
        self.elevator.elevator_stop()
