import commands2
from subsystems.elevator_subsystem import ElevatorSubsystem

class ElevatorMoveToGoalHeight(commands2.CommandBase):
    """
    A command that moves the elevator toward a goal height in inches.
    :param: goal  The height to move toward.
    :param: elev  The elevator subsystem to operate on.
    """
    def __init__(self, goal_height: float, elev: ElevatorSubsystem):
        super().__init__()
        self.elev = elev
        self.goal = goal_height
        self.addRequirements(elev)

    def initialize(self):
        # Set the goal height in inches.
        self.elev.set_goal_height_inches(self.goal)

    def execute(self):
        # Move the elevator toward the goal height.
        self.elev.move_to_goal()

    def isFinished(self, interrupted: bool = False) -> bool:
        # Check if the elevator has reached the goal height.
        if interrupted:
            return True
        else:
            return self.elev.is_at_goal()