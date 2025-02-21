import commands2

from constants.new_types import inches
from subsystems.elevator_subsystem import ElevatorSubsystem


class ElevatorMoveToGoalHeightContinuously(commands2.CommandBase):
    """
    A command that moves the elevator toward a goal height in inches.
    :param: goal  The height to move toward.
    :param: elev  The elevator subsystem to operate on.
    """

    def __init__(self, goal_height: inches, elev: ElevatorSubsystem):
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
        # Since this is the default command, it should only end if it is interrupted.
        if interrupted:
            return True
        else:
            return False
