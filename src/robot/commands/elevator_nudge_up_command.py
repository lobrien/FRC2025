import commands2

from constants.new_types import inches
from subsystems.elevator_subsystem import ElevatorSubsystem


class ElevatorNudgeUpCommand(commands2.Command):
    """
    A command that moves the elevator toward a goal height in inches.
    :param: goal  The height to move toward.
    :param: elev  The elevator subsystem to operate on.
    """

    def __init__(self, elev: ElevatorSubsystem):
        super().__init__()
        self.elev = elev
        self.addRequirements(elev)

    def initialize(self):
        current_height = self.elev.get_current_goal_pos_inches()
        # Set the goal height in inches.
        self.elev.set_goal_height_inches(current_height + 0.25)

    def execute(self):
        # Move the elevator toward the goal height.
        self.elev.move_to_goal()

    def isFinished(self):
        # Since this is the default command, it should only end if it is interrupted.
        return self.elev.is_at_goal()
    
    def end(self, interrupted):
        self.elev.elevator_stop()