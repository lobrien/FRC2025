from commands2 import Command
from wpimath.geometry import Pose2d

from subsystems.drive_subsystem import DriveSubsystem


class DriveToGoal(Command):
    """
    Allows drive_to_goal to be initialized and uses the drive subsystem to drive to the goal
    """
    def __init__(self, drive_subsystem: DriveSubsystem, goal_pose: Pose2d):
        super().__init__()  # Allows "drive_to_goal" to be Initialized, executed, and end by itself
        self.drive_subsystem = drive_subsystem
        self.goal_pose = goal_pose
        self.addRequirements(drive_subsystem)
    """
    Initializes a goal position for drive_to_goal
    """
    def initialize(self):
        self.drive_subsystem.set_goal_pose(self.goal_pose)
    """
    Executes the drive_to_goal code
    """
    def execute(self):
        self.drive_subsystem.drive_to_goal()
    
    def isFinished(self) -> (bool):  
        """
        when the robot is at the goal, tells the drive command its on the goal so it can stop
        """
        return self.drive_subsystem.is_at_goal()

    def end(self, interrupted: bool):
        self.drive_subsystem.stop()


