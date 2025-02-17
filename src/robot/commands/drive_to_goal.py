import commands2.cmd
import wpimath.geometry
from commands2 import Command
from wpimath.geometry import Pose2d

from subsystems.drive_subsystem import DriveSubsystem

class DriveToGoal(Command):
    def __init__(self, drive_subsystem: DriveSubsystem, goal_pose: Pose2d): 
        super().__init__() # Allows "drive_to_goal" to be Initilized, executed, and end by itself
        self.drive_subsystem = drive_subsystem
        self.goal_pose = goal_pose

    def initialize(self):
        self.drive_subsystem.set_goal_pose(self.goal_pose)

    def execute(self):
        self.drive_subsystem.drive_to_goal()

    def end(self, interrupted: bool):
        self.drive_subsystem.stop()

    def isFinished(self) -> bool: # when the robot is at the goal, tells the drive command its on the goal so it can stop
        return self.drive_subsystem.is_at_goal()
