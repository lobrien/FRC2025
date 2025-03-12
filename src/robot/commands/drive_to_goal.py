from commands2 import Command
from wpimath.geometry import Pose2d

from subsystems.drive_subsystem import DriveSubsystem


class DriveToGoal(Command):
    def __init__(self, drive_subsystem: DriveSubsystem, goal_pose: Pose2d):
        super().__init__()  # Allows "drive_to_goal" to be Initialized, executed, and end by itself
        self.drive_subsystem = drive_subsystem
        self.goal_pose = goal_pose
        self.addRequirements(self.drive_subsystem)

    def initialize(self):
        self.drive_subsystem.set_goal_pose(self.goal_pose)
        print("Drive to goal is initializing")

    def execute(self):
        self.drive_subsystem.drive_to_goal()
    
    def isFinished(self) -> (bool):  
        """
        when the robot is at the goal, tells the drive command its on the goal so it can stop
        """
        return self.drive_subsystem.is_at_goal()

    def end(self, interrupted: bool):
        if interrupted:
            # for debugging
            print("DriveToGoal command was interrupted")
        print("drive to goal have stop")
        self.drive_subsystem.stop()


