import unittest
from commands.drive_to_goal import DriveToGoal
from subsystems.drive_subsystem import DriveSubsystem
from wpimath.geometry import Pose2d


class TestDriveToGoal(unittest.TestCase):
    def test_can_create(self):
        drive_subsystem = DriveSubsystem()
        goal_pose = Pose2d(0, 0, 0)

        command = DriveToGoal(drive_subsystem, goal_pose)
        self.assertIsNotNone(command)
