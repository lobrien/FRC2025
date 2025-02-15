import unittest

from wpimath.geometry import Pose2d, Rotation2d

from subsystems.drive_subsystem import DriveSubsystem


class TestDriveSubsystem(unittest.TestCase):
    def test_can_create(self):
        drive_subsystem = DriveSubsystem()
        self.assertIsNotNone(drive_subsystem)

    def test_initialize_controllers(self):
        drive_subsystem = DriveSubsystem()
        xc, yc, rc = drive_subsystem._initialize_pid_controllers()
        assert xc is not None
        assert yc is not None
        assert rc is not None

    def test_set_goal_pose(self):
        drive_subsystem = DriveSubsystem()
        goal_pose = Pose2d(1, 1, 0)
        drive_subsystem.set_goal_pose(goal_pose)
        xg, yg, rg = drive_subsystem.get_controllers_goals()
        self.assertEqual(xg, goal_pose.x)
        self.assertEqual(yg, goal_pose.y)
        self.assertEqual(rg, goal_pose.rotation().degrees())

    def test_drive_to_goal(self):
        drive_subsystem = DriveSubsystem()
        drive_subsystem.drive_to_goal()
        self.assertTrue(drive_subsystem.is_at_goal())
