import unittest

from wpimath.geometry import Pose2d

from constants.driveconstants import DriveConstants
from subsystems.drive_subsystem import DriveSubsystem


class TestDriveSubsystem(unittest.TestCase):
    def test_can_create(self):
        drive_subsystem = DriveSubsystem()
        self.assertIsNotNone(drive_subsystem)

    def test_initialize_controllers(self):
        drive_subsystem = DriveSubsystem()
        xc, yc, rc = drive_subsystem._initialize_pid_controllers()
        # Everything should be zeroed out.
        assert xc.getGoal().position == 0.0
        assert yc.getGoal().position == 0.0
        assert rc.getGoal().position == 0.0
        assert xc.getGoal().velocity == 0.0
        assert yc.getGoal().velocity == 0.0
        assert rc.getGoal().velocity == 0.0

        # Confirm PID values.
        assert xc.getP() == DriveConstants.PIDX_KP
        assert yc.getP() == DriveConstants.PIDY_KP
        assert rc.getP() == DriveConstants.PID_ROT_KP
        # If we ever use I and D, uncomment these.
        # assert xc.getI() == DriveConstants.PIDX_KI
        # assert yc.getI() == DriveConstants.PIDY_KI
        # assert rc.getI() == DriveConstants.PIDR_KI
        # assert xc.getD() == DriveConstants.PIDX_KD
        # assert yc.getD() == DriveConstants.PIDY_KD
        # assert rc.getD() == DriveConstants.PID_ROT_KD

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
        drive_subsystem.set_goal_pose(Pose2d(1, 1, 0))
        drive_subsystem.drive_to_goal()
        self.assertFalse(drive_subsystem.is_at_goal())
