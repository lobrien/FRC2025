import os

import choreo
import commands2
import wpilib
from subsystems.drive_subsystem import DriveSubsystem
from util.utils import is_red_alliance


class FollowTrajectoryCommand(commands2.Command):
    """
    A command to follow a Choreo trajectory
    :param: algae The subsystem that works with algae
    """

    def __init__(self, trajectory_filename : str, drive : DriveSubsystem):
        super().__init__()
        # roborio default path is /home/lvuser/deploy
        if os.path.exists("/home/lvuser/deploy"):
            trajectory_path = os.path.join("/home", "lvuser", "deploy", trajectory_filename)
        else:
            # Load from deploy/, which is sibling of commands/
            trajectory_path = os.path.join(os.path.dirname(__file__), "..", "deploy", trajectory_filename)
        self.trajectory = choreo.load_swerve_trajectory(trajectory_path) # Load the trajectory from a file

        self.drive = drive
        self.addRequirements(drive)
        # timer needed for trajectory following (samples the trajectory at a fixed rate)
        self.timer = wpilib.Timer()

    def initialize(self):
        initial_pose = self.trajectory.get_initial_pose(is_red_alliance())
        self.drive.reset_odometry(initial_pose)
        self.timer.restart()

    def execute(self):
        if self.trajectory:
            # Sample the trajectory at the current time
            sample = self.trajectory.sample(self.timer.get(), is_red_alliance())
            if sample:
                self.drive.follow_trajectory(sample)

    def isFinished(self) -> bool:
        if self.trajectory:
            return self.timer.hasPeriodPassed(self.trajectory.get_total_time() + 1.0)
        return True

    def end(self, interrupted: bool):
        if interrupted:
            print("Trajectory following was interrupted (time elapsed: {})".format(self.timer.get()))
        else:
            print("Trajectory following completed (time elapsed: {})".format(self.timer.get()))