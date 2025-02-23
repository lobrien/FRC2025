import commands2
import commands2.cmd
from wpimath.geometry import Pose2d

from commands.follow_trajectory_command import FollowTrajectoryCommand
from subsystems.drive_subsystem import DriveSubsystem
from commands.drive_to_goal import DriveToGoal


class Autos:
    """Class to hold autonomous command factories"""

    def __init__(self):
        raise Exception("This is a utility class, don't make instances of it.")

    @staticmethod
    def side_step(drive: DriveSubsystem):
        """Autonomous routine that drives forward, waits, then moves left."""
        return commands2.cmd.sequence(
            DriveToGoal(drive, Pose2d(36, 0, 10)),
            commands2.WaitCommand(1),
            DriveToGoal(drive, Pose2d(0, 48, 0)),
        )

    @staticmethod
    def goal_sequence(drive: DriveSubsystem, poses: list[Pose2d]):
        """Autonomous routine that drives to a list of poses"""
        return commands2.cmd.sequence(*[DriveToGoal(drive, pose) for pose in poses])

    @staticmethod
    def forward(drive: DriveSubsystem):
        """Autonomous routine that drives forward"""
        return commands2.cmd.sequence(DriveToGoal(drive, Pose2d(36, 0, 10)))

    @staticmethod
    def trajectory(drive: DriveSubsystem):
        """Autonomous routine that follows a trajectory"""
        # Note the filename here does not have the .traj extension (it MUST in the file system, though)
        return FollowTrajectoryCommand("Bottom_Algae", drive)

    # def forward_elevator(
    #     drive: subsystems.drivesubsystem.DriveSubsystem,
    #     elevator: subsystems.elevatorsubsystem.ElevatorSubsystem,
    # ):
    #     """Autonomous routine that drives forward and moves elevator to mid
    #     TODO: Must understand why ad8336 (2025-02-10) worked. Only change was flip order. But wpilib docs say order doesn't matter.
    #     """
    #     return commands2.cmd.parallel(
    #         ElevatorCommands.move_goal(ElevatorConsts.MID, elevator),
    #         DriveCommands.drive_goal(Positions.AWAY, drive),
    #     )
