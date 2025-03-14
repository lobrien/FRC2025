import commands2
import commands2.cmd
from wpimath.geometry import Pose2d
from wpimath.units import inchesToMeters, degreesToRadians
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import inchesToMeters

from commands.drive_forward_command import DriveForwardCommand
from subsystems.drive_subsystem import DriveSubsystem
from commands.drive_to_goal import DriveToGoal
from subsystems.elevator_subsystem import ElevatorSubsystem
from commands.elevator_command import ElevatorMoveToGoalHeightContinuously
from constants.elevatorconstants import ElevatorConstants
from subsystems.coral_subsystem import CoralSubsystem


class Autos:
    """Class to hold autonomous command factories"""

    def __init__(self):
        raise Exception("This is a utility class, don't make instances of it.")

    @staticmethod
    def side_step(drive: DriveSubsystem):
        """Autonomous routine that drives forward, waits, then moves left."""
        return commands2.cmd.sequence(
            DriveToGoal(drive, Pose2d(inchesToMeters(36), 0, 0)),
            commands2.WaitCommand(1),
            DriveToGoal(drive, Pose2d(inchesToMeters(36), inchesToMeters(48), 0))
        )

    @staticmethod
    def goal_sequence(drive: DriveSubsystem, poses: list[Pose2d]):
        """Autonomous routine that drives to a list of poses"""
        return commands2.cmd.sequence(*[DriveToGoal(drive, pose) for pose in poses])

    @staticmethod
    def forward(drive: DriveSubsystem):
        """Autonomous routine that drives forward"""
        return DriveToGoal(drive, Pose2d(inchesToMeters(120.0), 0, 0))
        #return DriveForwardCommand(drive, 1, 120)

    def forward_elevator_and_score(
        drive: DriveSubsystem,
        elevator: ElevatorSubsystem,
        coral: CoralSubsystem,
    ):
        """Autonomous routine that drives forward and moves elevator to level 3
        TODO: Must understand why ad8336 (2025-02-10) worked. Only change was flip order. But wpilib docs say order doesn't matter.
        """
        return commands2.cmd.sequence(
            DriveToGoal(drive_subsystem = drive, goal_pose = Pose2d(inchesToMeters(69.75), 0.0, 0.0)),
            commands2.WaitCommand(seconds = 10),
            print("waited"),
            ElevatorMoveToGoalHeightContinuously(goal_height = ElevatorConstants.LEVEL_THREE,elev = elevator),
            coral.outtake(),
            print("outtaken")
        )
